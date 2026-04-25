/**
 * ik_mover — direct IK node (no OMPL, no path planning)
 *
 * Subscribes to  /target_pose  (geometry_msgs/PoseStamped)
 * Publishes to   /arm_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)
 *
 * Uses robot_state::setFromIK() directly — typically < 10 ms vs the 5-second
 * OMPL planning timeout.  Smooth motion comes from the JointTrajectoryController
 * interpolating to the target over TRAJ_DURATION seconds.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

static const std::string PLANNING_GROUP   = "arm";
static const std::string TRAJ_TOPIC       = "/arm_controller/joint_trajectory";

static constexpr double  IK_TIMEOUT_SEC   = 0.05;   // 50 ms per IK attempt
static constexpr double  TRAJ_DURATION    = 0.4;    // seconds for arm to reach target
static constexpr double  STRETCH_STEP     = 0.03;   // metres
static constexpr double  STRETCH_MIN_R    = 0.10;
static constexpr double  ARM_MAX_REACH    = 0.75;
static constexpr double  FAIL_EPSILON     = 0.02;   // 2 cm — "same pose" threshold
static constexpr double  FAIL_BACKOFF_SEC = 2.0;


class IKMoverNode : public rclcpp::Node
{
public:
  explicit IKMoverNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
  : Node("ik_mover", opts)
  {
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(TRAJ_TOPIC, 10);

    // Track current joint positions — used as IK seed for smooth, predictable solutions
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(js_mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i)
          current_joints_[msg->name[i]] = msg->position[i];
      });

    // Incoming target pose from the web UI
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 1,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(pending_mutex_);
        // Drop if within backoff window and nearly the same pose
        if (has_failed_ && this->now() < backoff_until_) {
          if (poseDist(msg->pose, last_failed_) < FAIL_EPSILON) {
            RCLCPP_DEBUG(get_logger(), "Backoff active — dropping repeated failed pose.");
            return;
          }
        }
        pending_ = *msg;
      });
  }

  void run()
  {
    // Load robot model from robot_description / robot_description_semantic parameters
    robot_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        shared_from_this());
    robot_model_  = robot_loader_->getModel();

    if (!robot_model_) {
      RCLCPP_FATAL(get_logger(), "Failed to load robot model — check robot_description param.");
      return;
    }

    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    jmg_ = robot_model_->getJointModelGroup(PLANNING_GROUP);
    if (!jmg_) {
      RCLCPP_FATAL(get_logger(), "Joint group '%s' not found.", PLANNING_GROUP.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(),
      "IK Mover ready (direct IK, no OMPL). Listening on /target_pose → %s",
      TRAJ_TOPIC.c_str());

    rclcpp::Rate rate(20);   // 20 Hz poll — IK is fast so we can check often
    while (rclcpp::ok()) {
      std::optional<geometry_msgs::msg::PoseStamped> pose;
      {
        std::lock_guard<std::mutex> lk(pending_mutex_);
        if (pending_) { pose = pending_; pending_.reset(); }
      }

      if (pose) {
        bool ok = solveAndPublish(pose->pose);
        std::lock_guard<std::mutex> lk(pending_mutex_);
        if (ok) {
          has_failed_ = false;
        } else {
          last_failed_  = pose->pose;
          has_failed_   = true;
          backoff_until_ = this->now() + rclcpp::Duration::from_seconds(FAIL_BACKOFF_SEC);
          RCLCPP_WARN(get_logger(),
            "IK failed for [%.3f, %.3f, %.3f]. Ignoring similar poses for %.1f s.",
            pose->pose.position.x, pose->pose.position.y, pose->pose.position.z,
            FAIL_BACKOFF_SEC);
        }
      }
      rate.sleep();
    }
  }

private:
  // ── IK + stretch-toward ────────────────────────────────────────────────────

  bool solveAndPublish(const geometry_msgs::msg::Pose & target)
  {
    if (tryIK(target)) return true;

    // Stretch-toward: walk back along origin→target ray
    const double tx = target.position.x, ty = target.position.y, tz = target.position.z;
    const double dist = std::sqrt(tx*tx + ty*ty + tz*tz);
    if (dist < 1e-6) return false;

    const double dx = tx/dist, dy = ty/dist, dz = tz/dist;
    for (double r = std::min(dist - STRETCH_STEP, ARM_MAX_REACH); r >= STRETCH_MIN_R; r -= STRETCH_STEP) {
      geometry_msgs::msg::Pose c = target;
      c.position.x = dx * r; c.position.y = dy * r; c.position.z = dz * r;
      if (tryIK(c)) {
        RCLCPP_INFO(get_logger(), "Stretch-toward IK succeeded at r=%.2f m.", r);
        return true;
      }
    }
    return false;
  }

  bool tryIK(const geometry_msgs::msg::Pose & target)
  {
    // Seed with current joint positions for smooth, local IK solutions
    moveit::core::RobotState ik_state(*robot_state_);
    {
      std::lock_guard<std::mutex> lk(js_mutex_);
      ik_state.setVariablePositions(current_joints_);
    }

    // Pose → Eigen
    Eigen::Isometry3d pose_e = Eigen::Isometry3d::Identity();
    pose_e.translation() << target.position.x, target.position.y, target.position.z;
    Eigen::Quaterniond q(
        target.orientation.w, target.orientation.x,
        target.orientation.y, target.orientation.z);
    pose_e.linear() = q.normalized().toRotationMatrix();

    if (!ik_state.setFromIK(jmg_, pose_e, IK_TIMEOUT_SEC)) return false;

    ik_state.enforceBounds();

    std::vector<double> jvals;
    ik_state.copyJointGroupPositions(jmg_, jvals);

    // Publish trajectory directly to the JointTrajectoryController
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->now();
    traj.joint_names  = jmg_->getVariableNames();

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = jvals;
    pt.velocities.assign(jvals.size(), 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(TRAJ_DURATION);
    traj.points.push_back(pt);

    traj_pub_->publish(traj);
    RCLCPP_INFO(get_logger(), "IK solved → trajectory sent [%.3f, %.3f, %.3f]",
      target.position.x, target.position.y, target.position.z);
    return true;
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  static double poseDist(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
  {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  // ── Members ────────────────────────────────────────────────────────────────

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr   traj_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr          js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_sub_;

  robot_model_loader::RobotModelLoaderPtr                robot_loader_;
  moveit::core::RobotModelConstPtr                       robot_model_;
  std::shared_ptr<moveit::core::RobotState>              robot_state_;
  const moveit::core::JointModelGroup*                   jmg_ {nullptr};

  // Current joint state (IK seed) — updated by /joint_states callback
  std::map<std::string, double> current_joints_;
  std::mutex                    js_mutex_;

  // Pending target pose — updated by /target_pose callback
  std::optional<geometry_msgs::msg::PoseStamped> pending_;
  std::mutex                                      pending_mutex_;

  // Failure backoff state
  geometry_msgs::msg::Pose last_failed_;
  bool                     has_failed_   {false};
  rclcpp::Time             backoff_until_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKMoverNode>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  std::thread spin_thread([&node]() { rclcpp::spin(node); });
  node->run();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
