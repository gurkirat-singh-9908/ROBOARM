#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <cmath>
#include <mutex>
#include <optional>
#include <thread>

static const std::string PLANNING_GROUP = "arm";

// Step size and floor for the stretch-toward search (metres)
static constexpr double STRETCH_STEP  = 0.03;
static constexpr double STRETCH_MIN_R = 0.10;
// Conservative upper bound on arm reach from base_link origin (metres)
static constexpr double ARM_MAX_REACH = 0.75;

class IKMoverNode : public rclcpp::Node
{
public:
  explicit IKMoverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ik_mover", options)
  {
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 1,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_pose_ = *msg;
        RCLCPP_INFO(get_logger(), "New target pose received: [%.3f, %.3f, %.3f]",
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      });
  }

  void run()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), PLANNING_GROUP);

    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setPlanningTime(5.0);

    RCLCPP_INFO(get_logger(),
      "IK Mover ready. Planning group: '%s' | End-effector: '%s' | "
      "Listening on /target_pose",
      PLANNING_GROUP.c_str(), move_group_->getEndEffectorLink().c_str());

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      std::optional<geometry_msgs::msg::PoseStamped> pose;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_pose_) {
          pose = pending_pose_;
          pending_pose_.reset();
        }
      }

      if (pose) {
        move_group_->setPoseTarget(pose->pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(get_logger(), "Plan found — executing trajectory...");
          move_group_->execute(plan);
          RCLCPP_INFO(get_logger(), "Execution complete.");
        } else {
          RCLCPP_WARN(get_logger(),
            "Target unreachable (error %d) — searching for closest point along ray...",
            result.val);
          if (!stretchToward(pose->pose)) {
            RCLCPP_ERROR(get_logger(), "No reachable point found in that direction.");
          }
        }
        move_group_->clearPoseTargets();
      }

      rate.sleep();
    }
  }

private:
  // Walk back from the target along the origin→target ray until IK succeeds,
  // then execute the found plan immediately.  Returns true on success.
  bool stretchToward(const geometry_msgs::msg::Pose & target)
  {
    const double tx = target.position.x;
    const double ty = target.position.y;
    const double tz = target.position.z;
    const double dist = std::sqrt(tx * tx + ty * ty + tz * tz);

    if (dist < 1e-6) {
      RCLCPP_WARN(get_logger(), "Target is at origin — cannot stretch.");
      return false;
    }

    const double dx = tx / dist;
    const double dy = ty / dist;
    const double dz = tz / dist;

    // Start one step inside the arm's max reach (or just inside the target
    // distance if the target was close but still unreachable).
    const double start_r = std::min(dist - STRETCH_STEP, ARM_MAX_REACH);

    for (double r = start_r; r >= STRETCH_MIN_R; r -= STRETCH_STEP) {
      geometry_msgs::msg::Pose candidate = target;
      candidate.position.x = dx * r;
      candidate.position.y = dy * r;
      candidate.position.z = dz * r;

      move_group_->setPoseTarget(candidate);
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(),
          "Stretching to [%.3f, %.3f, %.3f] (%.2fm along ray toward target)",
          candidate.position.x, candidate.position.y, candidate.position.z, r);
        move_group_->execute(plan);
        RCLCPP_INFO(get_logger(), "Stretch execution complete.");
        return true;
      }
    }
    return false;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IKMoverNode>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto spin_thread = std::thread([&node]() { rclcpp::spin(node); });

  node->run();

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
