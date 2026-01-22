import numpy as np
import math
from scipy.optimize import fsolve

# Define DH parameters (units consistent, angles in radians)
a1, a2, a3, a4, a5, a6 = 20.5, 28, 28.5, 4, 3.25, 20
L = a3 + a4  # offset from joint 3 to wrist center

input_angles_deg = [30, 45, 50, 20, 55, 40]

def dh_transform(theta, alpha_deg, r, d):
    alpha = math.radians(alpha_deg)
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0,                math.sin(alpha),                 math.cos(alpha),                d],
        [0,                0,                               0,                              1]
    ])

def forward_kinematics(angles):
    theta1, theta2, theta3, theta4, theta5, theta6 = angles
    T01 = dh_transform(theta1, 90, 0, a1)
    T12 = dh_transform(theta2, 0, a2, 0)
    T23 = dh_transform(theta3, 90, 0, 0)
    T34 = dh_transform(theta4, -90, 0, L)
    T45 = dh_transform(theta5, 90, 0, 0)
    T56 = dh_transform(theta6, 0, 0, (a5+a6))

    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)
    return T06

def T03_from_thetas(thetas):
    """Compute transformation from base to joint 3 given theta1, theta2, theta3."""
    theta1, theta2, theta3 = thetas
    T01 = dh_transform(theta1, 90, 0, a1)
    T12 = dh_transform(theta2, 0, a2, 0)
    T23 = dh_transform(theta3, 90, 0, 0)
    return np.dot(np.dot(T01, T12), T23)

def wrist_error(thetas, wc):
    """
    Given theta1, theta2, theta3, compute the error between the predicted wrist center
    and the desired wrist center.
    """
    T03 = T03_from_thetas(thetas)
    # Predicted wrist center from frame 3: p = [0, 0, L, 1]^T transformed to base frame.
    p_wrist = np.dot(T03, np.array([0, 0, L, 1]))
    return p_wrist[0:3] - wc

def inverse_kinematics(pos, R_desired):
    """
    Solve for all 6 joint angles.
    1. Compute the wrist center.
    2. Solve numerically for theta1, theta2, theta3 using wrist_error.
    3. Compute the wrist rotation and extract theta4, theta5, theta6.
    """
    d6 = a5 + a6
    # Compute wrist center (using current end-effector orientation's z-axis)
    wc = np.array(pos) - d6 * R_desired[:,2]

    # Initial guess for theta1, theta2, theta3 (could be zeros or based on geometric insight)
    initial_guess = [0.5, 0.0, 0.0]
    sol, infodict, ier, mesg = fsolve(wrist_error, initial_guess, args=(wc,), full_output=True)
    if ier != 1:
        print("Numerical IK for joints 1-3 did not converge:", mesg)
        return None
    theta1, theta2, theta3 = sol

    # Compute T03 and extract its rotation matrix
    T03 = T03_from_thetas([theta1, theta2, theta3])
    R03 = T03[0:3, 0:3]

    # Compute wrist rotation matrix: R36 = R03ᵀ * R_desired
    R36 = np.dot(np.linalg.inv(R03), R_desired)

    # Extract wrist angles from R36.
    # Using the derived relations for our DH parameters:
    theta5 = math.atan2(math.sqrt(R36[0,2]**2 + R36[1,2]**2), R36[2,2])
    theta4 = math.atan2(R36[1,2], R36[0,2])
    theta6 = math.atan2(R36[2,1], -R36[2,0])

    return [theta1, theta2, theta3, theta4, theta5, theta6]

def test_kinematics():
    """
    Test procedure:
      1. Start with a set of user-defined joint angles.
      2. Compute desired end-effector pose using forward kinematics.
      3. Use inverse kinematics to retrieve joint angles from the pose.
      4. Re-compute end-effector pose with the retrieved angles.
      5. Compare positions.
    """
    # Example input joint angles (in degrees)
    input_angles = [math.radians(a) for a in input_angles_deg]

    # Compute desired pose
    T_desired = forward_kinematics(input_angles)
    pos_desired = T_desired[0:3, 3]
    R_desired = T_desired[0:3, 0:3]
    print("Desired Position:", pos_desired)
    print("orientation",R_desired)

    # Solve IK
    calc_angles = inverse_kinematics(pos_desired, R_desired)
    if calc_angles is None:
        return
    # For comparison, print the recovered angles (in degrees)
    calc_angles_deg = [math.degrees(a) for a in calc_angles]
    print("Calculated Angles (deg):", calc_angles_deg)

    # Compute forward kinematics with calculated angles
    T_calc = forward_kinematics(calc_angles)
    pos_calc = T_calc[0:3, 3]
    print("Calculated Position:", pos_calc)

    tol = 1e-3
    if np.allclose(pos_desired, pos_calc, atol=tol):
        print("Yes, positions match.")
    else:
        print("No, positions do not match.")
        print("Desired position:", pos_desired)
        print("Calculated position:", pos_calc)

if __name__ == "__main__":
    test_kinematics()
