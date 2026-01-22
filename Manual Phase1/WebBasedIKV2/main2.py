import numpy as np
import math
from scipy.optimize import fsolve

# Define robot link lengths (modify as per your robot's dimensions)
link = [20.5, 28, 28.5, 4, 3.25, 20]  # link[0]=a1, link[1]=a2, link[2]=a3, link[3]=a4, link[4]=a5, link[5]=a6

# Function to compute the homogeneous transformation matrix from input [x,y,z] and [RZ,RY,RX] (in degrees)
def compute_desired_matrix(inp):
    """
    inp: 2x3 array
         First row: [x, y, z] position.
         Second row: [RZ, RY, RX] in degrees (RZ=yaw, RY=pitch, RX=roll).
    """
    x, y, z = inp[0, :]
    RZ_deg, RY_deg, RX_deg = inp[1, :]
    RZ = math.radians(RZ_deg)
    RY = math.radians(RY_deg)
    RX = math.radians(RX_deg)
    
    R = np.array([
        [math.cos(RZ)*math.cos(RY),
         math.cos(RZ)*math.sin(RY)*math.sin(RX) - math.sin(RZ)*math.cos(RX),
         math.cos(RZ)*math.sin(RY)*math.cos(RX) + math.sin(RZ)*math.sin(RX)],
        [math.sin(RZ)*math.cos(RY),
         math.sin(RZ)*math.sin(RY)*math.sin(RX) + math.cos(RZ)*math.cos(RX),
         math.sin(RZ)*math.sin(RY)*math.cos(RX) - math.cos(RZ)*math.sin(RX)],
        [-math.sin(RY),
         math.cos(RY)*math.sin(RX),
         math.cos(RY)*math.cos(RX)]
    ])
    
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [x, y, z]
    return T

# Example input: first row is [x, y, z], second row is [RZ (yaw), RY (pitch), RX (roll)] in degrees.
inp = np.array([[0.0, 0.6, -0.0],
                [0.0,  0.0, 0.0]])
desired = compute_desired_matrix(inp)

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
    T01 = dh_transform(theta1, 90, 0, link[0])
    T12 = dh_transform(theta2, 0, link[1], 0)
    T23 = dh_transform(theta3, 90, 0, 0)
    T34 = dh_transform(theta4, -90, 0, link[2]+link[3])
    T45 = dh_transform(theta5, 90, 0, 0)
    T56 = dh_transform(theta6, 0, 0, (link[4]+link[5]))
    
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)
    return T06

def T03_from_thetas(thetas):
    """Compute transformation from base to joint 3 given theta1, theta2, theta3."""
    theta1, theta2, theta3 = thetas
    T01 = dh_transform(theta1, 90, 0, link[0])
    T12 = dh_transform(theta2, 0, link[1], 0)
    T23 = dh_transform(theta3, 90, 0, 0)
    return np.dot(np.dot(T01, T12), T23)

def wrist_error(thetas, wc):
    """
    Given theta1, theta2, theta3, compute the error between the predicted wrist center
    and the desired wrist center.
    """
    T03 = T03_from_thetas(thetas)
    # Predicted wrist center from frame 3 (using offset L = link[2] + link[3])
    p_wrist = np.dot(T03, np.array([0, 0, link[2]+link[3], 1]))
    return p_wrist[0:3] - wc

def inverse_kinematics():
    """
    Solve for all 6 joint angles.
    1. Compute the desired end-effector position and orientation from 'desired' matrix.
    2. Compute the wrist center.
    3. Solve numerically for theta1, theta2, theta3 using wrist_error.
    4. Compute the wrist rotation and extract theta4, theta5, theta6.
    """
    pos_desired = desired[0:3, 3]
    R_desired = desired[0:3, 0:3]
    print("Desired Position:", pos_desired)
    print("Desired Orientation:\n", R_desired)

    d6 = link[4] + link[5]  # offset from wrist center to end-effector

    wc = np.array(pos_desired) - d6 * R_desired[:,2]

    initial_guess = [0.5, 0.0, 0.0]

    sol, infodict, ier, mesg = fsolve(wrist_error, initial_guess, args=(wc,), full_output=True)
    if ier != 1:
        print("Numerical IK for joints 1-3 did not converge:", mesg)
        return None
    theta1, theta2, theta3 = sol

    T03 = T03_from_thetas([theta1, theta2, theta3])
    R03 = T03[0:3, 0:3]

    # Compute wrist rotation matrix R36 = R03⁻¹ * R_desired
    R36 = np.dot(np.linalg.inv(R03), R_desired)

    # Extract wrist angles using standard relations
    theta5 = math.atan2(math.sqrt(R36[0,2]**2 + R36[1,2]**2), R36[2,2])
    theta4 = math.atan2(R36[1,2], R36[0,2])
    theta6 = math.atan2(R36[2,1], -R36[2,0])

    calc_angles = [theta1, theta2, theta3, theta4, theta5, theta6]
    calc_angles_deg = [math.degrees(a) for a in calc_angles]
    print("Calculated Angles (deg):", calc_angles_deg)

    T_calc = forward_kinematics(calc_angles)
    pos_calc = T_calc[0:3, 3]
    print("Calculated End-Effector Position:", pos_calc)

    tol = 1e-3
    if np.allclose(pos_desired, pos_calc, atol=tol):
        print("Yes, positions match.")
    else:
        print("No, positions do not match.")
        print("Desired position:", pos_desired)
        print("Calculated position:", pos_calc)

if __name__ == "__main__":
    inverse_kinematics()
