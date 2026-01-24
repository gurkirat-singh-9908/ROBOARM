import numpy as np 
import math
from scipy.optimize import fsolve

link = [20.5, 28, 28.5, 4, 3.25, 20]

desired = np.array([
 [-8.31329387e-02,  9.81009526e-01,  1.75240474e-01,  6.02289817e+00],
 [-9.81009526e-01, -4.96392896e-02, -1.87500000e-01, -3.23437500e+00],
 [-1.75240474e-01, -1.87500000e-01,  9.66506351e-01,  9.53658096e+01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

def T03_from_thetas(thetas):
    theta1, theta2, theta3 = thetas
    T01 = dh_transform(theta1, 90, 0, link[0])
    T12 = dh_transform(theta2, 0, link[1], 0)
    T23 = dh_transform(theta3, 90, 0, 0)
    return np.dot(np.dot(T01, T12), T23)

def wrist_error(thetas, wc):
    T03 = T03_from_thetas(thetas)
    p_wrist = np.dot(T03, np.array([0, 0, link[2]+link[3], 1]))
    return p_wrist[0:3] - wc

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


def dh_transform(theta, alpha_deg, r, d):
    alpha = math.radians(alpha_deg)
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0,                math.sin(alpha),                 math.cos(alpha),                d],
        [0,                0,                               0,                              1]
    ])

def inverse_kinematics():

    pos_desired = desired[0:3, 3]
    R_desired = desired[0:3, 0:3]
    print("Desired Position:", pos_desired)
    print("orientation",R_desired)

    d6 = link[4]+link[5]

    wc = np.array(pos_desired) - d6 * R_desired[:,2]

    initial_guess = [0.5, 0.0, 0.0]

    sol, infodict, ier, mesg = fsolve(wrist_error, initial_guess, args=(wc,), full_output=True)
    if ier != 1:
        print("Numerical IK for joints 1-3 did not converge:", mesg)
        return None
    theta1, theta2, theta3 = sol

    T03 = T03_from_thetas([theta1, theta2, theta3])
    R03 = T03[0:3, 0:3]

    R36 = np.dot(np.linalg.inv(R03), R_desired)

    theta5 = math.atan2(math.sqrt(R36[0,2]**2 + R36[1,2]**2), R36[2,2])
    theta4 = math.atan2(R36[1,2], R36[0,2])
    theta6 = math.atan2(R36[2,1], -R36[2,0])

    calc_angles = [theta1, theta2, theta3, theta4, theta5, theta6]

    calc_angles_deg = [math.degrees(a) for a in calc_angles]
    print("Calculated Angles (deg):", calc_angles_deg)

    T_calc = forward_kinematics(calc_angles)
    pos_calc = T_calc[0:3, 3]

    tol = 1e-3
    if np.allclose(pos_desired, pos_calc, atol=tol):
        print("Yes, positions match.")
    else:
        print("No, positions do not match.")
        print("Desired position:", pos_desired)
        print("Calculated position:", pos_calc)


if __name__ == "__main__":
    inverse_kinematics()
