import numpy as np
from numpy import pi, rad2deg,deg2rad
import math
from param import dh_params,link
import csv

np.set_printoptions(precision=1, suppress=True)

def inverse_kinematics(inp):
    un = False
    """
    input: 2x3 array
         First row: [x, y, z] position.
         Second row: [RZ, RY, RX] in degrees (RZ=yaw, RY=pitch, RX=roll).
         
    Solve for all 6 joint angles.
    1. Compute the desired end-effector position and orientation from 'desired' matrix.
    2. Compute the wrist center.
    3. Solve numerically for theta1, theta2, theta3 using wrist_error.
    4. Compute the wrist rotation and extract theta4, theta5, theta6.
    """

    #desired_matrix = compute_desired_matrix(inp)
    desired_matrix = inp
    global input

    try:
        pos_desired = desired_matrix[0:3, 3]
        R_desired = desired_matrix[0:3, 0:3]

        d6 = dh_params[-1][-1]

        wc = np.array(pos_desired) - d6 * R_desired[:,2]

        theta1 = np.atan2(wc[1],wc[0])

        if theta1<0:
            theta1 = np.pi+theta1
        else:
            theta1=theta1

        r_sq = wc[0]**2 + wc[1]**2

        s = wc[2] - dh_params[0][-1]

        numerator = r_sq + s**2 - (-dh_params[1][-2])**2 - ((-dh_params[3][-1]))**2
        denominator = 2 * (-dh_params[1][-2]) * ((-dh_params[3][-1]))
        D = numerator / denominator 

        if abs(D) > 1:
            print("Unreachable position!")
            un = True
            theta3 = np.pi/2

        else:
            theta3 = np.pi/2 - np.atan2(np.sqrt(1 - D**2), D) 

        k = np.atan2(s,np.sqrt(r_sq))
        theta2 = np.atan2((((-dh_params[3][-1]))*np.cos(theta3)),((-dh_params[1][-2])+((-dh_params[3][-1]))*np.sin(theta3)))+k

    
        T03 = T03_from_thetas([theta1, theta2, theta3])
        R03 = T03[0:3, 0:3]

        # Compute wrist rotation matrix R36 = R03⁻¹ * R_desired
        R36 = np.dot(np.linalg.inv(R03), R_desired)

        # Extract wrist angles using standard relations
        theta5 = np.atan2(-R36[2,2], np.sqrt(R36[0,2]**2 + R36[1,2]**2))
        theta4 = np.atan2(R36[0,2],-R36[1,2])
        theta6 = np.atan2(R36[2,1], -R36[2,0]) + np.pi
        if un:
            theta5 = np.pi/2

        cal = [theta1, theta2, theta3, theta4, theta5, theta6]
        cal_deg = [np.degrees(a) for a in cal]

        servo_angles = {
            's1': cal_deg[0],
            's2': cal_deg[1],
            's3': cal_deg[2],
            's4': cal_deg[3],
            's5': cal_deg[4],
            's6': cal_deg[5]
        }

        T_calc = forward_kinematics(cal)
        pos_calc = T_calc[0:3, 3]
        
        tol = 1e-3
        if np.allclose(pos_desired, pos_calc, atol=tol):
            print("Yes, positions match.")
            pass
        else:
            print("No, positions do not match.")
            print(un,pos_desired,pos_calc)
            pass
    
        return cal_deg
    
    except Exception as e:
        print(f"Error in inverse_kinematics: {e}")
        import traceback
        traceback.print_exc()
        raise
        
#this func here is to convert the input from the website to the desired matrix
def compute_desired_matrix(inp):
    """
    inp: 2x3 array
         First row: [x, y, z] position.
         Second row: [RZ, RY, RX] in degrees (RZ=yaw, RY=pitch, RX=roll).
    """
    try:
        # Ensure all values are float
        x, y, z = map(float, inp[0, :])
        RZ_deg, RY_deg, RX_deg = map(float, inp[1, :])
        
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
    except Exception as e:
        print(f"Error in compute_desired_matrix: {e}")
        import traceback
        traceback.print_exc()
        raise

def T03_from_thetas(thetas):
    """Compute transformation from base to joint 3 given theta1, theta2, theta3."""
    T = np.eye(4)
    for i, theta in enumerate(thetas):
        alpha = dh_params[i][1]
        r = dh_params[i][2]
        d = dh_params[i][3]
        T = np.dot(T, dh_transform(theta+dh_params[i][0], alpha, r, d))
    return T

def dh_transform(theta, alpha, r, d):
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0,                math.sin(alpha),                 math.cos(alpha),                d],
        [0,                0,                               0,                              1]
    ])

def forward_kinematics(angles):
    """
    Accepts theetas in radnians.
    """
    
    # Initialize transformation matrix
    T = np.eye(4)
    
    # Compute forward kinematics using DH parameters
    for i, theta in enumerate(angles):
        alpha = dh_params[i][1]
        r = dh_params[i][2]
        d = dh_params[i][3]
        T = np.dot(T, dh_transform(theta+dh_params[i][0], alpha, r, d))
    return T

def Position(filename,x, y, z):
        with open(filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            # Write header if file is empty
            if file.tell() == 0:
                writer.writerow(['x', f'y', f'z'])
            writer.writerow([x, y, z])

if __name__=="__main__":
    st = 15
    st2 = 30
    for i in range(0, 181, st):
        for j in range(0, 181, st):
            for k in range(0, 181, st):
                for l in range(0, 181, st2):
                    for m in range(0, 181, st2):
                        for n in range(0, 181, st2):
                            print(i,j,k,l,m,n)
                            angles = [rad2deg(i), rad2deg(j), rad2deg(k), rad2deg(l), rad2deg(m), rad2deg(n)]
                            Pos = forward_kinematics(angles)
                            x, y, z = Pos[0][-1], Pos[1][-1], Pos[2][-1]
                            filename = "workenvelope.csv"
                            Position(filename, x, y, z)