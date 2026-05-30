import numpy as np 
from param import dh_params
np.set_printoptions(precision=1, suppress=True)
tol = 1e-3

def inverse_kinematics(inp):
    """
    input: 2x3 array
         First row: [x, y, z] position.
         Second row: [RZ, RY, RX] in degrees (RZ=yaw, RY=pitch, RX=roll).
    """
    desired_matrix = compute_desired_matrix(inp)
    un = False

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
        #
        #print(r_sq, s**2, -dh_params[1][-2], - (-dh_params[1][-2])**2,-dh_params[3][-1], - ((-dh_params[3][-1]))**2)
        #print(numerator)
        #print(denominator)
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

        error(desired_matrix,T_calc)
    
        return servo_angles
    
    except Exception as e:
        print(f"Error in inverse_kinematics: {e}")
        import traceback
        traceback.print_exc()
        raise

def compute_desired_matrix(inp):
    """
    inp: 2x3 array
         First row: [x, y, z] position.
         Second row: [RZ, RY, RX] in degrees (RZ=yaw, RY=pitch, RX=roll).
    """
    try:
        # Ensure all values are float
        x, y, z = float(inp[0][0]), float(inp[0][1]), float(inp[0][2])
        RZ_deg, RY_deg, RX_deg = float(inp[1][0]), float(inp[1][1]), float(inp[1][2])
        
        RZ = np.radians(RZ_deg)
        RY = np.radians(RY_deg)
        RX = np.radians(RX_deg)
        
        R = np.array([
            [np.cos(RZ)*np.cos(RY),             np.cos(RZ)*np.sin(RY)*np.sin(RX) - np.sin(RZ)*np.cos(RX),           np.cos(RZ)*np.sin(RY)*np.cos(RX) + np.sin(RZ)*np.sin(RX)],
            [np.sin(RZ)*np.cos(RY),             np.sin(RZ)*np.sin(RY)*np.sin(RX) + np.cos(RZ)*np.cos(RX),           np.sin(RZ)*np.sin(RY)*np.cos(RX) - np.cos(RZ)*np.sin(RX)],
            [-np.sin(RY),                                       np.cos(RY)*np.sin(RX),                                                  np.cos(RY)*np.cos(RX)]
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

#TODO: instead of figuring out t03 formulate it and then paste the matrix here
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
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
        [0,                np.sin(alpha),                 np.cos(alpha),                d],
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

def error(desrired_pos, calculated_pos):
    """
    desired pos: the desired position generated from compute_desired_matrix
    calculated_pos: the position calculated from the derived thetas
    """

    if np.allclose(desrired_pos, calculated_pos, atol=tol):
        print(f"Postion is achived")

    else:
        print(f"Position couldn't be achived, desired position: \nnp.array({desrired_pos})\ncalculated position:\nnp.array({calculated_pos}) ")


if __name__ == "__main__":
    inp = np.array([[50,20,30],
                   [0,0,0]])
    cal = inverse_kinematics(inp)
