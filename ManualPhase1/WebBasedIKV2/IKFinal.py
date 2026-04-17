import numpy as np
import math

link = [20.5, 28, 28.5, 4, 3.25, 20]

dh_params = [
    #[theta, alpha_deg, r, d]
    [0,     90,     0,          link[0]],
    [0,     0,      link[1],    0],
    [0,     90,     0,          0],    
    [0,    -90,     0,          link[2]+link[3]],
    [0,     90,     0,          0],
    [0,     0,      0,          link[4]+link[5]]
]

l = dh_params[-1][-1]

def ForwardKinematics(angles):

    '''
    arg:
        list of thetas in degrees.
    returns:
        position & orieentation of the robot with respect to the angles
    '''
    T = np.eye(4)
    # Compute forward kinematics using DH parameters
    for i, theta in enumerate(angles):
        theta = theta + dh_params[i][0]
        alpha_deg = dh_params[i][1]
        r = dh_params[i][2]
        d = dh_params[i][3]
        T = np.dot(T, DHTransform(theta, alpha_deg, r, d))
    
    return T

def DHTransform(theta_deg, alpha_deg, r, d):

    '''
    arg:
        dh params
    returns:
        homogenous matrix for those params
    '''

    alpha = math.radians(alpha_deg)
    theta = math.radians(theta_deg)
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0,                math.sin(alpha),                 math.cos(alpha),                d],
        [0,                0,                               0,                              1]
    ])

def A(desired_pos, Formulation):
    return R(desired_pos) if Formulation=='R' else U(desired_pos)

def R(desired_pos):

    wrist_pos = CalWristPos(desired_pos)
    print(f"wrist_pos = {wrist_pos}")
    r = math.sqrt(wrist_pos[0]**2 + wrist_pos[1]**2)
    s = wrist_pos[2] - dh_params[0, 3]
    k = dh_params[3,3]
    print(f" r s k = {r} {s} {k}")
    angles = [0, 0, 0, 0, 0, 0]
    return angles 
    

    #return angles

def U(desired_pos):
    wrist_pos = CalWristPos(desired_pos)
    angles = [0, 0, 0, 0, 0, 0]
    return angles 

def N(desired_pos):
    wrist_pos = CalWristPos(desired_pos)
    angles = [0, 0, 0, 0, 0, 0]
    return angles 

def CalWristPos(desired_position):
    wrist_pos = np.array([desired_position[0, -1] - l*desired_position[0, 2],
                          desired_position[1, -1] - l*desired_position[1, 2],
                          desired_position[2, -1] - l*desired_position[2, 2]])
    return wrist_pos

def main(
        Devlopment = True, 
        Position = [20,20,40],
        RPY = [90,90,90],
        IK_Mode = 'A', 
        Formulation = 'R', 
        Ploting = False, 
        Visualition = False,
        ):
    
    '''
        agrs: 

            Devlopment phase: flag for testing. Default False. covers all the angles within the range of 0 to 180 and generats a failure percentage.

            Position: X Y Z coords of the robot. Default 20 20 40 cms.
            RPY: Roll Pitch Yaw orientation of the Robot. Default 90 90 90 degrees.

            IK Mode: Default 'A' 'R' 
                'A' Analytical - for solving IK using Analytical method 
                    'R' Research Paper - opts the formulas described in Research paper 
                    'U' User - opts the formulas described by the User  
                'N' Numerical - for solving IK using Numerical method 
            
            Ploting: flag for error plots. Default Fasle.

            Visualization: flag for robot visulations. Default False.
    '''
    if Devlopment:
        for q in range(0, 180, 10):
            theta = [q, q, q, q, q, q]
            print(f"state = {q}")
            desired_Position = ForwardKinematics(theta)
            print(f"desired_Position = {desired_Position}")
            angles = A(desired_Position, Formulation) if IK_Mode=='A' else N(desired_Position)
            print(angles)
    

if __name__ == "__main__":

    '''
    this code is for testing purpose not for deployment. NOT OPTIMIZED
    '''

    args = [True,[20,20,40],[90,90,90],'A','R',False,False,]

    inp = input("\nPlease select the Mode and enter values accordingly\n\tO Devolpment = T/F\n\n\tO IK Mode = A/N\n\n\tO Ploting = T/F\n\n\tO Visualition = T/F\n\tFormat for input = T A R F F\n")
    values = inp.split(" ")

    if values[0]=="T":
        args[0] = True
    else:
        args[0] = False
        position = input('\nPlease enter the position(cm) and the desired Roll Pitch Yaw value(degrees).\nFormat 20 20 40 90 90 90\n')
        position = position.split(" ")
        position = [int(x) for x in position]
        args[1] = position[0:3]
        args[2] = position[3:]

    if values[1]=="A":
        args[3] = 'A'
        formulation = input("please defien the formulation type you wanna use\nFormulation = R/U\n")
        args[4] = 'U' if formulation=="U" else 'R'
    else:
        args[3] = 'N'

    args[-2] = False if values[-2]=='F' else True
    args[-1] = False if values[-1]=='F' else True

    print(args)
    main(*args)
