import numpy as np
from numpy import rad2deg as c2d
from dh import dh

link = [20.5, 28, 28.5, 4, 3.25, 20]

#minor axis  
def minor(theeta,desired_rot):
    final = np.eye(4)
    dh_matrix = np.array([
    [theeta[0]        , +np.pi/2, 0.0     , link[0]],
    [theeta[1]+np.pi/2, 0.0     , +link[1], 0.0],
    [theeta[2]        , +np.pi/2, 0.0     , 0.0],
    [theeta[3]        , -np.pi/2, 0.0     , link[2] + link[3]],
    [theeta[4]        , +np.pi/2, 0.0     , 0.0]              ,
    [theeta[5]        , 0.0     , 0.0     , link[5] + link[4]]
    ])  
    for i in range(3):
        val = dh_matrix[i]
        #print(val)
        tb = np.array([
                [np.cos(val[0]), -np.sin(val[0]) * np.cos(val[1]), np.sin(val[0]) * np.sin(val[1]), val[2] * np.cos(val[0])],
                [np.sin(val[0]), np.cos(val[0]) * np.cos(val[1]), -np.cos(val[0]) * np.sin(val[1]), val[2] * np.sin(val[0])],
                [0.0, np.sin(val[1]), np.cos(val[1]), val[3]],
                [0.0, 0.0, 0.0, 1.0]
            ])
        final = np.dot(final, tb)
    rotation_matrix_03=final

    inv_rotation_matrix_03 = np.linalg.inv(rotation_matrix_03)

    rotation_matrix_36 = np.dot(inv_rotation_matrix_03, desired_rot)
    print(rotation_matrix_36)

    theeta5 = np.arccos(rotation_matrix_36[2,2])

    theeta4 = np.arcsin(rotation_matrix_36[1, 2] / np.sin(theeta5))
    theeta4 = np.arccos(desired_rot[0, 2] / np.sin(theeta5))

    theeta6 = np.arccos(rotation_matrix_36[2, 0] / -np.sin(theeta5))
    theeta6 = np.arcsin(desired_rot[2, 1] / np.sin(theeta5))

    cal_theeta = [theeta[0],theeta[1],theeta[2],theeta4, theeta5, theeta6]

    print(f"theetas={np.round(c2d(cal_theeta))}")
    #print(desired_rot)
    error,pos = dh(theeta=cal_theeta)
    error = error - desired_rot
    #print(error)




for i in range(-90,91,10):
    print(i)
    theeta = [i,i,i,i,i,i]
    final,pos=dh(theeta=theeta)
    theeta=minor(theeta=theeta,desired_rot=final)









'''

def calinv(desired):
    desired_rot = desired[:3, :3]
    goal = [desired[0, 3], desired[1, 3], desired[2, 3]]

    # Step 1: Calculate wrist position
    wrist_pos = np.array([goal[0] - (dh_matrix[5,3])*desired_rot[0, 2],
                          goal[1] - (dh_matrix[5,3])*desired_rot[1, 2],
                          goal[2] - (dh_matrix[5,3])*desired_rot[2, 2]])

    # Step 2: Inverse kinematics for Theta1, Theta2, Theta3
    theeta1 = np.arctan(wrist_pos[1]/wrist_pos[0])
    #theeta1 = np.arctan2(wrist_pos[1], wrist_pos[0])
    r1 = np.sqrt(np.square(wrist_pos[0]) + np.square(wrist_pos[1]))
    #print(f"r1={r1}")
    r2 = wrist_pos[2] - link[0]
    #print(f"r2={r2}")
    pi2 = np.arctan2(r2,r1)
    #print(f"pi2={c2d(pi2)}")
    r3 = np.sqrt(np.square(r1) + np.square(r2))
    if r3 > link[1] + (link[2] + link[3]):
        print("Warning: Desired position is out of reach")
    #print(f"r3={r3}")
    ca = link[2]+link[3]
    #print(f"ca{ca}")
    value = np.clip((np.square(r3) + np.square(link[1]) - np.square(ca)) / (2 * r3 * link[1]), -1, 1)
    #print(f"val={value}")
    pi1 = np.arccos(value)
    #print(f"pi1 = {pi1}")
    #alternative method for pi for elbow up and down 
    pi3 = np.arccos(np.clip((-np.square(r3) + np.square(link[1]) + np.square(link[2] + link[3])) / (2 * (link[2] + link[3]) * link[1]), -1, 1))
    #print(f"pi3 = {c2d(pi3)}")

    # Elbow up
    theeta3_up = -np.pi/2 + pi3
    theeta2_up = -np.pi/2 + pi2 + pi1

    # Elbow down
    theeta3_down = np.pi/2 - pi3
    theeta2_down = -np.pi/2 + pi2 - pi1

    #print(c2d(theeta2_down),c2d(theeta2_up))

    # Choose the configuration based on your criteria
    # For example, you could choose the one closer to the current configuration
    if abs(theeta3_up - theeta[2]) < abs(theeta3_down - theeta[2]):
        theeta3 = theeta3_up
        theeta2 = theeta2_up
    else:
        theeta3 = theeta3_down
        theeta2 = theeta2_down


    theeta[0]=theeta1
    theeta[1]=theeta2
    theeta[2]=theeta3
    return(theeta)


for i in range(-90,91,10):
    print(i)
    theeta = [i,i,i,i,i,i]
    final,pos=dh(theeta=theeta)
    theeta=calinv(desired=final)
    print(f"theeta2={np.round(c2d(theeta[1]))}")
    print(f"error={np.round(c2d(theeta[1])-i)}")
'''