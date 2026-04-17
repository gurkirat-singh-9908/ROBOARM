import numpy as np
from numpy import radians as c2r
from numpy import rad2deg as c2d
#import dh
#from dh import dh


link = [20.5, 28, 28.5, 4, 3.25, 20]

link_positions = [0, 0, 0,
                  0, 0, link[0],
                  0, 0, link[0]+link[1],
                  link[2], 0, link[0]+link[1],
                  link[2]+link[3], 0, link[0]+link[1],
                  link[2]+link[3]+link[4], 0, link[0]+link[1],
                  link[2]+link[3]+link[4]+link[5], 0, link[0]+link[1]]

desired = np.array([
 [-8.31329387e-02,  9.81009526e-01,  1.75240474e-01,  6.02289817e+00],
 [-9.81009526e-01, -4.96392896e-02, -1.87500000e-01, -3.23437500e+00],
 [-1.75240474e-01, -1.87500000e-01,  9.66506351e-01,  9.53658096e+01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#desired = main(False)

desired_rot = desired[:3, :3]

goal = [desired[0, 3], desired[1, 3], desired[2, 3]]

#print(f"goal={goal}")

# step 1 calxulate wrist position 
wrist_pos = np.array([goal[0] - (link[-1]+link[-2])*desired_rot[0, 2],
                      goal[1] - (link[-1]+link[-2])*desired_rot[1, 2],
                      goal[2] - (link[-1]+link[-2])*desired_rot[2, 2]])

print(f"wrist_pos={wrist_pos}")

# Step 2: Using inverse kinematics (graphical method)
theeta1 = np.arctan(wrist_pos[1]/wrist_pos[0])

r1 = np.sqrt(np.square(wrist_pos[0]) + np.square(wrist_pos[1]))
#print(f"r1={r1}")
r2 = wrist_pos[2] - link[0]
#print(f"r2={r2}")
pi2 = np.arctan2(r2,r1)
#print(f"pi2={c2d(pi2)}")
r3 = np.sqrt(np.square(r1) + np.square(r2))
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

#elbow up
theeta3 = -np.pi/2 + pi3

theeta2 = -np.pi/2 + pi2 + pi1
#print(f"t2 = {c2d(theeta2)}")



# Step 3: Calculate rotation matrix 03 for the calculated theetas
# done :update matrix_03
rotation_matrix_03 = np.array([
    [-np.sin(theeta2) * np.cos(theeta1) * np.cos(theeta3) - np.sin(theeta3) * np.cos(theeta1) * np.cos(theeta2), np.sin(theeta1),-np.sin(theeta2) * np.sin(theeta3) * np.cos(theeta1) + np.cos(theeta1) * np.cos(theeta2) * np.cos(theeta3)],
    [-np.sin(theeta1) * np.sin(theeta2) * np.cos(theeta3) - np.sin(theeta1) * np.sin(theeta3) * np.cos(theeta2), -np.cos(theeta1),-np.sin(theeta1) * np.sin(theeta2) * np.sin(theeta3) + np.sin(theeta1) * np.cos(theeta2) * np.cos(theeta3)],
    [-np.sin(theeta2) * np.sin(theeta3) + np.cos(theeta2) * np.cos(theeta3), 0, np.sin(theeta2) * np.cos(theeta3) + np.sin(theeta3) * np.cos(theeta2)]
])


# Step 4: Inverse of rotation matrix 03
inv_rotation_matrix_03 = np.linalg.inv(rotation_matrix_03)

# Step 5: Define desired rotation matrix and calculate rotation matrix 36
rotation_matrix_36 = np.dot(inv_rotation_matrix_03, desired_rot)

# Step 6: Calculate the rest of the angles base on dh matrix solution
# done: update formulas

#theeta 5
theeta5 = np.arccos(desired_rot[2,2])

#theeta 4
theeta4 = np.arcsin(desired_rot[1, 2] / np.sin(theeta5))
#2nd set
#theeta4 = np.arccos(desired_rot[0, 2] / np.sin(theeta5))

#theeta 6
theeta6 = np.arccos(desired_rot[2, 0] / -np.sin(theeta5))
#2nd set 
#theeta6 = np.arcsin(desired_rot[2, 1] / np.sin(theeta5))

theeta = [theeta1, theeta2, theeta3, theeta4, theeta5, theeta6]

#final,pos = dh(goal, theeta=theeta, link=link, link_position=link_positions)


#print("\n\n")

print(f"theetas={np.round(c2d(theeta))}")
'''
print("\n\n")

print(f"desired position \n{(desired)}")

print("\n\n")

print(f"position of the end effector \n{(final)}")

error = final-desired

print("\n\n")

print(f"error = {error}")'''