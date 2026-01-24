import numpy as np
from numpy import radians as c2r
from numpy import rad2deg as c2d
import math


link = [20.5, 28, 28.5, 4, 3.25, 20]



def dh_transform(theta, alpha_deg, r, d):
    alpha = math.radians(alpha_deg)
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), r*math.cos(theta)],
        [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), r*math.sin(theta)],
        [0,                math.sin(alpha),                 math.cos(alpha),                d],
        [0,                0,                               0,                              1]
    ])



def forward_kinematics(angles):
    theta1, theta2, theta3, theta4, theta5, theta6 = angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]
    T01 = dh_transform(theta1   , 90, 0       , link[0])
    T12 = dh_transform(theta2   , 0  , link[1], 0)
    T23 = dh_transform(theta3   , 90, 0       , 0)
    T34 = dh_transform(theta4   , -90, 0       , link[2]+link[3])
    T45 = dh_transform(theta5   , 90, 0       , 0)
    T56 = dh_transform(theta6   , 0  , 0       , (link[4]+link[5]))
    
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)
    T05 = np.dot(T04, T45)
    T06 = np.dot(T05, T56)
    return T06



user_input = input("\nEnter values: ")
values = user_input.split(" ")
inp = [float(values[0]), float(values[1]), float(values[2]), float(values[3]), float(values[4]), float(values[5])]



desired = forward_kinematics(inp)
desired_rot = desired[:3, :3]
goal = [desired[0, 3], desired[1, 3], desired[2, 3]]



l = link[4]+link[5]
r = math.sqrt((goal[1]**2+goal[0]**2))
s = goal[-1] - link[0]
k = link[2] + link[3]



wrist_pos = np.array([goal[0] - l*desired_rot[0, 2],
                      goal[1] - l*desired_rot[1, 2],
                      goal[2] - l*desired_rot[2, 2]])



theeta1 = math.atan2(wrist_pos[1],wrist_pos[0])

d = (r**2 + s**2 - link[1]**2 - k**2)/(2*link[1]*k)
theeta3 = math.atan2(math.sqrt(1-d**2),d)

h1 = math.atan2((k*math.sin(theeta3)),(link[1]+k*math.cos(theeta3)))
h2 = math.atan2(s,r)
theeta2 = h2 - h1

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

print(desired)
print(forward_kinematics(theeta))