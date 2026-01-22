import numpy as np
from numpy import pi, cos, sin, arctan, arctan2, arcsin, array, eye, dot, rad2deg, deg2rad, sqrt, square,abs,arccos
import matplotlib.pyplot as plt

link = array([20.5, 28, 28.5, 4, 3.25, 20])
dh_table = None
theta = []  # Global theta to store angles
PLT = False
count_true = 0
count_false = 0

def matrix_multipler(theta, limit):
    final = eye(4)
    global dh_table
    dh_table = array([
        [theta[0], pi/2, 0, link[0]],
        [theta[1] + pi/2, 0, link[1], 0],
        [theta[2], pi/2, 0, 0],
        [theta[3], -pi/2, 0, link[2] + link[3]],
        [theta[4], pi/2, 0, 0],
        [theta[5], 0, 0, link[4] + link[5]]
    ])

    for i in range(limit):
        dh_values = dh_table[i]
        dh_formula = array([
            [cos(dh_values[0]), -sin(dh_values[0]) * cos(dh_values[1]), sin(dh_values[0]) * sin(dh_values[1]), dh_values[2] * cos(dh_values[0])],
            [sin(dh_values[0]), cos(dh_values[0]) * cos(dh_values[1]), -cos(dh_values[0]) * sin(dh_values[1]), dh_values[2] * sin(dh_values[0])],
            [0, sin(dh_values[1]), cos(dh_values[1]), dh_values[3]],
            [0, 0, 0, 1]
        ])
        final = dot(final, dh_formula)
    return final

def forward_kinematics(theta):
    theta = deg2rad(theta)
    position = matrix_multipler(theta, 6)
    return position

def inverse_kinematics(position):
    global theta
    theta = []  # Reset theta to a list of scalars

    wrist_pos = array([
        position[0, -1] - dh_table[-1, -1] * position[0, -2],
        position[1, -1] - dh_table[-1, -1] * position[1, -2],
        position[2, -1] - dh_table[-1, -1] * position[2, -2]
    ])

    # Append scalar values to theta
    #print(abs(round(wrist_pos[0],5)),wrist_pos[1])
    if (abs(round(wrist_pos[0],5))==0 and wrist_pos[1]!=abs(wrist_pos[1])):
        theta1 = pi/2
    elif (abs(round(wrist_pos[0],5))==0 and wrist_pos[1]==abs(wrist_pos[1])):
        theta1 = -pi/2
    else:
        theta1 = arctan(wrist_pos[1] / wrist_pos[0])
    theta.append(theta1)
    r1 = sqrt(square(wrist_pos[0]) + square(wrist_pos[1]))
    r2 = wrist_pos[2] - dh_table[0, 3]
    r3 = sqrt(square(r1) + square(r2))
    ca = link[2] + link[3]
    pi1 = (arccos((square(r3) + square(link[1]) - square(ca)) / (2 * r3 * link[1])))
    pi2 = arctan(r2 / r1)
    pi3 = arccos((-square(r3) + square(link[1]) + square(ca)) / (2 * ca * link[1]))

    theta.append((-pi / 2 + pi2 + pi1).item())
    theta.append((pi3 - pi / 2))
    theta += [0, 0, 0]  # Initialize the remaining angles
    # Todo : remove this 
    #theta[1]=theta[0]

    # Compute rotations
    rotation_03 = matrix_multipler(theta, 3)
    rotation_03 = rotation_03[:3, :3]
    #print(f"R03 = {rotation_03}")

    inverse_rot_03 = np.linalg.inv(rotation_03)    
    #print(f"invR03 = {inverse_rot_03}")

    rotation_36 = np.dot(inverse_rot_03, position[:3, :3])

    #print(f"R22 = {rotation_36}")
    # Calculate theta[4] and theta[5] as scalars
    theta[4] = (arccos(rotation_36[2, 2]))*inverse_rot_03[2,2]/abs(inverse_rot_03[2,2])
    #print(f"theta 5 = {rad2deg(theta[4])}")

    if (abs(theta[4])==0 and abs(round(rotation_36[1, 2],5))==0 and abs(round(rotation_36[2, 0],5))==0):
        theta[3] = 0.0
        theta[5] = 0.0
    else:
        theta[3] = arcsin(round(rotation_36[1, 2],5) / sin(theta[4]))
        theta[5] = arcsin(round(rotation_36[2, 1],5) / sin(theta[4]))
    #print(f"theta 5 values = {rotation_36[2, 2]}\ntheta 4 values = {rotation_36[1, 2]},{sin(theta[4])}\ntheta 6 values = {rotation_36[2, 0]},{-sin(theta[4])}")
    return rad2deg(theta)

def safe_check(desired_position, calculated_angles, tolerance=1e-2):
    global count_true
    global count_false

    calculated_position = forward_kinematics(calculated_angles)

    position_error = np.linalg.norm(desired_position[:3, 3] - calculated_position[:3, 3])

    if position_error <= tolerance:
        count_true += 1
        return True
    else:
        print(f"Warning: Incorrect angles - calculated position does not match the desired position within tolerance.\nPosition tried to reach\n{desired_position}\nPosition reached\n{forward_kinematics(calculated_angles)}")
        count_false += 1
        return False


def main():
    true_values = []
    calculated_values = []
    '''theta = [-30,-30,-30,-30,-30,-30]
    position = forward_kinematics(theta)
    angles = inverse_kinematics(position)'''

    # Sweep through a range of angles
    for i in range(-90, 91, 10):
        theta = [i, i, i, i, i, i]
        position = forward_kinematics(theta)
        angles = inverse_kinematics(position)
        err = safe_check(position,angles)
        if not(err):
            print(f"theta = {np.round(theta)} vs angles = {np.round(angles)}")
        true_values.append(theta)
        calculated_values.append(angles)

        #if (round(angles[0])!=round(angles[-1]) or round(angles[0])!=round(angles[-2]) or round(angles[0])!=round(angles[-3])):
        #print(f"true value {np.round(theta)} calculated {np.round(angles)} R22 {round(position[2,2],5)}")#R22 {round(position[2,2],5)}

    # Convert lists to arrays for plotting
    print(f"success percentage = {(count_true/(count_true+count_false))*100}\n total count = {count_true+count_false}")
    true_values = np.array(true_values)
    calculated_values = np.array(calculated_values)

    # Plotting
    if PLT:
        plt.figure(figsize=(12, 10))
        joint_labels = [f'Joint {i+1}' for i in range(6)]
        for j in range(6):
            plt.subplot(3, 2, j+1)
            plt.plot(true_values[1, j], label='True Value', color='blue')
            plt.plot(calculated_values[1, j], label='Calculated Value', color='orange', linestyle='--')
            plt.title(joint_labels[j])
            plt.xlabel("Iteration")
            plt.ylabel("Angle (degrees)")
            plt.legend()
            plt.grid()

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    main()
