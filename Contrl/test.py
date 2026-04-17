# Revised code to calculate the values of r1, r2, pi2, r3, and pi1 and plot them for analysis
import numpy as np
from numpy import rad2deg as c2d
from dh import dh
import matplotlib.pyplot as plt

theeta = [0,0,0,0,0,0]

link = [20.5, 28, 28.5, 4, 3.25, 20]

dh_matrix = np.array([
    [theeta[0]        , +np.pi/2, 0.0     , link[0]],
    [theeta[1]+np.pi/2, 0.0     , +link[1], 0.0],
    [theeta[2]        , +np.pi/2, 0.0     , 0.0],
    [theeta[3]        , -np.pi/2, 0.0     , link[2] + link[3]],
    [theeta[4]        , +np.pi/2, 0.0     , 0.0]              ,
    [theeta[5]        , 0.0     , 0.0     , link[5] + link[4]]
])

expected_theta2=[]

# Updated function to also collect intermediate values
def calculate_theta2_with_analysis(wrist_pos):
    # Calculate r1, r2, pi2, r3, pi1
    epsilon = 1e-10  # Small value to avoid division by zero
    r1 = np.sqrt(np.square(wrist_pos[0]) + np.square(wrist_pos[1]) + epsilon)
    r2 = wrist_pos[2] - link[0]
    r3 = np.sqrt(np.square(r1) + np.square(r2) + epsilon)
    
    pi2 = np.arctan2(r2, r1)
    
    ca = link[2] + link[3]
    cos_pi1 = np.clip((np.square(r3) + np.square(link[1]) - np.square(ca)) / (2 * r3 * link[1]), -1 + epsilon, 1 - epsilon)
    pi1 = np.arccos(cos_pi1)

    # Collect data for plotting
    return r1, r2, pi2, r3, pi1

# Variables to store the results for plotting
r1_values, r2_values, pi2_values, r3_values, pi1_values = [], [], [], [], []

# Extended main function to collect the intermediate values
def calinv_with_analysis(desired):
    desired_rot = desired[:3, :3]
    goal = [desired[0, 3], desired[1, 3], desired[2, 3]]

    # Calculate wrist position
    wrist_pos = np.array([goal[0] - (dh_matrix[5, 3]) * desired_rot[0, 2],
                          goal[1] - (dh_matrix[5, 3]) * desired_rot[1, 2],
                          goal[2] - (dh_matrix[5, 3]) * desired_rot[2, 2]])

    # Calculate theta 2 and collect intermediate values
    r1, r2, pi2, r3, pi1 = calculate_theta2_with_analysis(wrist_pos)
    
    # Append intermediate values for plotting
    r1_values.append(r1)
    r2_values.append(r2)
    pi2_values.append(c2d(pi2))  # Convert to degrees for consistency
    r3_values.append(r3)
    pi1_values.append(c2d(pi1))  # Convert to degrees for consistency

# Run the analysis across the range of angles
for i in range(-90, 91, 10):
    theeta = [i, i, i, i, i, i]
    final, pos = dh(theeta=theeta)
    calinv_with_analysis(desired=final)
    expected_theta2.append(i)

# Now plot the results
plt.figure(figsize=(12, 8))

# r1 and r2
plt.subplot(2, 2, 1)
plt.plot(expected_theta2, r1_values, label="r1", marker='o')
plt.plot(expected_theta2, r2_values, label="r2", marker='x')
plt.title("r1 and r2 vs Expected Theta 2")
plt.xlabel("Expected Theta 2 (degrees)")
plt.ylabel("r1, r2 values")
plt.legend()
plt.grid(True)

# pi2 (in degrees)
plt.subplot(2, 2, 2)
plt.plot(expected_theta2, pi2_values, label="pi2 (degrees)", marker='o')
plt.title("pi2 vs Expected Theta 2")
plt.xlabel("Expected Theta 2 (degrees)")
plt.ylabel("pi2 (degrees)")
plt.grid(True)

# r3
plt.subplot(2, 2, 3)
plt.plot(expected_theta2, r3_values, label="r3", marker='o')
plt.title("r3 vs Expected Theta 2")
plt.xlabel("Expected Theta 2 (degrees)")
plt.ylabel("r3 values")
plt.grid(True)

# pi1 (in degrees)
plt.subplot(2, 2, 4)
plt.plot(expected_theta2, pi1_values, label="pi1 (degrees)", marker='x')
plt.title("pi1 vs Expected Theta 2")
plt.xlabel("Expected Theta 2 (degrees)")
plt.ylabel("pi1 (degrees)")
plt.grid(True)

plt.tight_layout()
plt.show()
