make sure the units are in radians and  centi meter throwghout the process 
for printing r2c and c2r funtion can be used to convert to radians or degree

preety close code 
import numpy as np
from math import atan2, sqrt, cos, sin, pi

class RoboticArm6DOF:
    def __init__(self):
        # Define DH parameters [theta, d, a, alpha]
        self.dh_params = np.array([
            [0, 20.5, 0, np.pi/2],    # Joint 1
            [0, 0, 28, 0],            # Joint 2
            [0, 0, 0, np.pi/2],       # Joint 3
            [0, 32.5, 0, -np.pi/2],   # Joint 4
            [0, 0, 0, np.pi/2],       # Joint 5
            [0, 23.25, 0, 0]          # Joint 6
        ])
        
    def transform_matrix(self, theta, d, a, alpha):
        """Calculate transformation matrix based on DH parameters"""
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def inverse_kinematics(self, target_matrix):
        """
        Calculate inverse kinematics for 6-DOF robotic arm
        Args:
            target_matrix: 4x4 homogeneous transformation matrix
        Returns:
            joint_angles: List of 6 joint angles in radians
        """
        target_pos = target_matrix[:3, 3]
        target_rot = target_matrix[:3, :3]
        
        # Wrist center position calculation
        wrist_length = self.dh_params[5][1]  # d6
        wrist_center = target_pos - wrist_length * target_rot[:3, 2]
        
        # Joint 1 (base rotation)
        theta1 = atan2(wrist_center[1], wrist_center[0])
        
        # Calculate positions relative to base
        r = sqrt(wrist_center[0]**2 + wrist_center[1]**2)
        z = wrist_center[2] - self.dh_params[0][1]  # Subtract d1
        
        # Link lengths
        a2 = self.dh_params[1][2]  # a2 = 28
        d4 = self.dh_params[3][1]  # d4 = 32.5
        
        # For joints 2 and 3
        r2 = r**2 + z**2
        s = sqrt(r2)
        D = (r2 - a2**2 - d4**2)/(2*a2*d4)
        
        # Joint 3
        theta3 = atan2(-sqrt(1 - D**2), D)  # Elbow up configuration
        
        # Joint 2
        theta2 = atan2(z, r) - atan2(d4*sin(theta3), a2 + d4*cos(theta3))
        
        # Calculate R03
        R03 = self.get_R03(theta1, theta2, theta3)
        
        # Calculate wrist rotation matrix
        R36 = np.dot(R03.T, target_rot)
        
        # Extract Euler angles for the spherical wrist
        theta4 = atan2(R36[1, 2], R36[0, 2])
        theta5 = atan2(sqrt(R36[0, 2]**2 + R36[1, 2]**2), R36[2, 2])
        theta6 = atan2(R36[2, 1], -R36[2, 0])
        
        return [theta1, theta2, theta3, theta4, theta5, theta6]
    
    def get_R03(self, theta1, theta2, theta3):
        """Calculate rotation matrix R03 based on first three joint angles"""
        T01 = self.transform_matrix(theta1, self.dh_params[0][1], self.dh_params[0][2], self.dh_params[0][3])
        T12 = self.transform_matrix(theta2, self.dh_params[1][1], self.dh_params[1][2], self.dh_params[1][3])
        T23 = self.transform_matrix(theta3, self.dh_params[2][1], self.dh_params[2][2], self.dh_params[2][3])
        
        T03 = np.dot(T01, np.dot(T12, T23))
        return T03[:3, :3]

    def forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics
        Args:
            joint_angles: List of 6 joint angles in radians
        Returns:
            4x4 homogeneous transformation matrix
        """
        T = np.eye(4)
        
        for i in range(6):
            theta = joint_angles[i]
            d = self.dh_params[i][1]
            a = self.dh_params[i][2]
            alpha = self.dh_params[i][3]
            Ti = self.transform_matrix(theta, d, a, alpha)
            T = np.dot(T, Ti)
        
        return T

# Example usage with the given target matrix
if __name__ == "__main__":
    robot = RoboticArm6DOF()
    
    # Given target homogeneous matrix
    target_matrix = np.array([
        [2.52243649e-01, 1.04367061e-01, 9.62019053e-01, 6.77419430e+01],
        [-7.87379763e-01, -5.55769053e-01, 2.66746825e-01, 3.23991321e+01],
        [5.62500000e-01, -8.24759526e-01, -5.80127019e-02, 1.69012047e+01],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
    
    # Calculate inverse kinematics
    joint_angles = robot.inverse_kinematics(target_matrix)
    print("\nCalculated joint angles (radians):", joint_angles)
    print("Calculated joint angles (degrees):", np.degrees(joint_angles))
    
    # Verify solution using forward kinematics
    final_matrix = robot.forward_kinematics(joint_angles)
    print("\nVerification using forward kinematics:")
    print("Final transformation matrix:\n", final_matrix)
    
    # Calculate position error
    pos_error = np.linalg.norm(final_matrix[:3, 3] - target_matrix[:3, 3])
    print("\nPosition error:", pos_error)
    
    # Calculate orientation error (Frobenius norm of rotation difference)
    ori_error = np.linalg.norm(final_matrix[:3, :3] - target_matrix[:3, :3], 'fro')
    print("Orientation error:", ori_error)

output = 29 30 -60 16 115 63 (error is in major three axis)