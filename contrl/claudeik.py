from math import atan2, sqrt, cos, sin, pi
import numpy as np

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
        ct, st = cos(theta), sin(theta)
        ca, sa = cos(alpha), sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def inverse_kinematics(self, target_matrix):
        """
        Calculate inverse kinematics for 6-DOF robotic arm using geometric approach
        Args:
            target_matrix: 4x4 homogeneous transformation matrix
        Returns:
            joint_angles: List of 6 joint angles in radians
        """
        # Extract position and rotation from target matrix
        pos = target_matrix[:3, 3]
        rot = target_matrix[:3, :3]
        print(f"haiji {rot}")
        
        # Constants from DH parameters
        d1 = self.dh_params[0][1]  # 20.5
        a2 = self.dh_params[1][2]  # 28
        d4 = self.dh_params[3][1]  # 32.5
        d6 = self.dh_params[5][1]  # 23.25
        
        # Calculate wrist center position
        wc = pos - d6 * rot[:3, 2]

        print(f"?{rot[:3, 2]}")
        
        # Solve for theta1 (base rotation)
        theta1 = atan2(wc[1], wc[0])
        
        # Calculate planar coordinates for arm triangle
        r = sqrt(wc[0]**2 + wc[1]**2)
        z = wc[2] - d1
        
        # Calculate arm triangle sides
        s = sqrt(r**2 + z**2)  # Distance from shoulder to wrist center
        
        # Law of cosines for theta3
        cos_theta3 = (s**2 - a2**2 - d4**2) / (2 * a2 * d4)
        # Bound cos_theta3 to prevent domain errors
        cos_theta3 = max(min(cos_theta3, 1), -1)
        theta3 = atan2(-sqrt(1 - cos_theta3**2), cos_theta3)  # Elbow-up configuration
        
        # Solve for theta2
        beta = atan2(z, r)
        psi = atan2(d4 * sin(theta3), a2 + d4 * cos(theta3))
        theta2 = beta - psi
        
        # Calculate R03
        R03 = self.get_R03(theta1, theta2, theta3)
        
        # Calculate R36 = (R03)^T * R06
        R36 = np.dot(R03.T, rot)
        
        # Extract Euler angles from R36
        theta4 = atan2(R36[1, 2], R36[0, 2])
        theta5 = atan2(sqrt(R36[0, 2]**2 + R36[1, 2]**2), R36[2, 2])
        theta6 = atan2(R36[2, 1], -R36[2, 0])
        
        # Normalize angles to [-pi, pi]
        angles = [theta1, theta2, theta3, theta4, theta5, theta6]
        angles = [(angle + pi) % (2*pi) - pi for angle in angles]
        
        return angles
    
    def get_R03(self, theta1, theta2, theta3):
        """Calculate rotation matrix R03 based on first three joint angles"""
        T01 = self.transform_matrix(theta1, self.dh_params[0][1], self.dh_params[0][2], self.dh_params[0][3])
        T12 = self.transform_matrix(theta2, self.dh_params[1][1], self.dh_params[1][2], self.dh_params[1][3])
        T23 = self.transform_matrix(theta3, self.dh_params[2][1], self.dh_params[2][2], self.dh_params[2][3])
        
        T03 = np.dot(T01, np.dot(T12, T23))
        return T03[:3, :3]

    def forward_kinematics(self, joint_angles):
        """Calculate forward kinematics"""
        T = np.eye(4)
        for i, theta in enumerate(joint_angles):
            d = self.dh_params[i][1]
            a = self.dh_params[i][2]
            alpha = self.dh_params[i][3]
            Ti = self.transform_matrix(theta, d, a, alpha)
            T = np.dot(T, Ti)
        return T

def test_ik_solution(robot, target_matrix):
    """Test the inverse kinematics solution with detailed error reporting"""
    print("Testing inverse kinematics solution...")
    print("\nTarget matrix:\n", target_matrix)
    
    # Calculate inverse kinematics
    joint_angles = robot.inverse_kinematics(target_matrix)
    print("\nCalculated joint angles (radians):", 
          [f"{angle:.6f}" for angle in joint_angles])
    print("Calculated joint angles (degrees):", 
          [f"{np.degrees(angle):.6f}" for angle in joint_angles])
    
    # Verify with forward kinematics
    result_matrix = robot.forward_kinematics(joint_angles)
    print("\nResult matrix:\n", result_matrix)
    
    # Calculate errors
    pos_error = np.linalg.norm(result_matrix[:3, 3] - target_matrix[:3, 3])
    rot_error = np.linalg.norm(result_matrix[:3, :3] - target_matrix[:3, :3], 'fro')
    
    print("\nError analysis:")
    print(f"Position error: {pos_error:.6f}")
    print(f"Rotation error: {rot_error:.6f}")
    
    return pos_error, rot_error

if __name__ == "__main__":
    robot = RoboticArm6DOF()
    
    # Test matrix
    target_matrix = np.array([
        [-2.18750000e-01, 9.20151992e-01, 3.24759526e-01, 2.86235718e+01],
        [-5.41265877e-02, -3.43750000e-01, 9.37500000e-01, -1.47024807e+01],
        [9.74278579e-01, 1.87500000e-01, 1.25000000e-01, 6.39049613e+01],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
    
    test_ik_solution(robot, target_matrix)