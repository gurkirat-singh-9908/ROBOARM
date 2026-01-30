import numpy as np
from scipy.optimize import fsolve

# DH parameters (example values; adjust as needed)
d = [20.5, 0, 0, 32.5, 0, 23.25]  # Link offsets
a = [0, 28, 0, 0, 0, 0]           # Link lengths
alpha = [np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0]  # Twist angles

def dh_matrix(theta, d, a, alpha):
    """Compute the DH transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(thetas):
    """Compute the end-effector position and orientation from joint angles."""
    T = np.eye(4)
    for i in range(6):
        T_i = dh_matrix(thetas[i], d[i], a[i], alpha[i])
        T = T @ T_i
    p = T[:3, 3]  # Position
    R = T[:3, :3]  # Rotation matrix
    return p, R

def rotation_from_matrix(R):
    """Convert a rotation matrix to angle and axis (rotation vector components)."""
    trace = np.trace(R)
    if trace > 3 - 1e-6:  # Nearly identity matrix
        angle = 0
        axis = np.array([1, 0, 0])  # Arbitrary axis
    elif trace < -1 + 1e-6:  # 180-degree rotation
        angle = np.pi
        axis = np.sqrt((R + np.eye(3)) / 2).diagonal()
        axis /= np.linalg.norm(axis)
    else:
        angle = np.arccos((trace - 1) / 2)
        axis = np.array([R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]]) / (2 * np.sin(angle))
    return angle, axis

def error_function(thetas, p_desired, R_desired):
    """Compute the 6-element error vector for fsolve."""
    p, R = forward_kinematics(thetas)
    # Position error (3 elements)
    error_p = p - p_desired
    # Orientation error (3 elements via rotation vector)
    R_error = R @ R_desired.T  # Relative rotation
    angle, axis = rotation_from_matrix(R_error)
    error_R = angle * axis  # Scale axis by angle
    # Combine into 6-element vector
    error = np.hstack((error_p, error_R))
    return error

def inverse_kinematics(p_desired, R_desired, initial_guess):
    """Solve for joint angles given desired position and orientation."""
    def error_func(thetas):
        return error_function(thetas, p_desired, R_desired)
    thetas_solution, info, ier, mesg = fsolve(error_func, initial_guess, full_output=True)
    if ier == 1:  # Success
        return thetas_solution
    else:
        print("fsolve did not converge:", mesg)
        return None

def test_inverse_kinematics(original_angles):
    """Test the inverse kinematics solution."""
    p_desired, R_desired = forward_kinematics(original_angles)
    thetas_solution = inverse_kinematics(p_desired, R_desired, original_angles)
    if thetas_solution is not None:
        p_calc, R_calc = forward_kinematics(thetas_solution)
        if np.allclose(p_calc, p_desired, atol=1e-6) and np.allclose(R_calc, R_desired, atol=1e-6):
            print("Yes")
        else:
            print("No", p_desired, p_calc)
    else:
        print("No solution found")

if __name__ == "__main__":
    original_angles = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    test_inverse_kinematics(original_angles)