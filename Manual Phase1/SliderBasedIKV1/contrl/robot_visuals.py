import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class RobotVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Define robot structure with DH parameters
        self.links = [
            {'name': 'base', 'length': 20.5, 'axis': 'z'},      # Base link
            {'name': 'shoulder', 'length': 28, 'axis': 'y'},    # Shoulder
            {'name': 'elbow', 'length': 28.5, 'axis': 'y'},     # Elbow
            {'name': 'wrist1', 'length': 4, 'axis': 'x'},       # Wrist pitch
            {'name': 'wrist2', 'length': 3.25, 'axis': 'z'},    # Wrist roll
            {'name': 'hand', 'length': 20, 'axis': 'z'}         # End effector
        ]
        
        self._setup_plot()
    
    def _setup_plot(self):
        """Set up the plot with proper limits and labels."""
        self.ax.cla()
        self.ax.set_xlim([-200, 200])
        self.ax.set_ylim([-200, 200])
        self.ax.set_zlim([-50, 100])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.grid(True)
    
    def transform_point(self, point, angle, axis='z'):
        """Apply rotation transformation to a point."""
        c = np.cos(np.radians(angle))
        s = np.sin(np.radians(angle))
        
        if axis == 'z':
            R = np.array([[c, -s, 0],
                         [s, c, 0],
                         [0, 0, 1]])
        elif axis == 'y':
            R = np.array([[c, 0, s],
                         [0, 1, 0],
                         [-s, 0, c]])
        elif axis == 'x':
            R = np.array([[1, 0, 0],
                         [0, c, -s],
                         [0, s, c]])
        
        return np.dot(R, point)
    
    def plot_robot_state(self, angles, show_frames=True, show_path=False, path_points=None):
        """
        Plot robot with given joint angles.
        
        Args:
            angles (list): List of 6 joint angles in degrees
            show_frames (bool): Whether to show coordinate frames at joints
            show_path (bool): Whether to show the end-effector path
            path_points (list): List of end-effector positions to show path
        """
        if len(angles) != 6:
            raise ValueError("Expected 6 angles, got {}".format(len(angles)))
            
        self._setup_plot()
        
        current_pos = np.array([0., 0., 0.])
        current_transform = np.eye(3)
        positions = [current_pos.copy()]  # Store positions for plotting
        
        # Plot each link
        for i, (angle, link) in enumerate(zip(angles, self.links)):
            # Calculate new position after rotation
            direction = np.array([0., 0., link['length']])
            if link['axis'] != 'z':
                direction = self.transform_point(direction, 90, link['axis'])
            
            # Apply current joint angle
            direction = np.dot(current_transform, direction)
            current_transform = np.dot(current_transform, 
                self.transform_point(np.eye(3), angle, link['axis']))
            
            # Calculate next position
            next_pos = current_pos + direction
            positions.append(next_pos.copy())
            
            # Plot link
            self.ax.plot([current_pos[0], next_pos[0]],
                        [current_pos[1], next_pos[1]],
                        [current_pos[2], next_pos[2]],
                        'b-', linewidth=2, label=link['name'])
            
            # Plot joint
            self.ax.scatter(current_pos[0], current_pos[1], current_pos[2],
                          color='red', s=100)
            
            if show_frames:
                # Plot coordinate frame at joint
                frame_size = 10
                for axis, color in zip(['x', 'y', 'z'], ['r', 'g', 'b']):
                    axis_dir = np.dot(current_transform, np.array([
                        frame_size if axis == 'x' else 0,
                        frame_size if axis == 'y' else 0,
                        frame_size if axis == 'z' else 0
                    ]))
                    self.ax.quiver(current_pos[0], current_pos[1], current_pos[2],
                                 axis_dir[0], axis_dir[1], axis_dir[2],
                                 color=color, alpha=0.5)
            
            current_pos = next_pos
        
        # Plot end-effector position
        self.ax.scatter(current_pos[0], current_pos[1], current_pos[2],
                       color='green', s=100, label='End-effector')
        
        # Plot path if requested
        if show_path and path_points is not None:
            path_points = np.array(path_points)
            self.ax.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2],
                        'g--', alpha=0.5, label='Path')
        
        self.ax.legend()
        plt.draw()
        return positions
    
    def show(self):
        """Display the plot."""
        plt.show()

def visualize_robot(angles, show_frames=True, show_path=False, path_points=None):
    """
    Convenience function to quickly visualize robot state.
    
    Args:
        angles (list): List of 6 joint angles in degrees
        show_frames (bool): Whether to show coordinate frames
        show_path (bool): Whether to show end-effector path
        path_points (list): List of end-effector positions for path
    """
    visualizer = RobotVisualizer()
    positions = visualizer.plot_robot_state(angles, show_frames, show_path, path_points)
    visualizer.show()
    return positions

def main():
    # Example usage
    test_angles = [30, 45, -30, 20, 15, 0]  # 6 angles in degrees
    visualize_robot(test_angles, show_frames=True)

if __name__ == "__main__":
    main()