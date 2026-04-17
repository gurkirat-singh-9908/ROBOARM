# stores a function that maps the latest_values from fetch_data.py to the servo angles

def mapVal(value, min_val, max_val):
    """
    Maps a value from one range to another (0-180 for servo angles)
    Formula: (current_value / (max - min)) * 180
    
    Args:
        value (float): Current value from slider
        min_val (float): Minimum value of the slider range
        max_val (float): Maximum value of the slider range
    
    Returns:
        float: Mapped value between 0 and 180
    """
    return (value / (max_val - min_val)) * 180

def map_sliders_to_servos(values):
    """
    Maps all slider values to their corresponding servo angles
    
    Args:
        values (dict): Dictionary containing slider values
    
    Returns:
        dict: Dictionary containing mapped servo angles
    """
    # Define the ranges for each slider
    ranges = {
        'slider_x': (0, 1.6),
        'slider_y': (0, 1.6),
        'slider_z': (0, 1.6),
        'roll': (0, 180),
        'pitch': (0, 180),
        'yaw': (0, 180),
        'slider_gripper': (0, 100)
    }
    
    # Convert string values to float before mapping
    try:
        float_values = {
            'slider_x': float(values['slider_x']),
            'slider_y': float(values['slider_y']),
            'slider_z': float(values['slider_z']),
            'roll': float(values['roll']),
            'pitch': float(values['pitch']),
            'yaw': float(values['yaw']),
            'slider_gripper': float(values['slider_gripper'])
        }
        
        # Map each slider to its corresponding servo
        servo_angles = {
            's1': mapVal(float_values['slider_x']+0.8, ranges['slider_x'][0], ranges['slider_x'][1]),
            's2': mapVal(float_values['slider_y']+0.8, ranges['slider_y'][0], ranges['slider_y'][1]),
            's3': mapVal(float_values['slider_z']+0.8, ranges['slider_z'][0], ranges['slider_z'][1]),
            's4': mapVal(float_values['roll'], ranges['roll'][0], ranges['roll'][1]),
            's5': mapVal(float_values['pitch'], ranges['pitch'][0], ranges['pitch'][1]),
            's6': mapVal(float_values['yaw'], ranges['yaw'][0], ranges['yaw'][1])
        }
        
        return servo_angles
    except (ValueError, TypeError) as e:
        print(f"Error converting values to float: {e}")
        print("Values received:", values)
        return None