import tkinter as tk
from tkinter import ttk
import math
import numpy as np
import threading
import time

def create_interface(update_interval=0.1):
    # Create the 2D array to store position and rotation values
    params = np.zeros((2, 3))  # [0,:] = position, [1,:] = rotation
    
    # Keep track of previous values to detect changes
    prev_params = np.zeros((2, 3))
    
    # Flag to track value changes
    values_changed = False
    
    # Create the main window
    root = tk.Tk()
    root.title("Position and Rotation Controls")
    root.geometry("500x450")
    
    # Flag to indicate if the interface is running
    running = True
    
    # Function to update param values
    def update_param(param_type, index, value):
        nonlocal values_changed
        
        if param_type == 'pos':
            new_value = float(value)
            if params[0, index] != new_value:
                params[0, index] = new_value
                values_changed = True
        else:  # rotation
            new_value = int(value)
            if params[1, index] != new_value:
                params[1, index] = new_value
                values_changed = True
    
    # Create position controls
    pos_frame = ttk.LabelFrame(root, text="Position (-0.8 to 0.8)")
    pos_frame.pack(fill="x", padx=10, pady=10)
    
    pos_labels = ['x', 'y', 'z']
    for i, label in enumerate(pos_labels):
        ttk.Label(pos_frame, text=f"{label}:").grid(row=i, column=0, padx=5, pady=5)
        
        slider = ttk.Scale(
            pos_frame,
            from_=-0.8,
            to=0.8,
            orient=tk.HORIZONTAL,
            length=350,
            command=lambda value, idx=i: update_param('pos', idx, value)
        )
        slider.set(0.0)
        slider.grid(row=i, column=1, padx=5, pady=5)
        
        val_label = ttk.Label(pos_frame, text="0.00")
        val_label.grid(row=i, column=2, padx=5, pady=5)
        slider.bind("<Motion>", lambda event, lbl=val_label, s=slider: 
                   lbl.config(text=f"{s.get():.2f}"))
    
    # Create rotation controls
    rot_frame = ttk.LabelFrame(root, text="Rotation (0° to 180°)")
    rot_frame.pack(fill="x", padx=10, pady=10)
    
    # Function to draw a knob
    def draw_knob(canvas, value, index):
        canvas.delete("all")
        
        # Draw circle and indicator line
        canvas.create_oval(10, 10, 90, 90, width=2)
        angle = math.radians(value - 90)
        x = 50 + 35 * math.cos(angle)
        y = 50 + 35 * math.sin(angle)
        canvas.create_line(50, 50, x, y, width=3, fill="red")
        canvas.create_text(50, 95, text=f"{value}°")
        
        # Update parameter value
        update_param('rot', index, value)
    
    # Function to handle knob interaction
    def on_knob_drag(event, canvas, index, value_label):
        cx, cy = 50, 50
        dx, dy = event.x - cx, event.y - cy
        angle = round(math.degrees(math.atan2(dy, dx)) + 90)
        
        # Clamp to range
        angle = max(0, min(180, angle))
        
        # Update knob and label
        draw_knob(canvas, angle, index)
        value_label.config(text=f"{angle}°")
    
    # Create rotation knobs
    rot_labels = ['roll', 'pitch', 'yaw']
    
    for i, label in enumerate(rot_labels):
        # Create frame for each knob
        frame = ttk.Frame(rot_frame)
        frame.grid(row=0, column=i, padx=15, pady=5)
        
        # Label
        ttk.Label(frame, text=label).pack(pady=2)
        
        # Canvas for knob
        canvas = tk.Canvas(frame, width=100, height=100, bg="white")
        canvas.pack()
        
        # Value label
        value_label = ttk.Label(frame, text="0°")
        value_label.pack(pady=2)
        
        # Draw initial knob
        draw_knob(canvas, 0, i)
        
        # Bind events - use lambda to capture the current values of variables
        canvas.bind("<B1-Motion>", 
                    lambda event, c=canvas, idx=i, lbl=value_label: 
                    on_knob_drag(event, c, idx, lbl))
    
    # Status panel showing current values
    status_frame = ttk.LabelFrame(root, text="Current Values")
    status_frame.pack(fill="x", padx=10, pady=10)
    
    # Create labels for displaying values
    pos_status = ttk.Label(status_frame, text="Position: [0.00, 0.00, 0.00]")
    pos_status.pack(pady=2)
    
    rot_status = ttk.Label(status_frame, text="Rotation: [0°, 0°, 0°]")
    rot_status.pack(pady=2)
    
    # Update status labels function
    def update_status():
        nonlocal values_changed, prev_params
        
        while running:
            # Check if values have changed
            if values_changed or not np.array_equal(params, prev_params):
                # Update status labels
                pos_str = f"Position: [{params[0,0]:.2f}, {params[0,1]:.2f}, {params[0,2]:.2f}]"
                rot_str = f"Rotation: [{int(params[1,0])}°, {int(params[1,1])}°, {int(params[1,2])}°]"
                
                # Use after method to update GUI from the main thread
                root.after_idle(lambda p=pos_str: pos_status.config(text=p))
                root.after_idle(lambda r=rot_str: rot_status.config(text=r))
                
                # Print to console for debugging
                #print(f"Parameters updated: {pos_str}, {rot_str}")
                
                # Reset change flag and store current values
                values_changed = False
                prev_params = np.copy(params)
            
            # Wait for the next update check
            time.sleep(update_interval)
    
    # Handle window close
    def on_closing():
        nonlocal running
        running = False
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Start the update thread
    update_thread = threading.Thread(target=update_status, daemon=True)
    update_thread.start()
    
    # Start the main loop (this will block until the window is closed)
    root.mainloop()
    
    # Return the parameters array
    return params

# Example usage
if __name__ == "__main__":
    result = create_interface()
    print(result)
    print("Final position (x, y, z):", result[0])
    print("Final rotation (roll, pitch, yaw):", result[1])