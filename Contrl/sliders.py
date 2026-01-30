import tkinter as tk

slider_val = [90,90,90,90,90,90]

def slider_value(value, index):
    global slider_val
    slider_val[index] = int(value)
    print(slider_val)

# Create the main window
root = tk.Tk()
root.title("Six Sliders with Default Values")

# Create 6 sliders dynamically
sliders = []

for i in range(6):
    tk.Label(root, text=f"Slider {i + 1}").pack()
    slider = tk.Scale(
        root, 
        from_=0, 
        to=180, 
        orient='horizontal', 
        command=lambda value, idx=i: slider_value(value, idx)
    )
    slider.pack()
    slider.set(90)  # Set default value
    sliders.append(slider)

root.mainloop()
