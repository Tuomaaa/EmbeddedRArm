import tkinter as tk
from tkinter import ttk
import RPi.GPIO as GPIO
import requests
import time
import threading

# ESP32 IP address
ESP32_IP = "192.168.1.53"
ESP32_SERVO_URL = f"http://{ESP32_IP}/servo"
ESP32_POT_URL = f"http://{ESP32_IP}/pot"
ESP32_MODE_URL = f"http://{ESP32_IP}/mode"
control_mode = "slider"
running = True

pot_angle = 0

def send_angle_to_esp32(angle):
    """Send angle command to ESP32"""
    try:
        params = {"angle": int(angle)}
        response = requests.get(ESP32_SERVO_URL, params=params, timeout=1)
        
        if response.status_code == 200:
            print(f"SUCCESS: {response.text}")
            return True
        else:
            print(f"ERROR: {response.status_code} - {response.text}")
            return False
    except Exception as e:
        print(f"FAILED: {e}")
        angle_label.config(text=f"**Error**")
        return False

def update_mode_to_esp32(mode):
    try:
        if mode == "slider":
            params = {"mode": 1}
        else:
            params = {"mode": 0}
        response = requests.get(ESP32_MODE_URL, params=params, timeout=1)
        
        if response.status_code == 200:
            print(f"SUCCESS: {response.text}")
            return True
        else:
            print(f"ERROR: {response.status_code} - {response.text}")
            return False
    except Exception as e:
        print(f"FAILED: {e}")
        angle_label.config(text=f"**Mode Error**")
        return False

def read_pot_from_esp32():
    """Read potentiometer angle from ESP32"""
    try:
        response = requests.get(ESP32_POT_URL, timeout=1)
        if response.status_code == 200:
            data = response.json()
            return data.get("angle", 0)
    except Exception as e:
        print(f"Error reading POT from ESP32: {e}")
    return None
    
def set_angle_from_slider(angle):
    """Called when slider moves"""
    if control_mode == "slider":
        angle_int = int(float(angle))
        angle_label.config(text=f"Current Angle: {angle_int}")
        send_angle_to_esp32(angle_int)

def toggle_mode():
    """Toggle between slider and potentiometer mode"""
    global control_mode
    
    if control_mode == "slider":
        control_mode = "pot"
        mode_button.config(text="Mode: Potentiometer (ESP32)")
        slider.config(state="disabled")
        status_label.config(text="Reading from ESP32 potentiometer...")
        update_mode_to_esp32(control_mode)
    else:
        control_mode = "slider"
        mode_button.config(text="Mode: Slider")
        slider.config(state="normal")
        status_label.config(text="Using slider control")
        update_mode_to_esp32(control_mode)

def pot_update_loop():
    """Continuously read potentiometer from ESP32 and update display"""
    global pot_angle
    while running:
        angle = read_pot_from_esp32()
        if angle is not None:
            pot_angle = angle
            # Update GUI in main thread
            if control_mode == "pot":
                root.after(0, lambda: angle_label.config(text=f"POT Angle: {pot_angle}"))
            root.after(0, lambda: pot_display_label.config(text=f"ESP32 POT: {pot_angle}"))
            time.sleep(0.5)
            
# Create GUI window
root = tk.Tk()
root.title("Servo Control")
root.geometry("400x200")

# Angle display label
angle_label = tk.Label(root, text="Current Angle: 0 degrees", font=("Arial", 16))
angle_label.pack(pady=20)

# Mode toggle button
mode_button = tk.Button(
    root,
    text="Mode: Slider",
    font=("Arial", 12),
    command=toggle_mode,
    width=25,
    height=2
)
mode_button.pack(pady=10)

status_label = tk.Label(root, text="Using slider control", font=("Arial", 10))
status_label.pack(pady=5)

pot_display_label = tk.Label(
    root, 
    text="ESP32 POT: -- ", 
    font=("Arial", 12),
    fg="blue"
)
pot_display_label.pack(pady=5)

angle_label = tk.Label(root, text="Current Angle: 90", font=("Arial", 16))
angle_label.pack(pady=10)

# Slider for angle control
slider = ttk.Scale(
    root, 
    from_=0, 
    to=180, 
    orient="horizontal",
    length=400,
    command=set_angle_from_slider
)
slider.set(90)  # Start at center position
slider.pack(pady=10)

connection_label = tk.Label(
    root, 
    text=f"ESP32: {ESP32_IP}", 
    font=("Arial", 9),
    fg="gray"
)
connection_label.pack(pady=5)

pot_thread = threading.Thread(target=pot_update_loop, daemon=True)
pot_thread.start()

update_mode_to_esp32(control_mode)

# Cleanup function
def on_closing():
    global running
    running = False
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
