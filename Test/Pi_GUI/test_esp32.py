import requests
import time

# ESP32 IP address
ESP32_IP = "192.168.1.53"
ESP32_URL = f"http://{ESP32_IP}/servo"

def send_angle(angle):
    """Send angle command to ESP32"""
    try:
        params = {"angle": angle}
        response = requests.get(ESP32_URL, params=params)
        
        if response.status_code == 200:
            print(f"SUCCESS: {response.text}")
            return True
        else:
            print(f"ERROR: {response.status_code} - {response.text}")
            return False
    except Exception as e:
        print(f"FAILED: {e}")
        return False

# Test different angles
print("Testing Pi -> ESP32 communication")
print("=" * 40)

angles = [0, 45, 90, 135, 180]

for angle in angles:
    print(f"\nSending angle: {angle}")
    send_angle(angle)
    time.sleep(2)

print("\nTest complete!")
