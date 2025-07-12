# drone/drone_forwarder.py
import time
import requests
from pymavlink import mavutil

DRONE_ID = "Drone1"
GCS_IP = "192.168.1.100"     # Replace with your GCS IP (LAN or VPN)
GCS_PORT = 14550              # Must match backend listener
PIXHAWK_PORT = "/dev/ttyACM0"  # Adjust if needed
BAUD_RATE = 57600

print(f"[INFO] Connecting to Pixhawk on {PIXHAWK_PORT} @ {BAUD_RATE} baud...")
try:
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD_RATE)
    print("[INFO] Connected to Pixhawk.")
except Exception as e:
    print(f"[ERROR] Cannot connect to Pixhawk: {e}")
    exit(1)

try:
    udp = mavutil.mavlink_connection(f"udpout:{GCS_IP}:{GCS_PORT}")
    print(f"[INFO] Forwarding MAVLink to GCS at {GCS_IP}:{GCS_PORT}")
except Exception as e:
    print(f"[ERROR] Cannot open UDP connection to GCS: {e}")
    exit(1)

while True:
    try:
        requests.post(f"http://{GCS_IP}:5000/register", json={
            'drone_id': DRONE_ID,
            'port': GCS_PORT
        })
        print(f"[INFO] Registered {DRONE_ID} with GCS")
        break
    except Exception as e:
        print(f"[WARN] Registration failed: {e}")
        time.sleep(3)

print("[INFO] Starting MAVLink forwarding loop...")
while True:
    try:
        data = mav.recv(1024)
        if data:
            udp.write(data)
    except Exception as e:
        print(f"[ERROR] Forwarding error: {e}")
        time.sleep(1)
