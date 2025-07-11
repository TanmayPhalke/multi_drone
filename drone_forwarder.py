# drone/drone_forwarder.py
import time
import requests
from pymavlink import mavutil

# CONFIGURE THIS:
DRONE_ID = "Drone1"
GCS_IP = "192.168.1.7"        # Replace with your Windows PC IP
GCS_PORT = 14550              # Must match what your Flask backend listens to
PIXHAWK_PORT = "/dev/ttyACM0" # Or /dev/ttyUSB0 if that's what Pixhawk shows up as
BAUD_RATE = 57600

# Connect to Pixhawk over USB
print(f"[INFO] Connecting to Pixhawk on {PIXHAWK_PORT} at {BAUD_RATE} baud...")
try:
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD_RATE)
    print("[INFO] Connected to Pixhawk.")
except Exception as e:
    print(f"[ERROR] Cannot connect to Pixhawk: {e}")
    exit(1)

# Connect to Ground Station via UDP
udp = mavutil.mavlink_connection(f"udpout:{GCS_IP}:{GCS_PORT}")
print(f"[INFO] Forwarding MAVLink to GCS at {GCS_IP}:{GCS_PORT}")

# Register with GCS Flask server
while True:
    try:
        requests.post(f"http://{GCS_IP}:5000/register", json={
            'drone_id': DRONE_ID,
            'port': GCS_PORT
        })
        print(f"[INFO] Registered {DRONE_ID} with GCS.")
        break
    except Exception as e:
        print(f"[WARN] GCS registration failed: {e}")
        time.sleep(3)

# Forward loop
print("[INFO] Starting MAVLink forwarder loop...")
while True:
    try:
        msg = mav.recv_match(blocking=True)
        if msg:
            udp.mav.send(msg)
    except Exception as e:
        print(f"[ERROR] Forwarding error: {e}")
        time.sleep(1)
