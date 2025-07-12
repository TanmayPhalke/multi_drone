import time
import requests
from pymavlink import mavutil

# ------------------------
# CONFIGURATION
# ------------------------
DRONE_ID = "Drone1"
GCS_IP = "100.113.240.132"      # VPN IP of your GCS (Windows PC)
GCS_PORT = 14550                # Port GCS listens on
PIXHAWK_PORT = "/dev/ttyACM0"   # Serial port to Pixhawk (check with `ls /dev/tty*`)
BAUD_RATE = 57600               # Common baud rate for Pixhawk

# ------------------------
# CONNECT TO PIXHAWK
# ------------------------
print(f"[INFO] Connecting to Pixhawk on {PIXHAWK_PORT} at {BAUD_RATE} baud...")
try:
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD_RATE)
    print("[INFO] Connected to Pixhawk.")
except Exception as e:
    print(f"[ERROR] Cannot connect to Pixhawk: {e}")
    exit(1)

# ------------------------
# CONNECT TO GCS (UDP)
# ------------------------
try:
    udp = mavutil.mavlink_connection(f"udpout:{GCS_IP}:{GCS_PORT}")
    print(f"[INFO] Forwarding MAVLink to GCS at {GCS_IP}:{GCS_PORT}")
except Exception as e:
    print(f"[ERROR] Could not open UDP connection to GCS: {e}")
    exit(1)

# ------------------------
# REGISTER WITH GCS SERVER
# ------------------------
while True:
    try:
        response = requests.post(f"http://{GCS_IP}:5000/register", json={
            'drone_id': DRONE_ID,
            'port': GCS_PORT
        })
        if response.status_code == 200:
            print(f"[INFO] Registered {DRONE_ID} with GCS.")
            break
        else:
            print(f"[WARN] GCS registration failed: {response.text}")
    except Exception as e:
        print(f"[WARN] GCS registration failed: {e}")
    time.sleep(3)

# ------------------------
# FORWARD MAVLINK (RAW BYTES)
# ------------------------
print("[INFO] Starting MAVLink forwarder loop...")
while True:
    try:
        data = mav.recv(1024)
        if data:
            udp.write(data)
    except Exception as e:
        print(f"[ERROR] Forwarding failed: {e}")
        time.sleep(1)
