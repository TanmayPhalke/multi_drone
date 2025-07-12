import time
import requests
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# ------------------------
# CONFIGURATION
# ------------------------
DRONE_ID = "Drone1"
GCS_IP = "192.168.1.100"     # 🟡 Your GCS (Windows) IP
GCS_PORT = 14550             # 🟡 The port GCS listens on
PIXHAWK_PORT = "/dev/ttyACM0"
BAUD_RATE = 57600

# ------------------------
# CONNECT TO PIXHAWK
# ------------------------
print(f"[INFO] Connecting to Pixhawk on {PIXHAWK_PORT} @ {BAUD_RATE} baud...")
try:
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD_RATE)
    print("[INFO] Connected to Pixhawk.")
except Exception as e:
    print(f"[ERROR] Cannot connect to Pixhawk: {e}")
    exit(1)

# ------------------------
# OPEN UDP TO GCS
# ------------------------
try:
    udp = mavutil.mavlink_connection(f"udpout:{GCS_IP}:{GCS_PORT}")
    print(f"[INFO] Forwarding MAVLink to GCS at {GCS_IP}:{GCS_PORT}")
except Exception as e:
    print(f"[ERROR] Cannot open UDP connection to GCS: {e}")
    exit(1)

# ------------------------
# SEND INITIAL GCS HEARTBEAT
# ------------------------
mav_sender = mavlink2.MAVLink(udp)
mav_sender.srcSystem = 255
mav_sender.srcComponent = 0

try:
    hb = mav_sender.heartbeat_encode(
        type=mavlink2.MAV_TYPE_GCS,
        autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavlink2.MAV_STATE_ACTIVE
    )
    udp.write(hb.pack())
    print("[INFO] Initial GCS heartbeat sent to Pixhawk")
except Exception as e:
    print(f"[ERROR] Failed to send initial heartbeat: {e}")

# ------------------------
# REGISTER WITH GCS BACKEND
# ------------------------
while True:
    try:
        requests.post(f"http://{GCS_IP}:5000/register", json={
            'drone_id': DRONE_ID,
            'port': GCS_PORT
        })
        print(f"[INFO] Registered {DRONE_ID} with GCS backend")
        break
    except Exception as e:
        print(f"[WARN] Registration failed: {e}")
        time.sleep(3)

# ------------------------
# FORWARD LOOP
# ------------------------
print("[INFO] Starting MAVLink forwarding loop...")
while True:
    try:
        data = mav.recv(1024)
        if data:
            print(f"[DEBUG] Forwarding {len(data)} bytes to GCS")
            udp.write(data)
    except Exception as e:
        print(f"[ERROR] Forwarding error: {e}")
        time.sleep(1)
