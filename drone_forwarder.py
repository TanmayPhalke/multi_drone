# drone_forwarder.py
import time
import requests
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import traceback

# -------------------
# CONFIGURATION
# -------------------
DRONE_ID = "Drone1"
GCS_IP = "110.113.114.176"       # Replace with your actual GCS IP
GCS_PORT = 14550
PIXHAWK_PORT = "/dev/ttyACM0"  # Could be /dev/ttyUSB0 on some systems
BAUD_RATE = 57600

print(f"[INFO] Starting Drone Forwarder for {DRONE_ID}")

# -------------------
# CONNECT TO PIXHAWK
# -------------------
try:
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD_RATE)
    print(f"[CONNECTED] Pixhawk via {PIXHAWK_PORT} @ {BAUD_RATE}")
except Exception as e:
    print(f"[ERROR] Cannot connect to Pixhawk: {e}")
    traceback.print_exc()
    exit(1)

# -------------------
# OPEN UDP TO GCS
# -------------------
try:
    udp = mavutil.mavlink_connection(f"udpout:{GCS_IP}:{GCS_PORT}")
    print(f"[CONNECTED] Opened UDP to GCS {GCS_IP}:{GCS_PORT}")
except Exception as e:
    print(f"[ERROR] Cannot open UDP connection to GCS: {e}")
    traceback.print_exc()
    exit(1)

# -------------------
# SEND GCS HEARTBEAT TO INITIATE HANDSHAKE
# -------------------
try:
    mav_sender = mavlink2.MAVLink(udp)
    mav_sender.srcSystem = 255
    mav_sender.srcComponent = 0
    heartbeat = mav_sender.heartbeat_encode(
        type=mavlink2.MAV_TYPE_GCS,
        autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavlink2.MAV_STATE_ACTIVE
    )
    udp.write(heartbeat.pack())
    print("[INFO] GCS heartbeat sent to Pixhawk ✅")
except Exception as e:
    print(f"[ERROR] Failed to send GCS heartbeat: {e}")
    traceback.print_exc()

# -------------------
# REGISTER WITH GCS BACKEND
# -------------------
while True:
    try:
        response = requests.post(f"http://{GCS_IP}:5000/register", json={
            'drone_id': DRONE_ID,
            'port': GCS_PORT
        })
        if response.ok:
            print(f"[REGISTERED] {DRONE_ID} with GCS backend ✅")
            break
        else:
            print(f"[WARN] GCS backend returned status {response.status_code}")
    except Exception as e:
        print(f"[WARN] Registration error: {e}")
        traceback.print_exc()
    time.sleep(3)

# -------------------
# FORWARD MAVLINK DATA TO GCS
# -------------------
print("[INFO] Forwarding loop started...")

while True:
    try:
        data = mav.recv(1024)
        if data:
            udp.write(data)
            print(f"[DEBUG] Forwarded {len(data)} bytes to GCS")
    except Exception as e:
        print(f"[ERROR] Forwarding loop error: {e}")
        traceback.print_exc()
        time.sleep(1)
