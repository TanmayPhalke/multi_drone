# drone_forwarder.py
import time
import requests
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import traceback

# ----------------------------
# CONFIGURATION
# ----------------------------
DRONE_ID = "Drone1"
GCS_IP = "100.113.114.176"  # ← Set this to your actual GCS IP
GCS_PORT = 14550
PIXHAWK_PORT = "/dev/ttyACM0"  # Or /dev/ttyUSB0 depending on setup
BAUD_RATE = 57600

print(f"[BOOT] Forwarder Starting for {DRONE_ID}")
print(f"[INFO] Pixhawk port: {PIXHAWK_PORT}, GCS: {GCS_IP}:{GCS_PORT}")

# ----------------------------
# CONNECT TO PIXHAWK
# ----------------------------
try:
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD_RATE)
    print(f"[OK] Connected to Pixhawk on {PIXHAWK_PORT}")
except Exception as e:
    print(f"[ERROR] Could not connect to Pixhawk: {e}")
    traceback.print_exc()
    exit(1)

# ----------------------------
# OPEN UDP LINK TO GCS
# ----------------------------
try:
    udp = mavutil.mavlink_connection(f"udpout:{GCS_IP}:{GCS_PORT}")
    print(f"[OK] UDP link established to GCS at {GCS_IP}:{GCS_PORT}")
except Exception as e:
    print(f"[ERROR] Could not open UDP link: {e}")
    traceback.print_exc()
    exit(1)

# ----------------------------
# SEND GCS HEARTBEAT
# ----------------------------
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
    udp.write(heartbeat.pack(mav_sender))
    print("[✓] GCS Heartbeat sent to Pixhawk")
except Exception as e:
    print(f"[ERROR] Failed to send GCS heartbeat: {e}")
    traceback.print_exc()

# ----------------------------
# REGISTER WITH GCS BACKEND
# ----------------------------
while True:
    try:
        print(f"[INFO] Attempting registration with GCS at http://{GCS_IP}:5000/register")
        res = requests.post(f"http://{GCS_IP}:5000/register", json={
            'drone_id': DRONE_ID,
            'port': GCS_PORT
        }, timeout=5)
        if res.ok:
            print(f"[✓] {DRONE_ID} registered with GCS backend")
            break
        else:
            print(f"[WARN] GCS registration failed with status {res.status_code}")
    except Exception as e:
        print(f"[WARN] Registration failed: {e}")
        traceback.print_exc()
    time.sleep(3)

# ----------------------------
# FORWARD MAVLINK PACKETS
# ----------------------------
print("[INFO] Forwarding MAVLink packets...")
while True:
    try:
        packet = mav.recv(1024)
        if packet:
            udp.write(packet)
            print(f"[DEBUG] Forwarded {len(packet)} bytes")
    except Exception as e:
        print(f"[ERROR] Packet forwarding error: {e}")
        traceback.print_exc()
        time.sleep(1)
