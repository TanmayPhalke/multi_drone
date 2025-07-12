# drone/drone_forwarder.py

import time
import requests
import traceback
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# ========== CONFIG ==========
DRONE_ID     = "Drone1"                        # Change for each drone
GCS_IP       = "100.113.114.176"               # GCS backend IP (VPN-facing)
GCS_PORT     = 14550                           # Must match GCS listening port
PIXHAWK_PORT = "/dev/ttyACM0"                  # Usually /dev/ttyACM0 or /dev/ttyUSB0
BAUD_RATE    = 57600

print(f"[BOOT] Forwarder Starting for {DRONE_ID}")
print(f"[INFO] Pixhawk port: {PIXHAWK_PORT}, GCS: {GCS_IP}:{GCS_PORT}")

# ========== CONNECT TO PIXHAWK ==========
try:
    mav = mavutil.mavlink_connection(PIXHAWK_PORT, baud=BAUD_RATE)
    print(f"[OK] Connected to Pixhawk on {PIXHAWK_PORT}")
except Exception as e:
    print(f"[ERROR] Cannot connect to Pixhawk: {e}")
    traceback.print_exc()
    exit(1)

# ========== CONNECT TO GCS VIA UDP ==========
try:
    udp = mavutil.mavlink_connection(f"udpout:{GCS_IP}:{GCS_PORT}")
    print(f"[OK] UDP link established to GCS at {GCS_IP}:{GCS_PORT}")
except Exception as e:
    print(f"[ERROR] Cannot open UDP connection to GCS: {e}")
    traceback.print_exc()
    exit(1)

# ========== SEND INITIAL HEARTBEAT ==========
try:
    mav_sender = mavlink2.MAVLink(udp)
    mav_sender.srcSystem = 255
    mav_sender.srcComponent = 0
    hb = mav_sender.heartbeat_encode(
        type=mavlink2.MAV_TYPE_GCS,
        autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=mavlink2.MAV_STATE_ACTIVE
    )
    udp.mav.send(hb)
    print("[✓] GCS Heartbeat sent to Pixhawk")
except Exception as e:
    print(f"[ERROR] Failed to send heartbeat: {e}")
    traceback.print_exc()

# ========== REGISTER WITH GCS BACKEND ==========
while True:
    try:
        res = requests.post(f"http://{GCS_IP}:5000/register", json={
            "drone_id": DRONE_ID,
            "port": GCS_PORT
        }, timeout=5)
        if res.ok:
            print(f"[✓] Registered {DRONE_ID} with GCS Backend")
            break
        else:
            print(f"[WARN] GCS returned HTTP {res.status_code}")
    except Exception as e:
        print(f"[WARN] Registration failed: {e}")
        traceback.print_exc()
    time.sleep(3)

# ========== START FORWARDING ==========
print("[LOOP] Forwarding MAVLink messages from Pixhawk → GCS")

while True:
    try:
        pkt = mav.recv(1024)
        if pkt:
            udp.write(pkt)
            print(f"[→] Forwarded {len(pkt)} bytes to GCS")
    except Exception as e:
        print(f"[ERROR] Forwarding loop: {e}")
        traceback.print_exc()
        time.sleep(1)
