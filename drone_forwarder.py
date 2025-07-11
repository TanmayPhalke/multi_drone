
# drone/drone_forwarder.py
import socket
import time
import requests
from pymavlink import mavutil

GCS_IP = "192.168.1.100"  # Update with your GCS IP
DRONE_ID = "Drone1"
PORT = 14550

mav = mavutil.mavlink_connection('udpout:127.0.0.1:14550')  # Forward MAVLink

while True:
    try:
        requests.post(f"http://{GCS_IP}:5000/register", json={
            'drone_id': DRONE_ID,
            'port': PORT
        })
        print(f"Registered {DRONE_ID} with GCS")
        break
    except:
        print("Retrying GCS registration...")
        time.sleep(3)

print("Forwarding MAVLink packets...")
while True:
    msg = mav.recv_match(blocking=True)
    if msg:
        pass  # could add filters or debugging here
