from flask import Flask, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
from dronekit import connect, VehicleMode
import subprocess
import threading
import time

app = Flask(__name__, static_folder="../frontend")
socketio = SocketIO(app, cors_allowed_origins="*")

drones = {}  # drone_id: {vehicle, ip, port}

@app.route('/')
def serve_dashboard():
    return send_from_directory(app.static_folder, 'index.html')

@app.route('/register', methods=['POST'])
def register():
    data = request.json
    ip = request.remote_addr
    drone_id = data.get('drone_id')
    port = data.get('port')
    if drone_id and port:
        try:
            vehicle = connect(f'udp:{ip}:{port}', wait_ready=True, timeout=60)
            drones[drone_id] = {'vehicle': vehicle, 'ip': ip, 'port': port}
            print(f"[CONNECTED] {drone_id} via udp:{ip}:{port}")
        except Exception as e:
            print(f"[ERROR] Could not connect to {drone_id}: {e}")
    return jsonify({'status': 'ok'})

@socketio.on('get_drones')
def handle_get_drones():
    output = {}
    for drone_id, info in drones.items():
        v = info['vehicle']
        output[drone_id] = {
            'ip': info['ip'],
            'port': info['port'],
            'altitude': getattr(v.location.global_relative_frame, 'alt', 0),
            'battery': getattr(v.battery, 'level', 0),
            'lat': getattr(v.location.global_frame, 'lat', 0),
            'lon': getattr(v.location.global_frame, 'lon', 0),
            'armed': v.armed
        }
    emit('drones', output)

@socketio.on('launch_mp')
def handle_launch_mp(data):
    drone_id = data.get('drone_id')
    if drone_id not in drones:
        emit('error', {'message': f"Drone {drone_id} not found"})
        return
    ip = drones[drone_id]['ip']
    port = drones[drone_id]['port']
    local_port = int(port) + 100
    threading.Thread(target=launch_mission_planner, args=(ip, port, local_port)).start()
    emit('mp_launched', {'drone_id': drone_id})

@socketio.on('toggle_arm')
def handle_toggle_arm(data):
    drone_id = data.get('drone_id')
    if drone_id in drones:
        v = drones[drone_id]['vehicle']
        v.armed = not v.armed
        emit('status_update', {'drone_id': drone_id, 'armed': v.armed}, broadcast=True)

@socketio.on('start_telemetry')
def start_telemetry():
    def loop():
        while True:
            for drone_id, info in drones.items():
                v = info['vehicle']
                data = {
                    'drone_id': drone_id,
                    'altitude': getattr(v.location.global_relative_frame, 'alt', 0),
                    'battery': getattr(v.battery, 'level', 0),
                    'lat': getattr(v.location.global_frame, 'lat', 0),
                    'lon': getattr(v.location.global_frame, 'lon', 0),
                }
                socketio.emit('telemetry', data)
            time.sleep(2)
    threading.Thread(target=loop, daemon=True).start()

def launch_mission_planner(ip, port, local_port):
    mp_path = "C:\\Program Files (x86)\\Mission Planner\\MissionPlanner.exe"
    try:
        subprocess.Popen([mp_path, f"--connect=udp:{ip}:{port}", f"--localport={local_port}"])
        print(f"[MP] Launched for {ip}:{port} on local {local_port}")
    except FileNotFoundError:
        print("[ERROR] Mission Planner not found")

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
