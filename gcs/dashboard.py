from flask import Flask, jsonify, render_template
import threading, time, socket, json

app = Flask(__name__)
drones = {}

def udp_listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 6060))
    while True:
        try:
            data, _ = s.recvfrom(1024)
            drone = json.loads(data.decode())
            drones[drone["drone_id"]] = {
                "battery": drone["battery"],
                "gps_fix": drone["gps_fix"],
                "last_seen": time.time()
            }
        except: pass

@app.route("/")
def home():
    now = time.time()
    view = {}
    for d, info in drones.items():
        age = now - info.get("last_seen", 0)
        status = "Online" if age < 5 else "Offline"
        view[d] = {
            "battery": info.get("battery", "N/A"),
            "gps": info.get("gps_fix", False),
            "last_seen": f"{int(age)}s ago",
            "status": status
        }
    return render_template("dashboard2.html", drones=view)

if __name__ == "__main__":
    threading.Thread(target=udp_listener, daemon=True).start()
    app.run(host="0.0.0.0", port=8000, debug=False)
