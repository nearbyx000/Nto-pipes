#!/usr/bin/env python
import rospy
import json
import threading
from flask import Flask, render_template_string, request, jsonify
from std_msgs.msg import String

# --- Flask App ---
app = Flask(__name__)

# --- Global State (Thread-Safe) ---
state_lock = threading.Lock()
mission_state = {
    'drone': [0.0, 0.0],
    'taps': [] # List of [x, y]
}
cmd_publisher = None

# --- HTML Template ---
HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Pipeline Inspector</title>
    <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;700&display=swap" rel="stylesheet">
    <style>
        body { font-family: 'Roboto', sans-serif; background: #222; color: #eee; text-align: center; }
        .container { display: flex; flex-direction: column; align-items: center; margin-top: 20px; }
        .panel { background: #333; padding: 20px; border-radius: 10px; margin-bottom: 20px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); }
        
        button {
            padding: 15px 30px; margin: 10px; font-size: 18px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; color: white;
        }
        .btn-start { background: #27ae60; } .btn-start:hover { background: #2ecc71; }
        .btn-stop { background: #f39c12; } .btn-stop:hover { background: #f1c40f; }
        .btn-kill { background: #c0392b; } .btn-kill:hover { background: #e74c3c; }

        #map {
            position: relative; width: 600px; height: 600px; background: #444; border: 2px solid #666; overflow: hidden;
            background-image: linear-gradient(#555 1px, transparent 1px), linear-gradient(90deg, #555 1px, transparent 1px);
            background-size: 50px 50px;
        }
        
        .drone { position: absolute; width: 20px; height: 20px; background: #3498db; border-radius: 50%; transform: translate(-50%, -50%); border: 2px solid white; box-shadow: 0 0 10px #3498db; transition: all 0.2s linear; }
        .tap { position: absolute; width: 16px; height: 16px; background: #e74c3c; transform: translate(-50%, -50%) rotate(45deg); border: 2px solid white; }
        
        .info { text-align: left; width: 600px; }
        .log-item { border-bottom: 1px solid #555; padding: 5px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Drone Pipeline Inspector</h1>
        
        <div class="panel">
            <button class="btn-start" onclick="send('start')">START MISSION</button>
            <button class="btn-stop" onclick="send('stop')">RETURN TO BASE</button>
            <button class="btn-kill" onclick="send('kill')">EMERGENCY KILL</button>
        </div>

        <div id="map">
            <div id="drone" class="drone" style="left: 50px; bottom: 50px;"></div>
        </div>

        <div class="panel info">
            <h3>Detected Anomalies</h3>
            <div id="log"></div>
        </div>
    </div>

    <script>
        const SCALE = 40; // px per meter
        const OFFSET_X = 50;
        const OFFSET_Y = 50;

        function send(cmd) {
            fetch('/cmd?action=' + cmd).then(r => console.log('Sent:', cmd));
        }

        function update() {
            fetch('/data').then(r => r.json()).then(state => {
                // Update Drone
                const d = document.getElementById('drone');
                d.style.left = (OFFSET_X + state.drone[0] * SCALE) + 'px';
                d.style.bottom = (OFFSET_Y + state.drone[1] * SCALE) + 'px';

                // Update Taps
                const map = document.getElementById('map');
                const log = document.getElementById('log');
                
                // Clear existing taps (inefficient but simple)
                document.querySelectorAll('.tap').forEach(e => e.remove());
                log.innerHTML = '';

                state.taps.forEach((t, i) => {
                    // Map Marker
                    const el = document.createElement('div');
                    el.className = 'tap';
                    el.style.left = (OFFSET_X + t[0] * SCALE) + 'px';
                    el.style.bottom = (OFFSET_Y + t[1] * SCALE) + 'px';
                    map.appendChild(el);

                    // Log Entry
                    const row = document.createElement('div');
                    row.className = 'log-item';
                    row.innerText = `Tap #${i+1}: Location [${t[0]}, ${t[1]}]`;
                    log.appendChild(row);
                });
            });
        }
        setInterval(update, 200);
    </script>
</body>
</html>
"""

# --- ROS Callbacks ---
def tubes_callback(msg):
    global mission_state
    try:
        data = json.loads(msg.data)
        with state_lock:
            mission_state = data
    except Exception:
        pass

# --- Flask Routes ---
@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/cmd')
def cmd():
    action = request.args.get('action')
    if action and cmd_publisher:
        cmd_publisher.publish(action)
    return "OK"

@app.route('/data')
def data():
    with state_lock:
        return jsonify(mission_state)

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

if __name__ == '__main__':
    rospy.init_node('web_interface')
    
    cmd_publisher = rospy.Publisher('/mission_cmd', String, queue_size=1)
    rospy.Subscriber('/tubes', String, tubes_callback)
    
    # Run Flask in thread
    t = threading.Thread(target=run_flask)
    t.daemon = True
    t.start()
    
    rospy.spin()