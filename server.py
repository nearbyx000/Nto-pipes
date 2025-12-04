#!/usr/bin/env python3

from flask import Flask, render_template_string, request, jsonify
import rospy
import threading
import json
from std_msgs.msg import String

app = Flask(__name__)

ros_data = {"drone": [0, 0], "taps": []}

def ros_thread():
    rospy.init_node('web_interface', anonymous=True)
    rospy.Subscriber('/tubes', String, data_callback)
    rospy.spin()

def data_callback(msg):
    global ros_data
    try:
        ros_data = json.loads(msg.data)
    except:
        pass

pub = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Pipeline Monitor</title>
    <style>
        body { font-family: sans-serif; padding: 20px; background: #f0f0f0; }
        .controls { margin-bottom: 20px; }
        button { padding: 10px 20px; font-size: 16px; cursor: pointer; margin-right: 10px; }
        .btn-start { background: #4CAF50; color: white; border: none; }
        .btn-stop { background: #f44336; color: white; border: none; }
        .btn-kill { background: #000; color: white; border: none; font-weight: bold; }
        #map { border: 2px solid #333; background: white; }
        .panel { display: flex; gap: 20px; }
        #info { width: 300px; background: white; padding: 10px; border: 1px solid #ccc; }
    </style>
</head>
<body>
    <h1>Управление миссией ТЭК</h1>
    <div class="controls">
        <button onclick="sendCommand('start')" class="btn-start" id="startBtn">СТАРТ</button>
        <button onclick="sendCommand('stop')" class="btn-stop">СТОП / ПОСАДКА</button>
        <button onclick="sendCommand('kill')" class="btn-kill">KILL SWITCH</button>
    </div>
    
    <div class="panel">
        <canvas id="map" width="800" height="600"></canvas>
        <div id="info">
            <h3>Статус</h3>
            <p>Дрон X: <span id="dx">0</span></p>
            <p>Дрон Y: <span id="dy">0</span></p>
            <h3>Найденные врезки:</h3>
            <ul id="tapsList"></ul>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('map');
        const ctx = canvas.getContext('2d');
        const scale = 50; 
        
        function sendCommand(cmd) {
            fetch('/cmd/' + cmd).then(r => {
                if(cmd === 'start') document.getElementById('startBtn').disabled = true;
                if(cmd === 'stop' || cmd === 'kill') document.getElementById('startBtn').disabled = false;
            });
        }

        function drawMap(data) {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Grid
            ctx.strokeStyle = '#eee';
            ctx.beginPath();
            for(let i=0; i<canvas.width; i+=scale) { ctx.moveTo(i,0); ctx.lineTo(i, canvas.height); }
            for(let i=0; i<canvas.height; i+=scale) { ctx.moveTo(0,i); ctx.lineTo(canvas.width, i); }
            ctx.stroke();

            // Origin
            ctx.fillStyle = 'black';
            ctx.beginPath(); ctx.arc(50, 550, 5, 0, Math.PI*2); ctx.fill();
            ctx.fillText("0,0", 55, 545);

            // Drone
            let dx = 50 + data.drone[0] * scale;
            let dy = 550 - data.drone[1] * scale;
            
            ctx.fillStyle = 'blue';
            ctx.beginPath(); ctx.arc(dx, dy, 8, 0, Math.PI*2); ctx.fill();
            ctx.fillText("Drone", dx+10, dy);

            // Taps
            const list = document.getElementById('tapsList');
            list.innerHTML = '';
            
            data.taps.forEach((tap, idx) => {
                let tx = 50 + tap[0] * scale;
                let ty = 550 - tap[1] * scale;
                
                ctx.fillStyle = 'red';
                ctx.beginPath(); ctx.arc(tx, ty, 6, 0, Math.PI*2); ctx.fill();
                
                let li = document.createElement('li');
                li.innerText = `Врезка ${idx+1}: [${tap[0]}, ${tap[1]}]`;
                list.appendChild(li);
            });

            document.getElementById('dx').innerText = data.drone[0];
            document.getElementById('dy').innerText = data.drone[1];
        }

        setInterval(() => {
            fetch('/data').then(r => r.json()).then(drawMap);
        }, 500);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/cmd/<cmd>')
def command(cmd):
    global pub
    if pub is None:
        pub = rospy.Publisher('/mission_cmd', String, queue_size=1)
    
    pub.publish(cmd)
    return jsonify(status="ok")

@app.route('/data')
def get_data():
    return jsonify(ros_data)

if __name__ == '__main__':
    threading.Thread(target=ros_thread).start()
    app.run(host='0.0.0.0', port=5000)