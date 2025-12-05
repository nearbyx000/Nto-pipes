from flask import Flask, render_template_string, jsonify
import rospy, threading, json
from std_msgs.msg import String

app = Flask(__name__)
d = {"drone": [0, 0], "taps": []}
pub = None

HTML = """<!DOCTYPE html><html><body style="font-family:sans-serif;padding:20px">
<h1>Control</h1><button onclick="f('start')">START</button><button onclick="f('stop')">STOP</button><button onclick="f('kill')">KILL</button>
<div style="display:flex;margin-top:20px"><canvas id="c" width="600" height="600" style="border:1px solid #000"></canvas>
<div style="margin-left:20px"><h3>Data</h3><pre id="info"></pre></div></div>
<script>
const c=document.getElementById('c').getContext('2d');
function f(x){fetch('/cmd/'+x)}
setInterval(()=>{fetch('/data').then(r=>r.json()).then(j=>{
    c.clearRect(0,0,600,600);
    c.fillStyle='blue'; c.fillRect(50+j.drone[0]*40, 550-j.drone[1]*40, 10, 10);
    c.fillStyle='red'; j.taps.forEach(t=>c.fillRect(50+t[0]*40, 550-t[1]*40, 8, 8));
    document.getElementById('info').innerText=JSON.stringify(j,null,2);
})},500)
</script></body></html>"""

@app.route('/')
def i(): return render_template_string(HTML)
@app.route('/cmd/<c>')
def c(c): pub.publish(c); return "ok"
@app.route('/data')
def dt(): return jsonify(d)

def ros():
    global pub
    rospy.init_node('web'); pub = rospy.Publisher('/mission_cmd', String, queue_size=1)
    rospy.Subscriber('/tubes', String, lambda m: d.update(json.loads(m.data)))
    rospy.spin()

if __name__ == '__main__':
    threading.Thread(target=ros).start()
    app.run(host='0.0.0.0', port=5000)