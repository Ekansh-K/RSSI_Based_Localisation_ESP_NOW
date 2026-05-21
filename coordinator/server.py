#!/usr/bin/env python3
"""
server.py — Host Computer Coordinator + Web UI
Run:  pip install flask flask-socketio eventlet numpy scipy
      python server.py
Open: http://localhost:8080
"""
import socket, struct, time, json, math, os, statistics
import numpy as np
from scipy.optimize import least_squares
from flask import Flask
from flask_socketio import SocketIO
import eventlet
eventlet.monkey_patch()

# ================================================================
#  CONFIGURE THESE
# ================================================================
UDP_PORT  = 5005
WEB_PORT  = 8080

# Anchor positions in metres. (0,0) = Anchor 1.
# Measure with tape measure after mounting anchors.
ANCHOR_POSITIONS = {
    1: (0.0, 0.0),
    2: (5.0, 0.0),
    3: (5.0, 4.0),
    4: (0.0, 4.0),
}
ROOM_W = 5.0   # room width in metres
ROOM_H = 4.0   # room height in metres

MIN_ANCHORS    = 3
MAX_STALE_SEC  = 1.5
MAX_DIST_M     = 15.0
OUTLIER_THRESH = 0.40
DEFAULT_A      = -55.0
DEFAULT_N      =  2.7
# ================================================================

# Load calibration
PATH_LOSS = {aid: (DEFAULT_A, DEFAULT_N) for aid in ANCHOR_POSITIONS}
CAL_FILE  = os.path.join(os.path.dirname(__file__), 'calibration_results.json')
if os.path.exists(CAL_FILE):
    try:
        with open(CAL_FILE) as f:
            cal_data = json.load(f)
        if not isinstance(cal_data, list):
            cal_data = [cal_data]
        for e in cal_data:
            PATH_LOSS[e['anchor_id']] = (e['recommended']['A'], e['recommended']['n'])
            print(f"[CAL] Anchor {e['anchor_id']}: A={e['recommended']['A']}  n={e['recommended']['n']}")
    except Exception as ex:
        print(f"[CAL] Could not load calibration: {ex}")
else:
    print("[CAL] No calibration_results.json found — using defaults. Run calibration.py first!")

REPORT_FMT  = '<BfBIBHB'
REPORT_SIZE = struct.calcsize(REPORT_FMT)

state_lock = eventlet.semaphore.Semaphore()
rssi_store = {}


def rssi_to_dist(rssi, A, n):
    if n == 0: return MAX_DIST_M
    return min(10.0 ** ((A - rssi) / (10.0 * n)), MAX_DIST_M)


def trilaterate(anchors, distances):
    def residuals(p):
        return [math.sqrt((p[0]-ax)**2 + (p[1]-ay)**2) - d
                for (ax, ay), d in zip(anchors, distances)]
    x0 = [sum(a[0] for a in anchors)/len(anchors),
          sum(a[1] for a in anchors)/len(anchors)]
    try:
        res = least_squares(residuals, x0, method='lm', max_nfev=200)
        return float(res.x[0]), float(res.x[1])
    except Exception as e:
        print(f"[TRILAT] {e}")
        return None


class Kalman2D:
    def __init__(self, pn=0.05, mn=0.8, dt=0.2):
        self.F  = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]], dtype=float)
        self.H  = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        self.Q  = np.eye(4)*pn
        self.R  = np.eye(2)*mn
        self.x  = np.zeros(4)
        self.P  = np.eye(4)*10.0
        self.ok = False

    def update(self, xy):
        z = np.array(xy, dtype=float)
        if not self.ok:
            self.x[:2] = z; self.ok = True; return z.copy()
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P  = (np.eye(4) - K @ self.H) @ self.P
        return self.x[:2].copy()

kalman = Kalman2D()


def filter_outliers(anchor_ids, anch_positions, distances):
    """Remove outlier anchors whose distance deviates >40% from the median.
    Returns (anchor_ids, anch_positions, distances) as consistent lists."""
    if len(distances) < 4:
        return list(anchor_ids), list(anch_positions), list(distances)
    med = statistics.median(distances)
    filtered = [(aid, pos, d) for aid, pos, d in zip(anchor_ids, anch_positions, distances)
                if abs(d - med) / (med + 0.001) <= OUTLIER_THRESH]
    if len(filtered) >= MIN_ANCHORS:
        aids, positions, dists = zip(*filtered)
        return list(aids), list(positions), list(dists)
    return list(anchor_ids), list(anch_positions), list(distances)


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.settimeout(0.5)
    print(f"[UDP] Listening on :{UDP_PORT}")
    while True:
        try:
            data, _ = sock.recvfrom(256)
            if len(data) < REPORT_SIZE: continue
            aid, avg_rssi, ch, ts, tid, sc, cal = struct.unpack(REPORT_FMT, data[:REPORT_SIZE])
            with state_lock:
                rssi_store[aid] = {'rssi': avg_rssi, 'recv_ts': time.time(),
                                   'channel': ch, 'samples': sc}
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] {e}")


def localization_loop(sio):
    print("[LOC] Running at 5 Hz")
    while True:
        eventlet.sleep(0.2)
        now = time.time()
        with state_lock:
            snap = dict(rssi_store)

        aid_v, anch_v, dist_v, rssi_out = [], [], [], {}
        for aid, pos in ANCHOR_POSITIONS.items():
            if aid not in snap: continue
            e = snap[aid]
            if now - e['recv_ts'] > MAX_STALE_SEC: continue
            A, n = PATH_LOSS.get(aid, (DEFAULT_A, DEFAULT_N))
            d    = rssi_to_dist(e['rssi'], A, n)
            if d >= MAX_DIST_M: continue
            aid_v.append(aid); anch_v.append(pos); dist_v.append(d)
            rssi_out[str(aid)] = round(float(e['rssi']), 1)

        if len(anch_v) < MIN_ANCHORS: continue
        aid_v, anch_v, dist_v = filter_outliers(aid_v, anch_v, dist_v)
        if len(anch_v) < MIN_ANCHORS: continue

        raw = trilaterate(anch_v, dist_v)
        if raw is None: continue
        sm  = kalman.update(raw)

        sio.emit('position_update', {
            'x':         round(float(sm[0]), 2),
            'y':         round(float(sm[1]), 2),
            'raw_x':     round(float(raw[0]), 2),
            'raw_y':     round(float(raw[1]), 2),
            'distances': {str(aid): round(d, 2) for aid, d in zip(aid_v, dist_v)},
            'rssi':      rssi_out,
            'n_anchors': len(anch_v),
            'timestamp': now,
        })


# ---- Web UI ----
HTML = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP-NOW Localization</title>
<script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:'Segoe UI',system-ui,sans-serif;background:#0d1117;color:#e6edf3;
      display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:20px;gap:14px}}
h1{{font-size:1.05rem;letter-spacing:.06em;color:#58a6ff;margin-top:4px}}
#mapwrap{{background:#161b22;border:1px solid #30363d;border-radius:10px;overflow:hidden}}
canvas{{display:block}}
#panels{{display:flex;gap:10px;flex-wrap:wrap;justify-content:center}}
.card{{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px 18px;
        min-width:130px;text-align:center}}
.card .val{{font-size:1.35rem;font-weight:700;color:#58a6ff;margin-bottom:3px}}
.card .lbl{{font-size:.72rem;color:#8b949e}}
#log{{font-size:.7rem;color:#6e7681}}
.led{{display:inline-block;width:8px;height:8px;border-radius:50%;
       background:#3fb950;margin-right:6px;box-shadow:0 0 5px #3fb950}}
</style>
</head>
<body>
<h1>&#x1F4CD; ESP-NOW RSSI Indoor Localization</h1>
<div id="mapwrap"><canvas id="map" width="560" height="420"></canvas></div>
<div id="panels">
  <div class="card"><div class="val" id="pos">—</div><div class="lbl">Position (x, y) m</div></div>
  <div class="card"><div class="val" id="nanch">—</div><div class="lbl">Active Anchors</div></div>
  <div class="card"><div class="val" id="r1">—</div><div class="lbl">Anchor 1 RSSI</div></div>
  <div class="card"><div class="val" id="r2">—</div><div class="lbl">Anchor 2 RSSI</div></div>
  <div class="card"><div class="val" id="r3">—</div><div class="lbl">Anchor 3 RSSI</div></div>
  <div class="card"><div class="val" id="r4">—</div><div class="lbl">Anchor 4 RSSI</div></div>
</div>
<div id="log"><span class="led"></span>Waiting...</div>
<script>
const ROOM_W={ROOM_W}, ROOM_H={ROOM_H};
const ANCHORS=[{{x:0,y:0}},{{x:{ANCHOR_POSITIONS[2][0]},y:{ANCHOR_POSITIONS[2][1]}}},
               {{x:{ANCHOR_POSITIONS[3][0]},y:{ANCHOR_POSITIONS[3][1]}}},
               {{x:{ANCHOR_POSITIONS[4][0]},y:{ANCHOR_POSITIONS[4][1]}}}];
const W=560,H=420,PAD=50;
const sx=x=>PAD+(x/ROOM_W)*(W-2*PAD);
const sy=y=>H-PAD-(y/ROOM_H)*(H-2*PAD);
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d');
let trail=[];
function drawBase(){{
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle='#21262d';ctx.lineWidth=1;
  for(let x=0;x<=ROOM_W;x++){{ctx.beginPath();ctx.moveTo(sx(x),PAD);ctx.lineTo(sx(x),H-PAD);ctx.stroke()}}
  for(let y=0;y<=ROOM_H;y++){{ctx.beginPath();ctx.moveTo(PAD,sy(y));ctx.lineTo(W-PAD,sy(y));ctx.stroke()}}
  ctx.strokeStyle='#30363d';ctx.lineWidth=2;ctx.strokeRect(PAD,PAD,W-2*PAD,H-2*PAD);
  ctx.fillStyle='#6e7681';ctx.font='11px system-ui';
  ctx.textAlign='center';for(let x=0;x<=ROOM_W;x++)ctx.fillText(x+'m',sx(x),H-PAD+16);
  ctx.textAlign='right';for(let y=0;y<=ROOM_H;y++)ctx.fillText(y+'m',PAD-6,sy(y)+4);
  ANCHORS.forEach((a,i)=>{{
    ctx.fillStyle='#f85149';ctx.beginPath();ctx.arc(sx(a.x),sy(a.y),9,0,2*Math.PI);ctx.fill();
    ctx.fillStyle='#fff';ctx.font='bold 10px system-ui';ctx.textAlign='center';
    ctx.fillText('A'+(i+1),sx(a.x),sy(a.y)+4);
  }});
}}
const socket=io();
socket.on('connect',()=>{{document.getElementById('log').innerHTML='<span class="led"></span>Connected'}});
socket.on('position_update',d=>{{
  document.getElementById('pos').textContent='('+d.x+', '+d.y+')';
  document.getElementById('nanch').textContent=d.n_anchors+' / 4';
  [1,2,3,4].forEach(i=>{{
    const el=document.getElementById('r'+i);
    el.textContent=d.rssi[i]!==undefined?d.rssi[i]+' dBm':'—';
  }});
  document.getElementById('log').innerHTML='<span class="led"></span>'+
    new Date(d.timestamp*1000).toLocaleTimeString()+'  raw=('+d.raw_x+','+d.raw_y+')';
  trail.push({{x:d.x,y:d.y}});if(trail.length>80)trail.shift();
  drawBase();
  if(d.distances)ANCHORS.forEach((a,i)=>{{
    const dist=d.distances[String(i+1)];if(!dist)return;
    ctx.strokeStyle='rgba(88,166,255,0.13)';ctx.lineWidth=1.5;ctx.beginPath();
    ctx.ellipse(sx(a.x),sy(a.y),dist/ROOM_W*(W-2*PAD),dist/ROOM_H*(H-2*PAD),0,0,2*Math.PI);
    ctx.stroke();
  }});
  if(trail.length>1){{
    ctx.strokeStyle='rgba(63,185,80,0.35)';ctx.lineWidth=2;ctx.beginPath();
    trail.forEach((p,i)=>i===0?ctx.moveTo(sx(p.x),sy(p.y)):ctx.lineTo(sx(p.x),sy(p.y)));ctx.stroke();
  }}
  ctx.fillStyle='rgba(210,153,34,0.7)';ctx.beginPath();ctx.arc(sx(d.raw_x),sy(d.raw_y),5,0,2*Math.PI);ctx.fill();
  ctx.fillStyle='#58a6ff';ctx.beginPath();ctx.arc(sx(d.x),sy(d.y),11,0,2*Math.PI);ctx.fill();
  ctx.fillStyle='#fff';ctx.font='bold 9px system-ui';ctx.textAlign='center';
  ctx.fillText('TAG',sx(d.x),sy(d.y)+3);
}});
drawBase();
</script>
</body></html>"""

app = Flask(__name__)
sio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

@app.route('/')
def index(): return HTML

@sio.on('connect')
def on_connect(): print("[WEB] Client connected")

if __name__ == '__main__':
    eventlet.spawn(udp_listener)
    eventlet.spawn(localization_loop, sio)
    print(f"[Server] http://localhost:{WEB_PORT}")
    sio.run(app, host='0.0.0.0', port=WEB_PORT)