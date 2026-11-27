#!/usr/bin/env python3
"""Live current plot for the FFW-SG2 arms, in a web browser.

Listens to /joint_states (read-only, like motor_profile.py) and serves a page that redraws about
once a second:   http://<robot-ip>:8765      (robot LAN IP, e.g. http://192.168.6.2:8765)

It never publishes, calls a service or talks to the servos. The picture is only rendered when a
browser asks for it, so it costs nothing while nobody is watching.

Run inside the ai_worker container while the bringup is up (Ctrl+C to stop):
    docker exec -it ai_worker bash -ic "python3 /root/ros2_ws/src/ai_worker/tools/motor_debug/live_current.py"
"""
import argparse
import collections
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

KEEP_S = 900.0      # history kept in memory
DECIMATE = 5        # 100 Hz -> 20 Hz (mean of 5 samples)
INK, INK2, GRID, SURFACE = "#191919", "#666666", "#e6e6e6", "#ffffff"
LEFT, RIGHT = "#257de0", "#f47230"
HOME = {"arm_l_joint1": -1.5994, "arm_l_joint2": 0.4105, "arm_l_joint3": -2.3, "arm_l_joint4": -1.0982,
        "arm_l_joint5": 2.0193, "arm_l_joint6": 0.7225, "arm_l_joint7": 0.1107,
        "arm_r_joint1": -1.5994, "arm_r_joint2": -0.4105, "arm_r_joint3": 2.3, "arm_r_joint4": -1.0982,
        "arm_r_joint5": -2.0193, "arm_r_joint6": 0.7225, "arm_r_joint7": -0.1107}

# /joint_states effort -> amperes. effort = raw Present Current x (factor in the driver's model file);
# raw unit = 0.01 A on the Y series (e-Manual), so amps = effort / factor * 0.01.
#   joint: (servo, amps per effort unit, continuous current rating in A or None, note shown on the plot)
_YM080 = ("YM080-230-R099", 0.01 / 0.07692307692, 10.7, "max 22.4 A")
_YM070 = ("YM070-210-R099", 0.01 / 0.04, None, "rating not looked up")
_YM070_ASSUMED = ("YM070 (model assumed, not read)", 0.01 / 0.04, None, "rating not looked up")
SCALE = {1: _YM080, 2: _YM080, 3: _YM080, 4: _YM070, 5: _YM070_ASSUMED, 6: _YM070_ASSUMED,
         7: ("PH42-020-S300", 0.001, None, "assumes 1 mA per raw unit (unverified)")}

lock = threading.Lock()
buf = collections.deque()          # (t, {joint: (pos, eff)})
acc = []
last_msg = [0.0]


class Listener(Node):
    def __init__(self):
        super().__init__("live_current_plot")
        self.create_subscription(JointState, "/joint_states", self.on_js, qos_profile_sensor_data)

    def on_js(self, msg):
        if len(msg.effort) != len(msg.name):
            return
        acc.append((list(msg.name), list(msg.position), list(msg.effort)))
        last_msg[0] = time.time()
        if len(acc) < DECIMATE:
            return
        names = acc[0][0]
        pos = np.mean([a[1] for a in acc], axis=0)
        eff = np.mean([a[2] for a in acc], axis=0)
        acc.clear()
        t = time.time()
        with lock:
            buf.append((t, {n: (p, e) for n, p, e in zip(names, pos, eff)}))
            while buf and t - buf[0][0] > KEEP_S:
                buf.popleft()


def render(joint, window):
    with lock:
        now = time.time()
        rows = [(t, d) for t, d in buf if now - t <= window]
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True, gridspec_kw={"height_ratios": [3, 1]},
                                  facecolor=SURFACE)
    stale = now - last_msg[0]
    title = f"Joint {joint} current, left vs right"
    servo, amps_per_unit, rating, note = SCALE[joint]
    decimals = 3 if joint == 7 else 1
    if rows:
        ts = np.array([t for t, _ in rows]) - now
        for name, side, color in (("Left", "l", LEFT), ("Right", "r", RIGHT)):
            j = f"arm_{side}_joint{joint}"
            if j not in rows[0][1]:
                continue
            cur = np.abs(np.array([d[j][1] for _, d in rows])) * amps_per_unit
            pos = np.array([d[j][0] for _, d in rows]) - HOME[j]
            ax.plot(ts, cur, color=color, lw=2, label=f"{name} (arm_{side}_joint{joint})", solid_capstyle="round")
            recent = cur[-100:].mean()   # last 5 s
            ax.annotate(f"{name}  {recent:.{decimals}f} A", (0, cur[-1]), xytext=(6, 0), textcoords="offset points",
                        va="center", fontsize=10, color=INK)
            ax2.plot(ts, pos, color=color, lw=2)
        ax.legend(loc="upper left", bbox_to_anchor=(0, 1.0), frameon=False, fontsize=9, ncol=2, labelcolor=INK)
    ax.set_title(title, loc="left", color=INK, fontsize=13, fontweight="bold", pad=24)
    status = "no /joint_states yet" if last_msg[0] == 0 else (
        f"LIVE  {time.strftime('%H:%M:%S', time.gmtime(now))} UTC" if stale < 1.0 else f"STALE: no data for {stale:.0f} s")
    ax.text(0, 1.035, f"{status}   |   labels at right = mean of last 5 s   |   {servo}: {note}",
            transform=ax.transAxes, color=INK2 if stale < 1.0 else "#dc3c3d", fontsize=9)
    ax.set_ylabel("motor current, size only (A)", color=INK2, fontsize=9)
    top = ax.get_ylim()[1]
    if rating:
        ax.axhline(rating, color=INK, lw=1.2, ls=(0, (5, 4)))
        ax.annotate(f"continuous rating {rating} A", (-window, rating), xytext=(4, 4), textcoords="offset points",
                    fontsize=9, color=INK)
        top = max(top, rating)
    ax.set_ylim(0, max(top * 1.15, 0.2 if joint == 7 else 2))
    ax.set_xlim(-window, 0)
    ax2.set_ylabel("angle from\nhome (rad)", color=INK2, fontsize=9)
    ax2.set_xlabel("seconds before now", color=INK2, fontsize=9)
    lo, hi = ax2.get_ylim()
    ax2.set_ylim(min(lo, -0.1), max(hi, 0.1))
    ax2.ticklabel_format(axis="y", style="plain", useOffset=False)
    for a in (ax, ax2):
        a.set_facecolor(SURFACE)
        a.grid(axis="y", color=GRID, lw=0.8)
        a.set_axisbelow(True)
        for s in ("top", "right", "left"):
            a.spines[s].set_visible(False)
        a.spines["bottom"].set_color(GRID)
        a.tick_params(colors=INK2, labelsize=8, length=0)
    fig.subplots_adjust(right=0.9, hspace=0.12, top=0.86)
    out = io.BytesIO()
    fig.savefig(out, format="png", dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return out.getvalue()


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Live arm current</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{font-family:system-ui,sans-serif;margin:16px;background:#f9f9f9;color:#191919}
img{max-width:100%;background:#fff;border:1px solid #e6e6e6;border-radius:6px}
label{margin-right:16px;font-size:14px}select{font-size:14px;padding:2px 4px}</style></head><body>
<div style="margin-bottom:10px">
<label>Joint <select id="j"><option value="1">1 shoulder</option><option value="2">2</option><option value="3">3</option>
<option value="4">4 elbow</option><option value="5">5</option><option value="6">6</option><option value="7">7 wrist</option></select></label>
<label>Window <select id="w"><option value="30">30 s</option><option value="120" selected>2 min</option>
<option value="600">10 min</option><option value="900">15 min</option></select></label>
<label><input type="checkbox" id="p"> pause</label></div>
<img id="plot" alt="live current plot">
<script>
const img=document.getElementById('plot');let busy=false;
function tick(){if(busy||document.getElementById('p').checked)return;busy=true;
 const n=new Image();n.onload=()=>{img.src=n.src;busy=false};n.onerror=()=>{busy=false};
 n.src='plot.png?j='+document.getElementById('j').value+'&w='+document.getElementById('w').value+'&t='+Date.now();}
setInterval(tick,1000);tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/plot.png":
            q = parse_qs(u.query)
            joint = min(max(int(q.get("j", ["1"])[0]), 1), 7)
            window = min(max(float(q.get("w", ["120"])[0]), 10.0), KEEP_S)
            body, ctype = render(joint, window), "image/png"
        elif u.path in ("/", "/index.html"):
            body, ctype = PAGE.encode(), "text/html; charset=utf-8"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    a = ap.parse_args()
    rclpy.init()
    node = Listener()
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    render_lock = threading.Lock()
    orig = Handler.do_GET

    def guarded(self):                       # matplotlib is not thread-safe: one render at a time
        with render_lock:
            orig(self)
    Handler.do_GET = guarded
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"live plot at http://<robot-ip>:{a.port}  (Ctrl+C to stop)", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
