"""Exercise 4 - record a trial and analyse it (the idea behind a rosbag).

    python examples/ex4_record_and_plot.py

Start it, perform a trial in the console window (press N or B), and it stops by
itself when the trial ends (or press Ctrl+C). It saves a CSV file in
recordings/ and plots the trajectory and the speed of the tool tip.

Metrics such as path length, time and smoothness are used in research to
assess surgical skill objectively.
"""
import csv
import json
import os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np

import minros
from minros.msg import PoseStamped, String

minros.init()
node = minros.Node("recorder")
rows, s = [], {"status": {}, "seen_running": False}


def on_pose(msg):
    if s["seen_running"]:
        p = msg.pose.position
        rows.append((msg.header.stamp, p.x, p.y, p.z))


def on_status(msg):
    s["status"] = json.loads(msg.data)
    if s["status"].get("state") == "running":
        s["seen_running"] = True
    elif s["seen_running"] and s["status"].get("state") == "done":
        minros.shutdown()


node.create_subscription(PoseStamped, "/PSM1/measured_cp", on_pose, 100)
node.create_subscription(String, "/scene/status", on_status, 10)
node.get_logger().info("recording PSM1: start a trial in the console (press N or B)")
minros.spin(node)

if len(rows) < 10:
    sys.exit("not enough data recorded")
os.makedirs(os.path.join(ROOT, "recordings"), exist_ok=True)
path = os.path.join(ROOT, "recordings", time.strftime("trial_%Y%m%d_%H%M%S.csv"))
with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["stamp", "x", "y", "z"])
    w.writerows(rows)

data = np.array(rows)
t = data[:, 0] - data[0, 0]
p = data[:, 1:] * 1000
step = np.linalg.norm(np.diff(p, axis=0), axis=1)
# Speed over 100 ms windows (human movements are slower than 10 Hz).
# Dividing each 10 ms step by its dt gives a comb, not a curve: commands and
# robot run on separate clocks, so between two samples the pose may change by
# zero, one or two command steps (on a real robot the encoder noise would do
# the same). Any numerical derivative needs a filter.
WINDOW = 0.1                                                # s
s_path = np.concatenate([[0.0], np.cumsum(step)])           # path length along the trial
j = np.searchsorted(t, t + WINDOW)
ok = j < len(t)
i, j = np.nonzero(ok)[0], j[ok]
t_speed = (t[i] + t[j]) / 2
speed = (s_path[j] - s_path[i]) / (t[j] - t[i])
print(f"saved {len(rows)} samples to {path}")
print(f"duration {t[-1]:.1f} s, path {step.sum():.0f} mm, mean speed {step.sum() / t[-1]:.1f} mm/s")
st = s["status"]
print(f"scene: {st.get('contacts', '?')} contact(s)")

import matplotlib.pyplot as plt
fig = plt.figure(figsize=(11, 4.5))
ax = fig.add_subplot(1, 2, 1, projection="3d")
ax.plot(p[:, 0], p[:, 1], p[:, 2], lw=1)
ax.set_title("tool-tip trajectory (mm)")
ax2 = fig.add_subplot(1, 2, 2)
ax2.plot(t_speed, speed, lw=0.8)
ax2.set_xlabel("time (s)"); ax2.set_ylabel("speed (mm/s)")
ax2.set_title("speed profile: fewer peaks = smoother movement")
plt.tight_layout()
plt.show()
