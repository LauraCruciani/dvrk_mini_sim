"""Exercise 3 - autonomy: the robot does the task on its own.

It starts a reach trial, reads the targets published by the scene and moves the
tool tip to each one along a straight line at constant speed.
Compare its time and path length with yours.

    python examples/ex3_autonomous_targets.py [speed_mm_s]

Only PSM1 moves (PSM2 stays under the left hand's control).
It ignores the vessels, so it may touch them. Your jobs: make it avoid
them (the vessel centre lines and radii are on /scene/vessels), then
split the targets between PSM1 and PSM2.
"""
import json
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np

import minros
from minros.msg import PoseStamped, PoseArray, String
from dvrk_sim.kinematics import msg_to_pose, pose_to_msg

SPEED = (float(sys.argv[1]) if len(sys.argv) > 1 else 30.0) / 1000      # m/s
DT = 0.01

minros.init()
node = minros.Node("autonomy")
pub = node.create_publisher(PoseStamped, "/PSM1/servo_cp", 10)
start_pub = node.create_publisher(String, "/scene/start", 10)
s = {"pose": None, "targets": None, "status": {}}
node.create_subscription(PoseStamped, "/PSM1/measured_cp", lambda m: s.update(pose=msg_to_pose(m)), 1)
node.create_subscription(PoseArray, "/scene/targets",
                         lambda m: s.update(targets=[np.array([p.position.x, p.position.y, p.position.z]) for p in m.poses]), 1)
node.create_subscription(String, "/scene/status", lambda m: s.update(status=json.loads(m.data)), 1)

node.set_remote_parameter("MTMR_PSM1", "enabled", False)
if not minros.spin_until(node, lambda: s["pose"] is not None, timeout_sec=5):
    sys.exit("no /PSM1/measured_cp: is the simulator running?")
start_pub.publish(String(data="reach"))
minros.spin_until(node, lambda: s["status"].get("state") == "running" and s["status"].get("task") == "reach", timeout_sec=3)
minros.spin_until(node, lambda: bool(s["targets"]), timeout_sec=3)

cmd, R = s["pose"]
targets = list(s["targets"])            # all the targets of this trial, in order
try:
    for i, goal in enumerate(targets):
        # straight line, constant speed
        last = time.monotonic()
        while np.linalg.norm(goal - cmd) > 1e-4:
            minros.spin_until(node, lambda: False, timeout_sec=DT)
            now = time.monotonic()                  # advance by the time really elapsed,
            dt, last = now - last, now              # not by DT: the loop is never exactly 10 ms
            step = goal - cmd
            dist = np.linalg.norm(step)
            cmd = cmd + step * min(1.0, SPEED * dt / dist)
            pub.publish(pose_to_msg(cmd, R, "PSM1_base"))
        # hold still until the scene accepts the target
        minros.spin_until(node, lambda: s["status"].get("index", 0) > i or s["status"].get("state") != "running",
                          timeout_sec=5)
    st = s["status"]
    node.get_logger().info(f"autonomous result: {st.get('time_s')} s, path {st.get('path_mm')} mm, "
                           f"{st.get('contacts')} contact(s)")
finally:
    node.set_remote_parameter("MTMR_PSM1", "enabled", True)
    node.spin_once(0.1)
