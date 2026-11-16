"""Exercise 2 - a publisher: the robot draws a circle by itself.

The script pauses the teleoperation (as you would on the real dVRK),
reads where the instrument is, then sends a new pose 100 times a second
on /PSM1/servo_cp so that the tip traces a horizontal circle.

    python examples/ex2_draw_circle.py [radius_mm] [seconds_per_lap]

ROS concepts: publisher, timer as a control loop, parameters of another node.
Try it: change the radius, the speed, draw a figure eight, a square...
"""
import math
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

import minros
from minros.msg import PoseStamped
from dvrk_sim.kinematics import msg_to_pose, pose_to_msg

RADIUS = float(sys.argv[1]) / 1000 if len(sys.argv) > 1 else 0.02
LAP_S = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
LAPS = 2

minros.init()
node = minros.Node("circle_drawer")
pub = node.create_publisher(PoseStamped, "/PSM1/servo_cp", 10)
state = {"pose": None, "t": 0.0}
node.create_subscription(PoseStamped, "/PSM1/measured_cp", lambda m: state.update(pose=msg_to_pose(m)), 1)

node.set_remote_parameter("MTMR_PSM1", "enabled", False)       # the surgeon lets go
if not minros.spin_until(node, lambda: state["pose"] is not None, timeout_sec=5):
    sys.exit("no /PSM1/measured_cp: is the simulator running?")
start, R = state["pose"]
centre = start - np.array([RADIUS, 0.0, 0.0])          # the circle passes through the current position
node.get_logger().info(f"drawing {LAPS} laps of a {RADIUS * 1000:.0f} mm circle")


def control_loop():
    state["t"] += 0.01
    a = 2 * math.pi * state["t"] / LAP_S
    p = centre + RADIUS * np.array([math.cos(a), math.sin(a), 0.0])
    pub.publish(pose_to_msg(p, R, "PSM1_base"))
    if state["t"] >= LAPS * LAP_S:
        minros.shutdown()


node.create_timer(0.01, control_loop)
try:
    minros.spin(node)
finally:
    minros.init()
    node.set_remote_parameter("MTMR_PSM1", "enabled", True)    # give control back
    node.spin_once(0.1)
    node.get_logger().info("done, teleoperation enabled again")
