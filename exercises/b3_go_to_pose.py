"""B3 - Commanding a robot: go to a point in a straight line.

    python exercises/b3_go_to_pose.py X Y Z        (millimetres, world frame)
    python exercises/b3_go_to_pose.py 10 0 -110

The script must
  1. pause the right-hand teleoperation:  node.set_remote_parameter("MTMR_PSM1", "enabled", False)
  2. read where the PSM1 tip is now (and its orientation) from /PSM1/measured_cp
  3. move the tip to (X, Y, Z) in a STRAIGHT LINE at SPEED_MM_S, by publishing a
     new pose on /PSM1/servo_cp every 10 ms, keeping the orientation unchanged
  4. give the teleoperation back (enabled True) and exit, even after Ctrl+C

Check it:  python exercises/check.py B3
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import minros
from minros.msg import PoseStamped
from dvrk_sim.kinematics import msg_to_pose, pose_to_msg     # PoseStamped <-> (position, rotation)

SPEED_MM_S = 20.0
DT = 0.01

minros.init()
args = minros.positional_args()
if len(args) != 3:
    sys.exit("usage: python exercises/b3_go_to_pose.py X Y Z   (millimetres)")
goal = np.array([float(a) for a in args]) / 1000.0           # metres

node = minros.Node("go_to_pose")
pub = node.create_publisher(PoseStamped, "/PSM1/servo_cp", 10)
current = {}
node.create_subscription(PoseStamped, "/PSM1/measured_cp",
                         lambda m: current.update(pose=msg_to_pose(m)), 1)

# TODO 1: pause the teleoperation of PSM1

# Wait (at most 5 s) for the first pose.
if not minros.spin_until(node, lambda: "pose" in current, timeout_sec=5):
    sys.exit("no /PSM1/measured_cp: is the simulator running?")
start, R = current["pose"]

try:
    # TODO 2: the straight line. Hints:
    #   distance = np.linalg.norm(goal - start)
    #   duration = distance / (SPEED_MM_S / 1000)
    #   for each step k: s = min(1, k * DT / duration); p = start + s * (goal - start)
    #   publish pose_to_msg(p, R, "world"), then wait DT with
    #   minros.spin_until(node, lambda: False, timeout_sec=DT)
    pass
finally:
    # TODO 3: give the teleoperation back, then let the message leave:
    node.spin_once(0.1)
