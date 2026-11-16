"""B1 - Your first node: the speed of the tool tip.

Write a node called "tip_speed" that
  - subscribes to /PSM1/measured_cp (PoseStamped)
  - computes the speed of the tip from two consecutive messages
  - publishes it on /PSM1/tip_speed (Float64) in mm/s

Run it:    python exercises/b1_tip_speed.py
Check it:  python exercises/check.py B1      (the simulator must be running)
Watch it:  python -m minros topic echo /PSM1/tip_speed
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import minros
from minros.msg import PoseStamped, Float64

minros.init()
node = minros.Node("tip_speed")

# TODO 1: create a publisher of Float64 messages on "/PSM1/tip_speed"
pub = None

previous = {"p": None, "t": None}      # the last position (x, y, z) and its time stamp


def on_pose(msg):
    p = msg.pose.position
    t = msg.header.stamp               # seconds: use the time written in the message
    # TODO 2: if there is a previous message, compute
    #           distance = sqrt(dx^2 + dy^2 + dz^2)   (metres)
    #           speed    = distance / dt              (m/s), then convert to mm/s
    #         and publish it with pub.publish(Float64(data=...)).
    #         Watch out: dt can be 0 if two messages carry the same stamp.
    # TODO 3: remember this message for next time (previous["p"], previous["t"])
    pass


# TODO 4: subscribe on_pose to "/PSM1/measured_cp"

minros.spin(node)
