"""B2 - Two subscriptions, two publishers: a safety monitor.

Instruments colliding inside the patient is a real risk. Write a node
called "safety_monitor" that
  - subscribes to /PSM1/measured_cp and /PSM2/measured_cp
  - publishes, 20 times a second (use a timer):
      /safety/tip_distance   Float64  distance between the two tips, in mm
      /safety/too_close      Bool     true when that distance is below 10 mm
  - prints a warning (node.get_logger().warn) when the tips get too close,
    at most once a second

Run it:    python exercises/b2_safety_monitor.py
Check it:  python exercises/check.py B2
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import minros
from minros.msg import PoseStamped, Float64, Bool

THRESHOLD_MM = 10.0

minros.init()
node = minros.Node("safety_monitor")
tips = {"PSM1": None, "PSM2": None}

# TODO 1: two publishers, /safety/tip_distance (Float64) and /safety/too_close (Bool)


def make_callback(arm):
    def on_pose(msg):
        # TODO 2: store the position of this arm's tip in tips[arm]
        pass
    return on_pose


# TODO 3: subscribe to both /PSM1/measured_cp and /PSM2/measured_cp,
#         using make_callback("PSM1") and make_callback("PSM2")


def check():
    # TODO 4: if both tips are known, compute their distance in mm and publish
    #         both messages. Use throttle_duration_sec=1 for the warning:
    #         node.get_logger().warn("...", throttle_duration_sec=1)
    pass


# TODO 5: a timer that calls check() 20 times a second

minros.spin(node)
