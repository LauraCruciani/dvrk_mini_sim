"""B5 - Challenge: a camera that follows the instruments.

Moving the camera interrupts the surgeon. Autonomous camera control is an
active research topic on the dVRK. Write a node called "auto_camera" that
keeps the endoscope pointed at the midpoint between the two tool tips.

  - subscribe to /PSM1/measured_cp, /PSM2/measured_cp and /ECM/measured_js
  - 10 times a second, compute the direction from the ECM's remote center
    of motion (ECM_BASE) to the midpoint, turn it into yaw and pitch, and
    publish /ECM/servo_jp with [yaw, pitch, insertion, roll], keeping the
    current insertion and roll
  - bonus: do nothing while the midpoint stays within 5 mm of the last aim
    point (a dead band, so the image does not shake)

The ECM shaft direction for given yaw and pitch is (see kinematics.py):
    u = [-sin(yaw) cos(pitch),  sin(pitch),  -cos(yaw) cos(pitch)]
so, from a unit vector u:  pitch = asin(u_y),  yaw = atan2(-u_x, -u_z)

Run it:    python exercises/b5_auto_camera.py
Check it:  python exercises/check.py B5
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import numpy as np
import minros
from minros.msg import PoseStamped, JointState
from dvrk_sim.kinematics import ECM_BASE, ECM_JOINT_NAMES

minros.init()
node = minros.Node("auto_camera")
state = {"PSM1": None, "PSM2": None, "ecm_q": None}

# TODO: everything! Use the previous exercises as a model.

minros.spin(node)
