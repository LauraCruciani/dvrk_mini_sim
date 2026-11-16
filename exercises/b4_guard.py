"""B4 - A node in the middle: a virtual fixture.

Research on the dVRK often inserts a node between the surgeon and the robot.
Here the teleoperation is remapped to publish on /PSM1/servo_cp_raw; your
node receives those commands, makes them safe and forwards them to the arm:

    MTMR_PSM1 --/PSM1/servo_cp_raw--> guard --/PSM1/servo_cp--> PSM1

Write a node called "guard" that
  - subscribes to /PSM1/servo_cp_raw (PoseStamped)
  - copies each pose, but never lets the tip go below FLOOR_MM (5 mm above
    the tissue) nor outside the box |x| <= 60 mm, |y| <= 50 mm
  - publishes the result on /PSM1/servo_cp, keeping header and orientation

Try it with a human:  python launch.py --guard     then   python exercises/b4_guard.py
Check it:             python exercises/check.py B4
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import minros
from minros.msg import PoseStamped

FLOOR_MM = -125.0
BOX_X_MM = 60.0
BOX_Y_MM = 50.0

minros.init()
node = minros.Node("guard")

# TODO 1: a publisher on /PSM1/servo_cp


def on_command(msg):
    p = msg.pose.position                      # metres!
    # TODO 2: clamp p.x, p.y and p.z (you can modify msg directly), then publish msg
    pass


# TODO 3: subscribe on_command to /PSM1/servo_cp_raw

minros.spin(node)
