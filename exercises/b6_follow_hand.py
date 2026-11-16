"""B6 - A camera that follows the hand you are using. A guided exercise.

Follow exercises/B6_GUIDE.md step by step. After each step:
  - run this node (simulator open):   python exercises/b6_follow_hand.py
    and read what it prints once a second: it tells you which steps work;
  - check that step on its own:        python exercises/check.py B6

The functions of steps 3, 4 and 5 are pure computations: the checker tests
them without the simulator, so you can work on them anywhere.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import math
import numpy as np
import minros
from minros.msg import PoseStamped, JointState, String
from dvrk_sim.kinematics import ECM_BASE, ECM_JOINT_NAMES

RATE_HZ = 20          # how often the node updates the camera
DEAD_BAND = 0.05      # rad (about 3 degrees): smaller errors are ignored
MAX_SPEED = 0.4       # rad/s: the camera never turns faster than this


# ============================================================ STEP 3: aiming
def aim_angles(target, rcm):
    """Yaw and pitch (radians) that point the endoscope from `rcm` at `target`.

    target, rcm: numpy arrays (x, y, z) in metres, world frame.
    The ECM's viewing direction for given yaw and pitch is
        u = [-sin(yaw) cos(pitch),  sin(pitch),  -cos(yaw) cos(pitch)]
    """
    # TODO 3: compute the unit vector u from rcm to target,
    #         then pitch = asin(u_y) and yaw = atan2(-u_x, -u_z)
    return None


# ========================================================= STEP 4: dead band
def update_goal(goal, wanted, dead_band):
    """Decide where the camera should point.

    goal:   (yaw, pitch) where the camera is going now
    wanted: (yaw, pitch) that would centre the instrument exactly
    Return `wanted` if it is more than `dead_band` away from `goal`
    (in yaw OR in pitch), otherwise return `goal` unchanged.
    """
    # TODO 4
    return goal


# ===================================================== STEP 5: smooth motion
def step_towards(current, goal, max_speed, dt):
    """One small step of the camera command from `current` towards `goal`.

    current, goal: (yaw, pitch). Each angle may change by at most
    max_speed * dt in one step. Return the new (yaw, pitch).
    """
    # TODO 5: for each of the two angles, limit the change to +- max_speed * dt
    return current


# ================================================== STEPS 1, 2, 6: the node
class FollowCamera(minros.Node):
    def __init__(self):
        super().__init__("follow_camera")
        self.tips = {"PSM1": None, "PSM2": None}
        self.active = None            # "PSM1" or "PSM2"
        self.ecm_q = None             # [yaw, pitch, insertion, roll] measured
        self.goal = None              # (yaw, pitch) chosen by update_goal
        self.command = None           # (yaw, pitch) sent to the ECM

        # STEP 1: listen.
        # TODO 1a: subscribe to /PSM1/measured_cp and /PSM2/measured_cp; store the
        #          tip positions as numpy arrays in self.tips["PSM1"] / ["PSM2"]
        #          (use a function that makes a callback for each arm, as in B2)
        # TODO 1b: subscribe to /ECM/measured_js and store list(msg.position) in self.ecm_q

        # STEP 2: which instrument is the surgeon using?
        # TODO 2: subscribe to /console/active_arm (String) and store msg.data in self.active

        # STEP 6: talk to the camera arm.
        # TODO 6a: a publisher of JointState on /ECM/servo_jp, in self.pub
        self.pub = None

        self.create_timer(1 / RATE_HZ, self.tick)
        self.create_timer(1.0, self.report)

    def tick(self):
        if self.ecm_q is None or self.active is None or self.tips.get(self.active) is None:
            return
        yaw, pitch, insertion, roll = self.ecm_q
        if self.command is None:                       # start from where the camera is
            self.command = (yaw, pitch)
            self.goal = (yaw, pitch)
        wanted = aim_angles(self.tips[self.active], ECM_BASE)
        if wanted is None:
            return
        self.goal = update_goal(self.goal, wanted, DEAD_BAND)
        self.command = step_towards(self.command, self.goal, MAX_SPEED, 1 / RATE_HZ)
        # TODO 6b: publish JointState(name=ECM_JOINT_NAMES,
        #          position=[command yaw, command pitch, insertion, roll])
        #          (keep the measured insertion and roll: we only turn the camera)

    def report(self):
        """Once a second: what works so far."""
        log = self.get_logger()
        ok1 = self.tips["PSM1"] is not None and self.tips["PSM2"] is not None and self.ecm_q is not None
        log.info(f"STEP 1 {'ok ' if ok1 else 'not yet'}: tips {'received' if ok1 else 'missing'}"
                 + (f", camera yaw {self.ecm_q[0]:+.2f} pitch {self.ecm_q[1]:+.2f} rad" if self.ecm_q else ""))
        log.info(f"STEP 2 {'ok ' if self.active else 'not yet'}: following {self.active or '?'} (Tab at the console switches)")
        if self.active and ok1:
            w = aim_angles(self.tips[self.active], ECM_BASE)
            log.info("STEP 3 " + (f"ok : to centre {self.active} the camera needs yaw {w[0]:+.2f} pitch {w[1]:+.2f} rad"
                                  if w is not None else "not yet: aim_angles returns None"))
        if self.goal is not None:
            log.info(f"STEPS 4-5: goal {self.goal[0]:+.2f} {self.goal[1]:+.2f}, command {self.command[0]:+.2f} {self.command[1]:+.2f}")
        log.info(f"STEP 6 {'ok ' if self.pub else 'not yet'}: " + ("commands sent to /ECM/servo_jp" if self.pub else "no publisher"))
        log.info("-" * 60)


def main():
    minros.init()
    node = FollowCamera()
    minros.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
