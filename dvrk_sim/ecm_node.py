"""Simulated ECM, the arm that holds the endoscope (camera).

  publishes   /ECM/measured_cp   PoseStamped  camera pose in the world frame, 50 Hz
              /ECM/measured_js   JointState   yaw, pitch, insertion, roll
  subscribes  /ECM/servo_jp      JointState   go to these joints

The camera frame follows the dVRK convention: z is the viewing direction,
x points to the right of the image and y to the bottom. The teleoperation
nodes read this pose to map hand motion in the image: moving your hand to
the right moves the instrument to the right on screen, wherever the camera
points. That is what makes a real console feel natural.

The endoscope cannot be pulled back into the abdominal wall: a motion that
would push the camera tip into it is refused (see kinematics.py). In the
anatomy world (/scene/world) a motion is also refused when the view would
reach past the reconstructed surface (anatomy.Anatomy.view_ok).
"""
import time
import numpy as np

import minros
from minros.msg import JointState, PoseStamped, String, Header
from . import anatomy
from .kinematics import (ECM_JOINT_NAMES, ECM_HOME, ECM_LOWER, ECM_UPPER, ecm_forward, pose_to_msg,
                         wall_height, ENDOSCOPE_R)


def wall_gap(q):
    p, _ = ecm_forward(q)
    return wall_height(p[0], p[1]) - p[2] - ENDOSCOPE_R


class ECM(minros.Node):
    def __init__(self):
        super().__init__("ECM")
        self.q = ECM_HOME.copy()
        self.goal = self.q.copy()
        self.vmax = np.array([0.8, 0.8, 0.08, 1.5])
        self.pub_cp = self.create_publisher(PoseStamped, "/ECM/measured_cp", 10)
        self.pub_js = self.create_publisher(JointState, "/ECM/measured_js", 10)
        self.pub_state = self.create_publisher(String, "/ECM/operating_state", 10)
        self.create_subscription(JointState, "/ECM/servo_jp", self.on_servo_jp, 1)
        self.anat = None
        self.create_subscription(String, "/scene/world", self.on_world, 1)
        self.last = time.monotonic()
        self.create_timer(1 / 50, self.step)
        self.create_timer(0.5, lambda: self.pub_state.publish(String(data="ENABLED")))
        self.get_logger().info("ECM ready")

    def on_servo_jp(self, msg):
        if len(msg.position) != 4:
            self.get_logger().warn("ECM servo_jp needs 4 joint positions", throttle_duration_sec=1)
            return
        self.goal = np.clip(np.array(msg.position, dtype=float), ECM_LOWER, ECM_UPPER)

    def on_world(self, msg):
        if msg.data == "anatomy" and self.anat is None and anatomy.available():
            self.anat = anatomy.Anatomy()
        elif msg.data != "anatomy":
            self.anat = None

    def view_ok(self, q):
        return self.anat is None or self.anat.view_ok(*ecm_forward(q))

    def step(self):
        now = time.monotonic()
        dt = min(0.1, now - self.last)
        self.last = now
        dq = np.clip(self.goal - self.q, -self.vmax * dt, self.vmax * dt)
        before = wall_gap(self.q)
        seen = self.view_ok(self.q) if dq.any() else True
        # a pose that shows the borders is only accepted while already showing them (to come back)
        ok = lambda q: ((g := wall_gap(q)) >= 0 or g >= before - 1e-9) and (not seen or self.view_ok(q))
        if not dq.any():
            pass
        elif ok(self.q + dq):
            self.q = self.q + dq
        else:                                               # keep only the joints that stay clear
            for i in range(4):
                trial = self.q.copy()
                trial[i] += dq[i]
                if ok(trial):
                    self.q = trial
        p, Rc = ecm_forward(self.q)
        stamp = time.time()
        self.pub_cp.publish(pose_to_msg(p, Rc, "world", stamp))
        self.pub_js.publish(JointState(header=Header(stamp, "ECM_base"), name=ECM_JOINT_NAMES,
                                       position=[float(v) for v in self.q]))


def main():
    minros.init()
    node = ECM()
    minros.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
