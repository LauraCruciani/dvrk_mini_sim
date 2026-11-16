"""Simulated ECM, the arm that holds the endoscope (camera).

  publishes   /ECM/measured_cp   PoseStamped  camera pose in the world frame, 50 Hz
              /ECM/measured_js   JointState   yaw, pitch, insertion, roll
  subscribes  /ECM/servo_jp      JointState   go to these joints

The camera frame follows the dVRK convention: z is the viewing direction,
x points to the right of the image and y to the bottom. The teleoperation
nodes read this pose to map hand motion in the image: moving your hand to
the right moves the instrument to the right on screen, wherever the camera
points. That is what makes a real console feel natural.
"""
import time
import numpy as np

import minros
from minros.msg import JointState, PoseStamped, String, Header
from .kinematics import (ECM_JOINT_NAMES, ECM_HOME, ECM_LOWER, ECM_UPPER, ecm_forward, pose_to_msg)


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
        self.last = time.monotonic()
        self.create_timer(1 / 50, self.step)
        self.create_timer(0.5, lambda: self.pub_state.publish(String(data="ENABLED")))
        self.get_logger().info("ECM ready")

    def on_servo_jp(self, msg):
        if len(msg.position) != 4:
            self.get_logger().warn("ECM servo_jp needs 4 joint positions", throttle_duration_sec=1)
            return
        self.goal = np.clip(np.array(msg.position, dtype=float), ECM_LOWER, ECM_UPPER)

    def step(self):
        now = time.monotonic()
        dt = min(0.1, now - self.last)
        self.last = now
        self.q = self.q + np.clip(self.goal - self.q, -self.vmax * dt, self.vmax * dt)
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
