"""Simulated PSM (instrument arm). Run one process per arm:

    python -m dvrk_sim.psm_node PSM1
    python -m dvrk_sim.psm_node PSM2

Topics (same names as the real dVRK, CRTK convention), for PSM1:
  publishes   /PSM1/measured_js        JointState    joint positions, 50 Hz
              /PSM1/measured_cp        PoseStamped   tool tip in the WORLD frame, 100 Hz
              /PSM1/local/measured_cp  PoseStamped   tool tip in the arm's own RCM frame
              /PSM1/jaw/measured_js    JointState    jaw angle
              /PSM1/operating_state    String        "ENABLED"
  subscribes  /PSM1/servo_cp           PoseStamped   go to this tip pose (world frame)
              /PSM1/servo_jp           JointState    go to these joint positions
              /PSM1/jaw/servo_jp       JointState    go to this jaw angle

As on the dVRK, "measured_cp" is expressed in a common frame shared by all
arms, while "local/measured_cp" is relative to the arm's own base. Here the
two differ only by the position of the RCM (see PSM_BASES in kinematics.py).
"""
import sys
import time
import numpy as np

import minros
from minros.msg import JointState, PoseStamped, String, Header
from .kinematics import (JOINT_NAMES, LOWER, UPPER, JAW_LIMITS, PSM_BASES, PSM_HOME,
                         forward, inverse, pose_to_msg, msg_to_pose)


class PSM(minros.Node):
    def __init__(self, name="PSM1"):
        super().__init__(name)
        self.arm = name
        self.base = PSM_BASES.get(name, np.zeros(3))
        self.q = np.array(PSM_HOME.get(name, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0]), dtype=float)
        self.goal = self.q.copy()
        self.jaw = 0.6
        self.jaw_goal = 0.6
        self.vmax = np.array([1.2, 1.2, 0.15, 4.0, 4.0, 4.0])   # rad/s, m/s
        self.jaw_vmax = 4.0
        self.declare_parameter("speed_factor", 1.0)

        self.pub_js = self.create_publisher(JointState, f"/{name}/measured_js", 10)
        self.pub_cp = self.create_publisher(PoseStamped, f"/{name}/measured_cp", 10)
        self.pub_local = self.create_publisher(PoseStamped, f"/{name}/local/measured_cp", 10)
        self.pub_jaw = self.create_publisher(JointState, f"/{name}/jaw/measured_js", 10)
        self.pub_state = self.create_publisher(String, f"/{name}/operating_state", 10)
        self.create_subscription(PoseStamped, f"/{name}/servo_cp", self.on_servo_cp, 1)
        self.create_subscription(JointState, f"/{name}/servo_jp", self.on_servo_jp, 1)
        self.create_subscription(JointState, f"/{name}/jaw/servo_jp", self.on_jaw, 1)

        self.last = time.monotonic()
        self.tick = 0
        self.create_timer(1 / 200, self.step)               # control loop, 200 Hz
        self.create_timer(0.5, lambda: self.pub_state.publish(String(data="ENABLED")))
        self.get_logger().info(f"{name} ready, RCM at x={self.base[0] * 1000:+.0f} mm in the world frame")

    # ------------------------------------------------------------ commands
    def on_servo_cp(self, msg):
        p, R = msg_to_pose(msg)
        try:
            q, ok = inverse(p - self.base, R)          # world frame -> this arm's RCM frame
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=1)
            return
        if not ok:
            bad = [JOINT_NAMES[i] for i in range(6) if not LOWER[i] <= q[i] <= UPPER[i]]
            self.get_logger().warn("servo_cp outside joint limits (" + ", ".join(bad) + "): clamping",
                                   throttle_duration_sec=1)
        self.goal = np.clip(q, LOWER, UPPER)

    def on_servo_jp(self, msg):
        if len(msg.position) != 6:
            self.get_logger().warn("servo_jp needs 6 joint positions", throttle_duration_sec=1)
            return
        self.goal = np.clip(np.array(msg.position, dtype=float), LOWER, UPPER)

    def on_jaw(self, msg):
        if msg.position:
            self.jaw_goal = float(np.clip(msg.position[0], *JAW_LIMITS))

    # ---------------------------------------------------------- simulation
    def step(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last)
        self.last = now
        k = self.get_parameter("speed_factor")
        dq = np.clip(self.goal - self.q, -self.vmax * dt * k, self.vmax * dt * k)
        self.q = self.q + dq
        self.jaw += float(np.clip(self.jaw_goal - self.jaw, -self.jaw_vmax * dt, self.jaw_vmax * dt))
        self.tick += 1
        if self.tick % 2:
            return                                          # publish at 100 Hz
        stamp = time.time()
        tip, R, _ = forward(self.q)
        self.pub_cp.publish(pose_to_msg(tip + self.base, R, "world", stamp))
        if self.tick % 4:
            return                                          # the rest at 50 Hz
        self.pub_js.publish(JointState(header=Header(stamp, f"{self.arm}_base"), name=JOINT_NAMES,
                                       position=[float(v) for v in self.q],
                                       velocity=[float(v) for v in dq / max(dt, 1e-6)]))
        self.pub_local.publish(pose_to_msg(tip, R, f"{self.arm}_base", stamp))
        self.pub_jaw.publish(JointState(header=Header(stamp, ""), name=["jaw"], position=[self.jaw]))


def main():
    minros.init()
    node = PSM(sys.argv[1] if len(sys.argv) > 1 else "PSM1")
    minros.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
