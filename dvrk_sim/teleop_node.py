"""Teleoperation from a master (MTM) to an instrument arm (PSM).

    python -m dvrk_sim.teleop_node MTMR PSM1     -> node MTMR_PSM1 (right hand)
    python -m dvrk_sim.teleop_node MTML PSM2     -> node MTML_PSM2 (left hand)

Topics for MTMR_PSM1 (the other pair is analogous):

  subscribes  /MTMR/measured_cp            PoseStamped  surgeon's hand
              /MTMR/gripper/measured_js    JointState   finger grip
              /footpedals/clutch           Bool         clutch pedal
              /PSM1/measured_cp            PoseStamped  used to anchor the motion
              /MTMR_PSM1/set_scale         Float64      as on the real dVRK
              /ECM/measured_cp             PoseStamped  camera pose (optional)
  publishes   /PSM1/servo_cp               PoseStamped  100 Hz while following
              /PSM1/jaw/servo_jp           JointState
              /MTMR_PSM1/scale             Float64
              /MTMR_PSM1/state             String       JSON with state and settings

Parameters (python -m minros param set MTMR_PSM1 <name> <value>):
  enabled          bool   follow the master or not
  scale            float  hand motion / instrument motion (3 means 3 cm -> 1 cm)
  tremor_filter    bool   2nd order low-pass on the hand position
  filter_cutoff_hz float  cut-off frequency of that filter
  latency_ms       float  artificial communication delay (remote telesurgery)
  follow_camera    bool   map hand motion in the camera image (needs an ECM)

Translation is relative and scaled; orientation follows the hand 1:1, as on
the clinical system. When an ECM is running, hand motion is expressed in the
camera frame: right is image right, forward is into the image. The clutch decouples the two: the hand can move while
the instrument stays still.
"""
import collections
import sys
import json
import math
import time
import numpy as np

import minros
from minros.msg import PoseStamped, JointState, Bool, Float64, String, Header
from .kinematics import pose_to_msg, msg_to_pose, view_rotation


class Teleop(minros.Node):
    def __init__(self, master="MTMR", slave="PSM1"):
        name = f"{master}_{slave}"
        super().__init__(name)
        self.master, self.slave, self.tname = master, slave, name
        self.declare_parameter("enabled", True)
        self.declare_parameter("scale", 3.0)
        self.declare_parameter("tremor_filter", False)
        self.declare_parameter("filter_cutoff_hz", 3.0)
        self.declare_parameter("latency_ms", 0.0)
        self.declare_parameter("follow_camera", True)
        self.Rv = np.eye(3)
        self.add_on_set_parameters_callback(self.on_param)

        self.mtm_p = self.mtm_R = None
        self.psm_p = self.psm_R = None
        self.grip = 0.6
        self.clutch = False
        self.f1 = self.f2 = None
        self.anchored = False
        self.last_cmd = None
        self.queue = collections.deque()
        self.state = "WAITING"

        self.create_subscription(PoseStamped, f"/{master}/measured_cp", self.on_mtm, 1)
        self.create_subscription(JointState, f"/{master}/gripper/measured_js", self.on_grip, 1)
        self.create_subscription(Bool, "/footpedals/clutch", self.on_clutch, 10)
        self.create_subscription(PoseStamped, f"/{slave}/measured_cp", self.on_psm, 1)
        self.create_subscription(PoseStamped, "/ECM/measured_cp", self.on_ecm, 1)
        self.create_subscription(Float64, f"/{name}/set_scale", lambda m: self.set_parameter("scale", float(m.data)), 10)
        self.pub_cp = self.create_publisher(PoseStamped, f"/{slave}/servo_cp", 10)
        self.pub_jaw = self.create_publisher(JointState, f"/{slave}/jaw/servo_jp", 10)
        self.pub_scale = self.create_publisher(Float64, f"/{name}/scale", 10)
        self.pub_state = self.create_publisher(String, f"/{name}/state", 10)

        self.last = time.monotonic()
        self.create_timer(1 / 100, self.tick)
        self.n_sent = 0
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info(f"teleoperation {master} -> {slave}, scale {self.get_parameter('scale')}")

    # ----------------------------------------------------------- inputs
    def on_mtm(self, msg):
        self.mtm_p, self.mtm_R = msg_to_pose(msg)

    def on_psm(self, msg):
        self.psm_p, self.psm_R = msg_to_pose(msg)

    def on_ecm(self, msg):
        if self.get_parameter("follow_camera"):
            _, Rc = msg_to_pose(msg)
            self.Rv = view_rotation(Rc)

    def on_grip(self, msg):
        if msg.position:
            self.grip = float(msg.position[0])

    def on_clutch(self, msg):
        if msg.data != self.clutch:
            self.clutch = bool(msg.data)
            self.anchored = False

    def on_param(self, name, value):
        if name == "scale":
            if value <= 0:
                self.get_logger().warn("scale must be positive")
                self.set_parameter("scale", 1.0)
            self.anchored = False                      # re-anchor so nothing jumps
        if name == "enabled":
            self.anchored = False
            self.last_cmd = None                       # someone else may have moved the arm
            self.queue.clear()

    # ------------------------------------------------------------ loop
    def tick(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last)
        self.last = now
        if self.mtm_p is None or self.psm_p is None:
            self.state = "WAITING"
            return

        # tremor filter: two first-order low-pass stages in series
        m = self.mtm_p
        if self.get_parameter("tremor_filter"):
            if self.f1 is None:
                self.f1, self.f2 = m.copy(), m.copy()
            a = 1 - math.exp(-2 * math.pi * self.get_parameter("filter_cutoff_hz") * dt)
            self.f1 += a * (m - self.f1)
            self.f2 += a * (self.f1 - self.f2)
            m = self.f2.copy()
        else:
            self.f1 = self.f2 = None

        self.flush(now)
        if not self.get_parameter("enabled"):
            self.state = "DISABLED"
            return
        if self.clutch:
            self.state = "CLUTCHED"
            return

        if self.anchored and np.abs(self.Rv - self.Rv0).max() > 1e-5:
            self.anchored = False                          # the camera moved: re-anchor, no jump
        if not self.anchored:
            self.Rv0 = self.Rv.copy()
            self.mtm0, self.mtm_R0 = m.copy(), self.mtm_R.copy()
            if self.last_cmd is not None:
                self.psm0, self.psm_R0 = self.last_cmd
            else:
                self.psm0, self.psm_R0 = self.psm_p.copy(), self.psm_R.copy()
            self.anchored = True

        Rv = self.Rv0                                      # hand frame -> camera-aligned world frame
        p = self.psm0 + Rv @ (m - self.mtm0) / self.get_parameter("scale")
        R = Rv @ self.mtm_R @ self.mtm_R0.T @ Rv.T @ self.psm_R0
        self.last_cmd = (p, R)
        self.queue.append((now + self.get_parameter("latency_ms") / 1000.0, p, R, self.grip))
        self.flush(now)
        self.state = "FOLLOWING"

    def flush(self, now):
        """Send the commands whose (artificial) delay has expired."""
        while self.queue and self.queue[0][0] <= now:
            _, p, R, grip = self.queue.popleft()
            self.pub_cp.publish(pose_to_msg(p, R, "world"))
            self.n_sent += 1
            if self.n_sent % 2 == 0:                       # jaw commands at half rate
                self.pub_jaw.publish(JointState(header=Header.now(), name=["jaw"], position=[grip]))

    def publish_state(self):
        self.pub_scale.publish(Float64(data=float(self.get_parameter("scale"))))
        info = {"state": self.state, "scale": self.get_parameter("scale"),
                "tremor_filter": self.get_parameter("tremor_filter"),
                "filter_cutoff_hz": self.get_parameter("filter_cutoff_hz"),
                "latency_ms": self.get_parameter("latency_ms"),
                "enabled": self.get_parameter("enabled")}
        self.pub_state.publish(String(data=json.dumps(info)))


def main():
    minros.init()
    args = sys.argv[1:3] if len(sys.argv) > 2 else ["MTMR", "PSM1"]
    node = Teleop(*args)
    minros.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
