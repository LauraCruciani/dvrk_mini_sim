"""Simulated PSM (instrument arm). Run one process per arm:

    python -m dvrk_sim.psm_node PSM1
    python -m dvrk_sim.psm_node PSM2

Topics (same names as the real dVRK, CRTK convention), for PSM1:
  publishes   /PSM1/measured_js        JointState    joint positions, 50 Hz
              /PSM1/measured_cp        PoseStamped   tool tip in the WORLD frame, 100 Hz
              /PSM1/local/measured_cp  PoseStamped   tool tip in the arm's own RCM frame
              /PSM1/jaw/measured_js    JointState    jaw angle
              /PSM1/operating_state    String        "ENABLED"
              /PSM1/collision          String        JSON list of what the instrument touches:
                                                     "tissue", "wall", "PSM2" (the other instrument)
  subscribes  /PSM1/servo_cp           PoseStamped   go to this tip pose (world frame)
              /PSM1/servo_jp           JointState    go to these joint positions
              /PSM1/jaw/servo_jp       JointState    go to this jaw angle
              /PSM2/measured_cp, /PSM2/jaw/measured_js   the other instrument, to avoid it

Collisions (geometry in kinematics.py): the arm refuses the part of a motion
that would push its instrument into the tissue, into the abdominal wall or
into the other instrument. A tip command below the tissue is first projected
onto it, so the tip slides along the surface instead of stopping. There are
no forces: this is a kinematic safety layer, like virtual fixtures.

As on the dVRK, "measured_cp" is expressed in a common frame shared by all
arms, while "local/measured_cp" is relative to the arm's own base. Here the
two differ only by the position of the RCM (see PSM_BASES in kinematics.py).
"""
import json
import sys
import time
import numpy as np

import minros
from minros.msg import JointState, PoseStamped, String, Header
from .kinematics import (JOINT_NAMES, LOWER, UPPER, JAW_LIMITS, PSM_BASES, PSM_HOME,
                         forward, inverse, pose_to_msg, msg_to_pose,
                         instrument_capsules, instrument_clearances, capsule_clearance, tissue_height,
                         JAW_R)

TOUCH = 0.0003          # closer than this to an obstacle counts as touching it (m)
TIP_ABOVE_TISSUE = JAW_R + 0.0002   # where a tip command below the tissue is projected


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
        self.declare_parameter("collisions", True)
        self.other = next((a for a in PSM_BASES if a != name), None) if name in PSM_BASES else None
        self.other_caps = None
        self.other_pose, self.other_jaw = None, 0.6
        self.touching, self.last_touching = [], None
        self._ckey, self._cval = None, None

        self.pub_js = self.create_publisher(JointState, f"/{name}/measured_js", 10)
        self.pub_cp = self.create_publisher(PoseStamped, f"/{name}/measured_cp", 10)
        self.pub_local = self.create_publisher(PoseStamped, f"/{name}/local/measured_cp", 10)
        self.pub_jaw = self.create_publisher(JointState, f"/{name}/jaw/measured_js", 10)
        self.pub_state = self.create_publisher(String, f"/{name}/operating_state", 10)
        self.pub_collision = self.create_publisher(String, f"/{name}/collision", 10)
        self.create_subscription(PoseStamped, f"/{name}/servo_cp", self.on_servo_cp, 1)
        self.create_subscription(JointState, f"/{name}/servo_jp", self.on_servo_jp, 1)
        self.create_subscription(JointState, f"/{name}/jaw/servo_jp", self.on_jaw, 1)
        if self.other:
            self.create_subscription(PoseStamped, f"/{self.other}/measured_cp", self.on_other_cp, 1)
            self.create_subscription(JointState, f"/{self.other}/jaw/measured_js", self.on_other_jaw, 1)

        self.last = time.monotonic()
        self.tick = 0
        self.create_timer(1 / 200, self.step)               # control loop, 200 Hz
        self.create_timer(0.5, lambda: self.pub_state.publish(String(data="ENABLED")))
        self.create_timer(0.2, lambda: self.publish_collision(force=True))
        self.get_logger().info(f"{name} ready, RCM at x={self.base[0] * 1000:+.0f} mm in the world frame")

    # ------------------------------------------------------------ commands
    def on_servo_cp(self, msg):
        p, R = msg_to_pose(msg)
        if self.get_parameter("collisions"):             # slide on the tissue instead of digging into it
            floor = tissue_height(p[0], p[1]) + TIP_ABOVE_TISSUE
            if p[2] < floor:
                p = np.array([p[0], p[1], floor])
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

    def on_other_cp(self, msg):
        self.other_pose = msg_to_pose(msg)
        self.update_other()

    def on_other_jaw(self, msg):
        if msg.position:
            self.other_jaw = float(msg.position[0])
            self.update_other()

    def update_other(self):
        if self.other_pose is not None:
            p, R = self.other_pose
            self.other_caps = instrument_capsules(p, R, self.other_jaw, PSM_BASES[self.other])

    # ----------------------------------------------------------- collisions
    def clearances(self, x):
        """Gap to each obstacle for the configuration x = 6 joints + jaw (negative = penetration)."""
        key = (x.tobytes(), id(self.other_caps))
        if key == self._ckey:                               # nothing moved since the last check
            return self._cval
        tip, R, _ = forward(x[:6])
        caps = instrument_capsules(tip + self.base, R, x[6], self.base)
        tissue, wall = instrument_clearances(caps)
        out = {"tissue": tissue, "wall": wall}
        if self.other_caps is not None:
            out[self.other] = capsule_clearance(caps, self.other_caps)
        self._ckey, self._cval = key, out
        return out

    def allowed(self, x, before):
        """True if x penetrates nothing, or at least does not go deeper than before (to get out)."""
        c = self.clearances(x)
        return all(v >= 0 or v >= before[k] - 1e-7 for k, v in c.items())

    def safe_step(self, x, dx):
        """The largest part of the step dx that does not push into an obstacle."""
        before = self.clearances(x)
        if self.allowed(x + dx, before):
            return x + dx, before
        lo, hi = 0.0, 1.0                                   # bisection on the length of the step
        for _ in range(8):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if self.allowed(x + mid * dx, before) else (lo, mid)
        x = x + lo * dx
        rest = (1 - lo) * dx
        for i in np.argsort(-np.abs(rest)):                 # then slide: keep the joints that are still free
            if rest[i] != 0:
                trial = x.copy()
                trial[i] += rest[i]
                if self.allowed(trial, before):
                    x = trial
        return x, before

    def publish_collision(self, force=False):
        if force or self.touching != self.last_touching:
            self.pub_collision.publish(String(data=json.dumps(self.touching)))
            self.last_touching = list(self.touching)

    # ---------------------------------------------------------- simulation
    def step(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last)
        self.last = now
        k = self.get_parameter("speed_factor")
        dq = np.clip(self.goal - self.q, -self.vmax * dt * k, self.vmax * dt * k)
        djaw = float(np.clip(self.jaw_goal - self.jaw, -self.jaw_vmax * dt, self.jaw_vmax * dt))
        x = np.append(self.q, self.jaw)
        dx = np.append(dq, djaw)
        if self.get_parameter("collisions"):
            new, c = self.safe_step(x, dx)
            blocked = np.abs(new - (x + dx)).max() > 1e-9
            touching = sorted(k for k, v in c.items() if v < TOUCH)
            if blocked and not touching:                    # stopped just short of something
                touching = sorted(k for k, v in self.clearances(x + dx).items() if v < 0)
            self.touching = touching
        else:
            new, self.touching = x + dx, []
        dq = new[:6] - self.q
        self.q, self.jaw = new[:6], float(new[6])
        self.publish_collision()
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
    pos = minros.positional_args()
    node = PSM(pos[0] if pos else "PSM1")
    minros.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
