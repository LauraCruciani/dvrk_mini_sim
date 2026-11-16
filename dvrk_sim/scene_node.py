"""The surgical scene, shared by both arms (all coordinates in the world frame).

Two tasks:
  reach     touch the yellow targets in order with either instrument
  handover  pick up each bead with the LEFT arm (PSM2), pass it to the RIGHT
            arm (PSM1) and put it down on its ring. A classic bimanual
            training exercise.
Both tasks count contacts with the blood vessels.

  subscribes  /PSM1/measured_cp, /PSM2/measured_cp          PoseStamped
              /PSM1/jaw/measured_js, /PSM2/jaw/measured_js  JointState
              /scene/start      String   "reach" or "handover"
  publishes   /scene/targets    PoseArray      remaining reach targets, 5 Hz
              /scene/vessels    PolylineArray  vessel centre lines and radii, 1 Hz
              /scene/status     String         JSON with everything else, 20 Hz
              /scene/contact    Bool           true while a tip touches a vessel

Grasping: closing the jaws (below 0.35 rad) within 6 mm of a bead picks it
up. Opening them releases it; a released bead falls on the tissue (z = -130 mm); the vessels lie on it.
If the other instrument is also holding the bead, it simply keeps it: that
is a handover.

Parameters: n_targets, tolerance_mm, hold_s, n_beads, seed (-1 = random).
"""
import json
import time
import numpy as np

import minros
from minros.msg import (PoseStamped, PoseArray, Pose, Point, Bool, String, Header,
                        Polyline, PolylineArray, JointState)
from .kinematics import msg_to_pose

ARMS = ("PSM1", "PSM2")
FLOOR = -0.130            # tissue surface
BEAD_R = 0.0025
REST_Z = FLOOR + BEAD_R   # height of a bead lying on the tissue
GRASP_DIST = 0.006
JAW_CLOSED = 0.35
PLACE_TOL = 0.007


def _vessel(a, b, bend, n=30):
    a, b, bend = map(np.asarray, (a, b, bend))
    t = np.linspace(0, 1, n)[:, None]
    return a + (b - a) * t + bend * np.sin(np.pi * t)


VESSELS = [
    (_vessel([-0.075, -0.010, -0.1270], [0.075, 0.015, -0.1270], [0.0, 0.020, 0.0]), 0.0030),
    (_vessel([-0.015, 0.065, -0.1275], [0.030, -0.065, -0.1275], [0.020, 0.0, 0.0]), 0.0025),
]


def seg_distance(p, pts):
    a, b = pts[:-1], pts[1:]
    ab = b - a
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.einsum("ij,ij->i", ab, ab), 0, 1)
    return float(np.min(np.linalg.norm(a + ab * t[:, None] - p, axis=1)))


def vessel_distance(p):
    return min(seg_distance(p, pts) - r for pts, r in VESSELS)


class Scene(minros.Node):
    def __init__(self):
        super().__init__("scene")
        self.declare_parameter("n_targets", 5)
        self.declare_parameter("tolerance_mm", 4.0)
        self.declare_parameter("hold_s", 0.5)
        self.declare_parameter("n_beads", 3)
        self.declare_parameter("seed", -1)
        self.tip = {a: None for a in ARMS}
        self.jaw = {a: 0.6 for a in ARMS}
        self.task, self.state = "none", "idle"
        self.targets, self.index, self.hold = [], 0, 0.0
        self.beads = []
        self.t0 = self.elapsed = 0.0
        self.path = 0.0
        self.contacts, self.drops, self.handovers = 0, 0, 0
        self.in_contact = False
        self.prev_tip = {a: None for a in ARMS}
        self.last = time.monotonic()

        for a in ARMS:
            self.create_subscription(PoseStamped, f"/{a}/measured_cp", self._tip_cb(a), 1)
            self.create_subscription(JointState, f"/{a}/jaw/measured_js", self._jaw_cb(a), 1)
        self.create_subscription(String, "/scene/start", self.on_start, 10)
        self.pub_targets = self.create_publisher(PoseArray, "/scene/targets", 10)
        self.pub_vessels = self.create_publisher(PolylineArray, "/scene/vessels", 10)
        self.pub_status = self.create_publisher(String, "/scene/status", 10)
        self.pub_contact = self.create_publisher(Bool, "/scene/contact", 10)
        self.create_timer(1 / 100, self.step)
        self.create_timer(0.2, self.publish_targets)
        self.create_timer(1.0, self.publish_vessels)
        self.create_timer(0.05, self.publish_status)
        self.get_logger().info('scene ready: publish "reach" or "handover" on /scene/start')

    def _tip_cb(self, arm):
        def cb(msg):
            self.tip[arm], _ = msg_to_pose(msg)
        return cb

    def _jaw_cb(self, arm):
        def cb(msg):
            if msg.position:
                self.on_jaw(arm, float(msg.position[0]))
        return cb

    # ------------------------------------------------------------- set-up
    def rng(self):
        seed = self.get_parameter("seed")
        return np.random.default_rng(None if seed < 0 else seed)

    def make_targets(self):
        rng, out = self.rng(), []
        for _ in range(20000):
            if len(out) >= self.get_parameter("n_targets"):
                break
            p = np.array([rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), rng.uniform(-0.125, -0.100)])
            if vessel_distance(p) < 0.008 or (out and np.linalg.norm(p - out[-1]) < 0.03):
                continue
            out.append(p)
        return out

    def make_beads(self):
        rng, beads = self.rng(), []
        n = self.get_parameter("n_beads")
        for _ in range(20000):
            if len(beads) >= n:
                break
            y = rng.uniform(-0.035, 0.035)
            start = np.array([rng.uniform(-0.050, -0.030), y, REST_Z])
            goal = np.array([rng.uniform(0.030, 0.050), rng.uniform(-0.035, 0.035), FLOOR])
            if vessel_distance(start) < 0.008 or vessel_distance(goal) < 0.010:
                continue
            if any(np.linalg.norm(start - b["p"]) < 0.015 or np.linalg.norm(goal - b["goal"]) < 0.018 for b in beads):
                continue
            beads.append({"p": start, "goal": goal, "holders": [], "placed": False, "by": set()})
        return beads

    def on_start(self, msg):
        task = (msg.data or "reach").strip().lower()
        if task not in ("reach", "handover"):
            self.get_logger().warn(f"unknown task '{task}': use reach or handover")
            return
        self.task, self.state = task, "running"
        self.targets = self.make_targets() if task == "reach" else []
        self.beads = self.make_beads() if task == "handover" else []
        self.index, self.hold, self.path = 0, 0.0, 0.0
        self.contacts, self.drops, self.handovers = 0, 0, 0
        self.t0, self.elapsed = time.monotonic(), 0.0
        self.get_logger().info(f"{task} trial started")

    # ------------------------------------------------------------ grasping
    def on_jaw(self, arm, value):
        before = self.jaw[arm]
        self.jaw[arm] = value
        if self.task != "handover" or self.state != "running" or self.tip[arm] is None:
            return
        if before > JAW_CLOSED >= value:                        # jaws just closed
            free = [b for b in self.beads if not b["placed"] and arm not in b["holders"]]
            if free:
                b = min(free, key=lambda b: np.linalg.norm(self.tip[arm] - b["p"]))
                if np.linalg.norm(self.tip[arm] - b["p"]) < GRASP_DIST:
                    b["holders"].append(arm)
                    b["by"].add(arm)
        elif before <= JAW_CLOSED < value:                      # jaws just opened
            for b in self.beads:
                if arm in b["holders"]:
                    b["holders"].remove(arm)
                    if b["holders"]:
                        if arm == "PSM2" and b["holders"] == ["PSM1"]:
                            self.handovers += 1
                            self.get_logger().info("handover PSM2 -> PSM1")
                    else:
                        self.release(b)

    def release(self, b):
        b["p"] = np.array([b["p"][0], b["p"][1], REST_Z])
        if np.linalg.norm(b["p"][:2] - b["goal"][:2]) < PLACE_TOL:
            b["placed"] = True
            b["p"] = b["goal"].copy()
            self.get_logger().info("bead placed" + ("" if b["by"] == {"PSM1", "PSM2"} else " (without handover)"))
        else:
            self.drops += 1

    # ----------------------------------------------------------- main loop
    def step(self):
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        tips = [p for p in self.tip.values() if p is not None]
        if not tips:
            return
        contact = min(vessel_distance(p) for p in tips) < 0.0005
        if contact and not self.in_contact:
            if self.state == "running":
                self.contacts += 1
            self.get_logger().warn("contact with a vessel!")
        if contact != self.in_contact:
            self.pub_contact.publish(Bool(data=contact))
        self.in_contact = contact

        for b in self.beads:                                   # held beads follow the jaws
            if b["holders"]:
                b["p"] = np.mean([self.tip[a] for a in b["holders"]], axis=0)

        if self.state == "running":
            self.elapsed = now - self.t0
            for a, p in self.tip.items():
                if p is not None and self.prev_tip[a] is not None:
                    self.path += float(np.linalg.norm(p - self.prev_tip[a]))
        self.prev_tip = dict(self.tip)
        if self.state != "running":
            return

        if self.task == "reach":
            target = self.targets[self.index]
            if min(np.linalg.norm(p - target) for p in tips) < self.get_parameter("tolerance_mm") / 1000:
                self.hold += dt
                if self.hold >= self.get_parameter("hold_s"):
                    self.index, self.hold = self.index + 1, 0.0
                    if self.index >= len(self.targets):
                        self.finish()
            else:
                self.hold = max(0.0, self.hold - 2 * dt)
        elif self.beads and all(b["placed"] for b in self.beads):
            self.finish()

    def finish(self):
        self.state = "done"
        self.get_logger().info(f"{self.task} done in {self.elapsed:.1f} s, path {self.path * 1000:.0f} mm, "
                               f"{self.contacts} contact(s), {self.drops} drop(s), {self.handovers} handover(s)")

    # ------------------------------------------------------------- outputs
    def publish_targets(self):
        remaining = self.targets[self.index:] if self.state == "running" and self.task == "reach" else []
        self.pub_targets.publish(PoseArray(header=Header.now("world"),
                                           poses=[Pose(position=Point(*map(float, p))) for p in remaining]))

    def publish_vessels(self):
        self.pub_vessels.publish(PolylineArray(header=Header.now("world"), lines=[
            Polyline(points=[Point(*map(float, p)) for p in pts], radius=r) for pts, r in VESSELS]))

    def publish_status(self):
        r3 = lambda v: [round(float(x), 5) for x in v]
        self.pub_status.publish(String(data=json.dumps({
            "task": self.task, "state": self.state,
            "index": self.index, "n": len(self.targets),
            "time_s": round(self.elapsed, 2), "path_mm": round(self.path * 1000, 1),
            "contacts": self.contacts, "drops": self.drops, "handovers": self.handovers,
            "hold": round(self.hold / self.get_parameter("hold_s"), 2),
            "beads": [{"p": r3(b["p"]), "goal": r3(b["goal"]), "held": b["holders"], "placed": b["placed"]}
                      for b in self.beads]})))


def main():
    minros.init()
    node = Scene()
    minros.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
