"""The surgical scene, shared by both arms (all coordinates in the world frame).

Three tasks:
  reach     touch the yellow targets in order with either instrument
  handover  pick up each bead with the LEFT arm (PSM2), pass it to the RIGHT
            arm (PSM1) and put it down on its ring. A classic bimanual
            training exercise.
  anatomy   a realistic scene: the instruments work inside a laparoscopic
            surface reconstructed from a real video frame (dvrk_sim/anatomy.py).
            Free exploration; contacts are counted per organ.

The first two tasks use the "classic" world (flat tissue, vessels, dome), the
third the "anatomy" world: /scene/world tells every node which one is active.
Both tasks count contacts with the blood vessels, touches of the tissue or
of the abdominal wall, and clashes between the two instruments.

Each bead has its own ring, drawn in the same colour: a bead counts as placed
only in its own ring. Once placed, bead and ring turn green.

  subscribes  /PSM1/measured_cp, /PSM2/measured_cp          PoseStamped
              /PSM1/jaw/measured_js, /PSM2/jaw/measured_js  JointState
              /PSM1/collision, /PSM2/collision              String (JSON list)
              /scene/start      String   "reach" or "handover"
  publishes   /scene/targets    PoseArray      remaining reach targets, 5 Hz
              /scene/vessels    PolylineArray  vessel centre lines and radii, 1 Hz
              /scene/status     String         JSON with everything else, 20 Hz
              /scene/contact    Bool           true while a tip touches a vessel
              /scene/world      String         "classic" or "anatomy", 2 Hz

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
from .kinematics import msg_to_pose, pose_to_msg, ECM_HOME, ECM_JOINT_NAMES, PSM_BASES, shaft_rotation
from . import anatomy

PARK_S = 1.5              # s to bring the instruments to their start poses in the anatomy

ARMS = ("PSM1", "PSM2")
FLOOR = -0.130            # tissue surface
BEAD_R = 0.0025
REST_Z = FLOOR + BEAD_R   # height of a bead lying on the tissue
GRASP_DIST = 0.006
JAW_CLOSED = 0.35
PLACE_TOL = 0.007
# one colour per bead and its ring; never green (green means "placed") nor the instruments' colours
BEAD_COLORS = ["#3d8bff", "#e254d6", "#f5d33b", "#2fd3e6", "#f2f2f2", "#9a6bff"]


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
        self.clashes = 0
        self.tissue_hits = 0
        self.collision = {a: [] for a in ARMS}
        self.in_clash = self.in_tissue = False
        self.in_contact = False
        self.prev_tip = {a: None for a in ARMS}
        self.last = time.monotonic()

        for a in ARMS:
            self.create_subscription(PoseStamped, f"/{a}/measured_cp", self._tip_cb(a), 1)
            self.create_subscription(JointState, f"/{a}/jaw/measured_js", self._jaw_cb(a), 1)
            self.create_subscription(String, f"/{a}/collision", self._collision_cb(a), 10)
        self.create_subscription(String, "/scene/start", self.on_start, 10)
        self.pub_targets = self.create_publisher(PoseArray, "/scene/targets", 10)
        self.pub_vessels = self.create_publisher(PolylineArray, "/scene/vessels", 10)
        self.pub_status = self.create_publisher(String, "/scene/status", 10)
        self.pub_contact = self.create_publisher(Bool, "/scene/contact", 10)
        self.create_timer(1 / 100, self.step)
        self.create_timer(0.2, self.publish_targets)
        self.create_timer(1.0, self.publish_vessels)
        self.world = "classic"
        self.organ_contacts, self.touching_organs = {}, set()
        self.arm_on_tissue = {}
        self.rot = {a: None for a in ARMS}
        self.park_until = 0.0
        self.pub_world = self.create_publisher(String, "/scene/world", 10)
        self.pub_servo = {a: self.create_publisher(PoseStamped, f"/{a}/servo_cp", 10) for a in ARMS}
        self.pub_ecm = self.create_publisher(JointState, "/ECM/servo_jp", 10)
        self.create_timer(0.5, lambda: self.pub_world.publish(String(data=self.world)))
        self.create_timer(0.05, self.publish_status)
        self.get_logger().info('scene ready: publish "reach" or "handover" on /scene/start')

    def _tip_cb(self, arm):
        def cb(msg):
            self.tip[arm], self.rot[arm] = msg_to_pose(msg)
        return cb

    def _collision_cb(self, arm):
        def cb(msg):
            self.collision[arm] = json.loads(msg.data or "[]")
            mine = [c for c in self.collision[arm] if c not in ARMS and c != "wall"]
            if mine and not self.arm_on_tissue.get(arm) and self.state == "running":
                o = mine[0]                                         # a new contact of this instrument
                self.organ_contacts[o] = self.organ_contacts.get(o, 0) + 1
            self.arm_on_tissue[arm] = bool(mine)
            organs = {c for a in ARMS for c in self.collision[a] if c not in ARMS}
            self.touching_organs = organs
            tissue = bool(organs)
            if tissue and not self.in_tissue:
                if self.state == "running":
                    self.tissue_hits += 1
                self.get_logger().warn("contact with the tissue!")
            self.in_tissue = tissue
            clash = any(b in self.collision[a] for a in ARMS for b in ARMS if b != a)
            if clash and not self.in_clash and self.passing_bead():
                return                      # jaws meeting around a bead: that is a handover, not a clash
            if clash and not self.in_clash:
                if self.state == "running":
                    self.clashes += 1
                self.get_logger().warn("the instruments collided!")
            self.in_clash = clash
        return cb

    def passing_bead(self):
        """True while a held bead sits between both tips (the two jaws are allowed to meet there)."""
        if self.task != "handover" or any(self.tip[a] is None for a in ARMS):
            return False
        return any(b["holders"] and all(np.linalg.norm(self.tip[a] - b["p"]) < 2 * GRASP_DIST for a in ARMS)
                   for b in self.beads)

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
            beads.append({"p": start, "goal": goal, "holders": [], "placed": False, "by": set(),
                          "color": BEAD_COLORS[len(beads) % len(BEAD_COLORS)]})
        return beads

    def on_start(self, msg):
        task = (msg.data or "reach").strip().lower()
        if task not in ("reach", "handover", "anatomy"):
            self.get_logger().warn(f"unknown task '{task}': use reach, handover or anatomy")
            return
        if task == "anatomy" and not anatomy.available():
            self.get_logger().warn("the anatomy is not available: put surgaware_ply_export/ in the simulator "
                                   "folder and run python -m dvrk_sim.anatomy")
            return
        world = "anatomy" if task == "anatomy" else "classic"
        if world != self.world:
            self.world = world
            self.pub_world.publish(String(data=world))
            self.publish_vessels()
        if world == "anatomy":
            self.park()
        self.organ_contacts, self.touching_organs = {}, set()
        self.task, self.state = task, "running"
        self.targets = self.make_targets() if task == "reach" else []
        self.beads = self.make_beads() if task == "handover" else []
        self.index, self.hold, self.path = 0, 0.0, 0.0
        self.contacts, self.drops, self.handovers = 0, 0, 0
        self.clashes = self.tissue_hits = 0
        self.t0, self.elapsed = time.monotonic(), 0.0
        self.get_logger().info(f"{task} trial started")

    def park(self):
        """Bring the camera home and both instruments in front of the anatomy (teleoperation paused)."""
        if not hasattr(self, "anat"):
            self.anat = anatomy.Anatomy()
        self.park_goal = {"PSM2": self.anat.start_tip(-1), "PSM1": self.anat.start_tip(+1)}
        for t in ("MTMR_PSM1", "MTML_PSM2"):
            self.set_remote_parameter(t, "enabled", False)
        self.pub_ecm.publish(JointState(header=Header.now(), name=ECM_JOINT_NAMES,
                                        position=[float(v) for v in ECM_HOME]))
        self.park_until = time.monotonic() + PARK_S

    def parking(self, now):
        if now < self.park_until:
            for a in ARMS:
                u = self.park_goal[a] - PSM_BASES[a]              # straight wrist, jaws along the shaft
                u /= np.linalg.norm(u)
                R = shaft_rotation(np.arctan2(-u[0], -u[2]), np.arcsin(u[1]))
                self.pub_servo[a].publish(pose_to_msg(self.park_goal[a], R, "world"))
            return True
        if self.park_until:
            self.park_until = 0.0
            for t in ("MTMR_PSM1", "MTML_PSM2"):
                self.set_remote_parameter(t, "enabled", True)    # re-anchors on where the arm is now
            self.t0, self.path = now, 0.0                         # the trial starts now
            self.organ_contacts, self.clashes, self.tissue_hits = {}, 0, 0
        return False

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
        if np.linalg.norm(b["p"][:2] - b["goal"][:2]) < PLACE_TOL:          # its own ring only
            b["placed"] = True
            b["p"] = b["goal"].copy()
            self.get_logger().info("bead placed" + ("" if b["by"] == {"PSM1", "PSM2"} else " (without handover)"))
        else:
            self.drops += 1
            other = [o for o in self.beads if o is not b and not o["placed"]
                     and np.linalg.norm(b["p"][:2] - o["goal"][:2]) < PLACE_TOL]
            if other:
                self.get_logger().info("wrong ring: each bead goes in the ring of its own colour")

    # ----------------------------------------------------------- main loop
    def step(self):
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        tips = [p for p in self.tip.values() if p is not None]
        if not tips:
            return
        if self.parking(now):
            self.prev_tip = dict(self.tip)
            return
        contact = self.world == "classic" and min(vessel_distance(p) for p in tips) < 0.0005
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
                               f"{self.contacts} contact(s), {self.tissue_hits} tissue touch(es), {self.clashes} clash(es), {self.drops} drop(s), "
                               f"{self.handovers} handover(s)")

    # ------------------------------------------------------------- outputs
    def publish_targets(self):
        remaining = self.targets[self.index:] if self.state == "running" and self.task == "reach" else []
        self.pub_targets.publish(PoseArray(header=Header.now("world"),
                                           poses=[Pose(position=Point(*map(float, p))) for p in remaining]))

    def publish_vessels(self):
        lines = [] if self.world != "classic" else [
            Polyline(points=[Point(*map(float, p)) for p in pts], radius=r) for pts, r in VESSELS]
        self.pub_vessels.publish(PolylineArray(header=Header.now("world"), lines=lines))

    def publish_status(self):
        r3 = lambda v: [round(float(x), 5) for x in v]
        self.pub_status.publish(String(data=json.dumps({
            "task": self.task, "state": self.state,
            "index": self.index, "n": len(self.targets),
            "time_s": round(self.elapsed, 2), "path_mm": round(self.path * 1000, 1),
            "contacts": self.contacts, "drops": self.drops, "handovers": self.handovers,
            "clashes": self.clashes, "tissue_hits": self.tissue_hits, "collision": self.collision,
            "touching_tissue": self.in_tissue, "clashing": self.in_clash,
            "world": self.world, "organ_contacts": self.organ_contacts,
            "touching": sorted(self.touching_organs), "parking": self.park_until > 0,
            "hold": round(self.hold / self.get_parameter("hold_s"), 2),
            "beads": [{"p": r3(b["p"]), "goal": r3(b["goal"]), "held": b["holders"], "placed": b["placed"],
                       "color": b["color"]}
                      for b in self.beads]})))


def main():
    minros.init()
    node = Scene()
    minros.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
