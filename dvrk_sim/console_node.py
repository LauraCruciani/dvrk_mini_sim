"""The surgeon console as a minros node, shared by both user interfaces.

It publishes the two masters (MTML, MTMR) at 100 Hz, the clutch pedal, the
camera commands and /console/active_arm (the instrument driven by the active
hand, "PSM1" or "PSM2"), and keeps the latest state of the robots and of the
scene so that a window (web or Matplotlib) can draw it.

One hand is active at a time; the user interfaces call move_hand(),
move_depth(), set_grip() and handle_key().
"""
import collections
import json
import math
import threading
import time

import numpy as np

import minros
from minros.msg import PoseStamped, JointState, Bool, String, PoseArray, PolylineArray, Header
from .kinematics import (pose_to_msg, msg_to_pose, JAW_LENGTH, PSM_BASES, MTM_JOINT_NAMES,
                         mtm_inverse, mtm_forward_points, mtm_hand_rotation,
                         ECM_HOME, ECM_LOWER, ECM_UPPER, ECM_BASE, view_rotation)

SIDES = ("MTML", "MTMR")
ARM = {"MTML": "PSM2", "MTMR": "PSM1"}
TELEOP = {"MTML": "MTML_PSM2", "MTMR": "MTMR_PSM1"}
CONSOLE_X = {"MTML": -0.18, "MTMR": 0.18}           # where each master stands in the console
HAND_X, HAND_Y, HAND_Z = 0.15, 0.10, 0.12            # half sizes of each hand workspace (m)
TREMOR_AMP = 0.003
KEY_SPEED = 0.02                                     # m/s of the instrument with the keyboard


class ConsoleNode(minros.Node):
    def __init__(self):
        super().__init__("console")
        self.lock = threading.Lock()
        self.active = "MTMR"
        self.hand = {s: np.zeros(3) for s in SIDES}
        self.angles = {s: [0.0, 0.0, 0.0] for s in SIDES}
        self.pressed = {s: False for s in SIDES}
        self.latched = {s: False for s in SIDES}
        self.mtm_q = {s: np.zeros(7) for s in SIDES}
        self.hand_shown = {s: np.zeros(3) for s in SIDES}
        self.clutch = False
        self.tremor = False
        self.camera_mode = False
        self.tip = {a: None for a in ARM.values()}
        self.R = {a: None for a in ARM.values()}
        self.jaw = {a: 0.6 for a in ARM.values()}
        self.collision = {a: [] for a in ARM.values()}
        self.trail = {a: collections.deque(maxlen=300) for a in ARM.values()}
        self.ecm_pose = None
        self.ecm_goal = None
        self.ecm_cmd_time = 0.0
        self.targets, self.vessels = [], []
        self.status, self.teleop = {}, {}
        self.contact_until = 0.0
        self.n_pub = 0

        self.pub = {}
        for s in SIDES:
            self.pub[s] = (self.create_publisher(PoseStamped, f"/{s}/measured_cp", 10),
                           self.create_publisher(JointState, f"/{s}/measured_js", 10),
                           self.create_publisher(JointState, f"/{s}/gripper/measured_js", 10))
            self.create_subscription(String, f"/{TELEOP[s]}/state", self._teleop_cb(s), 1)
        for a in ARM.values():
            self.create_subscription(PoseStamped, f"/{a}/measured_cp", self._tip_cb(a), 1)
            self.create_subscription(JointState, f"/{a}/jaw/measured_js", self._jaw_cb(a), 1)
            self.create_subscription(String, f"/{a}/collision",
                                     lambda m, a=a: self.collision.__setitem__(a, json.loads(m.data or "[]")), 10)
        self.pub_clutch = self.create_publisher(Bool, "/footpedals/clutch", 10)
        self.pub_start = self.create_publisher(String, "/scene/start", 10)
        self.pub_ecm = self.create_publisher(JointState, "/ECM/servo_jp", 10)
        self.create_subscription(PoseStamped, "/ECM/measured_cp", self.on_ecm, 1)
        self.create_subscription(JointState, "/ECM/measured_js", self.on_ecm_js, 1)
        self.create_subscription(PoseArray, "/scene/targets", self.on_targets, 1)
        self.create_subscription(PolylineArray, "/scene/vessels", self.on_vessels, 1)
        self.create_subscription(String, "/scene/status", lambda m: setattr(self, "status", json.loads(m.data)), 1)
        self.create_subscription(Bool, "/scene/contact", self.on_contact, 10)
        # which instrument the surgeon is using now (Tab switches); another node can switch it too
        self.pub_active = self.create_publisher(String, "/console/active_arm", 10)
        self.create_subscription(String, "/console/set_active", self.on_set_active, 10)
        self.create_timer(1 / 100, self.publish_masters)
        self.create_timer(0.1, lambda: self.pub_clutch.publish(Bool(data=self.clutch)))
        self.create_timer(0.1, self.publish_active)

    # ----------------------------------------------------- the two masters
    def gripper(self, s):
        return 0.0 if (self.pressed[s] or self.latched[s]) else 0.8

    def publish_masters(self):
        t = time.monotonic()
        self.n_pub += 1
        for k, s in enumerate(SIDES):
            with self.lock:
                p, a = self.hand[s].copy(), list(self.angles[s])
            if self.tremor:                      # physiological tremor, around 10 Hz, both hands
                ph = 1.7 * k
                p += TREMOR_AMP / 1.6 * np.array([
                    math.sin(2 * math.pi * 9.2 * t + ph) + 0.6 * math.sin(2 * math.pi * 11.7 * t + 1.3 + ph),
                    0.5 * math.sin(2 * math.pi * 10.4 * t + 2.0 + ph),
                    math.sin(2 * math.pi * 10.1 * t + 0.7 + ph) + 0.6 * math.sin(2 * math.pi * 8.4 * t + 2.1 + ph)])
            q, _ = mtm_inverse(p, a)
            with self.lock:
                self.mtm_q[s], self.hand_shown[s] = q, p
            cp, js, grip = self.pub[s]
            cp.publish(pose_to_msg(p, mtm_hand_rotation(a), f"{s}_base"))            # 100 Hz
            if self.n_pub % 4 == 0:                                                   # 25 Hz
                js.publish(JointState(header=Header.now(f"{s}_base"), name=MTM_JOINT_NAMES,
                                      position=[float(v) for v in q]))
            if self.n_pub % 2 == 0:                                                   # 50 Hz
                grip.publish(JointState(header=Header.now(), name=["gripper"], position=[self.gripper(s)]))

    # ------------------------------------------------------------- inputs
    def move_hand(self, dx, dz):
        with self.lock:
            h = self.hand[self.active]
            h[0] = float(np.clip(h[0] + dx, -HAND_X, HAND_X))
            h[2] = float(np.clip(h[2] + dz, -HAND_Z, HAND_Z))

    def move_depth(self, d):
        with self.lock:
            h = self.hand[self.active]
            h[1] = float(np.clip(h[1] + d, -HAND_Y, HAND_Y))

    def nudge(self, right, forward, up, dt):
        """Keyboard motion of the active instrument, like in a video game.

        right/forward are horizontal, relative to where the camera looks;
        up is straight up in the world. Each is -1, 0 or +1. The instrument
        moves at KEY_SPEED; the hand moves `scale` times more, as with the
        mouse, so the clutch is still needed at the edges of the workspace.
        """
        if self.ecm_pose is not None:
            Rc = self.ecm_pose[1]
            Rv = view_rotation(Rc)
            fwd = np.array([Rc[0, 2], Rc[1, 2], 0.0])
            if np.linalg.norm(fwd) < 1e-3:                 # camera looking straight down
                fwd = np.array([-Rc[0, 1], -Rc[1, 1], 0.0])
        else:
            Rv, fwd = np.eye(3), np.array([0.0, 1.0, 0.0])
        fwd /= np.linalg.norm(fwd)
        rgt = np.array([fwd[1], -fwd[0], 0.0])
        v = KEY_SPEED * (right * rgt + forward * fwd + up * np.array([0.0, 0.0, 1.0]))
        scale = float(self.teleop.get(self.active, {}).get("scale", 3.0))
        d = Rv.T @ v * dt * scale                          # world motion -> hand motion
        with self.lock:
            h = self.hand[self.active]
            h[0] = float(np.clip(h[0] + d[0], -HAND_X, HAND_X))
            h[1] = float(np.clip(h[1] + d[1], -HAND_Y, HAND_Y))
            h[2] = float(np.clip(h[2] + d[2], -HAND_Z, HAND_Z))

    def set_grip(self, down):
        if down:
            self.pressed[self.active] = True
        else:
            for s in SIDES:
                self.pressed[s] = False

    def publish_active(self):
        self.pub_active.publish(String(data=ARM[self.active]))

    def set_active(self, side):
        if side != self.active:
            self.pressed[self.active] = False
            self.active = side
            self.publish_active()

    def on_set_active(self, msg):
        name = msg.data.strip().upper()
        side = {"PSM1": "MTMR", "PSM2": "MTML"}.get(name, name)
        if side in SIDES:
            self.set_active(side)

    def set_clutch(self, on):
        self.clutch = on
        self.pub_clutch.publish(Bool(data=on))

    def move_camera(self, dyaw=0.0, dpitch=0.0, dins=0.0):
        if self.ecm_goal is None:
            return
        self.ecm_cmd_time = time.monotonic()
        self.ecm_goal = np.clip(self.ecm_goal + [dyaw, dpitch, dins, 0.0], ECM_LOWER, ECM_UPPER)
        self.pub_ecm.publish(JointState(header=Header.now(), name=["yaw", "pitch", "insertion", "roll"],
                                        position=[float(v) for v in self.ecm_goal]))

    def handle_key(self, k):
        k = k.lower()
        tp = self.teleop.get("MTMR", {})
        if k in ("tab", "x"):
            self.set_active("MTML" if self.active == "MTMR" else "MTMR")
        elif k in ("c", " "):
            self.set_clutch(not self.clutch)
        elif k in ("1", "2", "3", "5"):
            for t in TELEOP.values():
                self.set_remote_parameter(t, "scale", float(k))
        elif k == "t":
            self.tremor = not self.tremor
        elif k == "f":
            for t in TELEOP.values():
                self.set_remote_parameter(t, "tremor_filter", not tp.get("tremor_filter", False))
        elif k in ("+", "=", "-"):
            v = tp.get("latency_ms", 0) + (50 if k != "-" else -50)
            for t in TELEOP.values():
                self.set_remote_parameter(t, "latency_ms", float(np.clip(v, 0, 1000)))
        elif k == "n":
            self.pub_start.publish(String(data="reach"))
        elif k == "b":
            self.pub_start.publish(String(data="handover"))
        elif k == "w":
            self.move_depth(0.005)
        elif k == "s":
            self.move_depth(-0.005)
        elif k == "g":
            self.latched[self.active] = not self.latched[self.active]
        elif k == "h":
            with self.lock:
                self.angles[self.active] = [0.0, 0.0, 0.0]
        elif k in ("j", "l", "i", "k", "u", "o"):
            idx, sign = {"j": (0, 1), "l": (0, -1), "i": (1, 1), "k": (1, -1), "u": (2, 1), "o": (2, -1)}[k]
            with self.lock:
                a = self.angles[self.active]
                a[idx] = float(np.clip(a[idx] + sign * 0.1, -1.2, 1.2))
        elif k == "v":
            self.camera_mode = not self.camera_mode
        elif k == "arrowleft":
            self.move_camera(dyaw=0.04)
        elif k == "arrowright":
            self.move_camera(dyaw=-0.04)
        elif k == "arrowup":
            self.move_camera(dpitch=0.04)
        elif k == "arrowdown":
            self.move_camera(dpitch=-0.04)
        elif k == "pageup":
            self.move_camera(dins=0.005)
        elif k == "pagedown":
            self.move_camera(dins=-0.005)
        elif k == "home":
            if self.ecm_goal is not None:
                self.ecm_goal = ECM_HOME.copy()
                self.move_camera()

    # ------------------------------------------------------------ feedback
    def _tip_cb(self, arm):
        def cb(msg):
            p, R = msg_to_pose(msg)
            with self.lock:
                self.tip[arm], self.R[arm] = p, R
                self.trail[arm].append(p * 1000.0)
        return cb

    def _jaw_cb(self, arm):
        def cb(msg):
            if msg.position:
                self.jaw[arm] = msg.position[0]
        return cb

    def _teleop_cb(self, side):
        def cb(msg):
            self.teleop[side] = json.loads(msg.data)
        return cb

    def on_ecm(self, msg):
        self.ecm_pose = msg_to_pose(msg)

    def on_ecm_js(self, msg):
        # follow the measured camera unless we commanded it a moment ago: another
        # node (an autonomous camera, for example) may be moving it
        if len(msg.position) == 4 and time.monotonic() - self.ecm_cmd_time > 0.5:
            self.ecm_goal = np.array(msg.position, dtype=float)

    def on_targets(self, msg):
        with self.lock:
            self.targets = [np.array([q.position.x, q.position.y, q.position.z]) for q in msg.poses]

    def on_vessels(self, msg):
        with self.lock:
            self.vessels = [(np.array([[p.x, p.y, p.z] for p in line.points]), line.radius) for line in msg.lines]

    def on_contact(self, msg):
        if msg.data:
            self.contact_until = time.monotonic() + 0.6

    # ------------------------------------------------------------ snapshot
    def snapshot(self):
        """Everything a window needs to draw one frame, as plain Python data (metres)."""
        r = lambda v: [round(float(x), 6) for x in v]
        with self.lock:
            arms = {}
            for a, base in PSM_BASES.items():
                if self.tip[a] is None:
                    continue
                R = self.R[a]
                arms[a] = {"rcm": r(base), "tip": r(self.tip[a]), "wrist": r(self.tip[a] - R[:, 2] * JAW_LENGTH),
                           "R": [r(row) for row in R], "jaw": round(float(self.jaw[a]), 4),
                           "collision": list(self.collision[a])}
            mtm = {}
            for s in SIDES:
                off = np.array([CONSOLE_X[s], 0.0, 0.0])
                S, E, H = (p + off for p in mtm_forward_points(self.mtm_q[s]))
                mtm[s] = {"S": r(S), "E": r(E), "H": r(H), "R": [r(row) for row in mtm_hand_rotation(self.angles[s])],
                          "grip": self.gripper(s), "hand": r(self.hand[s]), "q": r(self.mtm_q[s]),
                          "latched": self.latched[s]}
            targets = [r(t) for t in self.targets]
        ecm = None
        if self.ecm_pose is not None:
            p, Rc = self.ecm_pose
            ecm = {"p": r(p), "R": [r(row) for row in Rc], "rcm": r(ECM_BASE)}
        return {"t": "state", "arms": arms, "mtm": mtm, "ecm": ecm, "active": self.active,
                "clutch": self.clutch, "tremor": self.tremor, "camera_mode": self.camera_mode,
                "teleop": self.teleop, "status": self.status, "targets": targets,
                "contact": time.monotonic() < self.contact_until,
                "workspace": [HAND_X, HAND_Y, HAND_Z]}

    def vessels_message(self):
        with self.lock:
            return {"t": "vessels", "lines": [{"pts": [[round(float(c), 6) for c in p] for p in pts], "r": rad}
                                              for pts, rad in self.vessels]}
