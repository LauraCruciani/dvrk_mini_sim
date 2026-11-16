"""Automatic checker for the exercises.

    python exercises/check.py A          Part A (your answers in exercises/answers_part_a.py)
    python exercises/check.py B1         one exercise of Part B (B1 ... B6)
    python exercises/check.py B3 --file my_solution.py   check another file

Part B needs the simulator running in another terminal (python launch.py,
or python launch.py --no-gui). The checker pauses the teleoperation while
it tests and gives it back at the end.
"""
import base64
import importlib.util
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

import numpy as np                                   # noqa: E402
import minros                                        # noqa: E402
from minros import broker                            # noqa: E402
from minros.msg import PoseStamped, JointState, String, Float64, Bool, get_type   # noqa: E402
from dvrk_sim.kinematics import msg_to_pose, pose_to_msg, ECM_HOME   # noqa: E402

OK, BAD = "[ok]", "[!!]"
EXPECTED = json.loads(base64.b64decode("eyJBMV8xIjogWyJzIiwgIm10bXJfcHNtMSJdLCAiQTFfMiI6IFsibCIsICJtdG1sX3BzbTIsbXRtcl9wc20xIl0sICJBMV8zIjogWyJzIiwgInBvbHlsaW5lYXJyYXkiXSwgIkExXzQiOiBbIm4iLCA3LCAwXSwgIkEyXzEiOiBbIm4iLCAxMDAsIDAuMTJdLCAiQTJfMiI6IFsibiIsIDUwLCAwLjEyXSwgIkEyXzMiOiBbIm4iLCAyNSwgMC4xMl0sICJBM18xIjogWyJuIiwgMjAwLCAwXSwgIkEzXzIiOiBbInMiLCAid29ybGQiXSwgIkEzXzMiOiBbInMiLCAicHNtMV9iYXNlIl0sICJBM180IjogWyJuIiwgMTUwLCAwLjAxXSwgIkEzXzUiOiBbInMiLCAiYiJdLCAiQTRfMSI6IFsibiIsIDEwLCAwLjAxXSwgIkE0XzIiOiBbInNldCIsICJhYmNkIl0sICJBNF8zIjogWyJuIiwgMiwgMF0sICJBNF80IjogWyJuIiwgMTAwLCAwXX0="))
HINTS = {
    "A1": "python -m minros graph   and   python -m minros topic info <topic>",
    "A2": "python -m minros topic hz <topic>  (give a round number)",
    "A3": "read dvrk_sim/psm_node.py: create_timer, pose_to_msg(..., frame_id), vmax, on_servo_cp",
    "A4": "read dvrk_sim/teleop_node.py: tick(), on_param(), on_clutch(), on_ecm(), create_timer",
}


# ===================================================================== part A
def check_a():
    path = os.path.join(HERE, "answers_part_a.py")
    spec = importlib.util.spec_from_file_location("answers", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    score = 0
    last_group = None
    for key, (kind, *exp) in EXPECTED.items():
        group = key[:2]
        if group != last_group:
            print(f"\n{group}   hint: {HINTS[group]}")
            last_group = group
        val = getattr(mod, key, None)
        try:
            if kind == "s":
                good = str(val).strip().strip("/").lower() == exp[0]
            elif kind == "l":
                good = ",".join(sorted(str(v).strip().strip("/").lower() for v in val)) == exp[0]
            elif kind == "set":
                good = "".join(sorted(set(str(val).lower().replace(",", "").replace(" ", "")))) == exp[0]
            else:
                good = abs(float(val) - exp[0]) <= max(exp[1] * exp[0], 1e-9)
        except (TypeError, ValueError):
            good = False
        score += good
        print(f"  {OK if good else BAD} {key} = {val!r}")
    print(f"\nPart A: {score}/{len(EXPECTED)} correct")
    return score == len(EXPECTED)


# ===================================================================== part B
class Harness:
    """A test node that can move the robots and listen to the student's node."""

    def __init__(self):
        if not broker.is_running():
            sys.exit("The simulator is not running. Start it in another terminal: python launch.py")
        minros.init([])
        self.node = minros.Node(f"_checker_{os.getpid()}")
        self.poses, self.ecm, self.teleop, self.inbox = {}, None, {}, {}
        for arm in ("PSM1", "PSM2"):
            self.node.create_subscription(PoseStamped, f"/{arm}/measured_cp", self._pose_cb(arm), 1)
        self.node.create_subscription(PoseStamped, "/ECM/measured_cp",
                                      lambda m: setattr(self, "ecm", msg_to_pose(m)), 1)
        self.node.create_subscription(String, "/MTMR_PSM1/state",
                                      lambda m: self.teleop.update(json.loads(m.data)), 1)
        self.cmd = {a: self.node.create_publisher(PoseStamped, f"/{a}/servo_cp", 10) for a in ("PSM1", "PSM2")}
        if not minros.spin_until(self.node, lambda: len(self.poses) == 2, timeout_sec=3):
            sys.exit("No /PSM1/measured_cp or /PSM2/measured_cp: is the simulator running?")
        self.proc = None

    def _pose_cb(self, arm):
        def cb(m):
            self.poses[arm] = msg_to_pose(m)
        return cb

    def listen(self, topic, msg_type):
        """Record every message on a topic as (arrival time, message)."""
        self.inbox[topic] = []
        self.node.create_subscription(msg_type, topic, lambda m: self.inbox[topic].append((time.monotonic(), m)), 1000)

    def wait(self, seconds):
        minros.spin_until(self.node, lambda: False, timeout_sec=seconds)

    def teleops(self, enabled):
        for t in ("MTMR_PSM1", "MTML_PSM2"):
            self.node.set_remote_parameter(t, "enabled", enabled)
        self.wait(0.3)

    def move(self, arm, goal_mm, speed_mm_s=40.0):
        """Straight line to a point (mm), keeping the orientation."""
        p, R = self.poses[arm]
        goal = np.array(goal_mm) / 1000.0
        n = max(1, int(np.linalg.norm(goal - p) / (speed_mm_s / 1000.0) / 0.01))
        for k in range(1, n + 1):
            self.cmd[arm].publish(pose_to_msg(p + (goal - p) * k / n, R, "world"))
            self.wait(0.01)
        self.wait(0.4)

    def start_student(self, path, *args):
        env = os.environ.copy()
        env["PYTHONPATH"] = ROOT + os.pathsep + env.get("PYTHONPATH", "")
        print(f"  starting {os.path.relpath(path, ROOT)} {' '.join(args)}")
        self.proc = subprocess.Popen([sys.executable, path, *args], cwd=ROOT, env=env,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        return self.proc

    def alive(self, settle=1.5):
        self.wait(settle)
        if self.proc.poll() is not None:
            print(f"  {BAD} your node stopped. Its output:\n" + self.proc.stdout.read())
            return False
        return True

    def stop_student(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def close(self):
        self.stop_student()
        self.teleops(True)
        self.node.destroy_node()


def result(ok, text):
    print(f"  {OK if ok else BAD} {text}")
    return ok


def values_between(h, topic, t0, t1):
    return [m.data for t, m in h.inbox[topic] if t0 <= t <= t1]


def check_b1(h, path):
    h.listen("/PSM1/tip_speed", Float64)
    h.start_student(path)
    if not h.alive():
        return False
    h.teleops(False)
    p0 = h.poses["PSM1"][0] * 1000
    h.wait(0.6)
    rest = values_between(h, "/PSM1/tip_speed", time.monotonic() - 0.5, time.monotonic())
    t0 = time.monotonic()
    h.move("PSM1", p0 + [30, 0, 0], speed_mm_s=20)
    t1 = time.monotonic()
    moving = values_between(h, "/PSM1/tip_speed", t0 + 0.4, t1 - 0.6)
    h.move("PSM1", p0, speed_mm_s=40)
    ok = result(len(h.inbox["/PSM1/tip_speed"]) > 0, "something is published on /PSM1/tip_speed")
    if not ok:
        return False
    rate = len(moving) / max(t1 - t0 - 1.0, 1e-3)
    ok &= result(rate > 40, f"publishes for every pose received (about {rate:.0f} Hz)")
    ok &= result(bool(rest) and float(np.median(rest)) < 1.0,
                 f"still arm: speed {np.median(rest) if rest else float('nan'):.2f} mm/s (expected about 0)")
    med = float(np.median(moving)) if moving else float("nan")
    ok &= result(abs(med - 20) < 3, f"moving at 20 mm/s: your node says {med:.1f} mm/s")
    return ok


def check_b2(h, path):
    h.listen("/safety/tip_distance", Float64)
    h.listen("/safety/too_close", Bool)
    h.start_student(path)
    if not h.alive():
        return False
    h.teleops(False)
    h.move("PSM2", [-15, 0, -110]); h.move("PSM1", [15, 0, -110])
    h.wait(1.0)
    now = time.monotonic()
    d = values_between(h, "/safety/tip_distance", now - 0.5, now)
    c = values_between(h, "/safety/too_close", now - 0.5, now)
    ok = result(len(d) >= 7, f"/safety/tip_distance at about 20 Hz ({len(d) * 2} Hz measured)")
    ok &= result(bool(d) and abs(d[-1] - 30) < 1, f"tips 30 mm apart: your node says {d[-1] if d else float('nan'):.1f} mm")
    ok &= result(bool(c) and c[-1] is False, f"too_close is false at 30 mm ({c[-1] if c else 'nothing received'})")
    h.move("PSM1", [-9, 0, -110], speed_mm_s=20)
    h.wait(0.8)
    now = time.monotonic()
    d = values_between(h, "/safety/tip_distance", now - 0.5, now)
    c = values_between(h, "/safety/too_close", now - 0.5, now)
    ok &= result(bool(d) and abs(d[-1] - 6) < 1, f"tips 6 mm apart: your node says {d[-1] if d else float('nan'):.1f} mm")
    ok &= result(bool(c) and c[-1] is True, f"too_close is true at 6 mm ({c[-1] if c else 'nothing received'})")
    h.move("PSM1", [20, 5, -110])
    return ok


def check_b3(h, path):
    p0 = h.poses["PSM1"][0] * 1000
    goal = np.clip(p0 + [25, -10, 8], [-40, -40, -125], [45, 40, -95])
    h.start_student(path, *(f"{v:.1f}" for v in goal))
    track = []
    t_end = time.monotonic() + 20
    while h.proc.poll() is None and time.monotonic() < t_end:
        h.wait(0.01)
        track.append((time.monotonic(), h.poses["PSM1"][0] * 1000))
    if h.proc.poll() is None:
        result(False, "the script did not finish within 20 s")
        return False
    out = h.proc.stdout.read()
    ok = result(h.proc.returncode == 0, "the script exits normally" + ("" if h.proc.returncode == 0 else ":\n" + out))
    h.wait(0.5)
    final = h.poses["PSM1"][0] * 1000
    err = float(np.linalg.norm(final - goal))
    ok &= result(err < 1.0, f"reaches the goal: {err:.2f} mm away")
    pts = np.array([p for _, p in track])
    ts = np.array([t for t, _ in track])
    if len(pts) > 10:
        line = (goal - p0) / max(np.linalg.norm(goal - p0), 1e-9)
        rel = pts - p0
        dev = np.linalg.norm(rel - np.outer(rel @ line, line), axis=1).max()
        ok &= result(dev < 1.5, f"straight line: largest deviation {dev:.2f} mm")
        idx = np.searchsorted(ts, ts + 0.1)
        sel = idx < len(ts)
        v = np.linalg.norm(pts[idx[sel]] - pts[sel], axis=1) / (ts[idx[sel]] - ts[sel])
        ok &= result(v.max() < 26, f"never faster than 20 mm/s (peak {v.max():.1f} mm/s)")
    h.teleop.clear()
    minros.spin_until(h.node, lambda: "enabled" in h.teleop, timeout_sec=1)
    ok &= result(h.teleop.get("enabled") is True, "gives the teleoperation back at the end")
    return ok


def check_b4(h, path):
    h.listen("/PSM1/servo_cp", PoseStamped)
    raw = h.node.create_publisher(PoseStamped, "/PSM1/servo_cp_raw", 10)
    h.start_student(path)
    if not h.alive():
        return False
    h.teleops(False)
    _, R = h.poses["PSM1"]
    cases = [([10, 5, -110], [10, 5, -110], "a pose inside the safe region passes unchanged"),
             ([10, 5, -140], [10, 5, -125], "below the virtual floor: z becomes -125 mm"),
             ([90, 5, -110], [60, 5, -110], "too far right: x becomes 60 mm"),
             ([-10, -80, -130], [-10, -50, -125], "outside in y and below the floor at once")]
    ok = True
    for i, (send, expect, text) in enumerate(cases):
        stamp = 1000.0 + i
        msg = pose_to_msg(np.array(send) / 1000, R, "world", stamp)
        h.inbox["/PSM1/servo_cp"] = []
        raw.publish(msg)
        minros.spin_until(h.node, lambda: any(m.header.stamp == stamp for _, m in h.inbox["/PSM1/servo_cp"]), 1.0)
        got = [m for _, m in h.inbox["/PSM1/servo_cp"] if m.header.stamp == stamp]
        if not got:
            ok &= result(False, f"{text}: nothing came out on /PSM1/servo_cp")
            continue
        p, Rg = msg_to_pose(got[0])
        good = np.allclose(p * 1000, expect, atol=0.01) and np.allclose(Rg, R, atol=1e-6) and got[0].header.frame_id == "world"
        ok &= result(good, f"{text}" + ("" if good else f" (got {np.round(p * 1000, 1)} mm)"))
    h.move("PSM1", h.poses["PSM1"][0] * 1000)
    return ok


def check_b5(h, path):
    ecm_pub = h.node.create_publisher(JointState, "/ECM/servo_jp", 10)
    h.start_student(path)
    if not h.alive():
        return False
    h.teleops(False)
    ok = True
    for p1, p2 in (([30, 15, -115], [-5, 20, -120]), ([10, -25, -105], [-35, -15, -118])):
        h.move("PSM1", p1); h.move("PSM2", p2)
        h.wait(3.0)
        cam, Rc = h.ecm
        mid = (np.array(p1) + np.array(p2)) / 2000
        to_mid = (mid - cam) / np.linalg.norm(mid - cam)
        angle = math.degrees(math.acos(np.clip(Rc[:, 2] @ to_mid, -1, 1)))
        ok &= result(angle < 3, f"camera aimed at the midpoint of the tips: {angle:.1f} degrees off")
    h.stop_student()
    ecm_pub.publish(JointState(name=["yaw", "pitch", "insertion", "roll"], position=[float(v) for v in ECM_HOME]))
    h.move("PSM1", [20, 5, -110]); h.move("PSM2", [-20, 5, -110])
    return ok


def load_module(path):
    spec = importlib.util.spec_from_file_location("student_b6", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def check_b6_offline(path):
    """Steps 3, 4 and 5: pure functions, tested without the simulator."""
    from dvrk_sim.kinematics import ECM_BASE, shaft_rotation
    mod = load_module(path)
    ok = True
    # step 3
    rng = np.random.default_rng(0)
    try:
        good = True
        for _ in range(50):
            target = rng.uniform([-0.05, -0.05, -0.13], [0.05, 0.05, -0.09])
            a = mod.aim_angles(target, ECM_BASE)
            if a is None:
                good = False
                break
            u = shaft_rotation(a[0], a[1])[:, 2]
            d = (target - ECM_BASE) / np.linalg.norm(target - ECM_BASE)
            good &= bool(np.allclose(u, d, atol=1e-6))
        ok &= result(good, "STEP 3 aim_angles points the camera exactly at the target")
    except Exception as e:
        ok &= result(False, f"STEP 3 aim_angles raised {type(e).__name__}: {e}")
    # step 4
    try:
        g = (0.10, 0.50)
        t1 = mod.update_goal(g, (0.13, 0.48), 0.05)
        t2 = mod.update_goal(g, (0.20, 0.50), 0.05)
        t3 = mod.update_goal(g, (0.10, 0.42), 0.05)
        good = tuple(t1) == g and np.allclose(t2, (0.20, 0.50)) and np.allclose(t3, (0.10, 0.42))
        ok &= result(good, "STEP 4 update_goal ignores small errors and follows big ones (yaw or pitch)")
    except Exception as e:
        ok &= result(False, f"STEP 4 update_goal raised {type(e).__name__}: {e}")
    # step 5
    try:
        c = mod.step_towards((0.0, 0.5), (1.0, 0.0), 0.4, 0.05)
        c2 = mod.step_towards((0.0, 0.5), (0.01, 0.495), 0.4, 0.05)
        good = np.allclose(c, (0.02, 0.48)) and np.allclose(c2, (0.01, 0.495))
        ok &= result(good, "STEP 5 step_towards limits the speed in both directions and stops on the goal"
                     + ("" if good else f" (got {np.round(c, 4)} and {np.round(c2, 4)})"))
    except Exception as e:
        ok &= result(False, f"STEP 5 step_towards raised {type(e).__name__}: {e}")
    return ok


def check_b6(h, path):
    ecm_pub = h.node.create_publisher(JointState, "/ECM/servo_jp", 10)
    set_active = h.node.create_publisher(String, "/console/set_active", 10)
    console = "console" in h.node.graph()["nodes"]
    own_active = None if console else h.node.create_publisher(String, "/console/active_arm", 10)
    state = {"arm": "PSM1"}

    def choose(arm):
        state["arm"] = arm
        set_active.publish(String(data=arm))
    if own_active:                                  # no console running: play its part
        h.node.create_timer(0.1, lambda: own_active.publish(String(data=state["arm"])))
    ecm_js = []
    h.node.create_subscription(JointState, "/ECM/measured_js",
                               lambda m: ecm_js.append((time.monotonic(), list(m.position))), 1000)

    def aimed_error(arm):
        cam, Rc = h.ecm
        tip = h.poses[arm][0]
        d = (tip - cam) / np.linalg.norm(tip - cam)
        return math.degrees(math.acos(np.clip(Rc[:, 2] @ d, -1, 1)))

    h.teleops(False)
    h.move("PSM1", [30, 15, -115]); h.move("PSM2", [-30, -5, -110])
    choose("PSM1")
    h.start_student(path)
    if not h.alive(0.5):
        return False
    h.wait(3.5)
    ok = result(aimed_error("PSM1") < 4, f"STEPS 1, 2, 6 the camera centres the active instrument PSM1 "
                                         f"({aimed_error('PSM1'):.1f} degrees off)")
    t0 = time.monotonic()
    choose("PSM2")
    h.wait(3.5)
    ok &= result(aimed_error("PSM2") < 4, f"STEP 2 after switching hand it centres PSM2 ({aimed_error('PSM2'):.1f} degrees off)")
    seg = [(t, q) for t, q in ecm_js if t >= t0]
    speeds = [max(abs(b[0] - a[0]), abs(b[1] - a[1])) / (tb - ta)
              for (ta, a), (tb, b) in zip(seg[:-5], seg[5:]) if tb > ta]
    peak = max(speeds) if speeds else 0.0
    ok &= result(0 < peak <= 0.5, f"STEP 5 the camera turns smoothly (peak {peak:.2f} rad/s, limit 0.4)")
    before = h.ecm[1][:, 2].copy()
    p = h.poses["PSM2"][0] * 1000
    h.move("PSM2", p + [3, 0, 0], speed_mm_s=10)
    h.wait(1.0)
    moved = math.degrees(math.acos(np.clip(before @ h.ecm[1][:, 2], -1, 1)))
    ok &= result(moved < 0.3, f"STEP 4 a 3 mm movement does not move the camera ({moved:.2f} degrees)")
    h.stop_student()
    if console:
        choose("PSM1")
    ecm_pub.publish(JointState(name=["yaw", "pitch", "insertion", "roll"], position=[float(v) for v in ECM_HOME]))
    h.move("PSM1", [20, 5, -110]); h.move("PSM2", [-20, 5, -110])
    return ok


CHECKS = {"B1": ("b1_tip_speed.py", check_b1), "B2": ("b2_safety_monitor.py", check_b2),
          "B3": ("b3_go_to_pose.py", check_b3), "B4": ("b4_guard.py", check_b4),
          "B5": ("b5_auto_camera.py", check_b5), "B6": ("b6_follow_hand.py", check_b6)}


def main():
    args = sys.argv[1:]
    if not args or args[0].upper() not in ["A", "ALL"] + list(CHECKS):
        sys.exit(__doc__)
    which = args[0].upper()
    custom = args[args.index("--file") + 1] if "--file" in args else None
    if which == "A":
        sys.exit(0 if check_a() else 1)
    names = list(CHECKS) if which == "ALL" else [which]
    if which == "B6":                               # steps 3-5 need no simulator
        path = os.path.abspath(custom or os.path.join(HERE, "b6_follow_hand.py"))
        print("\nB6, steps 3-5 (no simulator needed)")
        offline = check_b6_offline(path)
        if not broker.is_running():
            print("\nStart the simulator (python launch.py) to check steps 1, 2 and 6.")
            sys.exit(0 if offline else 1)
        if not offline:
            print("\nFix steps 3-5 first, then run the check again for steps 1, 2 and 6.")
            sys.exit(1)
    h = Harness()
    passed = []
    try:
        for name in names:
            default, fn = CHECKS[name]
            path = os.path.abspath(custom or os.path.join(HERE, default))
            print(f"\n{name}")
            try:
                good = fn(h, path)
            finally:
                h.stop_student()
            print(f"{name}: {'PASSED' if good else 'not yet, read the lines marked ' + BAD}")
            passed.append(good)
    except KeyboardInterrupt:
        pass
    finally:
        h.close()
    sys.exit(0 if passed and all(passed) else 1)


if __name__ == "__main__":
    main()
