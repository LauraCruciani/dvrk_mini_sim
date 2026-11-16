"""Classic surgeon console drawn with Matplotlib (python launch.py --classic).

The default console is the web one (endoscope view, web_console.py); this
one needs no browser and shows the scene from outside, which is useful to
explain the geometry.

Two masters: MTML drives PSM2, MTMR drives PSM1.

The window has three panels.
  Top left     the two master manipulators (MTMs), drawn from their joints.
  Bottom left  the hand pad: move the mouse here to move the ACTIVE hand.
               Movement is relative, like a trackpad. Tab or X switches hand.
  Right        the two instruments inside the patient.

Mouse and keys (they act on the active hand)
  mouse over the pad    move left/right and up/down
  wheel, W / S          depth (forward / back)
  left button held      close the jaws         G   keep the jaws closed (toggle)
  J L / I K / U O       rotate the wrist       H   reset the wrist
Keys for the whole console
  Tab or X   switch hand        C or space   clutch pedal (both arms)
  1 2 3 5    motion scale       T  tremor     F  tremor filter
  + / -      delay +-50 ms      N  reach task  B  handover task

The minros node behind the window is in console_node.py.
"""
import collections
import json
import math
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import minros
from .kinematics import JAW_LENGTH, PSM_BASES, mtm_forward_points, mtm_hand_rotation
from .console_node import ConsoleNode, SIDES, ARM, TELEOP, CONSOLE_X, HAND_X, HAND_Y, HAND_Z

COLOR = {"PSM1": "#1E6B5C", "PSM2": "#6A4C93", "MTMR": "#1E6B5C", "MTML": "#6A4C93"}
FLOOR = -130.0                                       # mm, tissue surface
MM = 1000.0


def _line3(ax, color, **kw):
    return ax.plot([], [], [], color=color, **kw)[0]


class ConsoleGUI:
    def __init__(self, node):
        self.node = node
        self.last_mouse = None
        for k in list(plt.rcParams):
            if k.startswith("keymap."):
                plt.rcParams[k] = []
        self.fig = plt.figure(figsize=(14, 8.2))
        try:
            self.fig.canvas.manager.set_window_title("dVRK mini simulator - surgeon console")
        except Exception:
            pass
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1, 1.45], height_ratios=[1.3, 1],
                                   left=0.03, right=0.99, top=0.93, bottom=0.12, wspace=0.04, hspace=0.12)
        self.ax_m = self.fig.add_subplot(gs[0, 0], projection="3d")
        self.ax_p = self.fig.add_subplot(gs[1, 0])
        self.ax = self.fig.add_subplot(gs[:, 1], projection="3d")
        self._setup_masters()
        self._setup_pad()
        self._setup_scene()
        self.hud = self.fig.text(0.03, 0.012, "", family="monospace", fontsize=9.5, va="bottom")
        self.fig.text(0.03, 0.965, "Tab: switch hand   mouse on the pad: move   wheel: depth   left button: jaws   "
                      "C/space: clutch   1 2 3 5: scale   T: tremor   F: filter   + -: delay   N: reach task   B: handover task",
                      fontsize=9, color="#555")
        c = self.fig.canvas
        c.mpl_connect("motion_notify_event", self.on_move)
        c.mpl_connect("axes_leave_event", lambda e: setattr(self, "last_mouse", None))
        c.mpl_connect("scroll_event", self.on_scroll)
        c.mpl_connect("button_press_event", self.on_press)
        c.mpl_connect("button_release_event", self.on_release)
        c.mpl_connect("key_press_event", self.on_key)
        self.timer = c.new_timer(interval=50)
        self.timer.add_callback(self.redraw)
        self.timer.start()

    # ------------------------------------------------------------- set-up
    def _setup_masters(self):
        a = self.ax_m
        a.set_xlim(-0.36, 0.36); a.set_ylim(-0.14, 0.50); a.set_zlim(-0.16, 0.36)
        try:
            a.set_box_aspect((0.72, 0.64, 0.52), zoom=1.25)
        except TypeError:
            a.set_box_aspect((0.72, 0.64, 0.52))
        a.view_init(elev=24, azim=-118)
        a.set_xticks([]); a.set_yticks([]); a.set_zticks([])
        a.set_title("Masters (MTML, MTMR) seen from behind the surgeon", fontsize=11, loc="left")
        self.m_art = {}
        for s in SIDES:
            c = COLOR[s]
            arts = {
                "pillar": _line3(a, "#8a8f93", lw=6),
                "upper": _line3(a, c, lw=6, solid_capstyle="round"),
                "fore": _line3(a, c, lw=5, solid_capstyle="round"),
                "ring": _line3(a, "#444", lw=2),
                "handle": _line3(a, "#222", lw=4, solid_capstyle="round"),
                "f1": _line3(a, "#c9a227", lw=3), "f2": _line3(a, "#c9a227", lw=3),
                "box": _line3(a, c, lw=0.8, alpha=0.4, ls="--"),
            }
            arts["label"] = a.text(CONSOLE_X[s], 0.42, 0.40, s, color=c, fontsize=10, weight="bold", ha="center")
            self.m_art[s] = arts

    def _setup_pad(self):
        a = self.ax_p
        a.set_xlim(-0.35, 0.35); a.set_ylim(-0.162, 0.135)
        a.set_aspect("equal"); a.set_xticks([]); a.set_yticks([])
        a.set_title("Hand pad: move the mouse here (relative, like a trackpad)", fontsize=11, loc="left")
        self.rects, self.dots, self.htrails, self.depth = {}, {}, {}, {}
        for s in SIDES:
            cx, c = CONSOLE_X[s], COLOR[s]
            self.rects[s] = a.add_patch(Rectangle((cx - HAND_X, -HAND_Z), 2 * HAND_X, 2 * HAND_Z,
                                                  fill=True, fc="white", ec=c, lw=1.2))
            a.text(cx, HAND_Z - 0.012, f"{s} → {ARM[s]}", ha="center", va="top", color=c, fontsize=10, weight="bold")
            self.htrails[s], = a.plot([], [], color=c, alpha=0.4, lw=1)
            self.dots[s], = a.plot([], [], "o", ms=12, color=c, mec="white", mew=2)
            self.depth[s] = a.barh(-HAND_Z + 0.012, 0, left=cx, height=0.008, color="#A86C12")[0]
        self.pad_msg = a.text(0, -0.158, "", ha="center", va="bottom", fontsize=9.5)

    def _setup_scene(self):
        a = self.ax
        a.set_xlim(-80, 80); a.set_ylim(-70, 70); a.set_zlim(-140, 50)
        try:
            a.set_box_aspect((1.15, 1, 1.15), zoom=1.15)
        except TypeError:
            a.set_box_aspect((1.15, 1, 1.15))
        a.set_xlabel("x (mm)"); a.set_ylabel("y (mm)"); a.set_zlabel("z (mm)")
        a.view_init(elev=20, azim=-80)
        a.set_title("Endoscope view: PSM2 (left) and PSM1 (right). Drag to rotate", fontsize=11, loc="left")
        g = np.array([-80, 80])
        X, Y = np.meshgrid(g, np.array([-70, 70]))
        a.plot_surface(X, Y, np.zeros_like(X), color="#e9bca6", alpha=0.22, shade=False)
        a.plot_surface(X, Y, np.full_like(X, FLOOR, dtype=float), color="#d98b7b", alpha=0.18, shade=False)
        self.s_art = {}
        for arm, base in PSM_BASES.items():
            b = base * MM
            a.plot([b[0]], [b[1]], [b[2]], "+", ms=12, mew=2.5, color=COLOR[arm])
            a.text(b[0], b[1], 8, arm, color=COLOR[arm], fontsize=9, weight="bold", ha="center")
            self.s_art[arm] = {
                "out": _line3(a, "#2e353a", lw=6, solid_capstyle="round"),
                "in": _line3(a, COLOR[arm], lw=4.5, solid_capstyle="round"),
                "j1": _line3(a, "#3b4247", lw=3), "j2": _line3(a, "#3b4247", lw=3),
                "tip": a.plot([], [], [], "o", ms=4, color="black")[0],
                "drop": _line3(a, "#555", lw=1, ls=":"),
                "shadow": a.plot([], [], [], "x", ms=6, color=COLOR[arm])[0],
                "trail": _line3(a, COLOR[arm], lw=0.8, alpha=0.5),
            }
        self.future = a.plot([], [], [], "o", ms=8, color="#e0b000", alpha=0.35)[0]
        self.current = a.plot([], [], [], "o", ms=13, color="#e0b000", mec="black")[0]
        self.cur_drop = _line3(a, "#b38a00", lw=1, ls=":")
        self.beads = a.plot([], [], [], "o", ms=10, color="#f28c28", mec="#7a3d00")[0]
        self.beads_held = a.plot([], [], [], "o", ms=10, color="#ffb347", mec="black")[0]
        self.beads_done = a.plot([], [], [], "o", ms=9, color="#2bbf7a", mec="#0d5c36")[0]
        self.rings = []
        self.vessel_lines = []

    # -------------------------------------------------------------- events
    def on_move(self, e):
        if e.inaxes is not self.ax_p or e.xdata is None:
            self.last_mouse = None
            return
        if self.last_mouse is not None:
            dx, dz = e.xdata - self.last_mouse[0], e.ydata - self.last_mouse[1]
            if abs(dx) < 0.06 and abs(dz) < 0.06:         # ignore jumps when the cursor re-enters
                self.node.move_hand(dx, dz)
        self.last_mouse = (e.xdata, e.ydata)

    def on_scroll(self, e):
        self.node.move_depth(0.005 * (1 if e.button == "up" else -1))

    def on_press(self, e):
        if e.inaxes is self.ax_p and e.button == 1:
            self.node.set_grip(True)

    def on_release(self, e):
        if e.button == 1:
            self.node.set_grip(False)

    def on_key(self, e):
        if e.key:
            self.node.handle_key(e.key)

    # ------------------------------------------------------------- drawing
    def draw_master(self, s, q, hand, angles, grip, active):
        art = self.m_art[s]
        off = np.array([CONSOLE_X[s], 0.0, 0.0])
        S, E, H = (p + off for p in mtm_forward_points(q))
        R = mtm_hand_rotation(angles)
        set3 = lambda l, pts: l.set_data_3d(*np.array(pts).T)
        set3(art["pillar"], [S + [0, 0, 0.15], S])
        set3(art["upper"], [S, E])
        set3(art["fore"], [E, H])
        th = np.linspace(0, 2 * np.pi, 20)
        set3(art["ring"], [H + 0.035 * (math.cos(t) * R[:, 0] + math.sin(t) * R[:, 2]) for t in th])
        back = H - R[:, 1] * 0.08
        set3(art["handle"], [H, back])
        for name, sign in (("f1", 1), ("f2", -1)):
            d = -math.cos(grip / 2) * R[:, 1] + sign * math.sin(grip / 2) * R[:, 0]
            start = H - R[:, 1] * 0.015
            set3(art[name], [start, start + d * 0.065])
        bx = [[-HAND_X, -HAND_Y, -HAND_Z], [HAND_X, -HAND_Y, -HAND_Z], [HAND_X, HAND_Y, -HAND_Z],
              [-HAND_X, HAND_Y, -HAND_Z], [-HAND_X, -HAND_Y, -HAND_Z]]
        set3(art["box"], [np.array(p) + off for p in bx] if active else [off, off])
        for key in ("upper", "fore"):
            art[key].set_alpha(1.0 if active else 0.45)

    def draw_instrument(self, arm, tip, R, jaw, trail):
        art = self.s_art[arm]
        base = PSM_BASES[arm] * MM
        t = tip * MM
        wrist = t - R[:, 2] * JAW_LENGTH * MM
        u = (wrist - base) / max(np.linalg.norm(wrist - base), 1e-9)
        outer = base - u * 50
        art["out"].set_data_3d([outer[0], base[0]], [outer[1], base[1]], [outer[2], base[2]])
        art["in"].set_data_3d([base[0], wrist[0]], [base[1], wrist[1]], [base[2], wrist[2]])
        for name, s in (("j1", 1), ("j2", -1)):
            d = math.cos(jaw / 2) * R[:, 2] + s * math.sin(jaw / 2) * R[:, 0]
            e = wrist + d * JAW_LENGTH * MM * 1.6
            art[name].set_data_3d([wrist[0], e[0]], [wrist[1], e[1]], [wrist[2], e[2]])
        art["tip"].set_data_3d([t[0]], [t[1]], [t[2]])
        art["drop"].set_data_3d([t[0], t[0]], [t[1], t[1]], [t[2], FLOOR])
        art["shadow"].set_data_3d([t[0]], [t[1]], [FLOOR])
        if len(trail):
            art["trail"].set_data_3d(trail[:, 0], trail[:, 1], trail[:, 2])

    def redraw(self):
        n = self.node
        with n.lock:
            q = {s: n.mtm_q[s].copy() for s in SIDES}
            hand = {s: n.hand_shown[s].copy() for s in SIDES}
            raw = {s: n.hand[s].copy() for s in SIDES}
            angles = {s: list(n.angles[s]) for s in SIDES}
            tips = {a: None if n.tip[a] is None else (n.tip[a].copy(), n.R[a].copy()) for a in n.tip}
            trails = {a: np.array(n.trail[a]) if n.trail[a] else np.zeros((0, 3)) for a in n.trail}
            targets = [t * MM for t in n.targets]
            vessels = list(n.vessels)
        st = n.status
        now = time.monotonic()
        active = n.active

        # masters and pad
        for s in SIDES:
            self.draw_master(s, q[s], hand[s], angles[s], n.gripper(s), s == active)
            cx = CONSOLE_X[s]
            self.dots[s].set_data([cx + hand[s][0]], [hand[s][2]])
            self.dots[s].set_markersize(14 if s == active else 9)
            self.depth[s].set_x(cx)
            self.depth[s].set_width(raw[s][1])
            self.rects[s].set_linewidth(3 if s == active else 1)
            self.rects[s].set_facecolor("#fbefd9" if n.clutch else ("#f3faf8" if s == active else "white"))
        h = raw[active]
        at_edge = min(HAND_X - abs(h[0]), HAND_Z - abs(h[2])) < 0.004
        self.pad_msg.set_text("CLUTCH pressed: hands move, instruments stay" if n.clutch else
                              f"{active} at the edge of its workspace: use the clutch (C)" if at_edge else
                              f"active: {active} (Tab to switch)   depth {h[1] * MM:+.0f} mm"
                              + ("   jaws held closed (G)" if n.latched[active] else ""))

        # vessels, sent once by the scene
        if vessels and not self.vessel_lines:
            for pts, r in vessels:
                pts = pts * MM
                line, = self.ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color="#8b1020", lw=2 + r * MM * 1.2, alpha=0.85)
                self.ax.plot(pts[:, 0], pts[:, 1], np.full(len(pts), FLOOR), color="#8b1020", lw=1, alpha=0.25)
                self.vessel_lines.append(line)
        hot = now < n.contact_until
        for l in self.vessel_lines:
            l.set_color("#ff2a2a" if hot else "#8b1020")

        for arm, v in tips.items():
            if v is not None:
                self.draw_instrument(arm, v[0], v[1], n.jaw[arm], trails[arm])

        # reach targets
        if targets:
            c = targets[0]
            self.current.set_data_3d([c[0]], [c[1]], [c[2]])
            self.cur_drop.set_data_3d([c[0], c[0]], [c[1], c[1]], [c[2], FLOOR])
            f = np.array(targets[1:]) if len(targets) > 1 else np.zeros((0, 3))
            self.future.set_data_3d(f[:, 0], f[:, 1], f[:, 2])
        else:
            for l in (self.current, self.cur_drop, self.future):
                l.set_data_3d([], [], [])

        # beads and their rings
        beads = st.get("beads", []) if st.get("task") == "handover" else []
        groups = {"free": [], "held": [], "done": []}
        for b in beads:
            p = np.array(b["p"]) * MM
            if b["held"] and all(tips.get(a) is not None for a in b["held"]):
                p = np.mean([tips[a][0] for a in b["held"]], axis=0) * MM     # smooth: follow the jaws
            groups["done" if b["placed"] else "held" if b["held"] else "free"].append(p)
        for key, line in (("free", self.beads), ("held", self.beads_held), ("done", self.beads_done)):
            pts = np.array(groups[key]) if groups[key] else np.zeros((0, 3))
            line.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])
        goals = [np.array(b["goal"]) * MM for b in beads]
        while len(self.rings) < len(goals):
            self.rings.append(_line3(self.ax, "#2bbf7a", lw=2))
        th = np.linspace(0, 2 * np.pi, 24)
        for i, ring in enumerate(self.rings):
            if i < len(goals):
                g = goals[i]
                ring.set_data_3d(g[0] + 7 * np.cos(th), g[1] + 7 * np.sin(th), np.full_like(th, g[2]))
            else:
                ring.set_data_3d([], [], [])

        # heads-up display
        tp = n.teleop.get("MTMR", {})
        states = "  ".join(f"{TELEOP[s]}: {n.teleop.get(s, {}).get('state', '?'):<9}" for s in SIDES)
        lines = [f"{states}  scale {tp.get('scale', '?')}:1  tremor {'ON ' if n.tremor else 'off'}  "
                 f"filter {'ON ' if tp.get('tremor_filter') else 'off'}  delay {tp.get('latency_ms', 0):.0f} ms"]
        arm = ARM[active]
        mine = tips.get(arm)
        state = st.get("state", "no scene")
        if state == "running" and st.get("task") == "reach" and targets and tips:
            near = min((v for v in tips.values() if v is not None), key=lambda v: np.linalg.norm(v[0] * MM - targets[0]))
            d = targets[0] - near[0] * MM
            lines.append(f"reach: target {st['index'] + 1}/{st['n']}  time {st['time_s']:.1f} s  path {st['path_mm']:.0f} mm  "
                         f"contacts {st['contacts']}   nearest tip to target: x {d[0]:+.0f} y {d[1]:+.0f} z {d[2]:+.0f} mm")
        elif state == "running" and st.get("task") == "handover":
            placed = sum(b["placed"] for b in beads)
            hint = ""
            if mine is not None:
                cand = [b for b in beads if not b["placed"] and arm not in b["held"]]
                if cand:
                    b = min(cand, key=lambda b: np.linalg.norm(np.array(b["p"]) - mine[0]))
                    d = (np.array(b["p"]) - mine[0]) * MM
                    hint = f"   {arm} to nearest bead: x {d[0]:+.0f} y {d[1]:+.0f} z {d[2]:+.0f} mm"
            lines.append(f"handover: placed {placed}/{len(beads)}  time {st['time_s']:.1f} s  handovers {st['handovers']}  "
                         f"drops {st['drops']}  contacts {st['contacts']}{hint}")
        elif state == "done":
            extra = f", {st['handovers']} handovers, {st['drops']} drops" if st.get("task") == "handover" else ""
            lines.append(f"{st['task']} complete: {st['time_s']:.1f} s, path {st['path_mm']:.0f} mm, "
                         f"{st['contacts']} contact(s){extra}. N or B to try again.")
        else:
            lines.append("N: reach task (yellow targets, either arm)    B: handover task (pick with PSM2, "
                         "pass to PSM1, put down on the green ring)")
        self.hud.set_text("\n".join(lines))
        self.hud.set_color("#c0392b" if hot else "black")
        self.fig.canvas.draw_idle()


def main():
    minros.init()
    node = ConsoleNode()
    stop = threading.Event()

    def spin():
        try:
            while not stop.is_set():
                node.spin_once(0.02)
        except ConnectionError as e:
            node.get_logger().error(str(e))

    threading.Thread(target=spin, daemon=True).start()
    ConsoleGUI(node)
    print(__doc__)
    try:
        plt.show()
    finally:
        stop.set()
        node.set_clutch(True)          # leave the instruments still when the console closes
        time.sleep(0.05)
        node.destroy_node()


if __name__ == "__main__":
    main()
