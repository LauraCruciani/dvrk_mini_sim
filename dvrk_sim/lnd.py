"""The dVRK Large Needle Driver (instrument 400006) drawn from its official CAD.

Meshes and kinematic chain come from jhu-dvrk/dvrk_model (MIT license):
roll 4162, wrist pitch/yaw 0091, tip 006. They are downloaded and converted
once with

    python -m dvrk_sim.lnd

into web/assets/lnd/ (one mesh per link, already in its link frame).

The simulator keeps its own (simplified) kinematics; link_poses() places the
CAD links so that the jaw pivot is at the simulator's wrist and the tool tip
frame matches the simulator's tip pose. The mapping to the URDF joints is
exact for the orientation: roll_urdf = roll - 90 deg, pitch_urdf = wrist_pitch,
yaw_urdf = -wrist_yaw. The 9.1 mm between the pitch and yaw axes of the real
wrist is drawn, but the simulator rotates the jaws about a single point.
"""
import json
import os
import sys
import urllib.request

import numpy as np

from .kinematics import rx, ry, rz, shaft_rotation, forward, JAW_LENGTH

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "web", "assets", "lnd")
URL = "https://media.githubusercontent.com/media/jhu-dvrk/dvrk_model/main/meshes/instruments/"
P2 = np.pi / 2
PITCH_TO_YAW = 0.0091


def rpy(r, p, y):
    return rz(y) @ ry(p) @ rx(r)


J = rpy(-P2, -P2, 0.0)                     # origin rotation of the wrist_pitch and wrist_yaw joints
TIP = rpy(0.0, P2, P2)                     # wrist_yaw_link -> tool_tip_link
C = rz(P2)                                 # simulator tip frame -> URDF tool_tip frame
# link: (file, scale, visual origin xyz, visual origin rpy), all from the dvrk_model xacro files
PARTS = {
    "pitch": ("wrist_pitch/0091/wrist_pitch_0091.obj", 0.1, (0.0295, 0, 0), (0, P2, P2)),
    "yaw": ("wrist_yaw/0091/wrist_yaw_0091.obj", 1.0, (0, 0, -0.000236), (0, 0, 0)),
    "jaw1": ("tip/006/tip_006_1.obj", 1.0, (0, 0, 0), (0, 0, 0)),
    "jaw2": ("tip/006/tip_006_2.obj", 1.0, (0, 0, 0), (0, 0, 0)),
}


def read_obj(text):
    V, T = [], []
    for line in text.splitlines():
        if line.startswith("v "):
            V.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
            idx = [i - 1 if i > 0 else len(V) + i for i in idx]
            for k in range(1, len(idx) - 1):                 # fan triangulation
                T.append([idx[0], idx[k], idx[k + 1]])
    return np.array(V), np.array(T, dtype=np.int64)


def build(out=OUT, log=print):
    os.makedirs(out, exist_ok=True)
    meta = {}
    for name, (path, scale, xyz, ang) in PARTS.items():
        log(f"[lnd] downloading {path}")
        with urllib.request.urlopen(URL + path, timeout=60) as r:
            V, T = read_obj(r.read().decode("utf-8", "replace"))
        V = np.asarray(xyz) + (V * scale) @ rpy(*ang).T            # into the link frame
        with open(os.path.join(out, name + ".bin"), "wb") as f:
            f.write(V.astype("<f4").tobytes())
            f.write(T.astype("<u4").tobytes())
        meta[name] = {"vertices": len(V), "triangles": len(T),
                      "min": V.min(0).round(5).tolist(), "max": V.max(0).round(5).tolist()}
        log(f"[lnd]   {name}: {len(V)} vertices, {len(T)} triangles")
    meta["source"] = "jhu-dvrk/dvrk_model (MIT), Large Needle Driver 400006"
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)


def available():
    return os.path.exists(os.path.join(OUT, "meta.json"))


def link_poses(q, jaw, base):
    """World poses (p, R) of the CAD links, and the end of the shaft, for joints q and jaw angle."""
    q = np.asarray(q, dtype=float)
    tip, R, wrist = forward(q)
    roll_R = shaft_rotation(q[0], q[1]) @ rz(q[3]) @ rz(-P2)
    pitch_R = roll_R @ J @ rz(q[4])
    yaw_R = pitch_R @ J @ rz(-q[5])
    w = wrist + base
    pitch_p = w - pitch_R @ np.array([PITCH_TO_YAW, 0.0, 0.0])
    return {"pitch": (pitch_p, pitch_R), "yaw": (w, yaw_R),
            "jaw1": (w, yaw_R @ rz(jaw / 2)), "jaw2": (w, yaw_R @ rz(-jaw / 2)),
            "shaft_end": pitch_p}


if __name__ == "__main__":
    build()
