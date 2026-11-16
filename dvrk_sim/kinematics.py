"""Simplified kinematics of a dVRK Patient Side Manipulator (PSM).

The base frame is placed at the remote center of motion (RCM), the point
where the instrument crosses the abdominal wall. World axes: x to the right,
y forward (away from the camera), z up. The patient is below the RCM (z < 0).

Joints, with the dVRK names:
  0 outer_yaw          rotation of the shaft around y   (left / right)
  1 outer_pitch        rotation of the shaft around x   (forward / back)
  2 outer_insertion    distance from the RCM to the wrist, in metres
  3 outer_roll         rotation of the instrument around its own shaft
  4 outer_wrist_pitch  EndoWrist pitch
  5 outer_wrist_yaw    EndoWrist yaw
plus the jaw opening angle (handled separately, as on the real dVRK).

Whatever the first three joints do, the shaft always passes through the
origin: that is the RCM, guaranteed here by the geometry exactly as the
double parallelogram guarantees it on the real arm.
"""
import numpy as np

JOINT_NAMES = ["outer_yaw", "outer_pitch", "outer_insertion",
               "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw"]
LOWER = np.array([-1.20, -0.80, 0.020, -2.20, -1.40, -1.40])
UPPER = np.array([+1.20, +0.80, 0.240, +2.20, +1.40, +1.40])
JAW_LENGTH = 0.0102          # metres from the wrist to the jaw tip
JAW_LIMITS = (0.0, 1.2)      # rad


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def shaft_rotation(yaw, pitch):
    """Orientation of the shaft. Its z axis points into the patient."""
    return ry(yaw) @ rx(pitch) @ rx(np.pi)


def forward(q):
    """Forward kinematics: joints -> (tip position, tip rotation, wrist position)."""
    q = np.asarray(q, dtype=float)
    Rs = shaft_rotation(q[0], q[1])
    wrist = q[2] * Rs[:, 2]
    R = Rs @ rz(q[3]) @ rx(q[4]) @ ry(q[5])
    tip = wrist + R[:, 2] * JAW_LENGTH
    return tip, R, wrist


def inverse(p, R):
    """Inverse kinematics, closed form.

    Returns (q, within_limits). q is not clipped: the caller decides.
    """
    p = np.asarray(p, dtype=float)
    R = np.asarray(R, dtype=float)
    wrist = p - R[:, 2] * JAW_LENGTH
    d = np.linalg.norm(wrist)
    if d < 1e-6:
        raise ValueError("the wrist cannot be on the RCM")
    u = wrist / d
    # shaft direction u = [-sin(yaw)cos(pitch), sin(pitch), -cos(yaw)cos(pitch)]
    pitch = np.arcsin(np.clip(u[1], -1.0, 1.0))
    yaw = np.arctan2(-u[0], -u[2])
    # what is left for the instrument: M = Rz(roll) Rx(wrist_pitch) Ry(wrist_yaw)
    M = shaft_rotation(yaw, pitch).T @ R
    wp = np.arcsin(np.clip(M[2, 1], -1.0, 1.0))
    wy = np.arctan2(-M[2, 0], M[2, 2])
    roll = np.arctan2(-M[0, 1], M[1, 1])
    q = np.array([yaw, pitch, d, roll, wp, wy])
    return q, bool(np.all(q >= LOWER - 1e-9) and np.all(q <= UPPER + 1e-9))


# ----------------------------------------------------------- quaternions
def mat_to_quat(R):
    """Rotation matrix -> quaternion [x, y, z, w]."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_to_mat(q):
    """Quaternion [x, y, z, w] -> rotation matrix."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# ------------------------------------------------- message helpers
def pose_to_msg(p, R, frame_id="", stamp=None):
    from minros.msg import PoseStamped, Pose, Point, Quaternion, Header
    import time
    qx, qy, qz, qw = (float(v) for v in mat_to_quat(R))
    return PoseStamped(header=Header(stamp=time.time() if stamp is None else stamp, frame_id=frame_id),
                       pose=Pose(position=Point(float(p[0]), float(p[1]), float(p[2])),
                                 orientation=Quaternion(qx, qy, qz, qw)))


def msg_to_pose(msg):
    """PoseStamped or Pose -> (position array, rotation matrix)."""
    pose = msg.pose if hasattr(msg, "pose") else msg
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    o = pose.orientation
    return p, quat_to_mat([o.x, o.y, o.z, o.w])


# =================================================================== PSM bases
# Where each PSM's remote center of motion sits in the shared "world" frame
# (think of it as the camera / operating-table frame). PSM1 is on the right,
# PSM2 on the left, as seen from the console.
PSM_BASES = {"PSM1": np.array([0.05, 0.0, 0.0]), "PSM2": np.array([-0.05, 0.0, 0.0])}
PSM_HOME = {"PSM1": [0.30, 0.0, 0.12, 0.0, 0.0, 0.0], "PSM2": [-0.30, 0.0, 0.12, 0.0, 0.0, 0.0]}


# ======================================================================== MTM
# A simplified Master Tool Manipulator. Frame: origin at the centre of the
# hand workspace, x right, y forward (towards the screen), z up.
#   0 outer_yaw       the whole arm turns around a vertical axis
#   1 shoulder_pitch  upper arm (link length MTM_L1), angle from horizontal
#   2 elbow_pitch     forearm relative to the upper arm (MTM_L2)
#   3 wrist_platform  the gimbal platform turns to face the hand
#   4 wrist_pitch
#   5 wrist_yaw       kept near zero by the platform, as the dVRK does
#   6 wrist_roll      rotation of the handle
# plus the gripper (the two finger levers).
# Link lengths are close to the real MTM; joint conventions are simplified.
MTM_JOINT_NAMES = ["outer_yaw", "shoulder_pitch", "elbow_pitch", "wrist_platform",
                   "wrist_pitch", "wrist_yaw", "wrist_roll"]
MTM_L1, MTM_L2 = 0.2794, 0.3645
MTM_SHOULDER = np.array([0.0, 0.42, 0.20])


def mtm_hand_rotation(angles):
    """Hand orientation from (yaw, pitch, roll); the handle points along +y."""
    return rz(angles[0]) @ rx(angles[1]) @ ry(angles[2])


def mtm_inverse(p, angles):
    """Hand position + orientation angles -> 7 MTM joints, and whether it is reachable."""
    d = np.asarray(p, dtype=float) - MTM_SHOULDER
    yaw = np.arctan2(d[0], -d[1])
    r, h = np.hypot(d[0], d[1]), d[2]
    D = (r * r + h * h - MTM_L1 ** 2 - MTM_L2 ** 2) / (2 * MTM_L1 * MTM_L2)
    reachable = -1.0 <= D <= 1.0
    e = np.arccos(np.clip(D, -1.0, 1.0))
    shoulder = np.arctan2(h, r) - np.arctan2(MTM_L2 * np.sin(e), MTM_L1 + MTM_L2 * np.cos(e))
    q = np.array([yaw, shoulder, e, angles[0] - yaw, angles[1], 0.0, angles[2]])
    return q, bool(reachable)


def mtm_forward_points(q):
    """Shoulder, elbow and hand positions for drawing the MTM."""
    plane = np.array([np.sin(q[0]), -np.cos(q[0]), 0.0])     # horizontal direction of the arm
    up = np.array([0.0, 0.0, 1.0])
    elbow = MTM_SHOULDER + MTM_L1 * (np.cos(q[1]) * plane + np.sin(q[1]) * up)
    a = q[1] + q[2]
    hand = elbow + MTM_L2 * (np.cos(a) * plane + np.sin(a) * up)
    return MTM_SHOULDER.copy(), elbow, hand


# ======================================================================== ECM
# Endoscopic Camera Manipulator: the same RCM structure as a PSM, but it
# carries the stereo endoscope. Joints: yaw, pitch, insertion, roll.
# Camera frame (as on the dVRK): z = viewing direction, x = image right,
# y = image down.
ECM_JOINT_NAMES = ["yaw", "pitch", "insertion", "roll"]
ECM_BASE = np.array([0.0, -0.09, 0.0])
ECM_HOME = np.array([0.0, 0.64, 0.035, 0.0])
ECM_LOWER = np.array([-0.9, -0.2, 0.02, -1.2])
ECM_UPPER = np.array([+0.9, +1.0, 0.11, +1.2])


def ecm_forward(q):
    """ECM joints -> camera position (world) and camera rotation."""
    Rc = shaft_rotation(q[0], q[1]) @ rz(q[3])
    return ECM_BASE + q[2] * Rc[:, 2], Rc


def view_rotation(Rc):
    """Columns: image right, viewing direction, image up (the MTM's x, y, z)."""
    return np.column_stack([Rc[:, 0], Rc[:, 2], -Rc[:, 1]])
