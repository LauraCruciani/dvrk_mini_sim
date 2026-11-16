"""Message types for minros.

They mirror the ROS message packages a dVRK user meets every day
(std_msgs, geometry_msgs, sensor_msgs). Each message is a plain Python
dataclass, serialised to JSON when it travels through the broker.
"""
from dataclasses import dataclass, field, fields, is_dataclass, asdict
import time

_REGISTRY = {}


def message(cls):
    """Turn a class into a minros message (a registered dataclass)."""
    cls = dataclass(cls)
    _REGISTRY[cls.__name__] = cls
    return cls


def get_type(name):
    """Return the message class registered under `name`."""
    try:
        return _REGISTRY[name]
    except KeyError:
        raise KeyError(f"unknown message type '{name}'. Known types: {', '.join(sorted(_REGISTRY))}")


def known_types():
    return sorted(_REGISTRY)


def to_dict(msg):
    return asdict(msg)


def from_dict(cls, data):
    """Rebuild a message object of type `cls` from a JSON dictionary."""
    kwargs = {}
    for f in fields(cls):
        if f.name not in data:
            continue
        value = data[f.name]
        elem = f.metadata.get("elem")
        if elem is not None and is_dataclass(elem):
            value = [from_dict(elem, v) for v in value]
        elif is_dataclass(f.type):
            value = from_dict(f.type, value)
        kwargs[f.name] = value
    return cls(**kwargs)


def _list_of(elem):
    return field(default_factory=list, metadata={"elem": elem})


# ---------------------------------------------------------------- std_msgs
@message
class Header:
    stamp: float = 0.0          # seconds since the epoch (ROS uses sec + nanosec)
    frame_id: str = ""

    @staticmethod
    def now(frame_id=""):
        return Header(stamp=time.time(), frame_id=frame_id)


@message
class Bool:
    data: bool = False


@message
class Int32:
    data: int = 0


@message
class Float64:
    data: float = 0.0


@message
class String:
    data: str = ""


# ----------------------------------------------------------- geometry_msgs
@message
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@message
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@message
class Wrench:
    force: Vector3 = field(default_factory=Vector3)       # newtons
    torque: Vector3 = field(default_factory=Vector3)      # newton metres


@message
class WrenchStamped:
    header: Header = field(default_factory=Header)
    wrench: Wrench = field(default_factory=Wrench)


@message
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@message
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@message
class PoseStamped:
    header: Header = field(default_factory=Header)
    pose: Pose = field(default_factory=Pose)


@message
class PoseArray:
    header: Header = field(default_factory=Header)
    poses: list = _list_of(Pose)


# ------------------------------------------------------------- sensor_msgs
@message
class JointState:
    header: Header = field(default_factory=Header)
    name: list = field(default_factory=list)
    position: list = field(default_factory=list)
    velocity: list = field(default_factory=list)
    effort: list = field(default_factory=list)


# ---------------------------------------------- simulator-specific messages
@message
class Polyline:
    """A tube in 3D (used for the blood vessels in the scene)."""
    points: list = _list_of(Point)
    radius: float = 0.0


@message
class PolylineArray:
    header: Header = field(default_factory=Header)
    lines: list = _list_of(Polyline)
