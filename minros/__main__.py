"""Command-line tools, modelled on the `ros2` command.

    python -m minros broker                      start the broker (ROS master)
    python -m minros node list                   ros2 node list
    python -m minros topic list [-v]             ros2 topic list
    python -m minros topic info /PSM1/measured_cp
    python -m minros topic echo /PSM1/measured_cp [--once] [--field pose.position]
    python -m minros topic hz /PSM1/measured_cp
    python -m minros topic pub /footpedals/clutch Bool '{"data": true}' [-r 10]
    python -m minros param set MTMR_PSM1 scale 2
    python -m minros graph                       a text version of rqt_graph
"""
import argparse
import json
import os
import sys
import time

from . import core
from .msg import get_type, from_dict, to_dict, known_types


def _cli_node():
    return core.Node(f"_cli_{os.getpid()}", connect_timeout=2.0)


def _graph():
    n = _cli_node()
    g = n.graph()
    n.destroy_node()
    g["nodes"] = [x for x in g["nodes"] if not x.startswith("_")]
    g["topics"] = {k: v for k, v in g["topics"].items() if not k.startswith("/_")}
    for t in g["topics"].values():
        t["publishers"] = [x for x in t["publishers"] if not x.startswith("_")]
        t["subscribers"] = [x for x in t["subscribers"] if not x.startswith("_")]
    return g


def _yaml(value, indent=0):
    pad = "  " * indent
    out = []
    if isinstance(value, dict):
        for k, v in value.items():
            if isinstance(v, dict) or (isinstance(v, list) and v and isinstance(v[0], (dict, list))):
                out.append(f"{pad}{k}:")
                out.append(_yaml(v, indent + 1))
            else:
                out.append(f"{pad}{k}: {_fmt(v)}")
    elif isinstance(value, list):
        for v in value:
            if isinstance(v, (dict, list)):
                out.append(f"{pad}-")
                out.append(_yaml(v, indent + 1))
            else:
                out.append(f"{pad}- {_fmt(v)}")
    else:
        out.append(f"{pad}{_fmt(value)}")
    return "\n".join(out)


def _fmt(v):
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, list):
        return "[" + ", ".join(_fmt(x) for x in v) + "]"
    return str(v)


def _topic_type(topic):
    g = _graph()
    if topic not in g["topics"]:
        sys.exit(f"topic {topic} does not exist. Try: python -m minros topic list")
    return get_type(g["topics"][topic]["type"])


def cmd_node_list(a):
    for n in _graph()["nodes"]:
        print("/" + n)


def cmd_topic_list(a):
    for t, e in _graph()["topics"].items():
        print(f"{t} [{e['type']}]" if a.verbose else t)


def cmd_topic_info(a):
    e = _graph()["topics"].get(a.topic)
    if not e:
        sys.exit(f"topic {a.topic} does not exist")
    print(f"Type: {e['type']}")
    print(f"Publishers ({len(e['publishers'])}): " + ", ".join("/" + x for x in e["publishers"]))
    print(f"Subscribers ({len(e['subscribers'])}): " + ", ".join("/" + x for x in e["subscribers"]))


def cmd_topic_echo(a):
    cls = _topic_type(a.topic)
    node = _cli_node()
    state = {"done": False}

    def cb(msg):
        d = to_dict(msg)
        if a.field:
            for part in a.field.split("."):
                d = d[part]
        print(_yaml(d) if isinstance(d, (dict, list)) else _fmt(d))
        print("---", flush=True)
        if a.once:
            state["done"] = True

    node.create_subscription(cls, a.topic, cb, 1 if a.once else 10)
    try:
        core.spin_until(node, lambda: state["done"])
    except KeyboardInterrupt:
        pass


def cmd_topic_hz(a):
    cls = _topic_type(a.topic)
    node = _cli_node()
    stamps = []

    def cb(msg):
        stamps.append(time.monotonic())
        del stamps[:-a.window]

    def report():
        if len(stamps) < 2:
            print("no new messages")
            return
        d = [b - c for b, c in zip(stamps[1:], stamps[:-1])]
        mean = sum(d) / len(d)
        print(f"average rate: {1 / mean:.1f} Hz   min: {min(d) * 1000:.1f} ms  max: {max(d) * 1000:.1f} ms  window: {len(stamps)}", flush=True)

    node.create_subscription(cls, a.topic, cb, 1000)
    node.create_timer(1.0, report)
    core.spin(node)


def cmd_topic_pub(a):
    cls = get_type(a.type)
    msg = from_dict(cls, json.loads(a.values)) if a.values else cls()
    node = _cli_node()
    pub = node.create_publisher(cls, a.topic, 10)
    time.sleep(0.2)
    if a.rate <= 0:
        pub.publish(msg)
        print(f"published once on {a.topic}")
        time.sleep(0.2)
        return
    print(f"publishing on {a.topic} at {a.rate} Hz (Ctrl+C to stop)")
    node.create_timer(1.0 / a.rate, lambda: pub.publish(msg))
    core.spin(node)


def cmd_param_set(a):
    node = _cli_node()
    time.sleep(0.1)
    node.set_remote_parameter(a.node, a.name, a.value)
    time.sleep(0.2)
    print(f"requested /{a.node.strip('/')} {a.name} = {a.value} (the node prints a confirmation)")


def cmd_graph(a):
    g = _graph()
    for t, e in g["topics"].items():
        pubs = ", ".join("/" + x for x in e["publishers"]) or "(nobody)"
        subs = ", ".join("/" + x for x in e["subscribers"]) or "(nobody)"
        print(f"{pubs:>24}  --[{t}]-->  {subs}")


def cmd_types(a):
    print("\n".join(known_types()))


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m minros", description="minros command-line tools")
    sp = p.add_subparsers(dest="cmd", required=True)
    sp.add_parser("broker", help="start the broker").set_defaults(fn=lambda a: __import__("minros.broker", fromlist=["serve"]).serve())

    node = sp.add_parser("node").add_subparsers(dest="sub", required=True)
    node.add_parser("list").set_defaults(fn=cmd_node_list)

    topic = sp.add_parser("topic").add_subparsers(dest="sub", required=True)
    t = topic.add_parser("list"); t.add_argument("-v", "--verbose", action="store_true"); t.set_defaults(fn=cmd_topic_list)
    t = topic.add_parser("info"); t.add_argument("topic"); t.set_defaults(fn=cmd_topic_info)
    t = topic.add_parser("echo"); t.add_argument("topic"); t.add_argument("--once", action="store_true")
    t.add_argument("--field", help="print one field, e.g. pose.position"); t.set_defaults(fn=cmd_topic_echo)
    t = topic.add_parser("hz"); t.add_argument("topic"); t.add_argument("--window", type=int, default=200); t.set_defaults(fn=cmd_topic_hz)
    t = topic.add_parser("pub"); t.add_argument("topic"); t.add_argument("type"); t.add_argument("values", nargs="?", default="")
    t.add_argument("-r", "--rate", type=float, default=0.0); t.set_defaults(fn=cmd_topic_pub)

    param = sp.add_parser("param").add_subparsers(dest="sub", required=True)
    t = param.add_parser("set"); t.add_argument("node"); t.add_argument("name"); t.add_argument("value"); t.set_defaults(fn=cmd_param_set)

    sp.add_parser("graph").set_defaults(fn=cmd_graph)
    sp.add_parser("types", help="list message types").set_defaults(fn=cmd_types)

    a = p.parse_args(argv)
    try:
        a.fn(a)
    except ConnectionError as e:
        sys.exit(str(e))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
