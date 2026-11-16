"""minros: a tiny, readable imitation of the ROS 2 Python API (rclpy).

    import minros
    from minros.msg import String

    minros.init()
    node = minros.Node("talker")
    pub = node.create_publisher(String, "/chatter", 10)
    node.create_timer(0.5, lambda: pub.publish(String(data="hello")))
    minros.spin(node)

The method names follow rclpy so that code written here can be ported to a
real ROS 2 / dVRK setup by changing the imports and the message packages.
"""
import collections
import json
import os
import queue
import socket
import sys
import threading
import time

HOST = os.environ.get("MINROS_HOST", "127.0.0.1")
PORT = int(os.environ.get("MINROS_PORT", "11311"))   # same port as the ROS 1 master

_shutdown = threading.Event()


def init(args=None):
    _shutdown.clear()


def ok():
    return not _shutdown.is_set()


def shutdown():
    _shutdown.set()


def _json_default(o):
    if hasattr(o, "item"):
        return o.item()          # numpy scalars
    if hasattr(o, "tolist"):
        return o.tolist()        # numpy arrays
    raise TypeError(f"cannot serialise {type(o).__name__}")


def _norm(topic):
    return "/" + topic.strip().strip("/")


# ------------------------------------------------------------------ logging
class Logger:
    def __init__(self, name):
        self.name = name
        self._last = {}

    def _log(self, level, text, throttle_s):
        if throttle_s:
            now = time.monotonic()
            if now - self._last.get(text, -1e9) < throttle_s:
                return
            self._last[text] = now
        stream = sys.stderr if level in ("WARN", "ERROR") else sys.stdout
        print(f"[{level}] [{time.time():.3f}] [{self.name}]: {text}", file=stream, flush=True)

    def info(self, text, throttle_duration_sec=None):
        self._log("INFO", text, throttle_duration_sec)

    def warn(self, text, throttle_duration_sec=None):
        self._log("WARN", text, throttle_duration_sec)

    warning = warn

    def error(self, text, throttle_duration_sec=None):
        self._log("ERROR", text, throttle_duration_sec)


# --------------------------------------------------------------- transport
class _Connection:
    def __init__(self, node_name, timeout):
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.sock = socket.create_connection((HOST, PORT), timeout=1.0)
                break
            except OSError:
                if time.monotonic() > deadline:
                    raise ConnectionError(
                        f"cannot reach the minros broker at {HOST}:{PORT}. "
                        "Start the simulator with 'python launch.py' or the broker with "
                        "'python -m minros broker'.") from None
                time.sleep(0.1)
        self.sock.settimeout(None)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.wlock = threading.Lock()
        self.inbox = queue.Queue()
        self.replies = queue.Queue()
        self.closed = False
        self._file = self.sock.makefile("r", encoding="utf-8")
        threading.Thread(target=self._reader, daemon=True).start()
        self.send({"op": "hello", "node": node_name})

    def send(self, obj):
        data = (json.dumps(obj, separators=(",", ":"), default=_json_default) + "\n").encode()
        with self.wlock:
            self.sock.sendall(data)

    def request(self, obj, timeout=2.0):
        self.send(obj)
        return self.replies.get(timeout=timeout)

    def _reader(self):
        try:
            for line in self._file:
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                op = m.get("op")
                if op == "msg":
                    self.inbox.put(m)
                elif op == "error":
                    print(f"[ERROR] [broker]: {m.get('msg')}", file=sys.stderr, flush=True)
                else:
                    self.replies.put(m)
        except (OSError, ValueError):
            pass
        self.closed = True
        self.inbox.put(None)

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


# ------------------------------------------------------- publishers & co.
class Publisher:
    def __init__(self, node, msg_type, topic):
        self._node, self.msg_type, self.topic = node, msg_type, topic

    def publish(self, msg):
        if not isinstance(msg, self.msg_type):
            raise TypeError(f"{self.topic} expects {self.msg_type.__name__}, got {type(msg).__name__}")
        from .msg import to_dict
        self._node._conn.send({"op": "pub", "topic": self.topic,
                               "type": self.msg_type.__name__, "data": to_dict(msg)})


class Subscription:
    def __init__(self, msg_type, topic, callback, qos):
        self.msg_type, self.topic, self.callback = msg_type, topic, callback
        self.qos = max(1, int(qos))


class Timer:
    def __init__(self, period, callback):
        self.period, self.callback = float(period), callback
        self.next = time.monotonic() + self.period
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


def _coerce(text, like):
    if isinstance(like, bool):
        if text.lower() in ("1", "true", "yes", "on"):
            return True
        if text.lower() in ("0", "false", "no", "off"):
            return False
        raise ValueError(f"'{text}' is not a boolean")
    if isinstance(like, int):
        return int(float(text))
    if isinstance(like, float):
        return float(text)
    return text


# --------------------------------------------------------------------- node
class Node:
    def __init__(self, node_name, connect_timeout=5.0):
        self._name = node_name.strip("/")
        self._log = Logger(self._name)
        self._conn = _Connection(self._name, connect_timeout)
        self._subs = collections.defaultdict(list)
        self._timers = []
        self._params = {}
        self._param_callbacks = []
        self._param_pubs = {}
        from .msg import String
        self.create_subscription(String, f"/{self._name}/set_parameter", self._on_set_parameter, 10)

    # basic info
    def get_name(self):
        return self._name

    def get_logger(self):
        return self._log

    def now(self):
        return time.time()

    # communication
    def create_publisher(self, msg_type, topic, qos=10):
        topic = _norm(topic)
        self._conn.send({"op": "adv", "topic": topic, "type": msg_type.__name__})
        return Publisher(self, msg_type, topic)

    def create_subscription(self, msg_type, topic, callback, qos=10):
        topic = _norm(topic)
        sub = Subscription(msg_type, topic, callback, qos)
        self._subs[topic].append(sub)
        self._conn.send({"op": "sub", "topic": topic, "type": msg_type.__name__})
        return sub

    def create_timer(self, period_sec, callback):
        t = Timer(period_sec, callback)
        self._timers.append(t)
        return t

    # parameters (set from outside by publishing "name=value" on /<node>/set_parameter)
    def declare_parameter(self, name, default):
        self._params.setdefault(name, default)
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    def set_parameter(self, name, value):
        self._params[name] = value
        for cb in self._param_callbacks:
            cb(name, value)

    def add_on_set_parameters_callback(self, callback):
        """callback(name, value) is called after every parameter change."""
        self._param_callbacks.append(callback)

    def _on_set_parameter(self, msg):
        if "=" not in msg.data:
            self._log.warn(f"parameter request must be 'name=value', got '{msg.data}'")
            return
        name, value = (s.strip() for s in msg.data.split("=", 1))
        if name not in self._params:
            self._log.warn(f"unknown parameter '{name}'. Known: {', '.join(self._params) or 'none'}")
            return
        try:
            value = _coerce(value, self._params[name])
        except ValueError as e:
            self._log.warn(f"cannot set '{name}': {e}")
            return
        self.set_parameter(name, value)
        self._log.info(f"parameter {name} = {value}")

    def set_remote_parameter(self, node_name, name, value):
        """Ask another node to change one of its parameters."""
        from .msg import String
        node_name = node_name.strip("/")
        if node_name not in self._param_pubs:
            self._param_pubs[node_name] = self.create_publisher(String, f"/{node_name}/set_parameter", 10)
        if isinstance(value, bool):
            value = "true" if value else "false"
        self._param_pubs[node_name].publish(String(data=f"{name}={value}"))

    # executor
    def spin_once(self, timeout_sec=0.1):
        """Deliver waiting messages and fire due timers, waiting at most timeout_sec."""
        from .msg import from_dict
        now = time.monotonic()
        due = [t.next for t in self._timers if not t.cancelled]
        wait = max(0.0, min([timeout_sec] + [d - now for d in due]))
        batch = []
        try:
            first = self._conn.inbox.get(timeout=wait) if wait > 0 else self._conn.inbox.get_nowait()
            batch.append(first)
        except queue.Empty:
            pass
        while True:
            try:
                batch.append(self._conn.inbox.get_nowait())
            except queue.Empty:
                break
        if None in batch:
            raise ConnectionError("lost the connection to the minros broker")
        per_topic = collections.OrderedDict()
        for m in batch:
            per_topic.setdefault(m["topic"], []).append(m)
        for topic, msgs in per_topic.items():
            for sub in self._subs.get(topic, ()):
                for m in msgs[-sub.qos:]:          # like a ROS queue: keep the newest `qos` messages
                    if m["type"] != sub.msg_type.__name__:
                        self._log.warn(f"{topic}: received {m['type']}, expected {sub.msg_type.__name__}",
                                       throttle_duration_sec=2)
                        continue
                    sub.callback(from_dict(sub.msg_type, m["data"]))
        now = time.monotonic()
        for t in self._timers:
            if not t.cancelled and now >= t.next:
                t.next += t.period
                if t.next < now:              # we fell behind: skip missed ticks
                    t.next = now + t.period
                t.callback()

    def graph(self):
        """Return the broker's view of the system (nodes and topics)."""
        return self._conn.request({"op": "graph"})

    def destroy_node(self):
        self._conn.close()


def spin(node):
    """Process callbacks until Ctrl+C, shutdown() or broker loss."""
    try:
        while ok():
            node.spin_once(0.1)
    except KeyboardInterrupt:
        pass
    except ConnectionError as e:
        node.get_logger().error(str(e))


def spin_once(node, timeout_sec=0.1):
    node.spin_once(timeout_sec)


def spin_until(node, condition, timeout_sec=None):
    """Spin until condition() is true. Returns False on timeout."""
    end = None if timeout_sec is None else time.monotonic() + timeout_sec
    while ok() and not condition():
        if end is not None and time.monotonic() > end:
            return False
        node.spin_once(0.02)
    return True
