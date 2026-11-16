"""The minros broker: the equivalent of the ROS master plus the transport.

Every node opens one TCP connection to the broker and announces the topics it
publishes and subscribes to. The broker forwards each published message to
all subscribers of that topic and can describe the whole graph (nodes,
topics, types) to the command-line tools.

Real ROS 1 uses a master only for discovery and connects nodes directly;
ROS 2 uses DDS discovery with no master at all. A central broker keeps this
teaching version small and easy to read.
"""
import json
import socket
import socketserver
import threading
import itertools

from .core import HOST, PORT


class _Client:
    _ids = itertools.count(1)

    def __init__(self, sock):
        self.id = next(self._ids)
        self.sock = sock
        self.node = f"_unnamed_{self.id}"
        self.wlock = threading.Lock()
        self.alive = True

    def send(self, data: bytes):
        if not self.alive:
            return
        try:
            with self.wlock:
                self.sock.sendall(data)
        except OSError:
            self.alive = False


class _Graph:
    def __init__(self):
        self.lock = threading.Lock()
        self.clients = {}                 # id -> _Client
        self.topics = {}                  # name -> {"type", "pubs": set(ids), "subs": set(ids)}

    def register(self, topic, mtype, client, role):
        with self.lock:
            entry = self.topics.setdefault(topic, {"type": mtype, "pubs": set(), "subs": set()})
            if entry["type"] != mtype:
                return f"topic {topic} has type {entry['type']}, node '{client.node}' used {mtype}"
            entry[role].add(client.id)
            return None

    def remove(self, client):
        with self.lock:
            self.clients.pop(client.id, None)
            for name in list(self.topics):
                t = self.topics[name]
                t["pubs"].discard(client.id)
                t["subs"].discard(client.id)
                if not t["pubs"] and not t["subs"]:
                    del self.topics[name]

    def subscribers(self, topic):
        with self.lock:
            t = self.topics.get(topic)
            if not t:
                return []
            return [self.clients[i] for i in t["subs"] if i in self.clients]

    def describe(self):
        with self.lock:
            name = lambda i: self.clients[i].node if i in self.clients else "?"
            return {
                "nodes": sorted(c.node for c in self.clients.values()),
                "topics": {
                    t: {"type": e["type"],
                        "publishers": sorted(name(i) for i in e["pubs"]),
                        "subscribers": sorted(name(i) for i in e["subs"])}
                    for t, e in sorted(self.topics.items())
                },
            }


GRAPH = _Graph()


def _error(client, text):
    client.send((json.dumps({"op": "error", "msg": text}) + "\n").encode())


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client = _Client(self.request)
        with GRAPH.lock:
            GRAPH.clients[client.id] = client
        try:
            for raw in self.rfile:
                line = raw.decode("utf-8").strip()
                if not line:
                    continue
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                op = m.get("op")
                if op == "pub":
                    err = GRAPH.register(m["topic"], m["type"], client, "pubs")
                    if err:
                        _error(client, err)
                        continue
                    # Re-use the incoming text: only the operation name changes.
                    out = (line.replace('"op":"pub"', '"op":"msg"', 1) + "\n").encode()
                    for sub in GRAPH.subscribers(m["topic"]):
                        sub.send(out)
                elif op == "hello":
                    client.node = m.get("node", client.node)
                elif op in ("adv", "sub"):
                    err = GRAPH.register(m["topic"], m["type"], client, "pubs" if op == "adv" else "subs")
                    if err:
                        _error(client, err)
                elif op == "graph":
                    client.send((json.dumps({"op": "graph", **GRAPH.describe()}) + "\n").encode())
        except (OSError, UnicodeDecodeError):
            pass
        finally:
            client.alive = False
            GRAPH.remove(client)


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def is_running(host=HOST, port=PORT):
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


def serve(host=HOST, port=PORT):
    """Run the broker in the foreground (blocking)."""
    if is_running(host, port):
        print(f"[minros] a broker is already running on {host}:{port}")
        return
    with _Server((host, port), _Handler) as srv:
        print(f"[minros] broker listening on {host}:{port} (Ctrl+C to stop)", flush=True)
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass
