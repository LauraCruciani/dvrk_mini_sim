"""Web surgeon console: the endoscope view, rendered in the browser.

    python -m dvrk_sim.web_console [--port 8080] [--no-browser]

This process is the minros node "console" (see console_node.py) plus a small
web server, a little like rosbridge in the ROS world: the browser receives
the robots' state through a WebSocket 50 times a second and renders the
camera view with three.js (WebGL); mouse and keyboard events travel back the
same way. Everything uses the Python standard library.
"""
import argparse
import base64
import functools
import hashlib
import http.server
import json
import os
import threading
import time
import webbrowser

import minros
from .console_node import ConsoleNode

WEB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MOUSE_GAIN = 0.0003          # metres of hand motion per pixel of mouse motion
CAMERA_GAIN = 0.002          # radians per pixel in camera mode


class WebSocket:
    """Just enough of RFC 6455 for text messages."""

    def __init__(self, sock, rfile):
        self.sock, self.rfile = sock, rfile
        self.lock = threading.Lock()
        self.alive = True

    def send(self, text):
        data = text.encode("utf-8")
        n = len(data)
        if n < 126:
            head = bytes([0x81, n])
        elif n < 65536:
            head = bytes([0x81, 126]) + n.to_bytes(2, "big")
        else:
            head = bytes([0x81, 127]) + n.to_bytes(8, "big")
        try:
            with self.lock:
                self.sock.sendall(head + data)
        except OSError:
            self.alive = False

    def _read(self, n):
        data = self.rfile.read(n)
        if len(data) < n:
            raise EOFError
        return data

    def recv(self):
        """Return the next text message, or None when the browser closes the page."""
        while True:
            h = self._read(2)
            op, n, masked = h[0] & 0x0F, h[1] & 0x7F, h[1] & 0x80
            if n == 126:
                n = int.from_bytes(self._read(2), "big")
            elif n == 127:
                n = int.from_bytes(self._read(8), "big")
            mask = self._read(4) if masked else b""
            payload = self._read(n)
            if masked:
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if op == 0x8:
                return None
            if op == 0x9:                                   # ping -> pong
                with self.lock:
                    self.sock.sendall(bytes([0x8A, len(payload)]) + payload)
            elif op == 0x1:
                return payload.decode("utf-8")


class Console:
    def __init__(self, node):
        self.node = node
        self.clients = set()
        self.clients_lock = threading.Lock()

    def handle(self, m):
        n, t = self.node, m.get("t")
        if t == "move":
            if n.camera_mode:
                n.move_camera(dyaw=-m["dx"] * CAMERA_GAIN, dpitch=-m["dy"] * CAMERA_GAIN)
            else:
                n.move_hand(m["dx"] * MOUSE_GAIN, -m["dy"] * MOUSE_GAIN)
        elif t == "wheel":
            step = 1 if m["d"] < 0 else -1
            if n.camera_mode:
                n.move_camera(dins=0.004 * step)
            else:
                n.move_depth(0.005 * step)
        elif t == "nudge":
            dt = min(0.1, max(0.0, float(m.get("dt", 0.0))))
            n.nudge(float(m["right"]), float(m["forward"]), float(m["up"]), dt)
        elif t == "present":                               # head in / out of the viewer (pointer lock)
            n.set_present(bool(m.get("on")))
        elif t == "grip":
            n.set_grip(bool(m["down"]))
        elif t == "key":
            n.handle_key(str(m["k"]))

    def serve_client(self, ws):
        with self.clients_lock:
            self.clients.add(ws)
        try:
            while True:
                text = ws.recv()
                if text is None:
                    break
                try:
                    m = json.loads(text)
                except ValueError:
                    continue
                if m.get("t") == "hello":
                    ws.send(json.dumps(self.node.vessels_message()))
                else:
                    self.handle(m)
        except (EOFError, OSError):
            pass
        finally:
            with self.clients_lock:
                self.clients.discard(ws)
                nobody = not self.clients
            self.node.set_grip(False)
            if nobody:
                self.node.set_present(False)                   # the page was closed: nobody at the console

    def broadcast_forever(self):
        while True:
            time.sleep(0.02)                                # 50 Hz
            with self.clients_lock:
                clients = list(self.clients)
            if not clients:
                continue
            text = json.dumps(self.node.snapshot(), separators=(",", ":"))
            for ws in clients:
                ws.send(text)


def make_handler(console):
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=WEB_DIR, **kw)

        def end_headers(self):
            self.send_header("Cache-Control", "no-store")
            super().end_headers()

        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path.startswith("/ws") and self.headers.get("Upgrade", "").lower() == "websocket":
                key = self.headers["Sec-WebSocket-Key"]
                accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
                self.send_response(101, "Switching Protocols")
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                http.server.BaseHTTPRequestHandler.end_headers(self)
                self.wfile.flush()
                self.close_connection = True
                console.serve_client(WebSocket(self.connection, self.rfile))
            else:
                super().do_GET()
    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description="web surgeon console")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)

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
    console = Console(node)
    threading.Thread(target=console.broadcast_forever, daemon=True).start()
    server = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(console))
    server.daemon_threads = True
    url = f"http://127.0.0.1:{args.port}/"
    node.get_logger().info(f"surgeon console at {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        node.set_clutch(True)
        time.sleep(0.05)
        server.server_close()
        node.destroy_node()


if __name__ == "__main__":
    main()
