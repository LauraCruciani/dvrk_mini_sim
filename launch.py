"""Start the whole simulator, like a ROS launch file.

    python launch.py              robots + web console (endoscope view in your browser)
    python launch.py --classic    robots + Matplotlib console (scene seen from outside, no camera arm)
    python launch.py --no-gui     robots only, for scripted experiments
    python launch.py --port 8090  web console on another port

Each node runs in its own process, as it would on a real dVRK. Open another
terminal and try:  python -m minros topic list
"""
import argparse
import os
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from minros import broker  # noqa: E402

ROBOTS = [["dvrk_sim.psm_node", "PSM1"], ["dvrk_sim.psm_node", "PSM2"],
          ["dvrk_sim.teleop_node", "MTMR", "PSM1"], ["dvrk_sim.teleop_node", "MTML", "PSM2"],
          ["dvrk_sim.scene_node"]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--classic", action="store_true", help="Matplotlib console instead of the web one")
    ap.add_argument("--no-gui", action="store_true", help="no console at all")
    ap.add_argument("--port", type=int, default=8080, help="port of the web console")
    ap.add_argument("--no-browser", action="store_true", help="do not open the browser automatically")
    args = ap.parse_args()
    # stop the child processes cleanly also when this process is terminated
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    env = os.environ.copy()
    env["PYTHONPATH"] = ROOT + os.pathsep + env.get("PYTHONPATH", "")
    env["PYTHONUNBUFFERED"] = "1"
    procs = []
    if broker.is_running():
        print("[launch] using the broker that is already running")
    else:
        procs.append(subprocess.Popen([sys.executable, "-m", "minros", "broker"], cwd=ROOT, env=env))
        for _ in range(50):
            if broker.is_running():
                break
            time.sleep(0.1)
    nodes = ROBOTS + ([] if args.classic else [["dvrk_sim.ecm_node"]])
    for mod in nodes:
        procs.append(subprocess.Popen([sys.executable, "-m", *mod], cwd=ROOT, env=env))
    try:
        if args.no_gui:
            print("[launch] running without a console, Ctrl+C to stop")
            while all(p.poll() is None for p in procs):
                time.sleep(0.5)
        elif args.classic:
            from dvrk_sim.console_gui import main as console
            console()
        else:
            from dvrk_sim.web_console import main as console
            console(["--port", str(args.port)] + (["--no-browser"] if args.no_browser else []))
    except KeyboardInterrupt:
        pass
    finally:
        for p in reversed(procs):
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        print("[launch] all nodes stopped")


if __name__ == "__main__":
    main()
