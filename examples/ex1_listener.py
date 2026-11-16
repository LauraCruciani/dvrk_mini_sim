"""Exercise 1 - subscribers.

Print the tool-tip positions of both instruments twice a second.
Run the simulator first (python launch.py), then in another terminal:
    python examples/ex1_listener.py

ROS concepts: node, subscription, callback, timer.
Try it: also subscribe to /PSM1/local/measured_cp and compare it with
/PSM1/measured_cp. Why do they differ by exactly 50 mm along x?
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import minros
from minros.msg import PoseStamped

minros.init()
node = minros.Node("listener")
latest = {}


def make_callback(arm):
    def on_pose(msg):
        # called every time the arm publishes its pose (100 times a second)
        latest[arm] = msg.pose.position
    return on_pose


for arm in ("PSM1", "PSM2"):
    node.create_subscription(PoseStamped, f"/{arm}/measured_cp", make_callback(arm), 10)


def print_tips():
    if not latest:
        node.get_logger().info("waiting for /PSM1/measured_cp and /PSM2/measured_cp ...")
        return
    text = "   ".join(f"{a}: x={p.x * 1000:6.1f} y={p.y * 1000:6.1f} z={p.z * 1000:6.1f} mm"
                      for a, p in sorted(latest.items()))
    node.get_logger().info(text)


node.create_timer(0.5, print_tips)
minros.spin(node)
