# Exercises: reading and writing ROS nodes

Two parts. In **Part A** you read nodes that already exist, using the
command-line tools and the source code. In **Part B** you write new nodes,
starting from files with `TODO` comments. A checker tells you straight away
whether your work is right.

## How to work

Open two terminals in the `dvrk_mini_sim` folder.

```bash
# terminal 1: the simulator (keep it running)
python launch.py

# terminal 2: your work
python -m minros node list
python exercises/check.py A
```

Every concept here exists in ROS 2 under almost the same name. The table at
the end shows the correspondence.

---

## Part A: reading nodes

Write your answers in `exercises/answers_part_a.py`, then run
`python exercises/check.py A`. The checker tells you which answers are
right but not what the right answer is; each group has a hint.

### A1. The graph

Use `python -m minros graph`, `python -m minros node list`,
`python -m minros topic list -v` and `python -m minros topic info <topic>`.

- **A1_1** Which node publishes `/PSM1/servo_cp`?
- **A1_2** Which nodes are subscribed to `/footpedals/clutch`?
- **A1_3** What is the message type of `/scene/vessels`?
- **A1_4** How many nodes are running after `python launch.py`? (The
  command-line tools hide their own temporary node.)

### A2. Measuring rates

Use `python -m minros topic hz <topic>` for a few seconds, then give a round
number.

- **A2_1** How often is `/PSM1/measured_cp` published?
- **A2_2** And `/PSM1/measured_js`?
- **A2_3** And `/MTMR/measured_js`?

Think about it: why would a robot publish its pose more often than its
joints, or the masters' joints less often than their pose?

### A3. Reading `dvrk_sim/psm_node.py`

- **A3_1** At what frequency does the PSM control loop run?
- **A3_2** What is the `frame_id` of `/PSM1/measured_cp`?
- **A3_3** What is the `frame_id` of `/PSM1/local/measured_cp`?
- **A3_4** What is the maximum insertion speed of the instrument, in mm/s?
- **A3_5** What does the PSM do when it receives a `servo_cp` it cannot reach
  within its joint limits?
  a) it ignores the command and stays where it is;
  b) it goes as close as it can, limiting each joint, and prints a warning;
  c) it stops and switches to a fault state.

You can check A3_2 and A3_3 with
`python -m minros topic echo /PSM1/measured_cp --once --field header`.

### A4. Reading `dvrk_sim/teleop_node.py`

- **A4_1** At scale 3, the hand moves 30 mm. How far does the instrument move, in mm?
- **A4_2** The teleoperation re-anchors (it takes the current hand and
  instrument positions as the new starting point) when some events happen.
  Which ones? Answer with the letters, e.g. `"ace"`:
  a) the clutch pedal is pressed or released;
  b) the scale changes;
  c) the teleoperation is enabled or disabled;
  d) the camera moves;
  e) the jaws open or close;
  f) the delay changes.
- **A4_3** What is the order of the tremor filter (how many first-order
  stages in series)?
- **A4_4** At what frequency does the teleoperation loop run?

---

## Part B: writing nodes

Each file in `exercises/` starts with a description of the task. Fill in the
`TODO`s, run your node in terminal 2, and in a third terminal run the
checker, for example `python exercises/check.py B1`. The checker starts your
node by itself, so you can also just run the checker.

While it tests, the checker pauses the teleoperation and moves the arms on
its own; do not touch the console during that time.

| | File | You learn |
| --- | --- | --- |
| **B1** | `b1_tip_speed.py` | publisher, subscriber, callback, time stamps |
| **B2** | `b2_safety_monitor.py` | several subscriptions, timers, throttled logging |
| **B3** | `b3_go_to_pose.py` | commanding a robot, parameters of another node, a control loop, cleaning up with `finally` |
| **B4** | `b4_guard.py` | topic remapping, a node in the middle of a pipeline, virtual fixtures |
| **B5** | `b5_auto_camera.py` | a small piece of research: autonomous camera control (challenge) |
| **B6** | `b6_follow_hand.py` + `B6_GUIDE.md` | a guided project in six steps: a camera that follows the hand you are using |

### B1. The speed of the tool tip

Compute the speed of the PSM1 tip from consecutive poses and publish it in
mm/s. Use the time written in the message header, not the clock of your
computer: that is how it is done in ROS, because messages can arrive late or
in bursts. Watch your node with `python -m minros topic echo /PSM1/tip_speed`
while you move the right hand in the console.

### B2. A safety monitor

Watch both instruments and publish their distance and a warning flag. Then
try it at the console: bring the two instruments close and watch the
warning appear in the terminal of your node.

### B3. Go to a point

A script, not a node that runs for ever: it pauses the teleoperation,
drives PSM1 in a straight line at 20 mm/s and gives control back, even if
you stop it with Ctrl+C. This is the basic pattern of every "autonomous"
motion on the dVRK.

### B4. A virtual fixture

Start the simulator with `python launch.py --guard`: the right-hand
teleoperation now publishes on `/PSM1/servo_cp_raw` instead of
`/PSM1/servo_cp`. This is **remapping**: the teleoperation node is started
with `-r /PSM1/servo_cp:=/PSM1/servo_cp_raw`, and its code does not change.
Until your guard node runs, PSM1 does not move at all. With your node
running, try to push the instrument into the tissue: it stops 5 mm above it.

### B5. A camera that follows the instruments (challenge)

Keep the endoscope aimed at the midpoint of the two tips. Then try it at the
console with the handover task: is it more comfortable? What happens when
the instruments move quickly, and how would you make it smoother?

### B6. A camera that follows the hand you are using (guided)

A step-by-step project with its own guide, `exercises/B6_GUIDE.md`: listen to
the instruments and the camera, find out which hand is active, compute where
to point the endoscope, add a dead band and a speed limit, and send the
commands. Each step has a checkpoint, and `python exercises/check.py B6`
checks the steps one by one (steps 3 to 5 even without the simulator).

---

## minros and ROS 2

| minros | ROS 2 (rclpy) |
| --- | --- |
| `import minros` | `import rclpy` |
| `from minros.msg import PoseStamped` | `from geometry_msgs.msg import PoseStamped` |
| `from minros.msg import Float64, Bool, String` | `from std_msgs.msg import Float64, Bool, String` |
| `from minros.msg import JointState` | `from sensor_msgs.msg import JointState` |
| `minros.init()` | `rclpy.init()` |
| `node = minros.Node("name")` | `node = rclpy.create_node("name")` |
| `node.create_publisher(T, "/topic", 10)` | the same |
| `node.create_subscription(T, "/topic", cb, 10)` | the same |
| `node.create_timer(0.05, cb)` | the same |
| `node.get_logger().warn("...", throttle_duration_sec=1)` | the same |
| `minros.spin(node)` | `rclpy.spin(node)` |
| `msg.header.stamp` (seconds, a float) | `msg.header.stamp.sec`, `.nanosec` |
| `node.set_remote_parameter("node", "name", v)` | a call to the `/node/set_parameters` service |
| `python my_node.py -r /a:=/b` | `ros2 run pkg my_node --ros-args -r /a:=/b` |
