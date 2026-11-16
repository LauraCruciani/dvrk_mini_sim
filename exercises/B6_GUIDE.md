# B6: a camera that follows the hand you are using

In the console, every time the instrument you are using leaves the image you
must stop, press the camera pedal, move the camera and start again. In this
exercise you write a node that does it for you: the endoscope turns by
itself to keep the **active instrument** (the one driven by the hand you
selected with Tab) in the middle of the image. Autonomous camera control is
an active research topic on the da Vinci Research Kit.

You work in one file, `exercises/b6_follow_hand.py`, in six steps. After each
step you can run the node and read its report, and check your work:

```bash
python launch.py                          # terminal 1, the simulator
python exercises/b6_follow_hand.py        # terminal 2, your node (Ctrl+C to stop)
python exercises/check.py B6              # terminal 3, the checker
```

The node prints a report every second, one line per step, saying `ok` or
`not yet`. Steps 3, 4 and 5 are plain functions: the checker tests them even
without the simulator.

## The big picture

```
 /PSM1/measured_cp ──┐
 /PSM2/measured_cp ──┤                       ┌───────────────┐
 /ECM/measured_js ───┼──►  follow_camera  ───►  /ECM/servo_jp ├──► ECM
 /console/active_arm ┘    (your node)        └───────────────┘
```

Every 1/20 of a second the node does the same four things:

1. take the position of the active instrument's tip;
2. compute the camera angles that would put that tip in the centre (step 3);
3. decide whether it is worth moving (step 4);
4. move a little towards those angles, never too fast (step 5), and send
   the command (step 6).

The skeleton already contains the loop (`tick`). You fill in the pieces.

---

## Step 1: listen

The node needs to know where the two tips are and where the camera points.

- Subscribe to `/PSM1/measured_cp` and `/PSM2/measured_cp` (PoseStamped) and
  store each tip as a numpy array `np.array([p.x, p.y, p.z])` in
  `self.tips["PSM1"]` and `self.tips["PSM2"]`. One callback per arm: write a
  method that *returns* a callback, as in B2.
- Subscribe to `/ECM/measured_js` (JointState) and store
  `list(msg.position)` in `self.ecm_q`. The four values are yaw, pitch,
  insertion and roll of the camera arm.

**Checkpoint.** Run the node: the line `STEP 1` says `ok` and shows the
camera's yaw and pitch. Move the camera at the console (arrow keys) and
watch the numbers change.

## Step 2: which hand are you using?

The console publishes `/console/active_arm` (String) ten times a second:
`"PSM1"` when the right hand is active, `"PSM2"` for the left. Subscribe to
it and store `msg.data` in `self.active`.

**Checkpoint.** `STEP 2` says `following PSM1`. Press Tab at the console: it
becomes `PSM2`.

> Think about it: the real dVRK has no "active hand" (the surgeon uses
> both). How else could the node guess which instrument matters? Ideas: the
> one that moved last, the one whose jaws are closed, the one closest to the
> edge of the image.

## Step 3: where should the camera point?

Like the instruments, the ECM pivots around its own remote center of
motion, the point where the endoscope crosses the abdominal wall
(`ECM_BASE`). It cannot slide sideways: it can only **turn** around that
point (yaw and pitch) and go in or out (insertion). So putting the tip in
the centre of the image means pointing the endoscope along the line from
`ECM_BASE` to the tip.

```
          ECM_BASE (RCM, in the wall)
              ●
               ╲   u = (tip - ECM_BASE) / |tip - ECM_BASE|
                ╲
                 ╲   camera
                  ▣ ─ ─ ─ ─ ─ ─ ► ✕ tip
```

The direction of the endoscope for given yaw and pitch is (from
`dvrk_sim/kinematics.py`):

```
u = [ -sin(yaw)·cos(pitch),   sin(pitch),   -cos(yaw)·cos(pitch) ]
```

Read it backwards to get the angles from a unit vector u:

```
pitch = asin(u_y)
yaw   = atan2(-u_x, -u_z)
```

Write `aim_angles(target, rcm)`: compute `d = target - rcm`, divide it by its
length (`np.linalg.norm`) to get `u`, return `(yaw, pitch)`.

**Checkpoint.** `python exercises/check.py B6` marks STEP 3 `[ok]`. The node's
report shows the angles that would centre the active tip; they match the
camera's current angles only when the tip is in the middle of the image.

## Step 4: don't chase every millimetre

If the camera followed every tiny movement, the image would never stand
still: the surgeon would feel seasick and lose their bearings. Real systems
use a **dead band**: small errors are ignored, and the camera moves only
when the instrument has drifted far enough from the centre.

Write `update_goal(goal, wanted, dead_band)`: if `wanted` differs from
`goal` by more than `dead_band` in yaw **or** in pitch, return `wanted`;
otherwise return `goal` unchanged. With `DEAD_BAND = 0.05` rad (about 3
degrees) the instrument can wander a few millimetres around the centre
without moving the camera.

**Checkpoint.** STEP 4 `[ok]` in the checker.

## Step 5: move smoothly

When you switch hand, the target jumps from one instrument to the other. If
the camera jumped too, the image would whip across the screen. The node
therefore never sends the goal directly: it moves the command a little at a
time, at most `MAX_SPEED` radians per second.

Write `step_towards(current, goal, max_speed, dt)`: for each angle, the
change is `goal - current`, limited to the range `±max_speed·dt`; return the
new pair. With `MAX_SPEED = 0.4` and 20 updates a second, each step turns the
camera by 0.02 rad at most.

**Checkpoint.** STEP 5 `[ok]` in the checker: now steps 3 to 5 all pass.

## Step 6: talk to the camera arm

Create the publisher in `__init__` (JointState on `/ECM/servo_jp`) and, at
the end of `tick`, send

```python
JointState(name=ECM_JOINT_NAMES, position=[cmd_yaw, cmd_pitch, insertion, roll])
```

keeping the measured insertion and roll: this node only turns the camera.

**Checkpoint.** Run the node and sit at the console. Move the right hand
towards the edge of the image: once it drifts past the dead band, the
camera follows it. Press Tab: the camera glides over to the other
instrument. `python exercises/check.py B6` should now say `B6: PASSED`.

---

## Try it and discuss

- Do the handover task (B) with and without your node. Is it faster? Is it
  more comfortable?
- Notice something subtle: when the camera turns, *your hand directions
  change*. "Right" is always right in the image, so the teleoperation keeps
  re-mapping your hand as the camera moves (see `on_ecm` and the re-anchoring
  in `teleop_node.py`). Is it disturbing while you are in the middle of a
  movement?
- Change `DEAD_BAND` and `MAX_SPEED`. What feels best? Is it the same for
  everyone in the class?

## Going further

1. **Respect the surgeon.** Do not move the camera while the clutch is
   pressed (`/footpedals/clutch`, Bool). Then: stop following while the
   surgeon moves the camera by hand.
2. **Zoom.** Also adjust the insertion to keep the tip at about 80 mm from
   the camera (`/ECM/measured_cp` gives the camera position). Within the
   limits!
3. **Both instruments.** Aim at the active one but keep the other inside
   the image too, for example by aiming at a weighted average of the two tips.
4. **Guess the active instrument** without `/console/active_arm`, from
   which tip moved last. Now the node would work on a real dVRK.
