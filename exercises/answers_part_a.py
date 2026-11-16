"""Part A - reading nodes. Write your answers here, then run:

    python exercises/check.py A

Keep the variable names. Use the types shown (text in quotes, numbers
without quotes, lists in square brackets). Questions are in exercises/README.md.
"""

# ---------------------------------------------------------------- A1 the graph
A1_1 = ""        # name of the node that publishes /PSM1/servo_cp, e.g. "scene"
A1_2 = []        # nodes subscribed to /footpedals/clutch, e.g. ["a", "b"]
A1_3 = ""        # message type of /scene/vessels
A1_4 = 0         # how many nodes are running with "python launch.py"

# ------------------------------------------------------------ A2 measuring rates
A2_1 = 0         # rate of /PSM1/measured_cp, in Hz
A2_2 = 0         # rate of /PSM1/measured_js, in Hz
A2_3 = 0         # rate of /MTMR/measured_js, in Hz

# --------------------------------------------------------- A3 reading psm_node.py
A3_1 = 0         # frequency of the PSM control loop, in Hz
A3_2 = ""        # frame_id of /PSM1/measured_cp
A3_3 = ""        # frame_id of /PSM1/local/measured_cp
A3_4 = 0         # maximum insertion speed, in mm/s
A3_5 = ""        # "a", "b" or "c" (see README)

# ------------------------------------------------------ A4 reading teleop_node.py
A4_1 = 0         # instrument motion in mm when the hand moves 30 mm at scale 3
A4_2 = ""        # letters of the events that re-anchor the teleoperation, e.g. "ace"
A4_3 = 0         # order of the tremor filter
A4_4 = 0         # frequency of the teleoperation loop, in Hz
