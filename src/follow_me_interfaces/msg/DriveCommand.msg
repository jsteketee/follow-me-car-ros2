# Drive command for the car — one of two shapes, chosen by `shape`:
#   SETPOINT: speed + heading -> onboard PIDs (the default path; nav_controller uses this).
#   DIRECT:   throttle + steering + pan -> raw actuators, PIDs bypassed.
# The bridge emits the matching ESP32 wire frame (interface.md); the header stamp gates
# staleness — the bridge stops transmitting when the latched command is older than 500 ms.

std_msgs/Header header

uint8 SETPOINT = 0
uint8 DIRECT = 1
uint8 shape          # command shape; default 0 = SETPOINT

# SETPOINT fields (used when shape == SETPOINT)
float32 speed        # m/s, forward; >= 0 (no reverse — clamped by the bridge)
float32 heading      # rad, absolute heading in the odom frame, REP-103 (CCW positive)

# DIRECT fields (used when shape == DIRECT)
float32 throttle     # normalized drive effort [-1, 1]; < 0 = brake/reverse
float32 steering     # normalized steering [-1, 1]
float32 pan_deg      # pan target, degrees [-90, 90]
