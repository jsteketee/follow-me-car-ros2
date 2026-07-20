# Drive setpoint for the car: forward speed + absolute heading, SI / REP-103.
# The serial bridge converts to the ESP32 wire contract (mph / device compass deg)
# on write — see PROJECT_PLAN.md "Serial Protocol". The header stamp gates staleness:
# the bridge stops transmitting when the latched command's stamp is older than 500 ms.

std_msgs/Header header
float32 speed      # m/s, forward; >= 0 (no reverse — clamped by the bridge)
float32 heading    # rad, absolute heading in the odom frame, REP-103 (CCW positive)
