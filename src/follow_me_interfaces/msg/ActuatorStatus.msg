# Live actuator outputs reported by the ESP32 — what the car is actually driving, as
# opposed to what was commanded (cf. CommandStatus). throttle/steering are normalized,
# dimensionless efforts (no SI conversion); pan_angle is the live servo angle; the *_pwm
# fields are the raw pulse widths actually written. See interface.md.
std_msgs/Header header
float32 throttle      # normalized drive effort, [-0.25, 1] (< 0 = braking)
float32 steering      # normalized steering effort, [-1, 1] (wire sign convention)
float32 pan_angle     # rad, live pan servo angle (bridge converts from device deg)
uint16  esc_pwm       # us, actual ESC pulse width written (post smoothing/deadband); ~1500 neutral
uint16  steer_pwm     # us, actual steering-servo pulse width written; ~1500 neutral
uint16  pan_pwm       # us, actual pan-servo pulse width written; ~1500 neutral
