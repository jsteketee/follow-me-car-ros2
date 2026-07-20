# Live actuator outputs reported by the ESP32 — what the car is actually driving, as
# opposed to what was commanded (cf. CommandStatus). throttle/steering are normalized,
# dimensionless efforts (no SI conversion); pan_angle is the live servo angle.
# See PROJECT_PLAN.md "Serial Protocol".
std_msgs/Header header
float32 throttle      # normalized drive effort, [0, 1] (no reverse for now)
float32 steering      # normalized steering effort, [-1, 1] (wire sign convention)
float32 pan_angle     # rad, live pan servo angle (bridge converts from device deg)
