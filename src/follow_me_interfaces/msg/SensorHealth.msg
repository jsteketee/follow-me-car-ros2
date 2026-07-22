# Low-rate ESP32 sensor-health report: per-sensor update rates from the firmware's own
# counters, arriving as {"type":"health"} event frames (~1-2 Hz). Parallel arrays so new
# sensors need no interface change. Staleness of THIS message = the reporter is unhealthy.
std_msgs/Header header
string[]  names       # sensor keys as reported by the firmware: "imu", "uwb", "loop", ...
float32[] rates_hz    # update rate per sensor, index-matched to names; 0 = silent
uint32    max_loop_us # worst control-loop iteration gap since the last frame (us); 0 = not reported
uint16    telem_frames_1s # successful telemetry frames the bridge parsed in the last 1 s (Pi-derived)
