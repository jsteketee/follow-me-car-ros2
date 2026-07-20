# Drivetrain state co-sampled in one ESP32 telemetry frame — one message, one stamp.
# speed is the ESP32's onboard fused estimate, the value its throttle PID uses as
# feedback. See PROJECT_PLAN.md "Serial Protocol".
std_msgs/Header header
float32 speed        # m/s, ESP32 fused speed estimate (throttle PID feedback)
float32 distance     # m, accumulated odometer reading (monotonic; resets on ESP32 reboot)
bool    cogging      # ESP32 latching cogging flag (ESC deadband/stiction, inferred onboard)
