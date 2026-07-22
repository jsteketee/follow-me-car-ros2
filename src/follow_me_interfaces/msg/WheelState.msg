# Drivetrain state co-sampled in one ESP32 telemetry frame — one message, one stamp.
# speed is the ESP32's onboard fused estimate, the value its throttle PID uses as
# feedback. See interface.md.
std_msgs/Header header
float32 speed        # m/s, ESP32 fused speed estimate (throttle PID feedback); SIGNED, < 0 =
                     #   reverse/rollback. Fails HIGH on sensor flicker, never spuriously low.
float32 distance     # m, odometer; SIGNED (reverse subtracts). Stitched continuous across ESP32
                     #   reboots at the bridge, so it does NOT reset on reboot (interface.md).
bool    cogging      # ESP32 latching cogging flag (ESC deadband/stiction, inferred onboard)
bool    enc_fault    # AS5600 encoder unhealthy (absent/I2C fail/magnet lost). While true,
                     #   the phantom-pulse odometry guard is off -> trust distance less
