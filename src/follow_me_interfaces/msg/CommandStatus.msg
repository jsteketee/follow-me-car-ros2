# Echo of the last command the ESP32 ACCEPTED — the feedback half of the cmd_drive
# channel (cf. DriveCommand, the setpoint sent TO the car). Used to verify the TX path:
# cmd_age_ms resetting means frames are landing; cmd_rejects ticking means they're
# arriving but failing validation. Also carries the ESP32's live control mode, since the
# mode decides which command shape is acted on. Values are the ESP32's accepted setpoints
# converted to SI / REP-103 at the bridge. See PROJECT_PLAN.md "Serial Protocol".
std_msgs/Header header
string  mode          # ESP32 control mode this frame: "SETPOINT", "DIRECT", or "STOPPED"
float32 cmd_speed     # m/s, last accepted forward setpoint
float32 cmd_heading   # rad, last accepted absolute heading, odom frame (REP-103, CCW +);
                      #   the bridge converts device compass deg -> odom rad via the same
                      #   heading offset the TX path establishes (see T2b)
float32 cmd_pan       # rad, last accepted pan setpoint
int32   cmd_age_ms    # ms since the last accepted setpoint frame; -1 = none since boot
uint32  cmd_rejects   # commands rejected by ESP32 validation since boot (monotonic)
