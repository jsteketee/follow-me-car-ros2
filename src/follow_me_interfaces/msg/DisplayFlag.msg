# Display flag event: add or remove one labeled status item shown on the ESP32 OLED
# (and/or a dashboard). Sent on display/flag, merged into the serial_bridge TX frame
# (cf. DriveCommand, the drive setpoint on the same serial link). Edge-triggered — the
# consumer owns the displayed list.
std_msgs/Header header
string text        # flag label to show
int8   action      # +1 = add the label, -1 = remove the matching label
