# Raw DW3000 UWB range + bearing to the tag — the tag fix stream. "Raw" = unprocessed
# on the Pi side: Pi fusion (Phase 5) will consume these measurements and publish its
# own estimate topic. SI / REP-103; the bridge converts from the ESP32's cm / deg.
# See PROJECT_PLAN.md "Serial Protocol".
std_msgs/Header header
float32 distance   # m, raw range to tag; < 0 (wire -1) = no valid range this frame
float32 bearing    # rad, raw azimuth/AoA to tag, ANCHOR-relative (measured in the panned
                   #   uwb_link frame: absolute = yaw + pan_angle + uwb_bearing, see
                   #   PROJECT_PLAN "Serial Protocol"); REP-103 (+ = LEFT/CCW), so the
                   #   bridge negates the device value (+ = RIGHT)
int32   age_ms     # ms since this fix was measured; -1 = no fix / not reported this frame
