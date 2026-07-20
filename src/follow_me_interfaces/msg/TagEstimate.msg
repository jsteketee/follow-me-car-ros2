# Fused tag estimate — the EKF output that consumes uwb/raw (cf. UwbRaw, the raw fix
# stream). Range/bearings are recomputed from the filtered tag position in `odom` on
# every publish, so they stay fresh through car rotation even between UWB fixes.
# bearing_abs is directly consumable as DriveCommand.heading. SI / REP-103 throughout.
std_msgs/Header header
float32 distance       # m, anchor->tag range from the filtered estimate (always >= 0)
float32 bearing_abs    # rad, ABSOLUTE bearing to tag in the odom frame (REP-103, CCW +);
                       #   feeds DriveCommand.heading directly
float32 bearing_rel    # rad, car-relative bearing (base_link frame, pan NOT included);
                       #   for the pan-pointing consumer
float32 range_sigma    # m, 1-sigma uncertainty on distance
float32 bearing_sigma  # rad, 1-sigma uncertainty on bearing
int32   age_ms         # ms since the last ACCEPTED fix's measurement time; -1 = never fixed
bool    coasting       # true = no accepted fix for > threshold; estimate is dead-reckoned
                       #   and the sigmas are growing
