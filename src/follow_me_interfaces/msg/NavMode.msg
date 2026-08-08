# The active Pi-side navigation policy, owned by mode_manager (published latched,
# transient_local depth 1). Selects which controller node drives; distinct from the ESP32's
# own control mode (which the Pi infers from the command shape it sends). Set via set_nav_mode.
std_msgs/Header header
string mode           # "follow", "stopped", "waypoint", ... (open set; mode_manager validates)
