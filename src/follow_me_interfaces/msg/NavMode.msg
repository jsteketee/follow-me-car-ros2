# The active Pi-side navigation policy, owned by mode_manager (published latched,
# transient_local depth 1). Selects which controller node drives; distinct from
# CommandStatus.command_mode, the ESP32's control mode. Set via the set_nav_mode service.
std_msgs/Header header
string mode           # "follow", "stopped", "waypoint", ... (open set; mode_manager validates)
