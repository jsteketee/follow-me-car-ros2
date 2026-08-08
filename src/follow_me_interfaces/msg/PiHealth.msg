# Raspberry Pi host health, sampled ~1 Hz by the pi_health node (psutil + thermal sysfs).
# Independent of the ESP/link — answers "is the Pi host itself the bottleneck?".
std_msgs/Header header
float32 load_percent          # 1-min load average as % of total CPU capacity (load/cores*100)
float32 cpu_percent           # overall CPU utilization %, ~1 s (since last poll)
float32 bridge_cpu_percent    # serial_bridge process CPU, 0-100 per core; -1 = not found
uint16  mem_used_mb           # RAM in use (total - available)
uint16  mem_total_mb          # total RAM
float32 temp_c                # CPU temperature (C); NaN = unavailable
