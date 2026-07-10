# ESP32 Kalman-fused bearing/distance estimate to the TAG (not the car's own pose).
# Streamed from the ESP32 while Pi-side fusion is deferred (see PROJECT_PLAN Phase 5).
# Units are SI (REP-103); the bridge converts from the ESP32's deg/cm.
std_msgs/Header header
float32 angle        # fused bearing to tag, car-relative, rad
float32 distance     # fused distance, m
float32 uncertainty  # fused bearing variance, rad^2
float32 heading      # car compass heading (yaw) at time of fix, rad; absolute tag bearing = heading + angle
