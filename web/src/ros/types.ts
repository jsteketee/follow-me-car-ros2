// Minimal decoded shapes for the messages this dashboard consumes (tf2_msgs/msg/TFMessage).
export type Vector3 = { x: number; y: number; z: number };
export type Quaternion = { x: number; y: number; z: number; w: number };

export type TransformStamped = {
  header: { stamp: { sec: number; nanosec: number }; frame_id: string };
  child_frame_id: string;
  transform: { translation: Vector3; rotation: Quaternion };
};

export type TFMessage = { transforms: TransformStamped[] };

// follow_me_interfaces/msg/ActuatorStatus — live actuator outputs from the ESP32.
export type ActuatorStatus = {
  throttle: number;   // normalized drive effort [0,1]
  steering: number;   // normalized steering effort [-1,1]
  pan_angle: number;  // rad, live pan servo angle
};

// follow_me_interfaces/msg/TagEstimate — fused (EKF) tag estimate with uncertainty.
export type TagEstimate = {
  distance: number;       // m, anchor->tag range
  bearing_abs: number;    // rad, absolute bearing (odom)
  bearing_rel: number;    // rad, car-relative bearing
  range_sigma: number;    // m, 1-sigma range uncertainty
  bearing_sigma: number;  // rad, 1-sigma bearing uncertainty
  age_ms: number;         // ms since last accepted fix; -1 = never
  coasting: boolean;      // true = dead-reckoned, sigmas growing
};

// follow_me_interfaces/msg/CommandStatus — echo of the ESP32's accepted command + mode.
export type CommandStatus = {
  command_mode: string; // ESP32 control mode: "SETPOINT" | "DIRECT" | "STOPPED"
  cmd_speed: number;    // m/s
  cmd_heading: number;  // rad, odom frame
  cmd_pan: number;      // rad
  cmd_age_ms: number;   // ms since last accepted setpoint; -1 = none
  cmd_rejects: number;  // monotonic reject counter
};

// follow_me_interfaces/msg/NavMode — the active Pi-side navigation policy (latched).
export type NavMode = {
  mode: string;  // "follow" | "stopped" | future policies
};

// follow_me_interfaces/srv/SetNavMode response.
export type SetNavModeResponse = {
  accepted: boolean;
  message: string;  // rejection reason or transition echo
};

// follow_me_interfaces/msg/SensorHealth — per-sensor update rates from ESP32 health frames.
export type SensorHealthMsg = {
  names: string[];          // sensor keys: "imu", "uwb", "loop", ...
  rates_hz: number[];       // Hz per sensor, index-matched; 0 = silent
  max_loop_us: number;      // worst control-loop gap since last frame (us); 0 = not reported
  telem_frames_1s: number;  // Pi-parsed telemetry frames in the last 1 s (received rate)
};

// rcl_interfaces/msg/Log — one /rosout record from any ROS2 node.
export type RosoutLog = {
  stamp: { sec: number; nanosec: number };
  level: number;     // DEBUG=10 INFO=20 WARN=30 ERROR=40 FATAL=50
  name: string;      // logger (node) name
  msg: string;
  file: string;
  function: string;
  line: number;
};
