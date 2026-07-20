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
  mode: string;        // "SETPOINT" | "DIRECT" | "STOPPED"
  cmd_speed: number;   // m/s
  cmd_heading: number; // rad, odom frame
  cmd_pan: number;     // rad
  cmd_age_ms: number;  // ms since last accepted setpoint; -1 = none
  cmd_rejects: number; // monotonic reject counter
};
