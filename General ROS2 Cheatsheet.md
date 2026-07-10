# ROS2 CLI Cheatsheet

---

## Packages

```bash
# Install a package
sudo apt update
sudo apt install ros-jazzy-turtlesim

# List all available packages
ros2 pkg list

# List all executables across all packages
ros2 pkg executables

# List executables for a specific package
ros2 pkg executables turtlesim

# Create a new Python package
ros2 pkg create --build-type ament_python <package_name>
```

---

## Building

```bash
# Build all packages in the workspace
colcon build

# Build a single package (faster during development)
colcon build --packages-select <package_name>

# After building, source the workspace so ROS2 can find your packages
source install/setup.bash
```

---

## Running Nodes

```bash
ros2 run <package_name> <executable_name>
ros2 run turtlesim turtlesim_node

# Set a parameter at launch
ros2 run turtle_draw square_node --ros-args -p side_length:=6.0
```

---

## Launch Files

```bash
# Run a launch file
ros2 launch <package_name> <launch_file>
ros2 launch turtle_draw turtle_draw.launch.py
```

### Remapping topics at runtime

Use `--ros-args -r` to redirect a node's topic names without changing its code.
Useful for running multiple instances of the same node.

```bash
# Syntax
ros2 run <pkg> <exe> --ros-args -r <default_topic>:=<new_topic>

# Example: second teleop controlling turtle2 instead of turtle1
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtle2/cmd_vel
```

---

## Nodes

```bash
# List all running nodes
ros2 node list

# Get info about a specific node (publishers, subscribers, services)
ros2 node info /turtlesim
```

---

## Topics

```bash
# List all active topics
ros2 topic list

# Show the message type and publisher/subscriber counts
ros2 topic info /turtle1/cmd_vel

# Stream live messages from a topic
ros2 topic echo /turtle1/pose
ros2 topic echo /turtle1/cmd_vel

# Show the full definition of a message type
ros2 interface show geometry_msgs/msg/Twist

# Publish to a topic from the CLI
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}"   # single message
ros2 topic pub --rate 10 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}" # 10 Hz
```

### Common message packages

| Package | Common types |
|---|---|
| `std_msgs` | `String`, `Int32`, `Float64`, `Bool` |
| `geometry_msgs` | `Twist`, `Pose`, `Point`, `Vector3` |
| `sensor_msgs` | `Image`, `LaserScan`, `Imu`, `NavSatFix` |
| `nav_msgs` | `Odometry`, `OccupancyGrid`, `Path` |

### geometry_msgs/Twist — standard velocity command

Used by almost every mobile robot. For 2D ground robots, only two fields matter:

| Field | Meaning |
|---|---|
| `linear.x` | Forward / backward speed |
| `angular.z` | Turning speed (yaw) |

The other four (`linear.y`, `linear.z`, `angular.x`, `angular.y`) are zero for ground robots.

---

## Services

Point-to-point request/response calls. One-shot — they don't appear in rqt_graph by default.

```bash
# List all available services
ros2 service list

# List services with their types
ros2 service list -t

# Get the type of a specific service
ros2 service type /clear

# Call a service
ros2 service call <service> <type> "<args>"

# Examples
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, off: 0}"

# Show the request/response definition for a service type
ros2 interface show turtlesim/srv/Spawn
```

---

## Actions

For long-running tasks. Built on top of topics and services — not a separate primitive.

```bash
# List all active actions
ros2 action list

# Get info about an action (clients and servers)
ros2 action info /turtle1/rotate_absolute

# Send a goal and stream feedback
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback

# Show the goal / feedback / result definition
ros2 interface show turtlesim/action/RotateAbsolute
```

### How actions work under the hood

Actions are composed of 5 primitives:

| Part | Type | Visible in rqt_graph? |
|---|---|---|
| Send goal | Service | No (by default) |
| Cancel goal | Service | No (by default) |
| Get result | Service | No (by default) |
| Feedback | Topic | Yes |
| Status | Topic | Yes |

---

## Parameters

Runtime-configurable values on a node. Can be get/set without restarting the node.
Parameters are **node configuration**, not data — use them for values that affect how a node behaves (speeds, thresholds, colors, timeouts), not for real-time data flowing between nodes (that's what topics are for).

```bash
# List all parameters on a node
ros2 param list /turtlesim

# Get a parameter value
ros2 param get /turtlesim background_r

# Set a parameter value
ros2 param set /turtlesim background_r 255

# After changing background color, call /clear to apply it
ros2 service call /clear std_srvs/srv/Empty {}
```

---

## rqt_graph

Visualizes the live node/topic graph.

```bash
rqt_graph
```

---

## CLI Command Reference

| Command | Description |
|---|---|
| `ros2 run` | Run a package executable |
| `ros2 launch` | Run a launch file |
| `ros2 node` | Inspect running nodes |
| `ros2 topic` | Publish, subscribe, list topics |
| `ros2 service` | Call and list services |
| `ros2 action` | Send goals and list actions |
| `ros2 param` | Get/set node parameters |
| `ros2 interface` | Show message/service/action definitions |
| `ros2 pkg` | Package inspection |
| `ros2 bag` | Record and replay data |
| `ros2 doctor` | Check ROS setup for issues |
| `ros2 component` | Manage composable nodes |
| `ros2 lifecycle` | Manage lifecycle nodes |
