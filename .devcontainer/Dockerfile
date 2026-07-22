# ROS2 Jazzy dev container for follow-me-car-ros2 — matches the Pi (aarch64) on Apple Silicon.
# Provides the full ROS2 toolchain + workspace deps so IntelliSense and builds run on the Mac.
# Based on ros:jazzy (already local); rosdep pulls the message/tf packages via apt.
FROM ros:jazzy

# colcon, rosdep, pip, pyserial, pytest — the dev/build toolchain not in the base image.
# nodejs + npm back the Claude Code CLI installed below.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-pip \
      python3-serial \
      python3-pytest \
      nodejs \
      npm \
    && rm -rf /var/lib/apt/lists/*

# Claude Code CLI (the `claude` command) for agentic dev inside the container.
RUN npm install -g @anthropic-ai/claude-code

# Source ROS (and the workspace overlay, once built) in every interactive shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc \
 && echo '[ -f /workspaces/follow-me-car-ros2/install/setup.bash ] && source /workspaces/follow-me-car-ros2/install/setup.bash' >> /etc/bash.bashrc
