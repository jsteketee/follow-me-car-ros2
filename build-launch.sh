#!/usr/bin/env bash
# build-launch — from the Mac, in one command: flush the source sync so the
# Pi is current, then clean-build and launch the follow-me stack on the Pi.
#
# Usage:  build-launch [-q|--quick] [extra ros2 launch args...]
#   -q, --quick   incremental build (skip the clean `rm -rf build install log`)
#   extra args are forwarded to `ros2 launch` (e.g. namespace:=fmbot foxglove:=false)
#
# The && chain is the safety: a failed flush (Pi down / sync conflict) or a failed
# build never proceeds to the launch.

set -euo pipefail

HOST="${HOST:-followme-pi}"                # Pi ssh alias (from ~/.ssh/config)
SESSION="${SESSION:-fmcar}"               # mutagen source session to flush
PI_DIR="${PI_DIR:-follow-me-car-ros2}"    # workspace path on the Pi (relative to ~)

# Full clean build by default (needed when the .msg set changes); -q skips the wipe.
clean="rm -rf build install log && "
if [ "${1:-}" = "-q" ] || [ "${1:-}" = "--quick" ]; then
  clean=""
  shift
fi
launch_args="$*"

echo "==> flushing source sync ($SESSION) — making sure the Pi is current…"
mutagen sync flush "$SESSION"

echo "==> ${clean:+clean-}building + launching on $HOST  (Ctrl-C stops the launch)…"
exec ssh -t "$HOST" "cd ~/$PI_DIR \
  && source /opt/ros/jazzy/setup.bash \
  && ${clean}colcon build --symlink-install \
  && source install/setup.bash \
  && ros2 launch follow_me_nodes bringup.launch.py ${launch_args}"
