#!/usr/bin/env bash
# build-launch — from the Mac, in one command: flush the source sync so the
# Pi is current, then clean-build and launch the follow-me stack on the Pi.
#
# Usage:  build-launch [-q|--quick | -l|--launch-only] [extra ros2 launch args...]
#   -q, --quick        incremental build (skip the clean `rm -rf build install log`)
#   -l, --launch-only  skip the build entirely — just source + launch the existing install
#   extra args are forwarded to `ros2 launch` (e.g. namespace:=fmbot foxglove:=false)
#
# The && chain is the safety: a failed flush (Pi down / sync conflict) or a failed
# build never proceeds to the launch.

set -euo pipefail

HOST="${HOST:-followme-pi}"                # Pi ssh alias (from ~/.ssh/config)
SESSION="${SESSION:-fmcar}"               # mutagen source session to flush
PI_DIR="${PI_DIR:-follow-me-car-ros2}"    # workspace path on the Pi (relative to ~)

# Full clean build by default (needed when the .msg set changes); -q skips the wipe, -l skips the build.
clean="rm -rf build install log && "
build="colcon build --symlink-install && "
case "${1:-}" in
  -q|--quick)       clean="";            shift ;;
  -l|--launch-only) clean=""; build="";  shift ;;
esac
launch_args="$*"

echo "==> flushing source sync ($SESSION) — making sure the Pi is current…"
mutagen sync flush "$SESSION"

echo "==> ${build:+${clean:+clean-}building + }launching on $HOST  (Ctrl-C stops the launch)…"
exec ssh -t "$HOST" "cd ~/$PI_DIR \
  && source /opt/ros/jazzy/setup.bash \
  && ${clean}${build}source install/setup.bash \
  && ros2 launch follow_me_nodes bringup.launch.py ${launch_args}"
