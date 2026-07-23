#!/usr/bin/env bash
# dev-tmux-reset — tear down the follow-me dev tmux session, then relaunch it from scratch
# via dev-tmux.sh. Use when the session is wedged and you want a clean rebuild.
#
# Usage:  dev-tmux-reset [-q|--quick]
#   -q, --quick   forwarded to dev-tmux.sh (incremental ROS2 build; default is full clean build).

set -euo pipefail

SESSION="${SESSION:-followme}"                                  # tmux session name (matches dev-tmux.sh)
DEV_TMUX="${DEV_TMUX:-$HOME/repositories/follow-me-car-ros2/dev-tmux.sh}"

command -v tmux >/dev/null || { echo "tmux not found — install it (brew install tmux)"; exit 1; }

# Kill the session if it exists so dev-tmux.sh rebuilds instead of re-attaching.
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "==> killing existing session '$SESSION'"
  tmux kill-session -t "$SESSION"
else
  echo "==> no existing session '$SESSION' — building fresh"
fi

# Relaunch, forwarding any args (notably -q/--quick) straight through.
exec "$DEV_TMUX" "$@"
