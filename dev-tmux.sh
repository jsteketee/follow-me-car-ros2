#!/usr/bin/env bash
# dev-tmux — bring up the follow-me dev environment as a 4-quadrant tmux session
# on the Mac: ROS2 build+bringup, web build+serve, Pi monitor, local terminal.
# Idempotent: re-running attaches to the existing session instead of rebuilding.
#
# Usage:  dev-tmux [-q|--quick | -l|--launch-only]
#   -q, --quick        incremental ROS2 build (skip the clean wipe); default is a full clean build.
#   -l, --launch-only  launch only — skip the ROS2 build and the web build, just serve/launch.

set -euo pipefail

SESSION="${SESSION:-followme}"                                   # tmux session name
HOST="${HOST:-followme-pi}"                                      # Pi ssh alias (~/.ssh/config)
PI_DIR="${PI_DIR:-follow-me-car-ros2}"                           # workspace on the Pi (relative to ~)
REPO_DIR="${REPO_DIR:-$HOME/repositories/follow-me-car-ros2}"    # Mac repo root (holds mutagen.yml)
BUILD_LAUNCH="${BUILD_LAUNCH:-$REPO_DIR/build-launch.sh}"
BUILD_LAUNCH_ARGS="${BUILD_LAUNCH_ARGS:-}"                       # forwarded to build-launch.sh
PIMON="${PIMON:-$HOME/repositories/pi-monitor/pimon.sh}"         # Mac-side Pi health monitor
# /rosout one-line formatter; resolved against $REPO_DIR on the Mac and ~/$PI_DIR on the Pi.
ROSOUT_FMT="${ROSOUT_FMT:-rosout-format.py}"
SYNC_SESSION="${SYNC_SESSION:-fmcar}"                            # mutagen source session name
WEB_URL="${WEB_URL:-http://followme-pi.local:8080/}"             # dashboard URL (Pi mDNS + serve port)
OPEN_BROWSER="${OPEN_BROWSER:-1}"                                # 0 to skip auto-opening the dashboard
PI_WAIT="${PI_WAIT:-1}"                                          # 0 to skip the Pi connectivity gate
PI_WAIT_TIMEOUT="${PI_WAIT_TIMEOUT:-0}"                          # seconds to keep trying (0 = wait forever)

# A leading -q means an incremental ROS2 build; -l skips the ROS2+web builds and just launches.
WEB_BUILD=1                                                      # 0 to skip `npm run build` (launch-only)
case "${1:-}" in
  -q|--quick)       BUILD_LAUNCH_ARGS="-q $BUILD_LAUNCH_ARGS" ;;
  -l|--launch-only) BUILD_LAUNCH_ARGS="-l $BUILD_LAUNCH_ARGS"; WEB_BUILD=0 ;;
esac

# start_sync — ensure the mutagen sessions are up (build-launch only flushes an existing one).
start_sync() {
  command -v mutagen >/dev/null || { echo "==> mutagen not found — skipping sync bring-up"; return 0; }
  # mutagen.yml.lock is mutagen's own gate on `project start`; the session-name
  # check alone can miss a live project (stale lock, sessions still registering).
  if [ -f "$REPO_DIR/mutagen.yml.lock" ] || mutagen sync list 2>/dev/null | grep -qw "$SYNC_SESSION"; then
    echo "==> mutagen sync '$SYNC_SESSION' already running — skipping bring-up"
  else
    echo "==> starting mutagen sync (mutagen project start)"
    ( cd "$REPO_DIR" && mutagen project start ) || echo "==> mutagen project start reported an error — continuing"
  fi
}

# open_dashboard — poll the web server, then open it in Chrome once it answers (~2 min max).
open_dashboard() {
  local i
  for i in $(seq 1 60); do
    if curl -sf -o /dev/null --max-time 2 "$WEB_URL"; then
      open -a "Google Chrome" "$WEB_URL"
      return 0
    fi
    sleep 2
  done
  echo "==> web server never came up at $WEB_URL — skipping browser open" >&2
}

# wait_for_pi — block until the Pi answers a real SSH handshake, so bringing the env up while
# the Pi is down/booting waits for it instead of racing every quadrant into failure.
wait_for_pi() {
  [ "$PI_WAIT" = "1" ] || return 0
  local start=$SECONDS attempt=0
  # BatchMode = fail instead of prompting; a completed handshake (not ping) is the gate.
  until ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$HOST" true 2>/dev/null; do
    if [ "$PI_WAIT_TIMEOUT" -gt 0 ] && [ $((SECONDS - start)) -ge "$PI_WAIT_TIMEOUT" ]; then
      printf '\n' >&2
      echo "==> Pi '$HOST' still unreachable after ${PI_WAIT_TIMEOUT}s — giving up" >&2
      exit 1
    fi
    attempt=$((attempt + 1))
    printf '\r==> waiting for Pi %s to come up (attempt %d)... ' "$HOST" "$attempt"
    sleep 3
  done
  [ "$attempt" -gt 0 ] && printf '\n'
  echo "==> Pi '$HOST' is up"
}

# attach — attach to the session, switching client if we're already inside tmux.
attach() {
  if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "$SESSION"
  else
    tmux attach-session -t "$SESSION"
  fi
}

command -v tmux >/dev/null || { echo "tmux not found — install it (brew install tmux)"; exit 1; }

# Already up? Just attach — don't clobber running builds/launches.
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "==> session '$SESSION' already exists — attaching"
  attach
  exit 0
fi

# Gate on the Pi: everything below needs SSH to $HOST, so don't build anything until it answers.
wait_for_pi

# Bring the mutagen sync up first so build-launch's flush has a session to flush.
start_sync

echo "==> creating tmux session '$SESSION' (4 quadrants)"

# Create the session detached, then carve it into four tiled panes.
tmux new-session -d -s "$SESSION" -n main
tl=$(tmux display-message -p -t "$SESSION:main" '#{pane_id}')   # top-left
tr=$(tmux split-window -h -t "$tl" -P -F '#{pane_id}')          # top-right
bl=$(tmux split-window -v -t "$tl" -P -F '#{pane_id}')          # bottom-left
br=$(tmux split-window -v -t "$tr" -P -F '#{pane_id}')          # bottom-right
tmux select-layout -t "$SESSION:main" tiled

# Label the quadrants along the pane borders.
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format ' #{pane_title} '
tmux select-pane -t "$tl" -T "rosout"
tmux select-pane -t "$tr" -T "web build+serve"
tmux select-pane -t "$bl" -T "pi monitor"
tmux select-pane -t "$br" -T "ros2 build+bringup"

# Fire each quadrant's command. Top-left tails the unified ROS log stream (/rosout) on the Pi.
web_run="npm run serve"
[ "$WEB_BUILD" = "1" ] && web_run="npm run build && $web_run"

# rosout-format.py collapses each 8-line YAML block from `topic echo` into one timestamped,
# severity-colored line. It runs Pi-side (keeps the pane a plain ssh -t, so Ctrl-C still kills
# the remote echo), which means mutagen has to have pushed it; if the Mac copy is missing the
# Pi's will be too, so fall back to the raw echo rather than leaving the quadrant dead.
# PYTHONUNBUFFERED stops `topic echo` block-buffering now that its stdout is a pipe.
rosout_run="source /opt/ros/jazzy/setup.bash && PYTHONUNBUFFERED=1 ros2 topic echo /rosout"
if [ -r "$REPO_DIR/$ROSOUT_FMT" ]; then
  rosout_run="$rosout_run | python3 -u ~/$PI_DIR/$ROSOUT_FMT"
else
  echo "==> $ROSOUT_FMT not found in $REPO_DIR — rosout quadrant will show raw YAML" >&2
fi

tmux send-keys -t "$tl" "ssh -t $HOST '$rosout_run'" C-m
tmux send-keys -t "$tr" "mutagen sync flush $SYNC_SESSION && ssh -t $HOST 'cd ~/$PI_DIR/web && $web_run'" C-m
tmux send-keys -t "$bl" "$PIMON" C-m
tmux send-keys -t "$br" "$BUILD_LAUNCH $BUILD_LAUNCH_ARGS" C-m

tmux select-pane -t "$tl"

# Once the web server answers, pop the dashboard in Chrome (background; won't block attach).
[ "$OPEN_BROWSER" = "1" ] && open_dashboard >/dev/null 2>&1 &

attach
