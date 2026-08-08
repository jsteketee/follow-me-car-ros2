#!/usr/bin/env bash
# performance-benchmark — sample the health topics for a fixed window and report on them.
# Records 10 s of the *_health topics into a dated bag under bags/performance-benchmark/,
# then prints and writes a CSV of each recorded field's average and max over the run.
#
# Usage:  performance-benchmark.sh [existing-bag]
#   (no args)      record a new 10 s bag, then report on it
#   existing-bag   skip recording; only (re)build the report for that bag
#
# The report is written next to the bag as <bag-name>-report.csv.
# Env overrides: DURATION_S (window, default 10), HEALTH_TOPIC_REGEX (default '.*_health$').

set -euo pipefail

DURATION_S="${DURATION_S:-10}"                    # record window in seconds
TOPIC_REGEX="${HEALTH_TOPIC_REGEX:-.*_health$}"   # namespace-safe: matches pi_health, sensor_health
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_ROOT="$SCRIPT_DIR/bags/performance-benchmark"

# Source a setup file with `set -u` relaxed (ROS setup scripts reference unbound vars).
_src() { set +u; # shellcheck disable=SC1090
  source "$1"; set -u; }

# Make ros2 and the follow_me_interfaces message types available for record + deserialization.
if ! command -v ros2 >/dev/null 2>&1; then
  for s in /opt/ros/*/setup.bash; do [ -f "$s" ] && { _src "$s"; break; }; done
fi
[ -f "$SCRIPT_DIR/install/setup.bash" ] && _src "$SCRIPT_DIR/install/setup.bash"

# Print the one-shot description shown every run.
print_banner() {
  cat <<'BANNER'
================================================================================
 performance-benchmark — follow-me health-topic benchmark
--------------------------------------------------------------------------------
 Records a fixed 10 s sample of the health topics (pi_health + sensor_health)
 into a dated rosbag under bags/performance-benchmark/, then reports the average
 and max of every recorded field over the run and saves it as a CSV beside the
 bag. Pass an existing bag path to skip recording and just rebuild its report.
================================================================================
BANNER
}

# Deserialize a bag and emit the average/max report to stdout and a sibling CSV.
generate_report() {
  local bag="${1%/}"
  local csv="${bag}-report.csv"
  if [ ! -f "$bag/metadata.yaml" ]; then
    echo "Error: '$bag' is not a rosbag2 directory (no metadata.yaml)." >&2
    return 1
  fi
  python3 - "$bag" "$csv" <<'PY'
# Reads a rosbag2 directory, computes per-field average/max for the health message
# types, prints a table, and writes an equivalent <bag>-report.csv.
import csv
import math
import sys
from collections import OrderedDict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG_PATH, CSV_PATH = sys.argv[1], sys.argv[2]
HEALTH_TYPES = {
    "follow_me_interfaces/msg/PiHealth",
    "follow_me_interfaces/msg/SensorHealth",
}

# Open a bag, auto-detecting the storage plugin (mcap/sqlite3).
def open_reader(path):
    for sid in ("", "mcap", "sqlite3"):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=path, storage_id=sid),
                rosbag2_py.ConverterOptions("", ""),
            )
            return reader
        except Exception:
            continue
    raise RuntimeError("could not open bag: %s" % path)

stats = OrderedDict()  # (topic, field) -> running sum/count/max/nan

# Fold one numeric value into the running stats for (topic, field), skipping NaN.
def acc(topic, field, val):
    try:
        v = float(val)
    except (TypeError, ValueError):
        return
    s = stats.get((topic, field))
    if s is None:
        s = {"sum": 0.0, "cnt": 0, "max": None, "nan": 0}
        stats[(topic, field)] = s
    if math.isnan(v):
        s["nan"] += 1
        return
    s["sum"] += v
    s["cnt"] += 1
    s["max"] = v if s["max"] is None else max(s["max"], v)

# Walk a message's fields, accumulating scalars and (name-labeled) numeric arrays.
def process(topic, msg):
    names = getattr(msg, "names", None)
    if not (isinstance(names, (list, tuple)) and names and all(isinstance(x, str) for x in names)):
        names = None
    for f in msg.get_fields_and_field_types():
        val = getattr(msg, f)
        if isinstance(val, bool) or isinstance(val, (str, bytes, bytearray)):
            continue
        if isinstance(val, (int, float)):
            acc(topic, f, val)
            continue
        if hasattr(val, "get_fields_and_field_types"):
            continue  # nested message (e.g. Header)
        try:
            seq = list(val)
        except TypeError:
            continue
        if not seq or all(isinstance(x, str) for x in seq):
            continue  # empty, or a string array like `names`
        if names is not None and len(seq) == len(names):
            for label, x in zip(names, seq):
                acc(topic, "%s[%s]" % (f, label), x)
        else:
            for x in seq:
                acc(topic, "%s[]" % f, x)

reader = open_reader(BAG_PATH)
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
health_topics = {n: ty for n, ty in types.items() if ty in HEALTH_TYPES}
msg_classes, counts = {}, OrderedDict()
t_min = t_max = None

try:
    while reader.has_next():
        topic, data, t = reader.read_next()
        ty = health_topics.get(topic)
        if ty is None:
            continue
        cls = msg_classes.get(ty)
        if cls is None:
            cls = get_message(ty)
            msg_classes[ty] = cls
        process(topic, deserialize_message(data, cls))
        counts[topic] = counts.get(topic, 0) + 1
        t_min = t if t_min is None else min(t_min, t)
        t_max = t if t_max is None else max(t_max, t)
except ModuleNotFoundError as e:
    sys.exit("Error: %s. Source the workspace (install/setup.bash) so the message types load." % e)

# Format a number compactly for display; blank for missing values.
def fmt(x, sig="%.4g"):
    return "" if x is None or x == "" else sig % x

print("Bag:      %s" % BAG_PATH)
if not health_topics:
    print("\nNo pi_health/sensor_health topics in this bag — nothing to report.")
elif not counts:
    print("\nHealth topics are present but held zero messages — was the stack running?")
else:
    span = (t_max - t_min) / 1e9 if t_min is not None and t_max != t_min else 0.0
    print("Duration: %.1f s   Messages: %s"
          % (span, ", ".join("%s=%d" % (n, c) for n, c in counts.items())))

# Print a per-topic table of field | average | max | samples.
last_topic = None
for (topic, field), s in stats.items():
    if topic != last_topic:
        print("\n%s" % topic)
        print("  %-28s %14s %14s %9s" % ("field", "average", "max", "samples"))
        last_topic = topic
    avg = s["sum"] / s["cnt"] if s["cnt"] else None
    note = "" if not s["nan"] else "  (%d NaN skipped)" % s["nan"]
    print("  %-28s %14s %14s %9d%s"
          % (field, fmt(avg), fmt(s["max"]), s["cnt"], note))

with open(CSV_PATH, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["topic", "field", "average", "max", "samples", "nan_samples"])
    for (topic, field), s in stats.items():
        avg = s["sum"] / s["cnt"] if s["cnt"] else None
        w.writerow([topic, field, fmt(avg, "%.6g"), fmt(s["max"], "%.6g"), s["cnt"], s["nan"]])

print("\nReport CSV: %s" % CSV_PATH)
PY
}

# ---- entry point ---------------------------------------------------------------
print_banner

if [ "$#" -gt 0 ]; then
  case "$1" in
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
  esac
  echo "Mode: report-only (existing bag)"
  echo
  generate_report "$1"
  exit $?
fi

command -v ros2 >/dev/null 2>&1 || { echo "Error: ros2 not on PATH; source your ROS install." >&2; exit 1; }
mkdir -p "$BAG_ROOT"
bag="$BAG_ROOT/performance-benchmark_$(date +%Y%m%d_%H%M%S)"
echo "Mode: record ${DURATION_S}s of topics matching /${TOPIC_REGEX}/"
echo "Bag:  $bag"
echo "Recording... (finalizes automatically after ${DURATION_S}s; Ctrl-C stops early)"

# Background elapsed/remaining counter while record runs foreground and logs to a file.
rec_log="$(mktemp -t perfbench-record.XXXXXX)"
(
  for ((i = 1; i <= DURATION_S; i++)); do
    printf '\r  recording %2ds / %2ds  (%2ds left) ' "$i" "$DURATION_S" "$((DURATION_S - i))"
    sleep 1
  done
) &
counter_pid=$!

# timeout -s INT delivers a clean SIGINT so rosbag2 finalizes the mcap; 124 = timer fired.
timeout -s INT "$DURATION_S" ros2 bag record -s mcap -o "$bag" -e "$TOPIC_REGEX" >"$rec_log" 2>&1 || true

kill "$counter_pid" 2>/dev/null || true
wait "$counter_pid" 2>/dev/null || true
printf '\r%*s\r' 44 ''   # wipe the counter line

if [ ! -f "$bag/metadata.yaml" ]; then
  echo "Error: recording produced no bag at $bag (no topics matched, or record failed)." >&2
  echo "  see record log: $rec_log" >&2
  exit 1
fi
rm -f "$rec_log"

echo "Recording complete."
echo
generate_report "$bag"
