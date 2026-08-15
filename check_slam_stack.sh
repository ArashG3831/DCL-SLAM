#!/usr/bin/env bash
set -euo pipefail

# ---- config: edit only if your topic names differ ----
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
MAP_TOPIC="${MAP_TOPIC:-/map}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
TF_TOPIC="${TF_TOPIC:-/tf}"
TF_STATIC_TOPIC="${TF_STATIC_TOPIC:-/tf_static}"
SLAM_NODE_REGEX="${SLAM_NODE_REGEX:-slam_toolbox}"
# ------------------------------------------------------

red(){ printf "\033[31m%s\033[0m\n" "$*"; }
grn(){ printf "\033[32m%s\033[0m\n" "$*"; }
ylw(){ printf "\033[33m%s\033[0m\n" "$*"; }
blu(){ printf "\033[34m%s\033[0m\n" "$*"; }

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || { red "Missing command: $1"; exit 1; }
}

need_cmd ros2
need_cmd python3
need_cmd timeout
need_cmd awk
need_cmd grep
need_cmd sed

blu "=== ROS ENV ==="
echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo

blu "=== TOPIC PRESENCE ==="
ros2 topic list | sort > /tmp/ros_topics.txt
for t in "$SCAN_TOPIC" "$MAP_TOPIC" "$ODOM_TOPIC" "$TF_TOPIC" "$TF_STATIC_TOPIC"; do
  if grep -qx "$t" /tmp/ros_topics.txt; then grn "OK: topic exists: $t"
  else red "MISSING: topic not found: $t"
  fi
done
echo

blu "=== TOPIC TYPES (must match expectations) ==="
for t in "$SCAN_TOPIC" "$MAP_TOPIC" "$ODOM_TOPIC" "$TF_TOPIC" "$TF_STATIC_TOPIC"; do
  if grep -qx "$t" /tmp/ros_topics.txt; then
    ty="$(ros2 topic type "$t" 2>/dev/null || true)"
    echo "$t  ->  ${ty:-<unknown>}"
  fi
done
echo

blu "=== QUICK LIVE MESSAGE CHECK (one sample each) ==="
if grep -qx "$SCAN_TOPIC" /tmp/ros_topics.txt; then
  ylw "[scan] capture one message (3s timeout)..."
  if timeout 3 ros2 topic echo "$SCAN_TOPIC" --once >/tmp/scan_msg.txt 2>/dev/null; then
    grn "[scan] got message"
  else
    red "[scan] NO message received (timeout)"
  fi
fi

if grep -qx "$MAP_TOPIC" /tmp/ros_topics.txt; then
  ylw "[map] capture one message (3s timeout)..."
  if timeout 3 ros2 topic echo "$MAP_TOPIC" --once >/tmp/map_msg.txt 2>/dev/null; then
    grn "[map] got message"
  else
    red "[map] NO message received (timeout)"
  fi
fi
echo

blu "=== RATE CHECK (2s window each) ==="
rate_check () {
  local t="$1"
  if grep -qx "$t" /tmp/ros_topics.txt; then
    ylw "ros2 topic hz $t (2s)..."
    timeout 2 ros2 topic hz "$t" 2>/dev/null | tail -n 3 || true
    echo
  fi
}
rate_check "$SCAN_TOPIC"
rate_check "$ODOM_TOPIC"
rate_check "$TF_TOPIC"
rate_check "$MAP_TOPIC"

blu "=== TF FRAME SNAPSHOT ==="
ylw "Frame ids seen in /tf (2s)..."
timeout 2 ros2 topic echo "$TF_TOPIC" 2>/dev/null | \
  grep -E "frame_id:|child_frame_id:" | head -n 40 || true
echo

blu "=== SLAM TOOLBOX NODE + LIFECYCLE ==="
nodes="$(ros2 node list 2>/dev/null || true)"
echo "$nodes" | grep -i "$SLAM_NODE_REGEX" || ylw "No node matching '$SLAM_NODE_REGEX' found (maybe slam_toolbox not running)."

slam_node="$(echo "$nodes" | grep -i "$SLAM_NODE_REGEX" | head -n 1 || true)"
if [[ -n "${slam_node}" ]]; then
  ylw "Lifecycle state for $slam_node:"
  ros2 lifecycle get "$slam_node" 2>/dev/null || ylw "(lifecycle query failed; node may not be lifecycle-managed)"
fi
echo

blu "=== DEEP VALIDATION (Python): scan sanity + map sanity ==="
python3 - <<'PY'
import re

def read(path):
  try:
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
      return f.read()
  except FileNotFoundError:
    return ""

scan = read("/tmp/scan_msg.txt")
mapp = read("/tmp/map_msg.txt")

def get_field(text, field):
  m = re.search(rf"^{re.escape(field)}:\s*(.+)$", text, flags=re.MULTILINE)
  return m.group(1).strip() if m else None

print("---- /scan sanity ----")
if not scan.strip():
  print("FAIL: no /scan message captured")
else:
  frame = get_field(scan, "frame_id")
  ang_min = get_field(scan, "angle_min")
  ang_max = get_field(scan, "angle_max")
  inc = get_field(scan, "angle_increment")
  rng_min = get_field(scan, "range_min")
  rng_max = get_field(scan, "range_max")

  ranges_section = re.search(r"^ranges:\s*\n((?:\s*-\s*.*\n)+)", scan, flags=re.MULTILINE)
  n_ranges = 0
  if ranges_section:
    n_ranges = len(re.findall(r"^\s*-\s*", ranges_section.group(1), flags=re.MULTILINE))

  print(f"frame_id = {frame}")
  print(f"angle_min/max/inc = {ang_min} / {ang_max} / {inc}")
  print(f"range_min/max = {rng_min} / {rng_max}")
  print(f"ranges count ≈ {n_ranges}")

  problems = []
  if frame is None:
    problems.append("missing frame_id")
  if n_ranges == 0:
    problems.append("ranges array empty")
  if rng_min is not None and rng_max is not None:
    try:
      rmin = float(rng_min); rmax = float(rng_max)
      if rmax <= rmin:
        problems.append("range_max <= range_min")
    except:
      problems.append("range_min/max not parseable")
  if problems:
    print("FAIL:", "; ".join(problems))
  else:
    print("OK: /scan looks structurally valid")

print("\n---- /map sanity ----")
if not mapp.strip():
  print("FAIL: no /map message captured")
else:
  w = get_field(mapp, "width")
  h = get_field(mapp, "height")
  res = get_field(mapp, "resolution")

  data_empty = "data: []" in mapp
  data_section = re.search(r"^data:\s*\n((?:\s*-\s*.*\n)+)", mapp, flags=re.MULTILINE)
  n_data = 0
  if data_empty:
    n_data = 0
  elif data_section:
    n_data = len(re.findall(r"^\s*-\s*", data_section.group(1), flags=re.MULTILINE))

  print(f"resolution = {res}")
  print(f"width x height = {w} x {h}")
  print(f"data length ≈ {n_data}")

  problems = []
  try:
    wi = int(w) if w is not None else None
    hi = int(h) if h is not None else None
  except:
    wi = hi = None
    problems.append("width/height not parseable")

  if wi is None or hi is None:
    problems.append("missing width/height")
  else:
    if wi <= 0 or hi <= 0:
      problems.append("width/height <= 0 (malformed OccupancyGrid)")
    if wi is not None and hi is not None and n_data not in (0, wi*hi):
      problems.append(f"data length != width*height ({wi*hi})")

  if problems:
    print("FAIL:", "; ".join(problems))
  else:
    print("OK: /map looks structurally valid")
PY
echo

blu "=== SUMMARY ==="
if [[ -s /tmp/map_msg.txt ]]; then
  h="$(grep -E '^height:' /tmp/map_msg.txt | head -n 1 | awk '{print $2}' || true)"
  w="$(grep -E '^width:'  /tmp/map_msg.txt | head -n 1 | awk '{print $2}' || true)"
  if [[ "${h:-}" == "0" ]]; then
    red "MAP IS MALFORMED: height=0. This is NOT a 'needs motion' issue."
  else
    grn "Map non-zero: width=$w height=$h"
  fi
else
  red "No /map captured."
fi

if [[ -s /tmp/scan_msg.txt ]]; then
  frame="$(grep -E 'frame_id:' /tmp/scan_msg.txt | head -n 1 | sed 's/.*frame_id: *//')"
  ylw "Scan frame_id: ${frame:-<none>}"
else
  red "No /scan captured."
fi
echo
blu "Done."
