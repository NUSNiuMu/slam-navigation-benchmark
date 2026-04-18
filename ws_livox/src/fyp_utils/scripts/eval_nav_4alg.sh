#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash eval_nav_4alg.sh --waypoints <yaml> [options]

Required:
  --waypoints <yaml>              Waypoint file with frame_id/goals

Optional:
  --algorithms <csv>              Default: liosam,fastlio,pointlio,fasterlio
  --out-root <dir>                Default: /home/niumu/FYP/nav_eval_results/nav_eval_<timestamp>
  --run-slam-nav-sh <path>        Default: /home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh
  --map-yaml <path>               Default: /home/niumu/ws_livox/src/mid360_navigation/maps/map.yaml
  --show-rviz <true|false>        Default: false
  --goal-timeout-sec <float>      Default: 90
  --xy-tolerance <float>          Default: 0.25
  --settle-sec <float>            Default: 1.0
  --near-collision-threshold-m <float> Default: 0.25
  --ros-setup <path>              Default: /opt/ros/noetic/setup.bash
  --ws-setup <path>               Default: /home/niumu/ws_livox/devel/setup.bash
EOF
}

WAYPOINTS=""
ALGORITHMS="liosam,fastlio,pointlio,fasterlio"
OUT_ROOT="/home/niumu/FYP/nav_eval_results/nav_eval_$(date +%Y%m%d_%H%M%S)"
RUN_SLAM_NAV_SH="/home/niumu/ws_livox/src/fyp_utils/scripts/run_slam_nav.sh"
MAP_YAML="/home/niumu/ws_livox/src/mid360_navigation/maps/map.yaml"
SHOW_RVIZ="false"
GOAL_TIMEOUT_SEC="90"
XY_TOLERANCE="0.25"
SETTLE_SEC="1.0"
NEAR_COLLISION_THRESHOLD_M="0.25"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="/home/niumu/ws_livox/devel/setup.bash"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --waypoints) WAYPOINTS="$2"; shift 2 ;;
    --algorithms) ALGORITHMS="$2"; shift 2 ;;
    --out-root) OUT_ROOT="$2"; shift 2 ;;
    --run-slam-nav-sh) RUN_SLAM_NAV_SH="$2"; shift 2 ;;
    --map-yaml) MAP_YAML="$2"; shift 2 ;;
    --show-rviz) SHOW_RVIZ="$2"; shift 2 ;;
    --goal-timeout-sec) GOAL_TIMEOUT_SEC="$2"; shift 2 ;;
    --xy-tolerance) XY_TOLERANCE="$2"; shift 2 ;;
    --settle-sec) SETTLE_SEC="$2"; shift 2 ;;
    --near-collision-threshold-m) NEAR_COLLISION_THRESHOLD_M="$2"; shift 2 ;;
    --ros-setup) ROS_SETUP="$2"; shift 2 ;;
    --ws-setup) WS_SETUP="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown arg: $1"; usage; exit 1 ;;
  esac
done

[[ -n "$WAYPOINTS" ]] || { echo "[ERROR] --waypoints is required"; exit 1; }
[[ -f "$WAYPOINTS" ]] || { echo "[ERROR] waypoints not found: $WAYPOINTS"; exit 1; }
[[ -x "$RUN_SLAM_NAV_SH" ]] || { echo "[ERROR] not executable: $RUN_SLAM_NAV_SH"; exit 1; }
[[ -f "$MAP_YAML" ]] || { echo "[ERROR] map yaml not found: $MAP_YAML"; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_EVAL_PY="$SCRIPT_DIR/nav_waypoint_eval.py"
SUM_PY="$SCRIPT_DIR/summarize_nav_eval.py"
[[ -f "$NAV_EVAL_PY" ]] || { echo "[ERROR] missing $NAV_EVAL_PY"; exit 1; }
[[ -f "$SUM_PY" ]] || { echo "[ERROR] missing $SUM_PY"; exit 1; }

mkdir -p "$OUT_ROOT"
echo "[INFO] out root: $OUT_ROOT"

wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start
  start=$(date +%s)
  while true; do
    if bash -lc "source '$ROS_SETUP' && source '$WS_SETUP' && rostopic list" | grep -Fx "$topic" >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start >= timeout_sec )); then
      return 1
    fi
    sleep 1
  done
}

terminate_pg() {
  local pgid="$1"
  [[ -n "$pgid" ]] || return 0
  if kill -0 "-$pgid" >/dev/null 2>&1; then
    kill -TERM "-$pgid" >/dev/null 2>&1 || true
    sleep 2
    if kill -0 "-$pgid" >/dev/null 2>&1; then
      kill -KILL "-$pgid" >/dev/null 2>&1 || true
    fi
  fi
}

CURRENT_NAV_PGID=""
CLEANED_UP=0
cleanup() {
  (( CLEANED_UP )) && return 0
  CLEANED_UP=1
  terminate_pg "$CURRENT_NAV_PGID"
}
on_signal() {
  local sig="$1"
  echo ""
  echo "[STOP] Caught ${sig}, shutting down current evaluation run..."
  trap - EXIT INT TERM
  cleanup
  wait || true
  exit 130
}
trap cleanup EXIT
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM

IFS=',' read -r -a ALG_LIST <<< "$ALGORITHMS"

for raw_alg in "${ALG_LIST[@]}"; do
  alg="$(echo "$raw_alg" | xargs)"
  [[ -n "$alg" ]] || continue

  echo ""
  echo "=================================================="
  echo "[RUN] $alg"
  echo "=================================================="

  ALG_DIR="$OUT_ROOT/$alg"
  mkdir -p "$ALG_DIR"

  NAV_PGID=""
  setsid bash -lc "source '$ROS_SETUP' && source '$WS_SETUP' && SHOW_RVIZ='$SHOW_RVIZ' MAP_YAML='$MAP_YAML' bash '$RUN_SLAM_NAV_SH' '$alg'" >"$ALG_DIR/run.log" 2>&1 &
  NAV_PGID=$!
  CURRENT_NAV_PGID="$NAV_PGID"

  if ! wait_topic "/move_base_simple/goal" 60; then
    echo "[ERROR] /move_base_simple/goal not ready for $alg" | tee -a "$ALG_DIR/error.log"
    terminate_pg "$NAV_PGID"
    CURRENT_NAV_PGID=""
    continue
  fi

  if ! wait_topic "/slam/odom" 60; then
    echo "[ERROR] /slam/odom not ready for $alg" | tee -a "$ALG_DIR/error.log"
    terminate_pg "$NAV_PGID"
    CURRENT_NAV_PGID=""
    continue
  fi

  bash -lc "source '$ROS_SETUP' && source '$WS_SETUP' && python3 '$NAV_EVAL_PY' --algorithm '$alg' --waypoints '$WAYPOINTS' --out-dir '$ALG_DIR' --goal-timeout-sec '$GOAL_TIMEOUT_SEC' --xy-tolerance '$XY_TOLERANCE' --settle-sec '$SETTLE_SEC' --near-collision-threshold-m '$NEAR_COLLISION_THRESHOLD_M'" >"$ALG_DIR/eval.log" 2>&1 || true

  terminate_pg "$NAV_PGID"
  CURRENT_NAV_PGID=""
  sleep 3

done

python3 "$SUM_PY" --root "$OUT_ROOT" --algs "$ALGORITHMS" >"$OUT_ROOT/summarize.log" 2>&1 || true

echo "[DONE] nav eval finished"
echo "[DONE] summary: $OUT_ROOT/summary.md"
