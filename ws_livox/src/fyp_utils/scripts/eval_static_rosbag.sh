#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash eval_static_rosbag.sh --bag <bag_path> [options]

Required:
  --bag <path>                  Input static rosbag.

Optional:
  --algorithms <csv>            Default: liosam,fastlio,pointlio,fasterlio
  --gt-topic <topic>            Ground-truth topic in original bag. Default: /odom
  --input-topic <topic>         Input lidar topic for latency estimate. Default: (empty, disabled)
  --out-root <dir>              Result root dir. Default: /home/niumu/FYP/static_eval_results/static_eval_<timestamp>
  --play-rate <float>           rosbag play rate. Default: 1.0
  --settle-sec <int>            Wait seconds after play. Default: 3
  --run-slam-sh <path>          Default: /home/niumu/run_slam.sh
  --ros-setup <path>            Default: /opt/ros/noetic/setup.bash
  --ws-setup <path>             Default: /home/niumu/ws_livox/devel/setup.bash
  --no-rviz <true|false>        Default: true (recommended for CPU evaluation)

Example:
  bash /home/niumu/ws_livox/src/fyp_utils/scripts/eval_static_rosbag.sh \
    --bag /home/niumu/FYP/PointCloudBag/short_test_xxx.bag \
    --gt-topic /odom
EOF
}

BAG=""
ALGORITHMS="liosam,fastlio,pointlio,fasterlio"
GT_TOPIC="/odom"
INPUT_TOPIC=""
OUT_ROOT="/home/niumu/FYP/static_eval_results/static_eval_$(date +%Y%m%d_%H%M%S)"
PLAY_RATE="1.0"
SETTLE_SEC=3
RUN_SLAM_SH="/home/niumu/run_slam.sh"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="/home/niumu/ws_livox/devel/setup.bash"
NO_RVIZ="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG="$2"
      shift 2
      ;;
    --algorithms)
      ALGORITHMS="$2"
      shift 2
      ;;
    --gt-topic)
      GT_TOPIC="$2"
      shift 2
      ;;
    --input-topic)
      INPUT_TOPIC="$2"
      shift 2
      ;;
    --out-root)
      OUT_ROOT="$2"
      shift 2
      ;;
    --play-rate)
      PLAY_RATE="$2"
      shift 2
      ;;
    --settle-sec)
      SETTLE_SEC="$2"
      shift 2
      ;;
    --run-slam-sh)
      RUN_SLAM_SH="$2"
      shift 2
      ;;
    --ros-setup)
      ROS_SETUP="$2"
      shift 2
      ;;
    --ws-setup)
      WS_SETUP="$2"
      shift 2
      ;;
    --no-rviz)
      NO_RVIZ="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$BAG" ]]; then
  echo "[ERROR] --bag is required"
  usage
  exit 1
fi

if [[ ! -f "$BAG" ]]; then
  echo "[ERROR] Bag file not found: $BAG"
  exit 1
fi

if [[ ! -x "$RUN_SLAM_SH" ]]; then
  echo "[ERROR] Cannot execute run_slam.sh: $RUN_SLAM_SH"
  exit 1
fi

if [[ ! -f "$ROS_SETUP" || ! -f "$WS_SETUP" ]]; then
  echo "[ERROR] Missing setup file: $ROS_SETUP or $WS_SETUP"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MONITOR_PY="$SCRIPT_DIR/slam_eval_monitor.py"
BAG2TUM_PY="$SCRIPT_DIR/bag_to_tum.py"
SUM_PY="$SCRIPT_DIR/summarize_static_eval.py"

for f in "$MONITOR_PY" "$BAG2TUM_PY" "$SUM_PY"; do
  if [[ ! -f "$f" ]]; then
    echo "[ERROR] Missing helper: $f"
    exit 1
  fi
done

mkdir -p "$OUT_ROOT"
echo "[INFO] Result root: $OUT_ROOT"
ROS_HOME_DIR="$OUT_ROOT/.ros"
mkdir -p "$ROS_HOME_DIR"

for cmd in evo_ape evo_rpe rosbag rostopic rosnode python3; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "[ERROR] Missing command: $cmd"
    exit 1
  fi
done

odom_topic_for_alg() {
  local alg="$1"
  case "$alg" in
    liosam) echo "/odometry/imu" ;;
    fastlio) echo "/Odometry" ;;
    pointlio) echo "/aft_mapped_to_init" ;;
    fasterlio) echo "/Odometry" ;;
    *)
      echo ""
      ;;
  esac
}

wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start
  start=$(date +%s)
  while true; do
    if bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && source '$WS_SETUP' && rostopic list" | grep -Fx "$topic" >/dev/null 2>&1; then
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
  if [[ -z "$pgid" ]]; then
    return 0
  fi
  if kill -0 "-$pgid" >/dev/null 2>&1; then
    kill -TERM "-$pgid" >/dev/null 2>&1 || true
    sleep 1
    if kill -0 "-$pgid" >/dev/null 2>&1; then
      kill -KILL "-$pgid" >/dev/null 2>&1 || true
    fi
  fi
}

ROSCORE_STARTED=0
if ! bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && rostopic list" >/dev/null 2>&1; then
  echo "[INFO] roscore not detected, starting one..."
  setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && roscore" >"$OUT_ROOT/roscore.log" 2>&1 &
  ROSCORE_PID=$!
  ROSCORE_STARTED=1
  sleep 3
fi

cleanup_all() {
  if [[ ${SLAM_PGID:-} != "" ]]; then
    terminate_pg "$SLAM_PGID"
  fi
  if [[ ${RECORD_PGID:-} != "" ]]; then
    terminate_pg "$RECORD_PGID"
  fi
  if [[ ${MONITOR_PGID:-} != "" ]]; then
    terminate_pg "$MONITOR_PGID"
  fi
  if [[ "$ROSCORE_STARTED" -eq 1 && ${ROSCORE_PID:-} != "" ]]; then
    terminate_pg "$ROSCORE_PID"
  fi
}
trap cleanup_all EXIT INT TERM

IFS=',' read -r -a ALG_LIST <<< "$ALGORITHMS"

for raw_alg in "${ALG_LIST[@]}"; do
  alg="$(echo "$raw_alg" | xargs)"
  [[ -z "$alg" ]] && continue

  odom_topic="$(odom_topic_for_alg "$alg")"
  if [[ -z "$odom_topic" ]]; then
    echo "[WARN] Skip unknown algorithm: $alg"
    continue
  fi

  echo ""
  echo "=================================================="
  echo "[RUN] algorithm=$alg odom_topic=$odom_topic"
  echo "=================================================="

  ALG_DIR="$OUT_ROOT/$alg"
  mkdir -p "$ALG_DIR/metrics"

  SLAM_PGID=""
  RECORD_PGID=""
  MONITOR_PGID=""

  if [[ "$NO_RVIZ" == "true" ]]; then
    case "$alg" in
      liosam)
        setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && source '$WS_SETUP' && roslaunch lio_sam run6axis.launch enable_rviz:=false" >"$ALG_DIR/slam.log" 2>&1 &
        ;;
      fastlio)
        setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source /home/niumu/fastlio2_ws/devel/setup.bash && roslaunch fast_lio mapping_mid360.launch rviz:=false" >"$ALG_DIR/slam.log" 2>&1 &
        ;;
      pointlio)
        setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source /home/niumu/fastlio2_ws/devel/setup.bash && roslaunch point_lio mapping_mid360.launch rviz:=false" >"$ALG_DIR/slam.log" 2>&1 &
        ;;
      fasterlio)
        setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source /home/niumu/fasterlio_ws/devel/setup.bash && roslaunch faster_lio mapping_mid360.launch rviz:=false" >"$ALG_DIR/slam.log" 2>&1 &
        ;;
      *)
        setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && source '$WS_SETUP' && '$RUN_SLAM_SH' '$alg'" >"$ALG_DIR/slam.log" 2>&1 &
        ;;
    esac
  else
    setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && source '$WS_SETUP' && '$RUN_SLAM_SH' '$alg'" >"$ALG_DIR/slam.log" 2>&1 &
  fi
  SLAM_PGID=$!
  sleep 4

  if ! wait_topic "$odom_topic" 40; then
    echo "[ERROR] odom topic not found for $alg: $odom_topic"
    continue
  fi

  setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && source '$WS_SETUP' && python3 '$MONITOR_PY' --algorithm '$alg' --odom-topic '$odom_topic' --input-topic '$INPUT_TOPIC' --out-dir '$ALG_DIR/metrics' --exclude-rosbag" >"$ALG_DIR/monitor.log" 2>&1 &
  MONITOR_PGID=$!
  sleep 1

  EST_BAG="$ALG_DIR/estimated.bag"
  setsid bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && source '$WS_SETUP' && rosbag record -O '$EST_BAG' '$odom_topic'" >"$ALG_DIR/record.log" 2>&1 &
  RECORD_PGID=$!
  sleep 2

  bash -lc "export ROS_HOME='$ROS_HOME_DIR' && source '$ROS_SETUP' && source '$WS_SETUP' && rosbag play '$BAG' --clock -r '$PLAY_RATE'" >"$ALG_DIR/play.log" 2>&1 || true
  sleep "$SETTLE_SEC"

  terminate_pg "$RECORD_PGID"
  RECORD_PGID=""

  terminate_pg "$MONITOR_PGID"
  MONITOR_PGID=""

  terminate_pg "$SLAM_PGID"
  SLAM_PGID=""

  if [[ ! -f "$EST_BAG" ]]; then
    echo "[ERROR] No estimated bag for $alg"
    continue
  fi

  python3 "$BAG2TUM_PY" --bag "$EST_BAG" --topic "$odom_topic" --out "$ALG_DIR/traj_est.tum" >"$ALG_DIR/export_est.log" 2>&1 || true
  python3 "$BAG2TUM_PY" --bag "$BAG" --topic "$GT_TOPIC" --out "$ALG_DIR/traj_gt.tum" >"$ALG_DIR/export_gt.log" 2>&1 || true

  if [[ -f "$ALG_DIR/traj_est.tum" && -f "$ALG_DIR/traj_gt.tum" ]]; then
    evo_ape tum "$ALG_DIR/traj_gt.tum" "$ALG_DIR/traj_est.tum" -a --save_results "$ALG_DIR/evo_ape.zip" >"$ALG_DIR/evo_ape.txt" 2>&1 || true
    # Use frame-based delta to be compatible with evo versions that don't support seconds unit here.
    evo_rpe tum "$ALG_DIR/traj_gt.tum" "$ALG_DIR/traj_est.tum" -a --delta 1 --delta_unit f --save_results "$ALG_DIR/evo_rpe.zip" >"$ALG_DIR/evo_rpe.txt" 2>&1 || true
  else
    echo "[WARN] Skip evo for $alg because trajectory export failed" >"$ALG_DIR/evo_skip.txt"
  fi

done

python3 "$SUM_PY" --root "$OUT_ROOT" --algs "$ALGORITHMS" >"$OUT_ROOT/summarize.log" 2>&1 || true

echo ""
echo "[DONE] Static rosbag evaluation finished."
echo "[DONE] Check summary: $OUT_ROOT/summary.md"
