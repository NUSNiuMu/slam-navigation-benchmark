#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 [liosam|fastlio|pointlio|fasterlio]"
  exit 1
fi

SLAM_TYPE="$1"
RUN_SLAM_SH="${RUN_SLAM_SH:-/home/niumu/run_slam.sh}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
NAV_WS_SETUP="${NAV_WS_SETUP:-/home/niumu/ws_livox/devel/setup.bash}"
MAP_YAML="${MAP_YAML:-/home/niumu/ws_livox/src/mid360_navigation/maps/map.yaml}"
SHOW_RVIZ="${SHOW_RVIZ:-true}"

case "${SLAM_TYPE}" in
  liosam)
    ODOM_SRC="/odometry/imu"
    CLOUD_SRC="/lio_sam/mapping/cloud_registered_raw"
    SCAN_FRAME="base_link"
    ;;
  fastlio)
    ODOM_SRC="/Odometry"
    CLOUD_SRC="/cloud_registered"
    SCAN_FRAME="base_link"
    ;;
  pointlio)
    ODOM_SRC="/aft_mapped_to_init"
    # Prefer body-frame cloud for local obstacle updates to avoid
    # map->base_link retransform lag before pointcloud_to_laserscan.
    CLOUD_SRC="/cloud_registered_body"
    SCAN_FRAME="base_link"
    ;;
  fasterlio)
    ODOM_SRC="/Odometry"
    # Prefer body-frame cloud for local obstacle updates to avoid
    # map->base_link retransform lag before pointcloud_to_laserscan.
    CLOUD_SRC="/cloud_registered_body"
    SCAN_FRAME="base_link"
    ;;
  *)
    echo "Unsupported SLAM type: ${SLAM_TYPE}"
    echo "Use: liosam | fastlio | pointlio | fasterlio"
    exit 1
    ;;
esac

if [[ ! -x "${RUN_SLAM_SH}" ]]; then
  echo "[ERROR] Cannot execute ${RUN_SLAM_SH}"
  exit 1
fi

if [[ ! -f "${ROS_SETUP}" || ! -f "${NAV_WS_SETUP}" ]]; then
  echo "[ERROR] Missing setup file: ${ROS_SETUP} or ${NAV_WS_SETUP}"
  exit 1
fi

kill_owned_processes() {
  pkill -f "bash ${RUN_SLAM_SH} ${SLAM_TYPE}" >/dev/null 2>&1 || true
  pkill -f "roslaunch mid360_navigation navigation.launch" >/dev/null 2>&1 || true
  # Use short patterns (no argument matching) so they actually hit running processes
  pkill -f "move_base/move_base" >/dev/null 2>&1 || true
  pkill -f "pointcloud_to_laserscan/pointcloud_to_laserscan_node" >/dev/null 2>&1 || true
  pkill -f "map_server/map_server" >/dev/null 2>&1 || true
  pkill -f "rviz.*nav\.rviz" >/dev/null 2>&1 || true
  pkill -f "topic_tools/relay.*(/slam/odom|/slam/cloud_registered)" >/dev/null 2>&1 || true

  case "${SLAM_TYPE}" in
    liosam)
      pkill -f "roslaunch lio_sam run6axis.launch" >/dev/null 2>&1 || true
      # Kill LIO-SAM node binaries directly
      pkill -f "lio_sam_imuPreintegration" >/dev/null 2>&1 || true
      pkill -f "lio_sam_imageProjection" >/dev/null 2>&1 || true
      pkill -f "lio_sam_featureExtraction" >/dev/null 2>&1 || true
      pkill -f "lio_sam_mapOptmization" >/dev/null 2>&1 || true
      ;;
    fastlio)
      pkill -f "roslaunch fast_lio mapping_mid360.launch" >/dev/null 2>&1 || true
      ;;
    pointlio)
      pkill -f "roslaunch point_lio mapping_mid360.launch" >/dev/null 2>&1 || true
      ;;
    fasterlio)
      pkill -f "roslaunch faster_lio mapping_mid360.launch" >/dev/null 2>&1 || true
      ;;
  esac
}

kill_matching_pids() {
  local signal="$1"
  shift
  local pattern
  local pid
  local -a pids=()

  for pattern in "$@"; do
    while IFS= read -r pid; do
      [[ -n "${pid}" ]] || continue
      pids+=("${pid}")
    done < <(pgrep -f "${pattern}" || true)
  done

  if [[ ${#pids[@]} -eq 0 ]]; then
    return 0
  fi

  for pid in "${pids[@]}"; do
    kill "-${signal}" "${pid}" >/dev/null 2>&1 || true
  done
}

kill_owned_processes_hard() {
  local patterns=(
    "roslaunch mid360_navigation navigation.launch"
    "/opt/ros/noetic/lib/move_base/move_base"
    "/opt/ros/noetic/lib/pointcloud_to_laserscan/pointcloud_to_laserscan_node"
    "/opt/ros/noetic/lib/map_server/map_server"
    "/opt/ros/noetic/lib/rviz/rviz -d /home/niumu/ws_livox/src/mid360_navigation/rviz/nav.rviz"
    "/opt/ros/noetic/lib/topic_tools/relay .* /slam/odom"
    "/opt/ros/noetic/lib/topic_tools/relay .* /slam/cloud_registered"
    "bash ${RUN_SLAM_SH} ${SLAM_TYPE}"
  )

  case "${SLAM_TYPE}" in
    liosam)
      patterns+=("roslaunch lio_sam run6axis.launch")
      ;;
    fastlio)
      patterns+=("roslaunch fast_lio mapping_mid360.launch")
      ;;
    pointlio)
      patterns+=("roslaunch point_lio mapping_mid360.launch")
      ;;
    fasterlio)
      patterns+=("roslaunch faster_lio mapping_mid360.launch")
      ;;
  esac

  kill_matching_pids TERM "${patterns[@]}"
  sleep 1
  kill_matching_pids KILL "${patterns[@]}"
}

kill_owned_ros_nodes() {
  bash -lc "source '${ROS_SETUP}' && source '${NAV_WS_SETUP}' && \
    for n in /move_base /map_server /pointcloud_to_laserscan /rviz_nav \
             /lio_sam_imuPreintegration /lio_sam_imageProjection \
             /lio_sam_featureExtraction /lio_sam_mapOptmization /lio_sam_rviz \
             /robot_state_publisher /laserMapping; do \
      rosnode kill \"\$n\" >/dev/null 2>&1 || true; \
    done; \
    for n in \$(rosnode list 2>/dev/null | egrep '/(Odometry_relay_|cloud_registered_relay_|imu_relay_)' || true); do \
      rosnode kill \"\$n\" >/dev/null 2>&1 || true; \
    done" >/dev/null 2>&1 || true
}

echo "[0/4] Cleaning stale ROS nodes on current master"
bash -lc "source '${ROS_SETUP}' && source '${NAV_WS_SETUP}' && printf 'y\n' | rosnode cleanup >/dev/null 2>&1 || true"

# Kill stale nodes with the same names to avoid respawn loops caused by name collisions.
bash -lc "source '${ROS_SETUP}' && source '${NAV_WS_SETUP}' && \
  for n in \
    /lio_sam_imuPreintegration /lio_sam_imageProjection /lio_sam_featureExtraction /lio_sam_mapOptmization /lio_sam_rviz \
    /robot_state_publisher /move_base /map_server /pointcloud_to_laserscan /rviz_nav; do \
    rosnode kill \"\$n\" >/dev/null 2>&1 || true; \
  done; \
  for n in \$(rosnode list 2>/dev/null | egrep '/(Odometry_relay_|cloud_registered_relay_|imu_relay_)' || true); do \
    rosnode kill \"\$n\" >/dev/null 2>&1 || true; \
  done"
kill_owned_processes

sleep 1

PIDS=()
CLEANED_UP=0

terminate_group() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  # 1) Kill the process group (setsid makes PGID == PID)
  kill -TERM -"${pid}" >/dev/null 2>&1 || true
  # 2) Kill ALL processes in the session (catches roslaunch children
  #    like move_base that run in separate process groups)
  pkill -TERM -s "${pid}" >/dev/null 2>&1 || true
  sleep 2
  kill -KILL -"${pid}" >/dev/null 2>&1 || true
  pkill -KILL -s "${pid}" >/dev/null 2>&1 || true
}
cleanup() {
  (( CLEANED_UP )) && return 0
  CLEANED_UP=1
  echo "[CLEANUP] Terminating process groups..."
  for pid in "${PIDS[@]:-}"; do
    terminate_group "${pid}"
  done
  kill_owned_ros_nodes
  sleep 1
  kill_owned_processes
  sleep 1
  kill_owned_ros_nodes
  kill_owned_processes
  kill_owned_processes_hard
  sleep 1
  kill_owned_ros_nodes
  # Purge dead nodes from ROS master
  echo "[CLEANUP] Cleaning up ROS master..."
  bash -lc "source '${ROS_SETUP}' && source '${NAV_WS_SETUP}' && printf 'y\n' | rosnode cleanup >/dev/null 2>&1" || true
}
on_signal() {
  local sig="$1"
  echo ""
  echo "[STOP] Caught ${sig}, shutting down SLAM/navigation processes..."
  trap - EXIT INT TERM
  cleanup
  wait || true
  exit 130
}
trap cleanup EXIT
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM

echo "[1/4] Launching ${SLAM_TYPE}"
setsid bash -lc "exec '${RUN_SLAM_SH}' '${SLAM_TYPE}'" &
PIDS+=("$!")

sleep 3

echo "[2/4] Bridging odom: ${ODOM_SRC} -> /slam/odom"
setsid bash -lc "source '${ROS_SETUP}' && source '${NAV_WS_SETUP}' && exec rosrun topic_tools relay '${ODOM_SRC}' /slam/odom" &
PIDS+=("$!")

echo "[3/4] Bridging cloud: ${CLOUD_SRC} -> /slam/cloud_registered"
setsid bash -lc "source '${ROS_SETUP}' && source '${NAV_WS_SETUP}' && exec rosrun topic_tools relay '${CLOUD_SRC}' /slam/cloud_registered" &
PIDS+=("$!")

echo "[4/4] Launching navigation stack"
setsid bash -lc "source '${ROS_SETUP}' && source '${NAV_WS_SETUP}' && exec roslaunch mid360_navigation navigation.launch map_yaml:='${MAP_YAML}' odom_topic:=/slam/odom cloud_topic:=/slam/cloud_registered scan_frame:='${SCAN_FRAME}' show_rviz:='${SHOW_RVIZ}'" &
PIDS+=("$!")

wait "${PIDS[-1]}"
