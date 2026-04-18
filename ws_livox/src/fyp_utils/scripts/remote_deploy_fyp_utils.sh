#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-deploy}"

REMOTE_USER="${REMOTE_USER:-bingda}"
REMOTE_HOST="${REMOTE_HOST:-172.20.10.2}"
REMOTE_WS="${REMOTE_WS:-/home/${REMOTE_USER}/catkin_ws}"
REMOTE_SRC="${REMOTE_WS}/src"
REMOTE_PKG="${REMOTE_SRC}/fyp_utils"
REMOTE_BAG_DIR="${REMOTE_BAG_DIR:-/home/${REMOTE_USER}/PointCloudBag}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
USE_SSH_KEY="${USE_SSH_KEY:-0}"
SSH_PASSWORD="${SSH_PASSWORD:-${SSHPASS:-}}"

LOCAL_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_PKG_DIR="$(cd "${LOCAL_SCRIPT_DIR}/.." && pwd)"

SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o ConnectTimeout=8
)

ensure_password() {
  if [[ -z "${SSH_PASSWORD}" ]]; then
    echo "[ERROR] 未提供密码。请设置 SSH_PASSWORD 或 SSHPASS。" >&2
    exit 1
  fi
}

run_password_cmd() {
  ensure_password
  if command -v sshpass >/dev/null 2>&1; then
    sshpass -p "${SSH_PASSWORD}" "$@"
    return
  fi

  # 无 sshpass 时，使用 SSH_ASKPASS 维持无交互密码输入
  local askpass_script
  askpass_script="$(mktemp)"
  cat > "${askpass_script}" <<'EOS'
#!/usr/bin/env bash
printf '%s\n' "${SSH_PASSWORD}"
EOS
  chmod 700 "${askpass_script}"
  DISPLAY="${DISPLAY:-codex:0}" \
  SSH_PASSWORD="${SSH_PASSWORD}" \
  SSH_ASKPASS="${askpass_script}" \
  SSH_ASKPASS_REQUIRE=force \
  setsid -w "$@" < /dev/null
  local rc=$?
  rm -f "${askpass_script}"
  return ${rc}
}

remote_ssh() {
  local cmd="$1"
  if [[ "${USE_SSH_KEY}" == "1" ]]; then
    ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "$cmd"
  else
    run_password_cmd ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "$cmd"
  fi
}

sync_pkg() {
  # 使用本地 tar 包 + scp 上传，兼容 sshpass 和 SSH_ASKPASS 两种密码模式
  local tmp_tar
  tmp_tar="$(mktemp /tmp/fyp_utils_pkg.XXXXXX.tar)"
  tar -C "${LOCAL_PKG_DIR}" -cf "${tmp_tar}" .

  if [[ "${USE_SSH_KEY}" == "1" ]]; then
    scp "${SSH_OPTS[@]}" "${tmp_tar}" "${REMOTE_USER}@${REMOTE_HOST}:/tmp/fyp_utils_pkg.tar"
    ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '${REMOTE_PKG}' && tar -xf /tmp/fyp_utils_pkg.tar -C '${REMOTE_PKG}' && rm -f /tmp/fyp_utils_pkg.tar"
  else
    run_password_cmd scp "${SSH_OPTS[@]}" "${tmp_tar}" "${REMOTE_USER}@${REMOTE_HOST}:/tmp/fyp_utils_pkg.tar"
    run_password_cmd ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p '${REMOTE_PKG}' && tar -xf /tmp/fyp_utils_pkg.tar -C '${REMOTE_PKG}' && rm -f /tmp/fyp_utils_pkg.tar"
  fi

  rm -f "${tmp_tar}"
}

setup_key_auth() {
  ensure_password
  local key_path="${HOME}/.ssh/id_ed25519"

  if [[ ! -f "${key_path}" ]]; then
    ssh-keygen -t ed25519 -N "" -f "${key_path}"
  fi

  if ! command -v ssh-copy-id >/dev/null 2>&1; then
    echo "[ERROR] ssh-copy-id 不可用，请安装 openssh-client。" >&2
    exit 1
  fi

  run_password_cmd ssh-copy-id "${SSH_OPTS[@]}" -i "${key_path}.pub" "${REMOTE_USER}@${REMOTE_HOST}"
  echo "[OK] 已尝试配置免密登录。后续可使用: USE_SSH_KEY=1 $0 deploy"
}

deploy() {
  echo "[1/4] 连通性检查"
  remote_ssh "echo [REMOTE] connected to \$(hostname)"

  echo "[2/4] 远端目录准备"
  remote_ssh "mkdir -p '${REMOTE_SRC}' '${REMOTE_BAG_DIR}'"

  echo "[3/4] 同步 fyp_utils 包"
  sync_pkg

  echo "[4/4] 赋权并编译"
  remote_ssh "chmod +x '${REMOTE_PKG}/scripts/cmd_vel_smoother.py' '${REMOTE_PKG}/scripts/remote_deploy_fyp_utils.sh' && cd '${REMOTE_WS}' && /bin/bash -lc 'if [ -f \"${ROS_SETUP}\" ]; then source \"${ROS_SETUP}\"; elif [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; elif [ -f /opt/ros/melodic/setup.bash ]; then source /opt/ros/melodic/setup.bash; elif [ -f /opt/ros/kinetic/setup.bash ]; then source /opt/ros/kinetic/setup.bash; else echo \"[ERROR] no ROS setup.bash found\"; exit 1; fi; catkin_make'"

  echo "[DONE] 远端部署完成。"
  echo "[HINT] 远端启动命令: roslaunch fyp_utils record_dataset.launch"
}

case "${MODE}" in
  setup-key)
    setup_key_auth
    ;;
  deploy)
    deploy
    ;;
  *)
    echo "Usage: $0 [deploy|setup-key]"
    exit 1
    ;;
esac
