#!/usr/bin/env bash

# 地面站 UI 一键启动脚本
# 功能：
# 1. 加载 ROS 环境
# 2. 激活 Python 虚拟环境
# 3. 执行基础依赖自检
# 4. 自动检查并拉起 ROS master（roscore）
# 5. 启动 UI 程序

set -euo pipefail

# 允许从干净的 WSL shell 启动；ROS 的环境脚本在 nounset 模式下
# 会读取这些变量。
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
VENV_PATH="${SCRIPT_DIR}/.venv"
UI_ENTRY="${SCRIPT_DIR}/ui_app.py"
ROSCORE_LOG="/tmp/uav_front_end_roscore.log"

log() {
  echo "[run_ui] $*"
}

fail() {
  echo "[run_ui][ERROR] $*" >&2
  exit 1
}

# 检查 ROS master 是否可达。
# 返回值：0 表示可达，1 表示不可达。
check_master() {
  python - <<'PY'
import os
import socket
from urllib.parse import urlparse
master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
parsed = urlparse(master_uri)
host = parsed.hostname or "localhost"
port = parsed.port or 11311
try:
    with socket.create_connection((host, port), timeout=1.0):
        print(f"ROS master 可达: {master_uri}")
except OSError:
    raise SystemExit(1)
PY
}

log "启动前环境检查..."
[[ -f "${ROS_SETUP}" ]] || fail "未找到 ROS 环境脚本: ${ROS_SETUP}"
[[ -d "${VENV_PATH}" ]] || fail "未找到虚拟环境目录: ${VENV_PATH}"
[[ -f "${UI_ENTRY}" ]] || fail "未找到 UI 入口文件: ${UI_ENTRY}"

# 1) ROS 环境
# shellcheck disable=SC1090
source "${ROS_SETUP}"
log "ROS 环境已加载: ${ROS_SETUP}"

# 2) Python 虚拟环境
# shellcheck disable=SC1091
source "${VENV_PATH}/bin/activate"
log "虚拟环境已激活: ${VENV_PATH}"

# Debian/Ubuntu 的 ROS Python 包位于系统 dist-packages；部分通过旧路径
# 创建的虚拟环境不会自动继承该目录，因此显式加入 ROS 的 Python 路径。
ROS_PYTHON_PATHS=(
  "/opt/ros/noetic/lib/python3/dist-packages"
  "/usr/lib/python3/dist-packages"
)
for ROS_PYTHON_PATH in "${ROS_PYTHON_PATHS[@]}"; do
  if [[ -d "${ROS_PYTHON_PATH}" ]]; then
    export PYTHONPATH="${ROS_PYTHON_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
done

# 3) 依赖自检（快速失败，避免 UI 启动后才报错）
python - <<'PY'
import importlib
mods = ["rospy", "rospkg", "PySide6", "sensor_msgs.msg", "geometry_msgs.msg", "std_msgs.msg"]
missing = []
for m in mods:
    try:
        importlib.import_module(m)
    except Exception:
        missing.append(m)
if missing:
    raise SystemExit("缺少依赖: " + ", ".join(missing))
print("依赖检查通过: " + ", ".join(mods))
PY

# 4) 自动拉起 roscore（如果 master 未运行）
if check_master; then
  log "ROS master 已就绪，无需重复启动 roscore。"
else
  log "ROS master 不可达，正在后台启动 roscore..."
  nohup roscore > "${ROSCORE_LOG}" 2>&1 &
  ROSCORE_PID=$!
  log "roscore 已启动，PID=${ROSCORE_PID}，日志=${ROSCORE_LOG}"

  # 最多等待 10 秒，直到 master 可达。
  MASTER_READY=0
  for _ in $(seq 1 20); do
    if check_master >/dev/null 2>&1; then
      MASTER_READY=1
      break
    fi
    sleep 0.5
  done

  if [[ "${MASTER_READY}" -ne 1 ]]; then
    fail "roscore 启动超时，请检查日志: ${ROSCORE_LOG}"
  fi
  log "ROS master 已就绪。"
fi

# 5) 启动 UI
log "启动 UI: ${UI_ENTRY}"
# 解决 WSL2 / Wayland 兼容性问题，强制使用 X11 后端
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec python "${UI_ENTRY}"
