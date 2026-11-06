#!/usr/bin/env bash
# Desktop MIME handler for saved RViz map snapshots.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${PROJECT_DIR}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/rvizlog_viewer.log"

if [[ $# -ne 1 || ! -f "$1" ]]; then
  zenity --error --text="请选择有效的 RVizlog .bag 地图文件" 2>/dev/null || true
  exit 1
fi

export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source /opt/ros/noetic/setup.bash
source "${PROJECT_DIR}/.venv/bin/activate"
for ROS_PATH in /opt/ros/noetic/lib/python3/dist-packages /usr/lib/python3/dist-packages; do
  [[ ! -d "${ROS_PATH}" ]] || export PYTHONPATH="${ROS_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
done
VENV_SITE="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
export PYTHONPATH="${VENV_SITE}${PYTHONPATH:+:${PYTHONPATH}}"

if ! python "${PROJECT_DIR}/tools/rvizlog_viewer.py" "$1" >>"${LOG_FILE}" 2>&1; then
  zenity --error --text="地图打开失败，请查看 ${LOG_FILE}" 2>/dev/null || true
  exit 1
fi
