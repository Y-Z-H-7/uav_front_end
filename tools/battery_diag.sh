#!/usr/bin/env bash
# 电量诊断：定位“电量显示不出来”卡在哪一层。
#
#   机载电脑上跑（看飞控→MAVROS 这一段）：
#       bash battery_diag.sh            # 自动用 /mavros/battery
#   地面站上跑（看桥接→地面站 这一段）：
#       bash battery_diag.sh 3          # 用 /uav3/mavros/battery
#
# 重点看第 2 节的 percentage / voltage / cell_voltage：
#   percentage=nan 或 0        → 飞控侧没算出电量（电池参数没标定 / 没接电池）
#   voltage=0、cell_voltage=[] → 飞控没上报电压，MAVROS 拿到的是空帧
#   id 在两个值之间跳（如 0 和 24）→ MAVROS 交替发布空帧和有效帧
set -u

DRONE="${1:-}"
if [ -n "$DRONE" ]; then
  TOPIC="/uav${DRONE}/mavros/battery"
else
  TOPIC="/mavros/battery"
fi

. /opt/ros/noetic/setup.bash 2>/dev/null

echo "== 主机 =="
hostname; whoami
echo
echo "== 1. battery 话题是否存在 =="
timeout 6 rostopic list 2>/dev/null | grep -i "mavros/battery" || echo "  （没有 battery 话题）"
echo
echo "== 2. 原始消息（取 3 条，只看关键字段）=="
timeout 10 rostopic echo -n 3 "$TOPIC" 2>&1 \
  | grep -E "^\s*(id|percentage|voltage|current|present|power_supply_status|cell_voltage):|^---" \
  | head -40
echo
echo "== 3. 发布者 / 订阅者 =="
timeout 6 rostopic info "$TOPIC" 2>&1 | sed -n '1,14p'
echo
echo "== 4. 频率 =="
timeout 8 rostopic hz "$TOPIC" 2>/dev/null | tail -2
echo
echo "== 5. 桥接进程 =="
pgrep -af "bridge_node" | grep -v pgrep | head -3 || true
echo
echo "== 6. 桥接配置里的 battery 条目 =="
CFG_DIR="$HOME/swarm_ros_bridge_ws/src/swarm_ros_bridge/config"
for cfg in "$CFG_DIR"/ros_topics_drone*.yaml "$CFG_DIR"/ros_topics_gcs.yaml; do
  [ -f "$cfg" ] || continue
  echo "  --- $(basename "$cfg")"
  grep -B1 -A3 "mavros/battery" "$cfg" 2>/dev/null | head -8
done
echo
echo "== 7. 对照：本机能不能看到 mavros 的其它话题 =="
timeout 6 rostopic list 2>/dev/null | grep -c "^/mavros/" || true
