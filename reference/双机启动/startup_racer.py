#!/usr/bin/env python3
"""
RACER 多终端自动化启动脚本（对应 启动.md 的终端1-8）
使用 tmux 管理多个终端，自动执行环境准备，保留手动确认执行关键命令
"""

import subprocess
import time
import sys
import os

AUTO_TRIGGER_CMD = (
    "bash -lc \"source ~/.bashrc; source ~/racer/devel/setup.bash; "
    "echo '[auto_trigger] waiting for wait for trigger x3...'; "
    "rostopic echo /rosout | grep -m3 'wait for trigger\\.'; "
    "echo '[auto_trigger] publishing trigger to /move_base_simple/goal'; "
    "rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "
    "'{header: {stamp: now, frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'\""
)

# 对应 racer/启动.md 的终端1-9（新增自动触发）
TERMINALS = [
    {
        "name": "t1_px4_multi_vehicle",
        "setup_cmds": ["source ~/.bashrc"],
        "cd": "cd ~/PX4_Firmware",
        "main_cmd": "roslaunch px4 multi_vehicle.launch"
    },
    {
        "name": "t2_comm_iris0",
        "setup_cmds": ["source ~/.bashrc"],
        "cd": "cd ~/XTDrone/communication",
        "main_cmd": "python multirotor_communication.py iris 0"
    },
    {
        "name": "t3_comm_iris1",
        "setup_cmds": ["source ~/.bashrc"],
        "cd": "cd ~/XTDrone/communication",
        "main_cmd": "python multirotor_communication.py iris 1"
    },
    {
        "name": "t4_pose_gt",
        "setup_cmds": ["source ~/.bashrc"],
        "cd": "cd ~/XTDrone/sensing/pose_ground_truth",
        "main_cmd": "python get_local_pose.py iris 2"
    },
    {
        "name": "t5_px4ctrl",
        "setup_cmds": [
            "source ~/.bashrc",
            "chmod +x /home/hua/racer/src/RACER/realflight_modules/px4ctrl/launch/increase_frequency.sh",
            "chmod +x /home/hua/racer/src/RACER/realflight_modules/px4ctrl/launch/takeoff.sh",
            "source ~/racer/devel/setup.bash"
        ],
        "cd": "cd ~/racer",
        "main_cmd": "roslaunch px4ctrl multi_ctrl.launch"
    },
    {
        "name": "t6_rviz",
        "setup_cmds": ["source ~/.bashrc", "source ~/racer/devel/setup.bash"],
        "cd": "cd ~/racer",
        "main_cmd": "roslaunch exploration_manager rviz.launch"
    },
    {
        "name": "t7_bridge",
        "setup_cmds": ["source ~/.bashrc", "source ~/swarm_ros_bridge_ws/devel/setup.bash"],
        "cd": "cd ~/swarm_ros_bridge_ws",
        "main_cmd": "roslaunch swarm_ros_bridge ros_bridge_realworld.launch"
    },
    {
        "name": "t8_exploration",
        "setup_cmds": ["source ~/.bashrc", "source ~/racer/devel/setup.bash"],
        "cd": "cd ~/racer",
        "main_cmd": "roslaunch exploration_manager swarm_exploration_gazebo.launch 2>&1 | tee ~/sim_log/exploration_log_$(date +%Y%m%d_%H%M%S).txt"
    },
    {
        "name": "t9_auto_trigger",
        "setup_cmds": [],
        "cd": "",
        "main_cmd": AUTO_TRIGGER_CMD
    }
]

SESSION_NAME = "racer_startup"
SETUP_DELAY = 0.3
CMD_DELAY = 1.0
WINDOW_DELAY = 2.0


def run_tmux_cmd(args):
    """执行 tmux 命令"""
    try:
        result = subprocess.run(["tmux"] + args, capture_output=True, text=True, timeout=5)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"x tmux命令超时: {' '.join(args)}")
        return False
    except FileNotFoundError:
        print("x 错误: tmux 未安装。请先安装: sudo apt-get install tmux")
        return False


def send_keys(window, cmd, execute=False):
    """发送按键到 tmux 窗口"""
    args = ["send-keys", "-t", window, cmd]
    if execute:
        args.append("Enter")
    return run_tmux_cmd(args)


def create_session():
    """创建新的 tmux session"""
    if run_tmux_cmd(["has-session", "-t", SESSION_NAME]):
        print(f"! Session '{SESSION_NAME}' 已存在，先删除...")
        run_tmux_cmd(["kill-session", "-t", SESSION_NAME])
        time.sleep(0.5)

    if run_tmux_cmd(["new-session", "-d", "-s", SESSION_NAME]):
        print(f"v 创建 tmux session: {SESSION_NAME}")
        return True

    print("x 创建 session 失败")
    return False


def setup_terminal(window_idx, term_config):
    """配置单个终端窗口"""
    window = f"{SESSION_NAME}:{window_idx}"
    name = term_config["name"]

    print(f"\n[{name}] 窗口 {window_idx}:")

    if window_idx > 0:
        if not run_tmux_cmd(["new-window", "-t", SESSION_NAME, "-n", name]):
            print("  x 创建窗口失败")
            return False
        time.sleep(0.2)
    else:
        run_tmux_cmd(["rename-window", "-t", window, name])

    if term_config["setup_cmds"]:
        print("  执行环境准备...")
        for cmd in term_config["setup_cmds"]:
            if not send_keys(window, cmd, execute=True):
                print(f"    x 执行失败: {cmd}")
                return False
            time.sleep(SETUP_DELAY)

    if term_config["cd"]:
        if not send_keys(window, term_config["cd"], execute=True):
            print("  x cd 失败")
            return False
        time.sleep(CMD_DELAY)

    print("  执行主命令...")
    if not send_keys(window, term_config["main_cmd"], execute=True):
        print("  x 执行主命令失败")
        return False

    print("  v 已启动")
    return True


def start_all():
    """启动所有终端"""
    print("=" * 70)
    print("RACER 启动脚本（终端1-9）")
    print("=" * 70)

    if not create_session():
        return False

    for idx, term_config in enumerate(TERMINALS):
        if not setup_terminal(idx, term_config):
            print(f"\nx 终端 {idx} ({term_config['name']}) 配置失败")
            return False
        time.sleep(WINDOW_DELAY)

    print("\n" + "=" * 70)
    print("v 所有终端配置完成")
    print("=" * 70)
    print("\n使用方法:")
    print("  窗口切换: Ctrl+B, n/p 或 Ctrl+B, <数字>")
    print(f"  查看列表: tmux list-windows -t {SESSION_NAME}")
    print("  主命令已自动执行（包含 t9 自动触发）")
    print("\n建议执行顺序: 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8")
    print("触发逻辑: t9 检测到 wait for trigger. 出现 3 次后自动发布一次 trigger")
    print("停止所有: python3 startup_racer.py stop")
    print("=" * 70 + "\n")

    os.execvp("tmux", ["tmux", "attach-session", "-t", SESSION_NAME])
    return True


def status():
    """显示当前 session 状态"""
    print("检查 session 状态...")
    run_tmux_cmd(["list-windows", "-t", SESSION_NAME])


def stop():
    """停止所有终端"""
    if run_tmux_cmd(["kill-session", "-t", SESSION_NAME]):
        print(f"v 已停止 session: {SESSION_NAME}")
    else:
        print("x 停止失败或 session 不存在")


def main():
    if len(sys.argv) < 2:
        print(f"用法: {sys.argv[0]} <start|stop|status>")
        print("  start  - 启动所有终端")
        print("  stop   - 停止所有终端")
        print("  status - 显示当前状态")
        sys.exit(1)

    action = sys.argv[1].lower()

    if action == "start":
        success = start_all()
        sys.exit(0 if success else 1)
    elif action == "stop":
        stop()
    elif action == "status":
        status()
    else:
        print(f"未知命令: {action}")
        sys.exit(1)


if __name__ == "__main__":
    main()
