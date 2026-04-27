#!/usr/bin/env python3
"""
NUC无人机自动化启动脚本
使用tmux管理多个终端，自动化环境准备，保留手动确认执行关键命令
"""

import subprocess
import time
import sys
import os

# 终端配置（共9个）
TERMINALS = [
    {
        "name": "1_px4",
        "setup_cmds": [],
        "cd": "",
        "main_cmd": "echo 8888 | sudo -S bash -c 'source /opt/ros/noetic/setup.bash && chmod 777 /dev/ttyACM0 && roslaunch mavros px4.launch'",
        "note": "PX4飞控（先开启PX4地面站）"
    },
    {
        "name": "2_livox",
        "setup_cmds": [],
        "cd": "",
        "main_cmd": "roslaunch livox_ros_driver2 msg_MID360.launch",
        "note": "雷达驱动"
    },
    {
        "name": "3_fastlio",
        "setup_cmds": [],
        "cd": "",
        "main_cmd": "roslaunch fast_lio mapping_mid360.launch",
        "note": "建图（等建图稳了再继续）"
    },
    {
        "name": "4_offb_node",
        "setup_cmds": ["source ~/racer/devel/setup.bash"],
        "cd": "",
        "main_cmd": "rosrun cxr_egoctrl_v1 offb_node",
        "note": "传感器位置vision_pose转换"
    },
    {
        "name": "5_ego_transfer",
        "setup_cmds": [],
        "cd": "cd ~/racer/src/scripts",
        "main_cmd": "python3 ego_transfer.py",
        "note": "camera_pose发布"
    },
    {
        "name": "6_trans",
        "setup_cmds": ["source ~/racer/devel/setup.bash"],
        "cd": "",
        "main_cmd": "rosrun cxr_egoctrl_v1 trans.py",
        "note": "偏置转换"
    },
    {
        "name": "7_cxr_egoctrl",
        "setup_cmds": ["source ~/racer/devel/setup.bash"],
        "cd": "",
        "main_cmd": "rosrun cxr_egoctrl_v1 cxr_egoctrl_v2",
        "note": "【程控】最后开！！"
    },
    {
        "name": "8_bridge",
        "setup_cmds": ["source ~/swarm_ros_bridge_ws/devel/setup.bash"],
        "cd": "",
        "main_cmd": "roslaunch swarm_ros_bridge bridge_drone1.launch",
        "note": "双机通信"
    },
    {
        "name": "9_exploration",
        "setup_cmds": ["source ~/racer/devel/setup.bash"],
        "cd": "",
        "main_cmd": "roslaunch exploration_manager swarm_exploration_realworld.launch 2>&1 | tee \"/home/nuc/文档/swarm$(date +%Y%m%d%H%M%S).log\"",
        "note": "【主探索】等飞机稳定后再开启"
    }
]

SESSION_NAME = "nuc_drone"
SETUP_DELAY = 0.3
CMD_DELAY = 1.0
START_INTERVAL = 1.0
MANUAL_START_NAMES = {"7_cxr_egoctrl", "9_exploration"}


def run_tmux_cmd(args):
    try:
        result = subprocess.run(["tmux"] + args, capture_output=True, text=True, timeout=5)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print(f"✗ tmux命令超时: {' '.join(args)}")
        return False
    except FileNotFoundError:
        print("✗ 错误: tmux未安装。请先安装: sudo apt-get install tmux")
        return False


def send_keys(window, cmd, execute=False):
    args = ["send-keys", "-t", window, cmd]
    if execute:
        args.append("Enter")
    return run_tmux_cmd(args)


def create_session():
    if run_tmux_cmd(["has-session", "-t", SESSION_NAME]):
        print(f"⚠ Session '{SESSION_NAME}' 已存在，先删除...")
        run_tmux_cmd(["kill-session", "-t", SESSION_NAME])
        time.sleep(0.5)

    if run_tmux_cmd(["new-session", "-d", "-s", SESSION_NAME]):
        print(f"✓ 创建tmux session: {SESSION_NAME}")
        return True
    else:
        print(f"✗ 创建session失败")
        return False


def setup_terminal(window_idx, term_config):
    window = f"{SESSION_NAME}:{window_idx}"
    name = term_config["name"]
    note = term_config.get("note", "")

    print(f"\n[{name}] {note}")
    print(f"  窗口: {window}")

    if window_idx > 0:
        if not run_tmux_cmd(["new-window", "-t", SESSION_NAME, "-n", name]):
            print(f"  ✗ 创建窗口失败")
            return False
        time.sleep(0.2)
    else:
        run_tmux_cmd(["rename-window", "-t", window, name])

    # 执行setup命令
    if term_config["setup_cmds"]:
        for cmd in term_config["setup_cmds"]:
            if not send_keys(window, cmd, execute=True):
                print(f"  ✗ 执行失败: {cmd}")
                return False
            time.sleep(SETUP_DELAY)

    # 执行cd命令
    if term_config["cd"]:
        if not send_keys(window, term_config["cd"], execute=True):
            print(f"  ✗ cd失败")
            return False
        time.sleep(CMD_DELAY)

    # 发送主命令（手动节点不执行，其余自动执行）
    main_cmd = term_config["main_cmd"]
    auto_execute = name not in MANUAL_START_NAMES
    if not send_keys(window, main_cmd, execute=auto_execute):
        print(f"  ✗ 输入命令失败")
        return False

    if auto_execute:
        print(f"  ✓ 已自动启动")
    else:
        print(f"  ✓ 准备完成，命令已输入（手动回车启动）")
    return True


def start_all():
    print(f"{'='*70}")
    print(f"NUC无人机启动脚本")
    print(f"{'='*70}")

    if not create_session():
        return False

    for idx, term_config in enumerate(TERMINALS):
        if not setup_terminal(idx, term_config):
            print(f"\n✗ 终端 {idx} 配置失败")
            return False
        time.sleep(START_INTERVAL)

    print(f"\n{'='*70}")
    print(f"✓ 所有终端配置完成！")
    print(f"{'='*70}")
    print(f"\n建议执行顺序:")
    print(f"  1-3: PX4 → 雷达 → 建图（等建图稳了）")
    print(f"  4-6: offb_node → ego_transfer → trans")
    print(f"  7:   程控（确认后开启）")
    print(f"  8-9: 通信 → 探索（飞机稳定后）")
    print(f"\n窗口切换: Ctrl+B, n(下) | Ctrl+B, p(上) | Ctrl+B, <数字>")
    print(f"停止所有: python3 nuc.py stop")
    print(f"{'='*70}\n")

    os.execvp("tmux", ["tmux", "attach-session", "-t", SESSION_NAME])
    return True


def stop():
    if run_tmux_cmd(["kill-session", "-t", SESSION_NAME]):
        print(f"✓ 已停止: {SESSION_NAME}")
    else:
        print(f"✗ 停止失败或session不存在")


def status():
    print(f"检查session状态...")
    run_tmux_cmd(["list-windows", "-t", SESSION_NAME])


def main():
    if len(sys.argv) < 2:
        print(f"用法: {sys.argv[0]} <start|stop|status>")
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
