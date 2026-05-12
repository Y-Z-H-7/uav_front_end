# 开发进度记录（2026-04-23）

## 目标
完成 Qt 前端开发前的 Python 环境初始化（虚拟环境与依赖安装）。

## 已执行操作
1. 先安装 venv（Ubuntu/Debian）
 - apt update && apt install -y python3.8-venv
2. 创建并激活虚拟环境
 - cd /home/hua/uav_front_end
 - python3 -m venv .venv
 - source .venv/bin/activate
3. 在虚拟环境里安装依赖
 - python -m pip install -U pip
 - python -m pip install PySide6 pyyaml paramiko
4. 安装qt依赖
 - sudo apt install libxcb-cursor0

---

## 会话增量进展（2026-04-23，降落链路联调）

### 本次目标
1. 将前端降落按钮与机载状态机的降落触发链路打通。
2. 明确基于 swarm_ros_bridge 的跨机通信方式，避免依赖跨机 ROS master 配置。
3. 保持与原控制程序兼容（降落触发协议恢复为 Bool）。

### 已实现功能
1. 前端界面完成降落控制入口
 - 文件：ui_app.py
 - 已实现 3 个按钮：1号机降落、2号机降落、同时降落1和2。
 - 已实现日志窗口：记录时间戳、请求目标、发布结果。

2. 地面站通信适配层已接入 UI 并可发布降落请求
 - 文件：ros_adapter.py
 - 当前协议：std_msgs/Bool(data=True)。
 - 发布话题：/uav1/mission/land_request、/uav2/mission/land_request。
 - 初始化方式：按本机 ROS 环境初始化，不要求跨机 master 地址配置。

3. 启动脚本能力已完善
 - 文件：run_ui.sh
 - 已实现自动加载 ROS 环境、激活虚拟环境、依赖检查。
 - 已实现 ROS master 检查，master 不可达时自动后台启动 roscore，再拉起 UI。

4. 注释规范已升级
 - 本次改动代码均补充了详细注释（模块说明、关键流程、异常/边界解释）。

### 关键决策记录
1. 中间尝试过 String 协议以适配默认 bridge 示例，但最终已回退到原控制程序 Bool 协议。
2. 跨机通信推荐通过 swarm_ros_bridge topic 转发实现；不再走“跨机器 ROS master”部署模式。

### 当前待完成事项
1. 机载 v2 控制代码尚未修改：
 - 仍需在机载 reference/control/src/cxr_egoctrl_v2.cpp 中确认/接入 mission/land_request（std_msgs/Bool）触发链路。

2. 机载端 rosbridge 尚未配置：
 - 仍需在机载侧 ros_topics 配置中新增 /uav1/mission/land_request（必要时 /uav2）对应 recv_topics。
 - 仍需确认 bridge 类型分发已支持 std_msgs/Bool（若未支持需在 ros_sub_pub.hpp 增加并重编译）。

3. 地面站与机载两侧完成 ros_topics 联调：
 - 校验 srcIP/srcPort 对应关系与端口唯一性。

4. 在前端增加状态回读（已接收/执行中/完成/失败）与超时提示。

### 当前可验证项
1. run_ui.sh 可自动拉起 roscore 并启动 UI。
2. UI 可触发降落请求发布（地面站侧）。
3. Python 文件语法检查通过（ui_app.py、ros_adapter.py）。

---

## 会话增量进展（2026-05-12，手动 SSH 模式联调）

### 本次目标
1. 将前端从“内置自动 SSH 登录”切换为“手动 SSH 前置 + 前端触发命令”。
2. 在不改变主要按钮流程的前提下，完成 SSH 检测链路可用性验证。
3. 定位并解决“检查SSH显示基础状态异常”的根因。

### 已实现功能
1. mission_orchestrator.py 已改为 OpenSSH 命令驱动
 - 移除 paramiko 直连逻辑。
 - 统一通过本机 ssh 子进程执行远端命令。
 - 检测与执行参数：BatchMode=yes、ConnectTimeout=8。

2. UI/任务流程保持一致
 - 保留检查SSH、启动基础链路、启动程控(7)、启动探索(9)与降落按钮。
 - 检查SSH时新增基础探针输出（whoami/hostname/pwd/tmux）。

3. 联调目标已收敛为显式地址
 - 当前 DRONES 中两条 ssh_target 为 nvidia@192.168.18.100（同机占位验证）。
 - 便于在未完善 Host 别名前快速排查链路。

### 关键排障记录
1. 发现基础状态异常的直接原因不是网络不可达，而是非交互模式下认证失败。
2. 交互可登录 != BatchMode 可登录：
 - 前端后台执行无法输入密码，必须使用公钥免密。
3. 关键验证命令：
 - ssh -o BatchMode=yes -o ConnectTimeout=8 nvidia@192.168.18.100 "echo CONNECT_OK && whoami && hostname"

### 当前待完成事项
1. 远端完成/确认公钥免密配置（authorized_keys 与权限）。
2. 将双机占位目标替换为真实 1/2 号机（或 uav1/uav2 别名）。
3. 机载端按原计划继续完成降落状态回读、rosbridge 话题配置与联调。

### 当前可验证项
1. .venv/bin/python -m py_compile mission_orchestrator.py ui_app.py ros_adapter.py 通过。
2. 手动运行 check_ssh_connectivity() 可输出分步骤日志并定位认证问题。
3. 在免密条件满足时，BatchMode SSH 检测链路可通过。


---

## 会话增量进展（2026-05-12，tmux 基础链路联调与日志收敛）

### 本次目标
1. 让前端直接在远端构建 tmux 基础链路，不再依赖远端现成的 fly.py / nuc.py 脚本文件。
2. 将基础链路启动过程的输出压缩为“开始/结束”两行，并补充每台无人机最终执行到的窗口。
3. 在基础链路等待期间禁用“启动程控”和“启动探索”按钮，避免重复触发。

### 已实现功能
1. 基础链路由前端直接编排远端 tmux 窗口
 - 文件：mission_orchestrator.py
 - 远端通过 OpenSSH 直接创建/重建 tmux session，按窗口 1-9 组织命令。
 - 窗口 1-7 自动启动；窗口 8、9 仅预置命令，等待手动触发。
 - 1 号窗口与 2 号窗口间隔 7 秒，后续窗口间隔 3 秒。

2. 任务输出已经收敛
 - 基础链路、程控、探索、附着窗口任务都只保留开始/结束两行必要信息。
 - 基础链路结束日志会额外写出每台无人机最终执行到的窗口名，便于现场确认。

3. UI 任务按钮状态更清晰
 - 文件：ui_app.py
 - 基础链路执行期间，程控、探索、检查 SSH、基础链路、阶段日志入口会统一禁用，防止重复下发命令。
 - 任务结束后再恢复按钮状态。

### 关键决策记录
1. 不再把远端 `/home/hua/双机启动/nuc.py` 与 `fly.py` 作为前端依赖项；前端自己生成 tmux 启动序列。
2. 基础链路中先创建 detached tmux session，再通过 send-keys 写入命令，避免 shell 退出导致窗口跟着消失。
3. 对 PX4 启动命令增加 `/dev/ttyACM0` 存在性判断，减少无设备时的噪声报错。

### 当前待完成事项
1. 继续在实机上核对 tmux 窗口内容是否与预期一致。
2. 观察窗口 8/9 的手动触发是否完全符合现场节奏。
3. 后续如有需要，再把任务失败时的 stderr 进一步拆成按窗口展示。

### 当前可验证项
1. `python3 -m py_compile mission_orchestrator.py ui_app.py` 通过。
2. 基础链路启动日志会输出每台无人机对应的最终窗口名。
3. 基础链路等待间隔已经按 7 秒 / 3 秒生效。
