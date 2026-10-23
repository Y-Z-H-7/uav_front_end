# 地面站多无人机控制前端进度记录（2026-10-13）

## 1. 项目定位

本项目是基于 PySide6 的地面站控制前端，用于通过 SSH 编排多架无人机上的 ROS 节点，并在本机通过 ROS 话题发布控制请求、订阅基础状态。

当前地面站配置了 3 架无人机：

| 编号 | 显示名称 | SSH 目标 | 远端 tmux 会话 | 机间桥接 |
|---|---|---|---|---|
| 1 | `nuc` | `nuc@192.168.0.121` | `nuc_drone` | `bridge_drone1.launch` |
| 2 | `fly` | `fly@192.168.0.195` | `fly_drone` | `bridge_drone2.launch` |
| 3 | `nuc2` | `nuc2@192.168.0.243` | `nuc2_drone` | `bridge_drone3.launch` |

无人机配置集中在 `mission_orchestrator.py` 的 `MissionOrchestrator.DRONES` 中。UI 会根据该字典动态生成状态卡、选择按钮和结束/降落按钮，增加或移除无人机时不需要修改界面布局代码。

## 2. 已实现功能

### 2.1 SSH 连接与状态检查

- 统一通过本机 OpenSSH 命令执行远端操作，不在前端保存 SSH 密码，也不使用 Paramiko。
- SSH 命令使用 `BatchMode=yes` 和 8 秒连接超时，因此远端必须预先配置好公钥免密登录。
- “SSH连接”按钮使用 `StrictHostKeyChecking=accept-new` 执行轻量 `echo ok`，用于首次连接时写入主机密钥并验证可达性。
- “检查SSH状态”按钮会查询：
  - `whoami`
  - `hostname`
  - 当前工作目录
  - 对应 tmux 会话是否存在
- UI 状态卡显示 SSH 用户、远端主机名、tmux 会话状态和最近检查时间。
- 未启动基础链路时 tmux 会话不存在属于正常状态，不会被当作 SSH 失败。

### 2.2 基础链路 tmux 编排

“启动基础链路”会对选中的无人机并行执行，单架无人机内部按顺序创建和启动窗口：

| 窗口 | 名称 | 默认行为 |
|---|---|---|
| 1 | `px4` | 启动 MAVROS/PX4，存在 `/dev/ttyACM0` 时自动设置权限 |
| 2 | `livox` | 启动 Livox MID360 驱动 |
| 3 | `fastlio` | 启动 Fast-LIO 建图 |
| 4 | `offb_node` | 启动传感器位置转换节点 |
| 5 | `ego_transfer` | 启动 `ego_transfer.py` |
| 6 | `trans` | 启动偏置转换节点 |
| 7 | `bridge` | 启动对应无人机的 `swarm_ros_bridge` 桥接 |
| 8 | `cxr_egoctrl` | 只预置命令，等待“启动程控” |
| 9 | `exploration` | 只预置命令，等待“启动探索” |

具体命令、工作目录和 ROS 环境脚本均按无人机写在 `DroneConfig.windows` 中：

- `nuc`、`fly` 使用 `~/racer` 和对应的 `bridge_drone1/2.launch`。
- `nuc2` 的 Livox、Fast-LIO 使用独立工作空间，并启动 `bridge_drone3.launch`。
- 旧 tmux 同名会话会在启动前清理。
- 第一个窗口启动后等待 7 秒，后续窗口之间等待 3 秒。
- 基础链路完成后会校验 tmux 会话是否存在，并在日志中报告成功状态和最后处理的窗口。

### 2.3 程控、探索和终端查看

- “启动程控”向选中无人机的 `cxr_egoctrl` 窗口发送 Enter。
- “启动探索”向选中无人机的 `exploration` 窗口发送 Enter。
- 两类阶段默认按 `(3, 2, 1)` 的编号顺序触发，架间间隔为 1 秒。
- “查看终端”会为选中的无人机打开本地终端，并通过 SSH 附加到远端 tmux 会话。
- “结束任务”会杀掉指定无人机的 tmux 会话，并验证会话已消失。
- “启动桥接节点”会在本地终端启动 `bridge_node_gcs.launch`。
- “启动探索 RViz”会在本地终端启动 `exploration_manager rviz.launch`。

> 当前“查看终端”不是自动起飞流程；`run_takeoff_sequence_placeholder()` 仅保留阶段日志占位实现，尚未接入真实起飞 ROS 控制接口。

### 2.4 ROS 降落控制

`ros_adapter.py` 负责本机 ROS 适配：

- 降落话题：
  - `/uav1/mission/land_request`
  - `/uav2/mission/land_request`
  - `/uav3/mission/land_request`
- 消息类型为 `std_msgs/Bool`，降落请求使用 `Bool(data=True)`。
- 每架目标机异步连发 10 次，间隔 100 ms，用于降低 UDP 丢包导致的漏触发风险。
- 多架无人机可以并发发送，不阻塞 Qt 主线程。
- 机间话题转发由 `swarm_ros_bridge` 负责，地面站不依赖跨机器 ROS master。
- ROS 不可用时进入占位模式，UI 会记录“ROS 未就绪，无法发布降落请求”。

### 2.5 状态显示与日志

- 订阅每架无人机的：
  - `/<uav>/mavros/battery`
  - `/<uav>/mavros/local_position/pose`
- 电量按 1 Hz 更新，位置按 5 Hz 更新。
- UI 显示电量、电压和 `(x, y, z)` 位置。
- 所有任务日志带时间戳，同时输出到：
  - Qt 日志窗口
  - 启动地面站的 stdout
  - `logs/groundstation_YYYYMMDD.log`
- Qt 任务通过 `QThread` 执行，避免 SSH、tmux 和降落连发阻塞界面。
- 同一时间只允许一个后台任务执行；任务进行期间相关按钮会禁用。

### 2.6 通信链路状态监控（2026-10-22）

新增 `link_monitor.py` 和界面“链路监控”标签页，把“地面站收到了什么、多久一次、占多少带宽、
延迟多少”变成可读指标，对应开题中“新增链路信息监控”一节。

- **监控清单来源**：启动时读取 `swarm_ros_bridge/config/ros_topics_gcs.yaml` 的 `recv_topics`，
  用 `srcIP`（robot1/robot2/robot3）在 `IP` 表里查地址，再用地址反查本工程的 `DRONES`，
  得到“哪架飞机 → 哪些主题”。配置不存在时退化为只监控电量/位姿，界面照常可用；
  路径可用环境变量 `SWARM_BRIDGE_GCS_CONFIG` 覆盖。
- **监控范围**：每架 14 条接收主题（探索状态、配对协商、B 样条轨迹、栅格、地图块、
  可视化 Marker、点云、电量、位姿），3 架共 42 行。
- **指标口径**：
  - 频率 Hz：最近 2 秒窗口内平均到达间隔的倒数（窗口里只剩一条时，用它与上一条的间隔估算）；
  - 带宽 KB/s：窗口内载荷字节数 / 窗口时长，字节数取 `len(msg._buff)`，即链路上真实的序列化大小；
  - 时延 ms：只对带 `header` 的电量/位姿计算，并先减去“窗口内最小单向时延”估计的时钟偏差，
    得到排队/拥塞造成的额外时延（两端时钟已同步时即为真实单向时延）；
  - 抖动 ms：到达间隔标准差；静默 s：距最后一条消息的秒数。
- **状态判定**：等待（从未收到）/ 采集中（累计不足 5 条）/ 正常 / 稀疏（低于峰值 50%，
  且窗口内至少 3 条）/ 中断（超过 `max(3 s, 4 个周期)` 无消息）。
- **订阅方式**：电量/位姿沿用原有强类型订阅（顺带读 `header.stamp`）；其余主题用
  `rospy.AnyMsg` 订阅，只取原始缓冲长度、不反序列化，因此不需要把 RACER 的
  `exploration_manager` / `bspline` / `plan_env` 消息包 source 进前端虚拟环境。
- **界面与导出**：顶部每架飞机一行汇总（在线主题数 / 总带宽 / 时钟偏差 / 中断条数），
  下方 42 行明细表，500 ms 刷新一次；“清零统计”用于开始新的对照实验，“导出 CSV”把当前
  快照写到 `logs/link_monitor_YYYYmmdd_HHMMSS.csv`（UTF-8 BOM，Excel 可直接打开）。
- **线程模型**：`LinkMonitor` 内部用锁保护，ROS 回调（rospy 线程）只做 `record()`，
  UI 定时器只做 `snapshot()` 读取，两边不共享 Qt 对象。

## 3. 启动与运行条件

### 3.1 地面站启动

推荐在 Ubuntu/WSL 环境执行：

```bash
cd ~/uav_front_end
./run_ui.sh
```

`run_ui.sh` 会依次：

1. 加载 `/opt/ros/noetic/setup.bash`；
2. 激活项目 `.venv`；
3. 检查 `rospy`、`rospkg` 和 `PySide6`；
4. 检查 ROS master，不可达时后台启动 `roscore`；
5. 使用 X11 默认启动 `ui_app.py`。

### 3.2 SSH 前置配置

在启动地面站前，确认本机可以免密执行：

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 nuc@192.168.0.121 "echo CONNECT_OK"
ssh -o BatchMode=yes -o ConnectTimeout=8 fly@192.168.0.195 "echo CONNECT_OK"
ssh -o BatchMode=yes -o ConnectTimeout=8 nuc2@192.168.0.243 "echo CONNECT_OK"
```

如果连接失败，先检查：

- 远端 IP 是否正确；
- 本机公钥是否已写入远端 `~/.ssh/authorized_keys`；
- 远端 SSH 服务、防火墙和网络是否正常；
- 本地 `ssh` 命令是否在 PATH 中。

### 3.3 远端环境

每架无人机需要具备：

- `tmux`；
- ROS Noetic；
- `mavros`、Livox 驱动、Fast-LIO、`cxr_egoctrl_v1`；
- `swarm_ros_bridge_ws`；
- 与配置路径一致的工作空间和 launch 文件；
- PX4 对应串口设备（通常为 `/dev/ttyACM0`）。

## 4. 推荐操作顺序

1. 确认各机网络、PX4 和 SSH 免密登录正常。
2. 启动地面站并点击“SSH连接”。
3. 点击“检查SSH状态”，确认目标机可达。
4. 在状态卡中取消不参与本次任务的无人机。
5. 点击“启动基础链路”，等待窗口 1-7 启动完成。
6. 点击“查看终端”，现场观察远端 tmux 输出。
7. 确认飞机状态稳定后点击“启动程控”。
8. 确认任务条件满足后点击“启动探索”。
9. 需要结束某架无人机时使用对应“结束任务”按钮。
10. 需要降落时使用单机降落按钮，或使用“全部降落”向当前选中无人机下发请求。

## 5. 当前限制与待完成事项

1. **真实起飞控制尚未实现**：当前没有自动起飞 ROS 服务/话题，相关按钮只负责查看/附加 tmux 终端。
2. **降落链路仍需机载端配合**：机载控制程序必须订阅对应的 `std_msgs/Bool` 降落话题，并由 `swarm_ros_bridge` 正确转发。
3. **降落只有发送结果，没有执行状态回读**：目前只能确认请求是否由地面站发布，尚未显示“已接收、执行中、完成、失败”等状态。
4. **配置仍是固定地址**：无人机 SSH 目标和远端工作空间写在 Python 配置中，尚未提供 UI 或 YAML 配置入口。
5. **启动前置条件检查仍可加强**：当前只检查 SSH 和 tmux 会话，尚未自动检查远端 ROS 包、串口、桥接 launch 文件等。
6. **本地终端启动依赖图形环境**：查看终端、桥接节点和 RViz 需要可用的 Linux 终端模拟器与 X11/Wayland 图形环境。
7. **链路监控的时延只覆盖带 header 的主题**：自定义消息（探索状态、B 样条等）走 `AnyMsg`，
   只能统计频率和带宽；若要测它们的时延，需要改成强类型订阅或先给消息补 `header`。
8. **时钟偏差是估计值**：按“窗口内最小单向时延”估计，机上时钟漂移或重启后需要约 1 分钟重新收敛；
   若两端已用 NTP/PTP 同步，该值应接近 0。
9. **监控本身有开销**：每条被监控主题会在本机 loopback 上多一份订阅，点云等大主题会带来额外拷贝，
   首次实机联调时建议观察地面站 CPU 占用。
10. **未建立自动化硬件联调测试**：目前可做 Python 语法检查和无 ROS 占位模式验证，真实 SSH、ROS、PX4 和桥接链路仍需实机测试。

## 6. 已验证内容

- `mission_orchestrator.py`、`ui_app.py`、`ros_adapter.py` 已按当前代码结构完成联动。
- UI 支持 3 架无人机动态生成状态卡和控制按钮。
- 基础链路支持按选中无人机并行、单机内部串行创建 tmux 窗口。
- SSH 检查、tmux 会话检查、旧会话清理、阶段触发和会话结束均有明确成功/失败日志。
- 降落消息使用 Bool 类型异步连发，电量和位置订阅已接入 UI。
- 链路监控已通过离线测试覆盖：频率/带宽/时延/抖动/静默、慢主题与单样本场景、时钟偏差扣除、
  状态流转（等待→采集中→正常→稀疏→中断）、CSV 导出、配置解析与界面渲染
  （`tests/test_link_monitor.py` 17 项、`tests/test_link_ui.py` 5 项，均不依赖真实 ROS）。
- 文档内容以当前仓库代码为准；远端实机配置变化时，需要同步更新 `mission_orchestrator.py` 中的 `DRONES` 配置。
