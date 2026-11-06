# 多机任务控制前端

基于 PySide6 的地面站多无人机控制前端，通过 SSH 远程编排无人机端 tmux 会话，
通过 ROS 话题下发降落指令，替代"人工 SSH + 手工回车 + 遥控器介入"的操作流程。

## 适用场景

多架无人机（当前支持 3 架）野外飞行测试的一键控制：启动软件栈、触发程控、启动探索、远程降落。

## 系统架构

```
地面站 (Linux)
├── ui_app.py                PySide6 Qt 桌面界面
├── mission_orchestrator.py  SSH 远程编排引擎
├── ros_adapter.py           ROS 通信适配层
├── link_monitor.py          链路指标采集（频率/带宽/时延/抖动）
└── run_ui.sh                一键启动脚本

无人机 (每机)
├── tmux session (9 窗口)    按序启动的软件栈
├── MAVROS + PX4             飞控通信
├── cxr_egoctrl_v2           EGO 轨迹跟踪控制器
├── swarm_ros_bridge         跨机 ROS 话题桥接
└── exploration_manager      探索规划器
```

## 依赖与环境

| 组件 | 说明 |
|---|---|
| Python | 3.8+ |
| GUI | PySide6 (Qt) |
| ROS | ROS Noetic（`rospy`、`rospkg`） |
| 远程 | OpenSSH 密钥认证，`BatchMode=yes` |
| 无人机端 | tmux（每机预装） |

## 快速开始

### 1. 前置准备

地面站到每架无人机的 SSH 密钥认证需提前配置：

```bash
ssh-copy-id nuc@<IP>
ssh-copy-id fly@<IP>
ssh-copy-id nuc2@<IP>
```

### 2. 配置无人机

编辑 `mission_orchestrator.py` 中 `DRONES` 字典，填入各机实际 IP：

```python
DRONES = {
    1: DroneConfig(
        display_name="nuc",
        ssh_target="nuc@192.168.0.121",   # ← 改成实际 IP
        session_name="nuc_drone",
        windows=(...),
    ),
    2: DroneConfig(
        display_name="fly",
        ssh_target="fly@192.168.0.195",   # ← 改成实际 IP
        session_name="fly_drone",
        windows=(...),
    ),
    3: DroneConfig(
        display_name="nuc2",
        ssh_target="nuc2@192.168.0.xxx",  # ← 改成实际 IP
        session_name="nuc2_drone",
        windows=(...),
    ),
}
```

各无人机可定义不同启动命令（`WindowDef`），适配不同硬件与 workspace 路径。

### 3. 启动

```bash
./run_ui.sh
```

脚本自动完成：加载 ROS 环境 → 激活虚拟环境 → 检查依赖 → 确保 roscore 运行 → 启动 UI。

## 界面布局

```
┌──────────────────────────────────┬─────────────────────────┐
│  多机任务控制前端                  │                         │
│                                  │  [ SSH连接 ]            │
│  ┌──────────┬──────────┬───────┐ │  [ 检查SSH状态 ]         │
│  │ nuc      │ fly      │ nuc2  │ │  [ 顺序阶段起飞日志 ]     │
│  │ 192...   │ 192...   │ 19..  │ │                         │
│  │ 连接详情  │ 连接详情  │ 详情  │ │  [ 启动bridge_node_gcs] │
│  └──────────┴──────────┴───────┘ │  [ 启动exploration RViz] │
│                                  │                         │
│  操作说明                         │  [启动基础链路] [启动程控] │
│                                  │  [   启动探索         ]  │
│  日志区域                         │                         │
│                                  │  [结束1号][结束2号][结束3号]│
│                                  │  [1号降落][2号降落][3号降落]│
│                                  │  [      同时降落全部      ]│
└──────────────────────────────────┴─────────────────────────┘
```

## 按钮功能说明

### 辅助功能组

**SSH连接** — 后台并行探测 3 架无人机（`ssh -o StrictHostKeyChecking=accept-new` 执行 `echo ok`），验证可达性后自动更新三张状态卡片。适用于首次连接或更换网络环境后一键打通，无需手动开终端敲 SSH。

**检查SSH状态** — 独立执行 `whoami` / `hostname` / `pwd` / `tmux has-session` 四合一探测，逐行输出到日志并更新状态卡片。可在 SSH 连接后的任意时刻手动复查。

**顺序阶段起飞日志** — 当前为占位实现，打印起飞阶段日志（等待起飞 ROS 接口定稿后生效）。

### 地面站组

**启动 bridge_node_gcs** — 在本地终端窗口启动 `swarm_ros_bridge bridge_node_gcs.launch`，负责跨机 ROS 话题 UDP 转发。

**启动 exploration RViz** — 在本地终端窗口启动 `exploration_manager rviz.launch`，显示栅格地图与探索轨迹。

**保存 RViz 地图快照** — 前端运行期间缓存每架飞机最新的一帧 `/sdf_map/occupancy_all_N` 点云；任务结束后点击按钮，后台保存一个 `RVizlog/rviz_map_时间.bag` 文件。按钮会报告未收到地图的飞机及已保存帧的年龄。请保持地面站前端运行到保存完成；关闭前端会清空内存中的快照。保存只取已收到的地图，不再向无人机请求数据，也不会保存 RViz 窗口配置或规划器内部的空闲/未知栅格。

地面站首次使用前运行 `bash tools/install_rvizlog_handler.sh`。此后在文件管理器双击
`RVizlog/rviz_map_时间.bag`，会打开独立的 RViz 地图窗口，自动显示该文件中保存的各机点云。
点击 RViz 的 **Move Camera** 工具后拖动鼠标可以旋转，滚轮缩放；关闭窗口即结束回看。
查看器使用独立 ROS master 与 `/rvizlog/occupancy_all_N` 话题，不干扰正在运行的桥接和
普通 RViz。也可在终端运行 `bash tools/open_rvizlog.sh RVizlog/rviz_map_时间.bag`。
该文件只含保存时各机的最新一帧占据点云，不含实时过程或 RViz 界面布局。

### 飞行控制组

**启动基础链路** — 并行在 3 架无人机上：清理旧 tmux 会话 → 创建 9 窗口 tmux → 窗口 1-7 自动执行，窗口 8-9 预置命令但不执行。

自动执行的窗口（窗口 1 后等待 7 秒，后续窗口间隔 3 秒）：

| 窗口 | 运行内容 |
|---|---|
| `1_px4` | MAVROS + PX4 飞控连接 |
| `2_livox` | Livox LiDAR 驱动 |
| `3_fastlio` | FAST-LIO 激光里程计 |
| `4_offb_node` | Offboard 模式桥接节点 |
| `5_ego_transfer` | EGO 规划器数据传输 |
| `6_trans` | 坐标转换节点 |
| `7_bridge` | swarm_ros_bridge 跨机桥接 |

**启动程控** — 按配置顺序（默认 fly → nuc → nuc2，间隔 1 秒）向各机 `8_cxr_egoctrl` 窗口发送 Enter，启动 `cxr_egoctrl_v2` 轨迹跟踪控制器。**桨叶会转，切勿在室内使用。**

**启动探索** — 按同样顺序向各机 `9_exploration` 窗口发送 Enter，启动 `swarm_exploration_realworld.launch` 探索规划器。

### 结束与降落组

**结束 X 号机任务** — 远程执行 `tmux kill-session` 并验证会话已消失，该机所有进程随会话终止。

**X 号机降落** — 通过 ROS 话题 `/uavX/mission/land_request` 发布 `Bool(True)`，连发 10 次间隔 100ms 抵抗 UDP 丢包。无人机端 `cxr_egoctrl` 收到后触发降落流程。

**同时降落全部** — 向 3 架无人机并行发布降落请求。

## 状态卡片

三张卡片位于界面左上方，每张显示：

- **无人机名称**（粗体）：如 `nuc` / `fly` / `nuc2`
- **IP 地址**（灰色小字）：如 `192.168.0.121`
- **连接指示灯**：绿色 = SSH 可达，红色 = 不可达
- **详情**：`user@hostname`、tmux 会话状态、最后检查时间

初始全部显示红点不可达。点击"SSH连接"或"检查SSH状态"后更新。

### 电池提示和耗电记录

状态卡从 `/uavN/mavros/battery` 接收电量与总电压。按剩余电量分级提示：
≤80% 关注余量，≤60% 规划返航，≤30% 到达预留线并准备降落，≤20% 尽快安全降落。
这是地面站的提前提醒；飞控的低电保护参数由 PX4 独立执行。阈值附近留有 2 个百分点
回差，避免读数抖动时反复提示。数据超过 10 秒未更新或百分比无效时，界面不继续显示旧
电量和剩余时间。

“到 30% 预留电量”根据最近最多 10 分钟的百分比下降速度计算；至少连续观察 60 秒、
下降 2 个百分点后才给出参考分钟数。电池更换、充电、遥测断流后重新估算。它不是最大
可飞行时间保证，风、负载和电池状态会改变实际耗电速度。

连续遥测每积累至少 60 秒、2 个百分点下降，会把该段时长和电量消耗追加到
`logs/battery_discharge.csv`，方便后续对比。当前桥接没有转发飞控的解锁/落地状态，
因此记录的 `flight_confirmed=False`，表示观测到的耗电时间，不能当作已确认的飞行时间。
出现鼓包或异常的电池应停止使用，不继续充电。

## 链路监控

界面第二个标签页（“链路监控”）显示地面站实际收到的每条 ROS 主题的链路质量，用于判断
“链路断了还是只是发得慢”，并为桥接通信优化提供前后对照数据。

- 监控清单在启动时从 `swarm_ros_bridge/config/ros_topics_gcs.yaml` 读取（`srcIP` 查 IP 表，
  再用 IP 反查 `DRONES`），默认每架 14 条接收主题；找不到配置时只监控电量/位姿。
  可用环境变量 `SWARM_BRIDGE_GCS_CONFIG` 指定配置文件路径。
- 每条主题显示：频率 Hz、带宽 KB/s、本次峰值 KB/s、时延 ms、抖动 ms、静默 s、状态。
  - 频率 = 最近 2 秒窗口内平均到达间隔的倒数；带宽 = 窗口内载荷字节数 / 窗口时长。
  - 本次峰值为完整 2 秒窗口的最大带宽；悬停该列可看发生时间。“清零统计”重新开始记录。
  - 时延只对带 `header` 的电量/位姿主题测量，并先扣掉按“窗口内最小单向时延”估计的时钟
    偏差——两端时钟已同步时即为真实单向时延，未同步时反映的是排队/拥塞造成的额外时延。
  - 状态：等待 / 采集中 / 正常 / 稀疏（低于峰值 50%）/ 中断（超过 `max(3s, 4 个周期)` 无消息）。
- 顶部每架飞机一行汇总：在线主题数、总带宽、时钟偏差。“清零统计”用于开始新的对照实验，
  “导出 CSV”把当前快照写到 `logs/link_monitor_YYYYmmdd_HHMMSS.csv`，并另存
  `logs/link_peak_context_YYYYmmdd_HHMMSS.csv`：每条主题最终峰值前后各 5 秒内，
  同机各主题的频率、带宽、状态、时延及该机总带宽与中断数。前后状态按界面 0.5 秒刷新采样。

监控是只读的：电量/位姿沿用原有强类型订阅，其余主题用 `rospy.AnyMsg` 订阅，只取原始字节
长度、不做反序列化，因此不需要把 RACER 的自定义消息包 source 进前端虚拟环境。

## 典型操作流程

```
1. ./run_ui.sh                          # 启动前端
2. 点击 "SSH连接"                        # 自动打通三机 SSH
3. 确认三张状态卡片全绿                    # 验证连通性
4. 点击 "启动基础链路"                     # 远端创建 tmux，自动启动窗口 1-7
5. 等待约 30 秒（窗口依次启动）             # 观察日志确认正常
6. 点击 "启动 bridge_node_gcs"            # 启动地面站桥接
7. 点击 "启动 exploration RViz"           # 启动地面站可视化
8. 点击 "启动程控"                        # 三机 EGO 控制器启动（桨叶解锁）
9. 点击 "启动探索"                        # 三机探索规划器启动（开始飞行）
10. 点击 "同时降落全部"                    # 任务结束，三机降落
11. 点击 "结束 X 号机任务"                 # 清理远端 tmux 会话
```

## 无人机 tmux 窗口结构

每架无人机启动基础链路后，远端创建 9 窗口 tmux 会话：

| 窗口 | 名称 | 运行内容 | 触发方式 |
|---|---|---|---|
| 1 | `1_px4` | MAVROS + PX4 飞控连接 | 自动 |
| 2 | `2_livox` | Livox LiDAR 驱动 | 自动 |
| 3 | `3_fastlio` | FAST-LIO 激光里程计 | 自动 |
| 4 | `4_offb_node` | Offboard 模式桥接节点 | 自动 |
| 5 | `5_ego_transfer` | EGO 规划器数据传输 | 自动 |
| 6 | `6_trans` | 坐标转换节点 | 自动 |
| 7 | `7_bridge` | swarm_ros_bridge 跨机桥接 | 自动 |
| 8 | `8_cxr_egoctrl` | EGO 轨迹跟踪控制器 | "启动程控"按钮 |
| 9 | `9_exploration` | 探索规划器 | "启动探索"按钮 |

## 程控/探索启动顺序

点击"启动程控"或"启动探索"后，按固定顺序逐机触发（默认 fly → nuc → nuc2，间隔 1 秒）：

```
fly (2号机) ──等待 1s──→ nuc (1号机) ──等待 1s──→ nuc2 (3号机)
```

如需调整，修改 `MissionOrchestrator` 构造参数：

```python
MissionOrchestrator(
    manual_stage_start_order=(1, 3, 2),   # nuc → nuc2 → fly
    manual_stage_start_delay_sec=2.0,      # 间隔改为 2 秒
)
```

## 降落机制

降落不走 SSH，而是通过 ROS 话题 `std_msgs/Bool` 发布：

| 无人机 | 话题 |
|---|---|
| 1 号机 | `/uav1/mission/land_request` |
| 2 号机 | `/uav2/mission/land_request` |
| 3 号机 | `/uav3/mission/land_request` |

连发 10 次间隔 100ms，补偿 swarm_ros_bridge 的 UDP 丢包。飞机端 `cxr_egoctrl_v3` 收到 `Bool(True)` 后触发降落并防抖（忽略后续重复）。

## 参考代码

`reference/` 目录下包含无人机端 ROS 控制节点参考实现与启动文档：

- `reference/control/` — `cxr_egoctrl` (C++)、`offb_node` (C++)、`offegotrans` (Python)、PID 控制器
- `reference/双机启动/` — 早期独立启动脚本（已内联到 `mission_orchestrator.py`）
- `reference/第三架无人机直接启动文档.md` — 第三机手动启动参考
