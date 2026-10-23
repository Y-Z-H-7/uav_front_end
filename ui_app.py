#!/usr/bin/env python3
"""
Ground station multi-UAV control frontend.

Current scope:
1. SSH checks and startup orchestration buttons.
2. Manual stage triggers for tmux windows (control / exploration) after base startup.
3. Landing request controls via ROS.
4. Timestamped logs for field debugging.

代码结构说明：
- 本文件是整个地面站前端的入口，负责渲染 Qt 界面和分发按钮信号。
- 真正的业务动作通常分层到 mission_orchestrator.py 与 ros_adapter.py 中。
- UI 侧重点主要是：状态展示、用户交互、日志输出、后台任务调度。
- 这类程序的典型模式是：信号 (Signal) + 槽 (Slot) + QThread，保证界面响应不被阻塞。
"""

import math
import os
import subprocess
import sys
from datetime import datetime, timedelta
from typing import Dict, Optional

from PySide6.QtCore import Qt, QThread, QTimer, Signal, Slot
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QGridLayout,
    QFrame,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QTabWidget,
    QTextEdit,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtGui import QColor, QFont

from link_monitor import (
    STATUS_OK,
    STATUS_SPARSE,
    STATUS_STALE,
    STATUS_WAITING,
    LinkMonitor,
    build_link_specs,
)
from mission_orchestrator import MissionOrchestrator
from ros_adapter import RosAdapter, LANDING_REPEAT_COUNT, LANDING_REPEAT_INTERVAL


def _format_age(age_s: Optional[float]) -> str:
    """把静默时长格式化成一格之内的文本。

    秒数一路涨到三位数既占宽度又不提供信息，1 分钟以后改成“分:秒”，
    真正想知道的“几点断的”放在单元格的悬停提示里。
    """
    if age_s is None:
        return "--"
    if age_s < 10:
        return f"{age_s:.1f}"
    if age_s < 60:
        return f"{age_s:.0f}"
    minutes = int(age_s) // 60
    seconds = int(age_s) % 60
    return f"{minutes}:{seconds:02d}"


# 链路监控表格的列序号与表头（折叠树和刷新逻辑共用）
(
    COL_AIRCRAFT,
    COL_TOPIC,
    COL_DESC,
    COL_TYPE,
    COL_HZ,
    COL_KBPS,
    COL_LAT,
    COL_JIT,
    COL_AGE,
    COL_STATUS,
) = range(10)

LINK_HEADERS = (
    "飞机", "主题", "说明", "消息类型",
    "频率 Hz", "带宽 KB/s", "时延 ms", "抖动 ms", "静默 s", "状态",
)

# 各列初始宽度（px）。全部 Interactive，用户可以自由拖动；
# 点“列宽自适应”会按内容重新算一次。
LINK_WIDTHS = {
    COL_AIRCRAFT: 105,
    COL_TOPIC: 300,
    COL_DESC: 330,
    COL_TYPE: 185,
    COL_HZ: 75,
    COL_KBPS: 85,
    COL_LAT: 75,
    COL_JIT: 75,
    COL_AGE: 65,
    COL_STATUS: 70,
}


class TaskWorker(QThread):
    """用来在后台执行耗时任务，避免 UI 卡住。

    作用：
    - 接收一个 task_fn 回调函数；这个回调通常会调用 mission_orchestrator 中的 SSH / tmux / ROS 逻辑。
    - 在 worker.run() 中执行任务，不阻塞主线程事件循环，因此按钮点击、窗口拖拽、日志滚动等 UI 操作仍然保持流畅。
    - 任务完成后发送 finished_signal，让 UI 线程恢复按钮状态并刷新状态卡。
    """

    # Signal 是 Qt 的事件机制；这里使用命名信号向 UI 主线程发出日志/完成状态
    log_signal = Signal(str)
    finished_signal = Signal(bool)

    def __init__(self, task_fn):
        super().__init__()
        # task_fn 是一个可执行函数，它本身通常把 log_signal.emit 当作日志回调参数。
        self.task_fn = task_fn

    def run(self):
        try:
            # task_fn 一般是 lambda log: self.mission.auto_ssh_connect(log, drone_ids=selected)
            # 这里把 log_signal.emit 传入，使后台线程可以把日志写回主线程的 append_log()
            ok = bool(self.task_fn(self.log_signal.emit))
        except Exception as exc:
            # 兜底：任务异常时也必须向 UI 反馈，否则用户无法知道失败原因。
            self.log_signal.emit(f"[TASK] unexpected error: {exc}")
            ok = False
        self.finished_signal.emit(ok)


class UAVControlApp(QMainWindow):
    """主窗口，负责所有 UI 和逻辑"""

    ros_log_signal = Signal(str)                    # ROS 适配器发来的日志 → 转发到 append_log()
    battery_signal = Signal(int, float, float)      # (uav_id, 电量0~1, 电压V) → 刷新状态卡电量
    pose_signal = Signal(int, float, float, float)  # (uav_id, x, y, z，单位米) → 刷新状态卡位置

    def __init__(self):
        super().__init__()
        self.setWindowTitle("多机任务控制前端")
        self.resize(1100, 850)

        central_widget = QWidget()
        outer_layout = QVBoxLayout(central_widget)
        outer_layout.setContentsMargins(0, 0, 0, 0)

        # 用标签页把"任务控制"和新增的"链路监控"分开：监控表格有 9 列，
        # 直接塞进原来 1/3 宽的按钮区会把布局挤变形。
        self.tabs = QTabWidget()
        control_page = QWidget()
        main_layout = QHBoxLayout(control_page)

        self._status_cards: Dict[int, Dict[str, QLabel]] = {}
        self._select_buttons: Dict[int, QPushButton] = {}
        self._selected_drones: set = set()
        self._end_buttons: Dict[int, QPushButton] = {}
        self._land_buttons: Dict[int, QPushButton] = {}
        self._latest_ssh_snapshot: Optional[Dict[int, Dict[str, str]]] = None
        self._early_logs: list = []  # log_view 控件创建之前缓存的早期日志，创建后统一刷入
        self._log_file = None
        self._init_log_file()

        # 后端对象（状态卡片创建时需要引用 drone 配置）
        # MissionOrchestrator 负责统筹 SSH / tmux / 一键启动等远程操作
        self.mission = MissionOrchestrator()

        # 通信链路监测：主题清单直接读桥接的 ros_topics_gcs.yaml（找不到就只监控
        # 电量/位姿）。监控器由 ROS 回调写入（ROS 线程），界面定时器读快照（主线程）。
        self._link_specs, self._link_config_path = build_link_specs(self._drone_ips())
        self.link_monitor = LinkMonitor(self._link_specs)
        self._link_summary_labels: Dict[int, QLabel] = {}
        self._link_status_cache: Dict[int, str] = {}

        # 连接信号与槽：当后端线程或 ROS 回调发出信号时，UI 会更新日志和状态卡。
        self.ros_log_signal.connect(self.append_log)
        self.battery_signal.connect(self._on_battery, Qt.QueuedConnection)
        self.pose_signal.connect(self._on_pose, Qt.QueuedConnection)

        # 左侧面板：状态卡 + 操作说明 + 日志
        left_panel = QVBoxLayout()

        title_label = QLabel("多机任务控制前端")
        title_font = QFont()
        title_font.setPointSize(27)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        left_panel.addWidget(title_label)

        # 状态卡（并排两张）
        status_frame = QFrame()
        status_layout = QHBoxLayout(status_frame)
        status_layout.setSpacing(12)
        status_layout.setContentsMargins(0, 0, 0, 0)

        def make_status_card(drone_id, display, ip):
            """为每一架无人机构造一个状态卡片。

            一个状态卡片由 5 个关键元素组成：
            - 左上角小圆点：表示连接状态（绿色在线/红色离线）
            - 标题：无人机名称，例如 px4 / cxr_egoctrl
            - IP 地址：ssh_target 或连接目标
            - 电量：从 ROS 主题更新，显示当前电量
            - 位置：从 ROS 主题更新，显示 x/y/z 坐标
            - 底部选择按钮：决定这架无人机是否参与后续操作
            """
            f = QFrame()
            f.setObjectName('statusCard')
            # 尺寸策略：宽度按比例扩展、高度固定，避免某张卡片挤占过多横向空间
            f.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            f.setMinimumWidth(220)
            f.setMaximumHeight(120)
            v = QVBoxLayout(f)
            v.setContentsMargins(6, 4, 6, 4)
            v.setSpacing(2)
            h = QHBoxLayout()
            dot = QLabel()
            dot.setFixedSize(14, 14)
            dot.setObjectName('statusDot')
            name_lbl = QLabel(display)
            name_lbl.setStyleSheet('font-weight:600')
            ip_lbl = QLabel(ip)
            ip_lbl.setObjectName('statusIp')
            ip_font = QFont()
            ip_font.setPointSize(9)
            ip_lbl.setFont(ip_font)
            h.addWidget(dot)
            h.addWidget(name_lbl)
            h.addWidget(ip_lbl)
            h.addStretch()
            v.addLayout(h)
            battery = QLabel('电量: --')
            battery.setObjectName('statusBattery')
            battery_font = QFont()
            battery_font.setPointSize(10)
            battery.setFont(battery_font)
            v.addWidget(battery)
            position = QLabel('位置: --')
            position.setObjectName('statusPosition')
            position_font = QFont()
            position_font.setPointSize(9)
            position.setFont(position_font)
            v.addWidget(position)
            info = QLabel('...')
            info.setObjectName('statusInfo')
            info.setWordWrap(True)
            info.setMaximumHeight(40)
            self._status_cards[drone_id] = {"dot": dot, "info": info, "battery": battery, "position": position}

            # 每个无人机卡片底部都有一个选择开关，用户可决定是否参与后续批量起飞/检查/降落操作。
            # 绿色 = 选中，红色 = 未选中；这正是本程序中“批量操作作用对象”的核心控制点。
            select_btn = QPushButton("✓ 已选")
            select_btn.setObjectName('selectBtn')
            select_btn.setCheckable(True)
            select_btn.setChecked(True)
            self._selected_drones.add(drone_id)
            select_btn.clicked.connect(
                lambda *args, did=drone_id: self._toggle_drone_selection(did)
            )
            self._select_buttons[drone_id] = select_btn
            self._update_select_button_style(drone_id)
            v.addWidget(select_btn)

            return f

        for drone_id in sorted(self.mission.DRONES.keys()):
            drone = self.mission.DRONES[drone_id]
            status_layout.addWidget(make_status_card(drone_id, drone.display_name, drone.ssh_target))
        left_panel.addWidget(status_frame)

        # 操作说明
        hint_box = QTextEdit()
        hint_box.setReadOnly(True)
        hint_box.setObjectName('hintBox')
        hint_box.setHtml(
            "<b>Operational Guidelines</b><br>"
            "当前为自动 SSH 协作模式，使用本前端的按钮执行固定远程命令。<br>"
            "<b>飞机选择</b>：每张卡片下方的按钮，绿色=选中（参与起飞），红色=未选中（跳过）。<br>"
            "连接/检查SSH：输出各机基础状况（whoami/hostname/tmux 状态）。<br>"
            "启动基础链路：通过 SSH 在远端创建 tmux 会话并依次启动窗口1-7。<br>"
            "降落控制：分别控制各机降落，或同时发起全部降落。<br>"
            f"（降落请求会连发 {LANDING_REPEAT_COUNT} 次，间隔 {int(LANDING_REPEAT_INTERVAL*1000)}ms）"
        )
        left_panel.addWidget(hint_box)

        # 日志区域
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setObjectName('logView')
        self.log_view.setPlaceholderText("任务日志会显示在这里")
        # 把 log_view 创建前缓存的早期日志全部刷入显示区
        for line in self._early_logs:
            self.log_view.append(line)
        self._early_logs.clear()
        left_panel.addWidget(self.log_view)

        main_layout.addLayout(left_panel, 2)

        # 右侧面板：分块按钮（辅助 / 地面站 / 飞行控制 + 结束/降落）
        right_frame = QFrame()
        right_layout = QVBoxLayout(right_frame)
        right_layout.setSpacing(14)

        # 辅助功能组
        aux_frame = QFrame()
        aux_frame.setObjectName('auxFrame')
        aux_layout = QHBoxLayout(aux_frame)
        aux_layout.setSpacing(8)
        self.auto_ssh_button = QPushButton("SSH连接")
        self.auto_ssh_button.setProperty('role','core')
        self.check_ssh_button = QPushButton("检查SSH状态")
        self.check_ssh_button.setProperty('role','core')
        self.takeoff_seq_button = QPushButton("查看终端")
        self.takeoff_seq_button.setProperty('role','core')
        aux_layout.addWidget(self.auto_ssh_button)
        aux_layout.addWidget(self.check_ssh_button)
        aux_layout.addWidget(self.takeoff_seq_button)
        right_layout.addWidget(aux_frame)

        # 地面站组
        gcs_frame = QFrame()
        gcs_frame.setObjectName('gcsFrame')
        gcs_layout = QHBoxLayout(gcs_frame)
        gcs_layout.setSpacing(8)
        self.launch_bridge_button = QPushButton("启动 bridge_node_gcs")
        self.launch_bridge_button.setProperty('role','core')
        self.launch_exploration_rviz_button = QPushButton("启动 exploration RViz")
        self.launch_exploration_rviz_button.setProperty('role','core')
        gcs_layout.addWidget(self.launch_bridge_button)
        gcs_layout.addWidget(self.launch_exploration_rviz_button)
        right_layout.addWidget(gcs_frame)

        # 飞行控制组
        fc_frame = QFrame()
        fc_frame.setObjectName('fcFrame')
        fc_grid = QGridLayout(fc_frame)
        fc_grid.setSpacing(8)
        self.start_basic_button = QPushButton("启动基础链路")
        self.start_basic_button.setProperty('role','core')
        self.start_control_button = QPushButton("启动程控")
        self.start_control_button.setProperty('role','core')
        self.start_exploration_button = QPushButton("启动探索")
        self.start_exploration_button.setProperty('role','core')
        fc_grid.addWidget(self.start_basic_button, 0, 0)
        fc_grid.addWidget(self.start_control_button, 0, 1)
        fc_grid.addWidget(self.start_exploration_button, 1, 0)
        right_layout.addWidget(fc_frame)

        # 结束任务与降落组（放在底部）
        end_frame = QFrame()
        end_frame.setObjectName('endFrame')
        end_layout = QGridLayout(end_frame)
        end_layout.setSpacing(8)
        # 动态生成结束/降落按钮（根据 DRONES 配置自动适配数量）
        drone_ids_sorted = sorted(self.mission.DRONES.keys())
        drone_count = len(drone_ids_sorted)
        for col, drone_id in enumerate(drone_ids_sorted):
            drone = self.mission.DRONES[drone_id]
            end_btn = QPushButton(f"结束{drone_id}号机({drone.display_name})")
            end_btn.setProperty('role', 'end')
            land_btn = QPushButton(f"{drone_id}号机降落({drone.display_name})")
            land_btn.setProperty('role', 'end')
            self._end_buttons[drone_id] = end_btn
            self._land_buttons[drone_id] = land_btn
            end_layout.addWidget(end_btn, 0, col)
            end_layout.addWidget(land_btn, 1, col)
        self.land_all_button = QPushButton("同时降落全部")
        self.land_all_button.setProperty('role', 'end')
        end_layout.addWidget(self.land_all_button, 2, 0, 1, drone_count)
        right_layout.addWidget(end_frame)

        main_layout.addWidget(right_frame, 1)

        self.tabs.addTab(control_page, "任务控制")
        self.tabs.addTab(self._build_link_page(), "链路监控")
        outer_layout.addWidget(self.tabs)

        self.setCentralWidget(central_widget)

        # 重新连接信号（与原逻辑完全保持一致）
        self.auto_ssh_button.clicked.connect(self.auto_ssh_connect)
        self.check_ssh_button.clicked.connect(self.check_ssh)
        self.start_basic_button.clicked.connect(self.start_basic_chain)
        self.start_control_button.clicked.connect(self.start_control_stage)
        self.start_exploration_button.clicked.connect(self.start_exploration_stage)
        self.takeoff_seq_button.clicked.connect(self.run_takeoff_sequence)
        # 动态连接结束/降落按钮信号
        for drone_id in sorted(self.mission.DRONES.keys()):
            self._end_buttons[drone_id].clicked.connect(
                lambda checked=False, did=drone_id: self._run_background_task(
                    f"end_uav{did}_task",
                    lambda log, did=did: self.mission.terminate_tmux_session(did, log),
                )
            )
            self._land_buttons[drone_id].clicked.connect(
                lambda checked=False, did=drone_id: self.request_landing([did])
            )
        self.launch_bridge_button.clicked.connect(lambda: self._run_background_task(
            "launch_bridge", lambda log: self.mission.launch_bridge_node_gcs(log)))
        self.launch_exploration_rviz_button.clicked.connect(lambda: self._run_background_task(
            "launch_exploration_rviz", lambda log: self.mission.launch_exploration_rviz(log)))
        self.land_all_button.clicked.connect(
            lambda: self.request_landing(self._get_selected_drone_ids())
        )

        self._task_worker = None
        self._task_in_progress = False

        # 应用全局样式表（浅色主题；按钮按 role 属性分色：core=蓝色高亮，end=灰色）
        app = QApplication.instance()
        dark_style = '''
QWidget { background: #F7F7F9; color: #1A1A1A; }

/* 组件背景：状态卡、操作说明框、日志框 */
QFrame#statusCard { background: #FFFFFF; border: 1px solid #E0E0E0; border-radius: 8px; padding: 8px; }
QTextEdit#hintBox { background: #FFFFFF; border: 1px solid #E0E0E0; padding: 8px; border-radius:6px; color:#2D3436 }
QTextEdit#logView { background: #FFFFFF; border: 1px solid #E0E0E0; padding: 8px; border-radius:6px; font-family: Courier New, monospace; color:#4A4A4A }

/* 链路监控页：表格与汇总标签 */
QTabWidget::pane { border: 1px solid #E0E0E0; }
QTabBar::tab { padding: 8px 18px; }
QTreeWidget#linkTree { background: #FFFFFF; alternate-background-color: #FAFBFC; border: 1px solid #E0E0E0; }
QHeaderView::section { background: #F0F3F7; color: #2D3436; border: none; padding: 5px 6px; }
QLabel#linkSummary { color: #34495E; font-size: 12px; }
QLabel#linkHint { color: #7F8C8D; font-size: 11px; }

/* 文字颜色 */
QLabel { color: #2D3436 }

/* 状态圆点默认样式（灰色=未知；在线绿/离线红由代码动态设置） */
QLabel#statusDot { border-radius: 7px; background: #E0E0E0; }
/* 状态卡头部 IP 标签颜色 */
QLabel#statusIp { color: #999999; }

/* 核心操作按钮（role="core"，蓝色高亮） */
QPushButton[role="core"] { background: #3498DB; color: #FFFFFF; min-height: 42px; min-width: 140px; border-radius: 8px; }
QPushButton[role="core"]:hover { background: #2f85c8 }
QPushButton[role="core"]:pressed { background: #246ea8 }

/* 结束/降落按钮（role="end"，灰色） */
QPushButton[role="end"] { background: #E0E6ED; color: #4A4A4A; min-height: 42px; min-width: 140px; border-radius: 8px; }

/* 兜底按钮样式（未设置 role 属性时生效） */
QPushButton { background: #E0E6ED; color: #4A4A4A; font-size: 16px; }

/* 细节修饰 */
QFrame > QPushButton { background: transparent }
'''


        if app:
            app.setStyleSheet(dark_style)

        self._apply_ssh_snapshot({})
        # 控件创建完成后再接收遥测，回调通过 Qt 信号交给 UI 线程。
        self.ros_adapter = RosAdapter(
            self.ros_log_signal.emit,
            landing_topics={
                drone_id: f"/uav{drone_id}/mission/land_request"
                for drone_id in self.mission.DRONES
            },
            battery_callback=self.battery_signal.emit,
            pose_callback=self.pose_signal.emit,
            link_monitor=self.link_monitor,
        )
        drone_names = ', '.join(d.display_name for d in self.mission.DRONES.values())
        self.append_log(f"界面已启动，等待任务指令。（支持 {len(self.mission.DRONES)} 架无人机：{drone_names}）")
        self.append_log("提示：当前采用自动 SSH 连接模式，连接成功后按钮会通过 ssh alias 执行固定命令。")

        # 链路监控的界面刷新：500 ms 一次，只读快照、不做 ROS 调用，
        # 因此不会因为 ROS 掉线而卡住界面。
        if self._link_config_path:
            self.append_log(
                f"[LINK] 已加载 {len(self._link_specs)} 条链路主题"
                f"（{self._link_config_path}）"
            )
        else:
            self.append_log(
                f"[LINK] 未找到桥接配置（或解析失败），仅监控 {len(self._link_specs)} 条关键状态主题；"
                "可用 SWARM_BRIDGE_GCS_CONFIG 指定 ros_topics_gcs.yaml 路径"
            )
        self._link_timer = QTimer(self)
        self._link_timer.setInterval(500)
        self._link_timer.timeout.connect(self._refresh_link_view)
        self._link_timer.start()

    def _toggle_drone_selection(self, drone_id: int):
        """切换指定无人机的选中状态。

        这个方法是实现“批量操作”最关键的地方：
        - 每个状态卡的底部按钮都会调用它。
        - 如果某架飞机被取消选中，则后续的 SSH / 启动 / 降落等动作将不会作用于它。
        - 选中集合用 set() 保持去重，排序后用于显示和批量处理。
        """
        if drone_id in self._selected_drones:
            self._selected_drones.discard(drone_id)
            checked = False
        else:
            self._selected_drones.add(drone_id)
            checked = True
        self._update_select_button_style(drone_id)
        drone = self.mission.DRONES.get(drone_id)
        name = drone.display_name if drone else drone_id
        state = "已选" if checked else "未选"
        self.append_log(f"[SELECT] {name}({drone_id}号) {state}，当前选中: {sorted(self._selected_drones)}")

    def _update_select_button_style(self, drone_id: int):
        """按选中状态刷新选择按钮的文字和配色：绿色"✓ 已选" / 红色"✗ 未选"。"""
        btn = self._select_buttons.get(drone_id)
        if not btn:
            return
        if drone_id in self._selected_drones:
            btn.setText("✓ 已选")
            btn.setStyleSheet(
                "QPushButton { background: #2ECC71; color: #FFFFFF; "
                "min-height: 30px; border-radius: 6px; font-weight: bold; }"
            )
        else:
            btn.setText("✗ 未选")
            btn.setStyleSheet(
                "QPushButton { background: #E74C3C; color: #FFFFFF; "
                "min-height: 30px; border-radius: 6px; font-weight: bold; }"
            )

    def _get_selected_drone_ids(self) -> list:
        """返回当前已选中无人机编号的升序列表，供各任务按钮作为操作对象使用。"""
        return sorted(self._selected_drones)

    @Slot(int, float, float)
    def _on_battery(self, uav_id: int, pct: float, volt: float):
        """电量槽函数：由 battery_signal 触发，刷新对应状态卡上的电量文本。

        参数：uav_id=飞机编号；pct=电量(0~1，<0 表示暂无数据)；volt=电池电压(V)。
        """
        card = self._status_cards.get(uav_id)
        if not card or "battery" not in card:
            return
        percentage = f"{pct*100:.0f}%" if math.isfinite(pct) and 0 <= pct <= 1 else "--"
        voltage = f"{volt:.2f}V" if math.isfinite(volt) and volt > 0 else "--V"
        text = f"电量: {percentage}  ({voltage})"
        card["battery"].setText(text)

    @Slot(int, float, float, float)
    def _on_pose(self, uav_id: int, x: float, y: float, z: float):
        """位置槽函数：由 pose_signal 触发，刷新对应状态卡上的位置文本（单位：米）。"""
        card = self._status_cards.get(uav_id)
        if not card or "position" not in card:
            return
        card["position"].setText(f"位置: ({x:.1f}, {y:.1f}, {z:.1f}) m")

    def _init_log_file(self):
        """打开按天命名的日志文件 logs/groundstation_YYYYMMDD.log。

        - 目录不存在时自动创建；以追加模式打开，不覆盖历史日志。
        - 启动时写入一条分隔线，便于区分每次运行。
        - 打开失败则降级为 None，后续写日志自动跳过，不影响程序运行。
        """
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)
        log_name = f"groundstation_{datetime.now().strftime('%Y%m%d')}.log"
        self._log_file_path = os.path.join(log_dir, log_name)
        try:
            self._log_file = open(self._log_file_path, "a", encoding="utf-8")
            self._log_file.write(
                f"\n{'=' * 60}\n"
                f"Ground station started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"{'=' * 60}\n"
            )
            self._log_file.flush()
        except Exception:
            self._log_file = None

    def append_log(self, message):
        """Append one timestamped log line.

        这个函数是整个程序的统一日志入口，所有后台任务、ROS 回调、按钮动作都应尽量通过它发日志。

        设计目的：
        - UI 中的 QTextEdit 负责显示日志给用户看。
        - stdout 负责让启动脚本所属的 SSH 终端也能看到日志，便于远程排查。
        - 日志文件可以用 tail -f 进行实时查看，方便分析现场运行情况。
        - 实际上它还兼容在 log_view 尚未生成之前的“早期日志缓存”逻辑。
        """
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{timestamp}] {message}"
        # 1) UI 日志框：如果控件已创建，则立即追加；否则先缓存起来，等窗口初始化完成后再 flush
        if hasattr(self, "log_view") and self.log_view is not None:
            self.log_view.append(line)
        else:
            self._early_logs.append(line)
        # 2) stdout — 让 run_ui.sh 这样的启动脚本也能直接看到界面输出，方便现场调试
        print(line, flush=True)
        # 3) daily log file — 可用: tail -f logs/groundstation_YYYYMMDD.log
        if hasattr(self, "_log_file") and self._log_file is not None:
            try:
                self._log_file.write(line + "\n")
                self._log_file.flush()
            except Exception:
                pass

    def _set_status_card(self, drone_id: int, connected: bool, info_text: str):
        """更新某架无人机状态卡的"连接圆点"和"信息文本"。

        connected=True 时圆点显示绿色（在线），否则显示红色（离线）。
        """
        card = self._status_cards.get(drone_id)
        if not card:
            return

        dot = card["dot"]
        info = card["info"]
        color = "#2ECC71" if connected else "#E74C3C"
        dot.setStyleSheet(f"border-radius: 7px; background: {color};")
        info.setText(info_text)

    def _apply_ssh_snapshot(self, snapshot: Dict[int, Dict[str, str]]):
        """把 SSH 检查结果快照批量应用到各状态卡。

        snapshot 结构: {drone_id: {"reachable": "true"/"false", "user": ...,
                          "host": ..., "session": ..., "detail": ...}}
        快照中缺失的无人机视为不可达，状态卡显示 "unreachable"。
        """
        now_text = datetime.now().strftime("%H:%M:%S")
        for drone_id in self.mission.DRONES:
            item = snapshot.get(drone_id)
            if not item:
                self._set_status_card(drone_id, False, "unreachable")
                continue

            connected = item.get("reachable") == "true"
            user = item.get("user", "-")
            host = item.get("host", "-")
            session = item.get("session", "unknown")
            if connected:
                info_text = f"{user}@{host}\ntmux: {session}\n{now_text}"
            else:
                detail = item.get("detail") or "ssh unreachable"
                # 错误详情截断到 60 字符，保持卡片显示紧凑
                short_detail = detail[:60] + "…" if len(detail) > 60 else detail
                info_text = f"{short_detail}\n{now_text}"
            self._set_status_card(drone_id, connected, info_text)

    def _set_landing_buttons_enabled(self, enabled: bool):
        """批量启用/禁用所有降落按钮（各机降落 + 同时降落全部）。

        降落请求连发期间暂时禁用，防止用户重复点击导致指令重复下发。
        """
        for btn in self._land_buttons.values():
            btn.setEnabled(enabled)
        self.land_all_button.setEnabled(enabled)

    def _set_mission_buttons_enabled(self, enabled: bool):
        """批量启用/禁用所有"任务类"按钮（SSH、链路启动、结束、地面站等）。

        后台任务执行期间统一禁用，避免用户并发发起多个任务造成冲突。
        """
        self.auto_ssh_button.setEnabled(enabled)
        self.check_ssh_button.setEnabled(enabled)
        self.start_basic_button.setEnabled(enabled)
        self.start_control_button.setEnabled(enabled)
        self.start_exploration_button.setEnabled(enabled)
        self.takeoff_seq_button.setEnabled(enabled)
        for btn in self._end_buttons.values():
            btn.setEnabled(enabled)
        self.launch_bridge_button.setEnabled(enabled)
        self.launch_exploration_rviz_button.setEnabled(enabled)

    def _clear_worker(self):
        """清空后台线程引用（由 worker.finished 触发，帮助 Qt 回收线程对象）。"""
        self._task_worker = None

    def _on_task_finished(self, task_name: str, ok: bool):
        """后台任务完成的统一回调（由 finished_signal 触发）。

        - 复位任务互斥标志，恢复所有任务按钮可用；
        - 若是 SSH 相关任务，则用最新快照刷新状态卡；
        - 在日志中记录任务完成结果。
        """
        self._task_in_progress = False
        self._set_mission_buttons_enabled(True)
        if task_name in ("check_ssh", "auto_ssh") and self._latest_ssh_snapshot is not None:
            self._apply_ssh_snapshot(self._latest_ssh_snapshot)
        self.append_log(f"[TASK] finished: {task_name}, success={ok}")

    def _run_background_task(self, task_name: str, task_fn):
        """统一入口：启动后台线程执行任务并锁定控制按钮。

        这样做的好处：
        - 一个用户不能同时点击多个任务按钮，避免竞争和并发冲突。
        - UI 既不会卡住，也能在任务过程中给出明确反馈。
        - 任务完成后恢复按钮可用状态，并在日志中写出 finished 事件。
        """
        if self._task_in_progress:
            self.append_log(f"[TASK] busy, ignored: {task_name}")
            return

        self._task_in_progress = True
        self._set_mission_buttons_enabled(False)
        self.append_log(f"[TASK] started: {task_name}")

        worker = TaskWorker(task_fn)
        worker.log_signal.connect(self.append_log)
        worker.finished_signal.connect(lambda ok: self._on_task_finished(task_name, ok))
        worker.finished.connect(self._clear_worker)

        self._task_worker = worker
        worker.start()

    def auto_ssh_connect(self):
        """一键 SSH 连接选中无人机。

        逻辑：
        1. 读取当前已选飞机列表。
        2. 调用 MissionOrchestrator.auto_ssh_connect() 执行远端连接。
        3. 连接完成后用 collect_ssh_status_snapshot() 汇总状态，更新状态卡中的在线/离线信息。
        """
        selected = self._get_selected_drone_ids()
        if not selected:
            self.append_log("[TASK] 未选中任何飞机，已忽略")
            return
        self.append_log(f"[TASK] 目标飞机: {selected}")
        def task(log):
            # log 即 TaskWorker.log_signal.emit，后台线程通过它把日志写回 UI
            ok = self.mission.auto_ssh_connect(log, drone_ids=selected)
            self._latest_ssh_snapshot = self.mission.collect_ssh_status_snapshot()
            return ok
        self._run_background_task("auto_ssh", task)

    def check_ssh(self):
        """检查已选无人机的 SSH 可达性和 tmux 状态。"""
        selected = self._get_selected_drone_ids()
        if not selected:
            self.append_log("[TASK] 未选中任何飞机，已忽略")
            return
        self.append_log(f"[TASK] 目标飞机: {selected}")
        def task(log):
            # log 即 TaskWorker.log_signal.emit，后台线程通过它把日志写回 UI
            ok = self.mission.check_ssh_connectivity(log, drone_ids=selected)
            self._latest_ssh_snapshot = self.mission.collect_ssh_status_snapshot()
            return ok
        self._run_background_task("check_ssh", task)

    def start_basic_chain(self):
        """启动基础链路：通常会在远端创建 tmux 会话并依次执行窗口 1~7 的初始化步骤。"""
        selected = self._get_selected_drone_ids()
        if not selected:
            self.append_log("[TASK] 未选中任何飞机，已忽略")
            return
        self.append_log(f"[TASK] 目标飞机: {selected}")
        self._run_background_task(
            "start_basic_chain",
            lambda log: self.mission.start_basic_chain(log, drone_ids=selected),
        )

    def start_control_stage(self):
        """切换到程控阶段（control stage），通常会执行特定 tmux 窗口命令。"""
        selected = self._get_selected_drone_ids()
        if not selected:
            self.append_log("[TASK] 未选中任何飞机，已忽略")
            return
        self.append_log(f"[TASK] 目标飞机: {selected}")
        self._run_background_task(
            "start_control_stage",
            lambda log: self.mission.trigger_manual_stage("control", log, drone_ids=selected),
        )

    def start_exploration_stage(self):
        """切换到探索阶段（exploration stage）。"""
        selected = self._get_selected_drone_ids()
        if not selected:
            self.append_log("[TASK] 未选中任何飞机，已忽略")
            return
        self.append_log(f"[TASK] 目标飞机: {selected}")
        self._run_background_task(
            "start_exploration_stage",
            lambda log: self.mission.trigger_manual_stage("exploration", log, drone_ids=selected),
        )

    def run_takeoff_sequence(self):
        """打开本地终端，直接 attach 到远端 tmux 会话。"""
        selected = self._get_selected_drone_ids()
        if not selected:
            self.append_log("[TASK] 未选中任何飞机，已忽略")
            return
        self.append_log(f"[TASK] 目标飞机: {selected}")
        self._run_background_task(
            "open_terminals",
            lambda log: self.mission.open_base_chain_terminals(log, drone_ids=selected),
        )

    def request_landing(self, uav_ids):
        """对指定无人机发起降落请求（通过 RosAdapter 异步连发）。

        流程：
        1. 记录降落请求日志；
        2. 禁用降落按钮，防止连点重复下发；
        3. 调用 ros_adapter.request_landing()：为每架飞机启动一个后台线程，
           连发 LANDING_REPEAT_COUNT 次 Bool(True)（间隔 LANDING_REPEAT_INTERVAL 秒），
           以抵抗 UDP 丢包；多机并发下发，UI 不卡顿；
        4. 连发周期结束后（总时长 + 1s 余量），用 QTimer.singleShot 自动恢复降落按钮。
        """
        target_text = "、".join(str(uav_id) for uav_id in uav_ids)
        self.append_log(f"收到降落请求：{target_text} 号机。")

        # 发送期间锁住降落按钮，防止重复点击
        self._set_landing_buttons_enabled(False)

        # 异步下发（每机一个后台线程，立即返回）；False 表示 ROS 未就绪或有飞机未配置
        result = self.ros_adapter.request_landing(uav_ids)
        if not result:
            self.append_log(f"降落请求发起失败：{target_text} 号机。")

        # 连发总时长（次数 × 间隔）+ 1 秒余量，到期后自动恢复降落按钮
        total_duration_ms = int(
            LANDING_REPEAT_COUNT * LANDING_REPEAT_INTERVAL * 1000
        ) + 1000
        QTimer.singleShot(total_duration_ms, lambda: self._set_landing_buttons_enabled(True))

    # ------------------------------------------------------------------
    # 通信链路状态监控
    # ------------------------------------------------------------------
    def _drone_ips(self) -> Dict[int, str]:
        """从 DRONES 的 ssh_target（如 nuc@192.168.0.121）取出 IP。

        链路监控要用它把桥接配置里的 robot1/robot2/robot3 对回本工程的飞机编号。
        """
        ips: Dict[int, str] = {}
        for drone_id, drone in self.mission.DRONES.items():
            target = (drone.ssh_target or "").strip()
            if "@" in target:
                target = target.rsplit("@", 1)[-1]
            if target:
                ips[drone_id] = target
        return ips

    def _link_hint_text(self) -> str:
        return (
            "状态：等待（从未收到）· 采集中（累计不足 5 条）· 正常 · "
            "稀疏（低于峰值 50%）· 中断（超过 max(3 秒, 4 个周期) 无消息）。"
            "频率/带宽取最近 2 秒滑动窗口；时延只对带 header 的电量/位姿测量，"
            "并已扣掉按“窗口内最小单向时延”估计的时钟偏差。"
            "点左侧三角可折叠单架飞机，导出 CSV 可用于优化前后的对照实验。"
        )

    def _build_link_page(self) -> QWidget:
        """构建“链路监控”标签页：汇总行 + 表格 + 导出/清零。"""
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(8)

        header = QHBoxLayout()
        title = QLabel("通信链路状态监控")
        title.setStyleSheet("font-size: 18px; font-weight: 600;")
        header.addWidget(title)
        header.addStretch()
        self.link_expand_button = QPushButton("全部展开")
        self.link_collapse_button = QPushButton("全部折叠")
        self.link_fit_button = QPushButton("列宽自适应")
        self.link_reset_button = QPushButton("清零统计")
        self.link_export_button = QPushButton("导出 CSV")
        header.addWidget(self.link_expand_button)
        header.addWidget(self.link_collapse_button)
        header.addWidget(self.link_fit_button)
        header.addWidget(self.link_reset_button)
        header.addWidget(self.link_export_button)
        layout.addLayout(header)

        # 每架飞机一行汇总：在线主题数、总带宽、时钟偏差
        summary_row = QHBoxLayout()
        summary_row.setSpacing(24)
        for drone_id in sorted(self.mission.DRONES.keys()):
            label = QLabel(f"{drone_id} 号机：等待数据")
            label.setObjectName("linkSummary")
            self._link_summary_labels[drone_id] = label
            summary_row.addWidget(label)
        summary_row.addStretch()
        layout.addLayout(summary_row)

        hint = QLabel(self._link_hint_text())
        hint.setObjectName("linkHint")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        # 用可折叠的树代替平铺表格：每架飞机一个父节点，点前面的三角就能
        # 把 14 条主题一起收起，标题行还会顺带显示这架机的汇总。
        self.link_tree = QTreeWidget()
        self.link_tree.setObjectName("linkTree")
        self.link_tree.setColumnCount(len(LINK_HEADERS))
        self.link_tree.setHeaderLabels(list(LINK_HEADERS))
        self.link_tree.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.link_tree.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.link_tree.setAlternatingRowColors(True)
        self.link_tree.setUniformRowHeights(True)
        self.link_tree.setRootIsDecorated(True)
        head = self.link_tree.header()
        # 全部改成 Interactive：ResizeToContents 的列 Qt 不允许手动拖宽，
        # 之前说明列一长就没法调。这里给一个够用的初始宽度，剩下交给用户拖。
        head.setSectionsMovable(False)
        head.setStretchLastSection(False)
        for column, width in LINK_WIDTHS.items():
            head.setSectionResizeMode(column, QHeaderView.Interactive)
            head.resizeSection(column, width)
        layout.addWidget(self.link_tree, 1)

        # 父节点（每架飞机）与子节点（每条主题）只建一次，刷新时只改文本。
        parents = {}
        for drone_id in sorted(self.mission.DRONES.keys()):
            drone = self.mission.DRONES[drone_id]
            parent = QTreeWidgetItem([""] * len(LINK_HEADERS))
            parent.setText(COL_AIRCRAFT, f"{drone_id} 号机 {drone.display_name}")
            bold = QFont()
            bold.setBold(True)
            parent.setFont(COL_AIRCRAFT, bold)
            self.link_tree.addTopLevelItem(parent)
            parents[drone_id] = parent

        self._link_children: list = []   # 与 _link_specs、snapshot 顺序一一对应
        self._link_parents: list = []    # [(父节点, drone_id)]
        for spec in self._link_specs:
            parent = parents[spec.drone_id]
            child = QTreeWidgetItem([""] * len(LINK_HEADERS))
            child.setText(COL_AIRCRAFT, f"{spec.drone_id} 号机")
            child.setText(COL_TOPIC, spec.topic)
            child.setText(COL_DESC, spec.description or "--")
            child.setText(COL_TYPE, spec.msg_type)
            # 列被拖窄时会省略显示，悬停能看到完整内容
            child.setToolTip(COL_TOPIC, spec.topic)
            child.setToolTip(COL_DESC, spec.description or spec.topic)
            child.setToolTip(COL_TYPE, spec.msg_type)
            for column in (COL_HZ, COL_KBPS, COL_LAT, COL_JIT, COL_AGE):
                child.setText(column, "--")
            child.setText(COL_STATUS, STATUS_WAITING)
            parent.addChild(child)
            self._link_children.append(child)

        for drone_id in sorted(parents):
            count = sum(1 for spec in self._link_specs if spec.drone_id == drone_id)
            parents[drone_id].setText(COL_TOPIC, f"{count} 条主题")
            self._link_parents.append((parents[drone_id], drone_id))

        self.link_tree.expandAll()
        # 按钮的 clicked 会带一个 bool，用 lambda 吞掉，避免槽函数签名对不上
        self.link_fit_button.clicked.connect(lambda *_: self._fit_link_columns())
        self.link_export_button.clicked.connect(lambda *_: self._export_link_csv())
        self.link_reset_button.clicked.connect(lambda *_: self._reset_link_stats())
        self.link_expand_button.clicked.connect(lambda *_: self.link_tree.expandAll())
        self.link_collapse_button.clicked.connect(lambda *_: self.link_tree.collapseAll())
        return page

    @staticmethod
    def _set_item_text(item, column: int, text: str) -> None:
        """只在文本变化时写单元格，避免每次刷新都触发重绘。"""
        if item.text(column) != text:
            item.setText(column, text)

    def _refresh_link_view(self) -> None:
        """定时器回调：把监控器快照画到树上（只读，不碰 ROS）。"""
        if not hasattr(self, "link_tree"):
            return
        rows = self.link_monitor.snapshot()
        for child, row in zip(self._link_children, rows):
            self._set_item_text(
                child, COL_HZ, f"{row.hz:.1f}" if row.samples else "--")
            self._set_item_text(
                child, COL_KBPS, f"{row.kbytes_per_s:.1f}" if row.samples else "--")
            self._set_item_text(
                child, COL_LAT, "--" if row.latency_ms is None else f"{row.latency_ms:.1f}")
            self._set_item_text(
                child, COL_JIT, "--" if row.jitter_ms is None else f"{row.jitter_ms:.1f}")
            self._set_item_text(child, COL_AGE, _format_age(row.age_s))
            # 静默久了以后，"多少秒"没有意义，想知道的是"几点断的"——
            # 放在悬停提示里，不占表格宽度。
            if row.age_s is not None:
                last_seen = datetime.now() - timedelta(seconds=row.age_s)
                child.setToolTip(COL_AGE, f"最后收到：{last_seen.strftime('%H:%M:%S')}")
            self._set_item_text(child, COL_STATUS, row.status)
            self._color_link_status(child, row.status)

        summary = self.link_monitor.summary(rows)
        for parent, drone_id in self._link_parents:
            item = summary.get(drone_id) or {}
            total = int(item.get("total", 0))
            online = int(item.get("online", 0))
            kbps = float(item.get("kbytes_per_s", 0.0))
            parent.setText(COL_TOPIC, f"{total} 条主题 · 在线 {online}")
            parent.setText(COL_KBPS, f"{kbps:.1f}" if online else "--")
            interrupted = int(item.get("interrupted", 0))
            parent.setText(COL_STATUS, f"中断 {interrupted}" if interrupted else "")

        summary = self.link_monitor.summary(rows)
        for drone_id, label in self._link_summary_labels.items():
            item = summary.get(drone_id)
            if not item or not int(item["total"]):
                label.setText(f"{drone_id} 号机：等待数据")
                continue
            # 一条数据都没收到时（例如还没启动基础链路），不要显示成
            # “在线 0/14”，那看起来像故障；这里统一显示“等待数据”。
            if int(item["online"]) == 0 and int(item["waiting"]) == int(item["total"]):
                label.setText(f"{drone_id} 号机：等待数据")
                continue
            offset = item["offset_ms"]
            offset_text = "--" if offset is None else f"{float(offset):+.0f} ms"
            text = (
                f"{drone_id} 号机：在线 {item['online']}/{item['total']} · "
                f"{float(item['kbytes_per_s']):.1f} KB/s · 时钟偏差 {offset_text}"
            )
            if int(item["interrupted"]):
                text += f" · 中断 {item['interrupted']} 条"
            label.setText(text)

    def _color_link_status(self, item, status: str) -> None:
        """状态列配色：正常绿、稀疏橙、中断红，其余灰色。"""
        key = id(item)
        if self._link_status_cache.get(key) == status:
            return
        self._link_status_cache[key] = status
        colors = {
            STATUS_OK: "#27AE60",
            STATUS_SPARSE: "#E67E22",
            STATUS_STALE: "#C0392B",
            STATUS_WAITING: "#95A5A6",
        }
        item.setForeground(COL_STATUS, QColor(colors.get(status, "#95A5A6")))

    def _fit_link_columns(self) -> None:
        """按内容重算一遍列宽；之后仍然可以手动拖。"""
        for column in range(len(LINK_HEADERS)):
            self.link_tree.resizeColumnToContents(column)
            if self.link_tree.columnWidth(column) < 70:
                self.link_tree.setColumnWidth(column, 70)
        self.append_log("[LINK] 已按内容重排列宽")

    def _export_link_csv(self) -> None:
        """导出当前快照，用于优化前后的对照实验记录。"""
        if not self._link_specs:
            self.append_log("[LINK] 没有可导出的链路主题。")
            return
        rows = self.link_monitor.snapshot()
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        name = f"link_monitor_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            path = self.link_monitor.write_csv(os.path.join(log_dir, name), rows)
        except Exception as exc:
            self.append_log(f"[LINK] 导出失败：{exc}")
            return
        self.append_log(f"[LINK] 已导出 {len(rows)} 条链路快照：{path}")

    def _reset_link_stats(self) -> None:
        self.link_monitor.reset()
        self._link_status_cache.clear()
        self.append_log("[LINK] 链路统计已清零，开始新的统计窗口。")



def main():
    # 这是 PySide6 程序的标准入口：
    # 1. 创建 QApplication（整个 GUI 程序的应用对象）
    # 2. 实例化主窗口
    # 3. 显示窗口
    # 4. 进入 Qt 事件循环，等待用户操作和信号触发
    app = QApplication(sys.argv)
    window = UAVControlApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
