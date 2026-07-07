#!/usr/bin/env python3
"""
Ground station multi-UAV control frontend.

Current scope:
1. SSH checks and startup orchestration buttons.
2. Manual stage triggers for tmux windows (control / exploration) after base startup.
3. Landing request controls via ROS.
4. Timestamped logs for field debugging.
"""

import sys
from datetime import datetime
from typing import Dict, Optional

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication,
    QGridLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtGui import QFont

from mission_orchestrator import MissionOrchestrator
from ros_adapter import RosAdapter, LANDING_REPEAT_COUNT, LANDING_REPEAT_INTERVAL


class TaskWorker(QThread):
    """Run mission tasks in background to keep UI responsive."""

    log_signal = Signal(str)
    finished_signal = Signal(bool)

    def __init__(self, task_fn):
        super().__init__()
        self.task_fn = task_fn

    def run(self):
        try:
            ok = bool(self.task_fn(self.log_signal.emit))
        except Exception as exc:
            self.log_signal.emit(f"[TASK] unexpected error: {exc}")
            ok = False
        self.finished_signal.emit(ok)


class UAVControlApp(QMainWindow):
    """Dual-UAV control main window."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("多机任务控制前端")
        self.resize(1100, 850)

        central_widget = QWidget()
        main_layout = QHBoxLayout(central_widget)

        self._status_cards: Dict[int, Dict[str, QLabel]] = {}
        self._latest_ssh_snapshot: Optional[Dict[int, Dict[str, str]]] = None
        self._early_logs: list = []  # buffered before log_view exists

        # 后端对象（状态卡片创建时需要引用 drone 配置）
        self.mission = MissionOrchestrator()
        self.ros_adapter = RosAdapter(self.append_log)

        # 左侧面板：状态卡 + 操作说明 + 日志
        left_panel = QVBoxLayout()

        title_label = QLabel("多机任务控制前端")
        title_font = QFont()
        title_font.setPointSize(18)
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
            f = QFrame()
            f.setObjectName('statusCard')
            # Prevent card from growing beyond its fair share of horizontal space
            f.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            f.setMinimumWidth(220)
            f.setMaximumHeight(80)
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
            info = QLabel('...')
            info.setObjectName('statusInfo')
            info.setWordWrap(True)
            info.setMaximumHeight(40)
            self._status_cards[drone_id] = {"dot": dot, "info": info}
            return f

        for drone_id in (1, 2, 3):
            drone = self.mission.DRONES[drone_id]
            status_layout.addWidget(make_status_card(drone_id, drone.display_name, drone.ssh_target))
        left_panel.addWidget(status_frame)

        # 操作说明
        hint_box = QTextEdit()
        hint_box.setReadOnly(True)
        hint_box.setObjectName('hintBox')
        hint_box.setHtml(
            "<b>Operational Guidelines</b><br>"
            "当前为自动 SSH 协作模式"
            "使用本前端的按钮执行固定远程命令。<br>"
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
        # Flush any log lines buffered before log_view existed
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
        self.takeoff_seq_button = QPushButton("顺序阶段起飞日志")
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
        self.end_uav1_button = QPushButton("结束1号机任务")
        self.end_uav1_button.setProperty('role','end')
        self.end_uav2_button = QPushButton("结束2号机任务")
        self.end_uav2_button.setProperty('role','end')
        self.end_uav3_button = QPushButton("结束3号机任务")
        self.end_uav3_button.setProperty('role','end')
        self.land_uav1_button = QPushButton("1号机降落")
        self.land_uav1_button.setProperty('role','end')
        self.land_uav2_button = QPushButton("2号机降落")
        self.land_uav2_button.setProperty('role','end')
        self.land_uav3_button = QPushButton("3号机降落")
        self.land_uav3_button.setProperty('role','end')
        self.land_all_button = QPushButton("同时降落全部")
        self.land_all_button.setProperty('role','end')
        end_layout.addWidget(self.end_uav1_button, 0, 0)
        end_layout.addWidget(self.end_uav2_button, 0, 1)
        end_layout.addWidget(self.end_uav3_button, 0, 2)
        end_layout.addWidget(self.land_uav1_button, 1, 0)
        end_layout.addWidget(self.land_uav2_button, 1, 1)
        end_layout.addWidget(self.land_uav3_button, 1, 2)
        end_layout.addWidget(self.land_all_button, 2, 0, 1, 3)
        right_layout.addWidget(end_frame)

        main_layout.addWidget(right_frame, 1)

        self.setCentralWidget(central_widget)

        # 重新连接信号（与原逻辑完全保持一致）
        self.auto_ssh_button.clicked.connect(self.auto_ssh_connect)
        self.check_ssh_button.clicked.connect(self.check_ssh)
        self.start_basic_button.clicked.connect(self.start_basic_chain)
        self.start_control_button.clicked.connect(self.start_control_stage)
        self.start_exploration_button.clicked.connect(self.start_exploration_stage)
        self.takeoff_seq_button.clicked.connect(self.run_takeoff_sequence)
        self.end_uav1_button.clicked.connect(
            lambda: self._run_background_task(
                "end_uav1_task",
                lambda log: self.mission.terminate_tmux_session(1, log),
            )
        )
        self.end_uav2_button.clicked.connect(
            lambda: self._run_background_task(
                "end_uav2_task",
                lambda log: self.mission.terminate_tmux_session(2, log),
            )
        )
        self.end_uav3_button.clicked.connect(
            lambda: self._run_background_task(
                "end_uav3_task",
                lambda log: self.mission.terminate_tmux_session(3, log),
            )
        )
        self.launch_bridge_button.clicked.connect(lambda: self._run_background_task(
            "launch_bridge", lambda log: self.mission.launch_bridge_node_gcs(log)))
        self.launch_exploration_rviz_button.clicked.connect(lambda: self._run_background_task(
            "launch_exploration_rviz", lambda log: self.mission.launch_exploration_rviz(log)))
        self.land_uav1_button.clicked.connect(lambda: self.request_landing([1]))
        self.land_uav2_button.clicked.connect(lambda: self.request_landing([2]))
        self.land_uav3_button.clicked.connect(lambda: self.request_landing([3]))
        self.land_all_button.clicked.connect(lambda: self.request_landing([1, 2, 3]))

        self._task_worker = None
        self._task_in_progress = False

        # 应用深色主题样式表（分组配色：辅助灰、地面站蓝、飞行控制绿、结束/降落红）
        app = QApplication.instance()
        dark_style = '''
QWidget { background: #F7F7F9; color: #1A1A1A; }

/* Component backgrounds */
QFrame#statusCard { background: #FFFFFF; border: 1px solid #E0E0E0; border-radius: 8px; padding: 8px; }
QTextEdit#hintBox { background: #FFFFFF; border: 1px solid #E0E0E0; padding: 8px; border-radius:6px; color:#2D3436 }
QTextEdit#logView { background: #FFFFFF; border: 1px solid #E0E0E0; padding: 8px; border-radius:6px; font-family: Courier New, monospace; color:#4A4A4A }

/* Text colors */
QLabel { color: #2D3436 }

/* Status indicator default */
QLabel#statusDot { border-radius: 7px; background: #E0E0E0; }
/* IP label in status card header */
QLabel#statusIp { color: #999999; }

/* Core action buttons (highlight) */
QPushButton[role="core"] { background: #3498DB; color: #FFFFFF; min-height: 42px; min-width: 140px; border-radius: 8px; }
QPushButton[role="core"]:hover { background: #2f85c8 }
QPushButton[role="core"]:pressed { background: #246ea8 }

/* General / end buttons */
QPushButton[role="end"] { background: #E0E6ED; color: #4A4A4A; min-height: 42px; min-width: 140px; border-radius: 8px; }

/* Fallback button style */
QPushButton { background: #E0E6ED; color: #4A4A4A }

/* minor accents */
QFrame > QPushButton { background: transparent }
'''


        if app:
            app.setStyleSheet(dark_style)

        self._apply_ssh_snapshot({})
        self.append_log("界面已启动，等待任务指令。（支持 3 架无人机）")
        self.append_log("提示：当前采用自动 SSH 连接模式，连接成功后按钮会通过 ssh alias 执行固定命令。")

    def append_log(self, message):
        """Append one timestamped log line.

        Safe to call before log_view is created — early messages are
        buffered and flushed once the widget is ready.
        """
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{timestamp}] {message}"
        if hasattr(self, "log_view") and self.log_view is not None:
            self.log_view.append(line)
        else:
            self._early_logs.append(line)

    def _set_status_card(self, drone_id: int, connected: bool, info_text: str):
        card = self._status_cards.get(drone_id)
        if not card:
            return

        dot = card["dot"]
        info = card["info"]
        color = "#2ECC71" if connected else "#E74C3C"
        dot.setStyleSheet(f"border-radius: 7px; background: {color};")
        info.setText(info_text)

    def _apply_ssh_snapshot(self, snapshot: Dict[int, Dict[str, str]]):
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
                # Keep detail short so the card stays compact
                short_detail = detail[:60] + "…" if len(detail) > 60 else detail
                info_text = f"{short_detail}\n{now_text}"
            self._set_status_card(drone_id, connected, info_text)

    def _set_landing_buttons_enabled(self, enabled: bool):
        self.land_uav1_button.setEnabled(enabled)
        self.land_uav2_button.setEnabled(enabled)
        self.land_uav3_button.setEnabled(enabled)
        self.land_all_button.setEnabled(enabled)

    def _set_mission_buttons_enabled(self, enabled: bool):
        self.auto_ssh_button.setEnabled(enabled)
        self.check_ssh_button.setEnabled(enabled)
        self.start_basic_button.setEnabled(enabled)
        self.start_control_button.setEnabled(enabled)
        self.start_exploration_button.setEnabled(enabled)
        self.takeoff_seq_button.setEnabled(enabled)
        self.end_uav1_button.setEnabled(enabled)
        self.end_uav2_button.setEnabled(enabled)
        self.end_uav3_button.setEnabled(enabled)
        self.launch_bridge_button.setEnabled(enabled)
        self.launch_exploration_rviz_button.setEnabled(enabled)

    def _clear_worker(self):
        self._task_worker = None

    def _on_task_finished(self, task_name: str, ok: bool):
        self._task_in_progress = False
        self._set_mission_buttons_enabled(True)
        if task_name in ("check_ssh", "auto_ssh") and self._latest_ssh_snapshot is not None:
            self._apply_ssh_snapshot(self._latest_ssh_snapshot)
        self.append_log(f"[TASK] finished: {task_name}, success={ok}")

    def _run_background_task(self, task_name: str, task_fn):
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
        def task(log):
            ok = self.mission.auto_ssh_connect(log)
            self._latest_ssh_snapshot = self.mission.collect_ssh_status_snapshot()
            return ok

        self._run_background_task("auto_ssh", task)

    def check_ssh(self):
        def task(log):
            ok = self.mission.check_ssh_connectivity(log)
            self._latest_ssh_snapshot = self.mission.collect_ssh_status_snapshot()
            return ok

        self._run_background_task("check_ssh", task)

    def start_basic_chain(self):
        self._run_background_task("start_basic_chain", self.mission.start_basic_chain)

    def start_control_stage(self):
        self._run_background_task(
            "start_control_stage",
            lambda log: self.mission.trigger_manual_stage("control", log),
        )

    def start_exploration_stage(self):
        self._run_background_task(
            "start_exploration_stage",
            lambda log: self.mission.trigger_manual_stage("exploration", log),
        )

    def run_takeoff_sequence(self):
        """Open attached terminals only if tmux sessions already exist.

        This ensures the operator cannot attach to sessions that have not been
        created by `start_basic_chain`.
        """
        missing = []
        for drone in self.mission.DRONES.values():
            session_ok, _ = self.mission._check_session(drone, log=None)
            if not session_ok:
                missing.append(drone.display_name)

        if missing:
            self.append_log(f"[TASK] 未检测到 tmux 会话: {', '.join(missing)}。请先按“启动基础链路”创建会话，然后重试。")
            return

        self._run_background_task(
            "open_base_chain_terminals",
            self.mission.open_base_chain_terminals,
        )

    def request_landing(self, uav_ids):
        target_text = "、".join(str(uav_id) for uav_id in uav_ids)
        self.append_log(f"收到降落请求：{target_text} 号机。")

        self._set_landing_buttons_enabled(False)

        result = self.ros_adapter.request_landing(uav_ids)
        if not result:
            self.append_log(f"降落请求发起失败：{target_text} 号机。")

        total_duration_ms = int(
            LANDING_REPEAT_COUNT * LANDING_REPEAT_INTERVAL * 1000
        ) + 1000
        QTimer.singleShot(total_duration_ms, lambda: self._set_landing_buttons_enabled(True))



def main():
    app = QApplication(sys.argv)
    window = UAVControlApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
