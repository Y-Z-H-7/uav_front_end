#!/usr/bin/env python3

"""
地面站双机控制前端（当前阶段：降落链路打通）

职责说明：
1. 展示降落控制按钮（1号机、2号机、双机）。
2. 记录操作日志（带时间戳）。
3. 将 UI 操作转发给 RosAdapter，由其负责 ROS 话题发布。

本版改进：
1. 适配 RosAdapter 的异步连发机制。
2. 按钮防抖：发送过程中按钮置灰，避免重复点击。
3. 发送完成后按钮自动恢复。

注意：
- UI 层只发起"任务请求"，不直接操作飞控底层接口。
- 若 ROS 未就绪，UI 仍可启动，并在日志中明确提示失败原因。
"""

import sys
from datetime import datetime

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from ros_adapter import RosAdapter, LANDING_REPEAT_COUNT, LANDING_REPEAT_INTERVAL


class UAVControlApp(QMainWindow):
    """双机控制主窗口。"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("双机任务控制前端")
        self.resize(1000, 800)

        # 中央容器与整体纵向布局。
        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)

        # 标题区域：只做简要说明，避免界面信息噪声过大。
        title_label = QLabel("双机任务控制前端")
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # 功能提示：说明当前页面聚焦"降落控制"。
        landing_hint = QLabel(
            f"降落控制：分别控制 1 号机、2 号机，或同时发起双机降落。\n"
            f"（每次按钮按下会连发 {LANDING_REPEAT_COUNT} 次指令，"
            f"间隔 {int(LANDING_REPEAT_INTERVAL*1000)}ms，提高UDP传输可靠性）"
        )
        landing_hint.setWordWrap(True)
        layout.addWidget(landing_hint)

        # 按钮行：提供三种降落触发方式。
        button_row = QHBoxLayout()
        self.land_uav1_button = QPushButton("1号机降落")
        self.land_uav2_button = QPushButton("2号机降落")
        self.land_both_button = QPushButton("同时降落1和2")

        # 信号连接：UI 统一走 request_landing，便于后续接入编排层。
        self.land_uav1_button.clicked.connect(lambda: self.request_landing([1]))
        self.land_uav2_button.clicked.connect(lambda: self.request_landing([2]))
        self.land_both_button.clicked.connect(lambda: self.request_landing([1, 2]))

        button_row.addWidget(self.land_uav1_button)
        button_row.addWidget(self.land_uav2_button)
        button_row.addWidget(self.land_both_button)
        layout.addLayout(button_row)

        # 日志框：展示操作轨迹与错误原因，便于现场排障。
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setPlaceholderText("任务日志会显示在这里")
        layout.addWidget(self.log_view)

        # ROS 适配层：负责把"降落请求"转成 ROS 话题消息。
        # 这里在 log_view 初始化之后创建，避免初始化日志回调访问空控件。
        self.ros_adapter = RosAdapter(self.append_log)

        self.setCentralWidget(central_widget)
        self.append_log("界面已启动，等待降落指令。")

    def append_log(self, message):
        """追加一条带时间戳的日志。"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.log_view.append(f"[{timestamp}] {message}")

    def _set_buttons_enabled(self, enabled: bool):
        """统一启用/禁用所有降落按钮，防止发送过程中重复点击。"""
        self.land_uav1_button.setEnabled(enabled)
        self.land_uav2_button.setEnabled(enabled)
        self.land_both_button.setEnabled(enabled)

    def request_landing(self, uav_ids):
        """
        处理 UI 的降落请求。

        参数：
        - uav_ids: list[int]，例如 [1]、[2]、[1, 2]。

        防抖逻辑：
        - 按下按钮后立即置灰所有降落按钮
        - 等待发送总时长（连发次数×间隔 + 缓冲）后恢复
        """
        target_text = "、".join(str(uav_id) for uav_id in uav_ids)
        self.append_log(f"收到降落请求：{target_text} 号机。")

        # 禁用按钮，防止重复点击
        self._set_buttons_enabled(False)

        # 交给 ROS 适配层异步发送（连发10次）
        result = self.ros_adapter.request_landing(uav_ids)
        if not result:
            self.append_log(f"降落请求发起失败：{target_text} 号机。")

        # 计算预期发送总时长（毫秒），到时后恢复按钮
        # 连发次数 × 间隔 + 1秒缓冲（让用户能看到完整日志）
        total_duration_ms = int(
            LANDING_REPEAT_COUNT * LANDING_REPEAT_INTERVAL * 1000
        ) + 1000
        QTimer.singleShot(total_duration_ms, lambda: self._set_buttons_enabled(True))


def main():
    """程序入口。"""
    app = QApplication(sys.argv)
    window = UAVControlApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
