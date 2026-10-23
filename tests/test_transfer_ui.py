#!/usr/bin/env python3
"""图传控制页的界面测试：不跑 UAVLink，用替身控制器验证状态展示与按钮联动。"""

import contextlib
import io
import os
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication

import ros_adapter
import ui_app
from transfer_controller import TransferController
from ui_app import UAVControlApp


class FakeTransfer:
    """与 TransferController 同接口的替身：状态可注入，动作只做记录。"""

    def __init__(self, available: bool = True):
        self._available = available
        self.status = {"running": False, "stale": False, "age_s": None,
                       "state": None, "error": None}
        self.calls = []

    @property
    def available(self):
        return self._available

    @property
    def hint(self):
        return "（替身提示：UAVLink 未安装）"

    def read_status(self):
        return self.status

    def summarize_drones(self, state):
        return TransferController.summarize_drones(state)

    def start_photos(self):
        self.calls.append("start_photos")
        return True, "照片传输已在后台启动"

    def stop_photos(self):
        self.calls.append("stop_photos")
        return True, "已请求停止照片传输"

    def start_video(self, name):
        self.calls.append(("start_video", name))
        return True, f"{name} 视频已启动"

    def stop_video(self):
        self.calls.append("stop_video")
        return True, "视频已停止"


def running_status(video=None):
    """构造一份“主程序在跑”的替身状态（字段名与 ground_manager 一致）。"""
    state = {
        "session": "run_20261023_9999_ab12cd",
        "nodes": {
            "nuc": {"online": True, "status": {"counts": {"captures_saved": 30}}},
            "fly": {"online": False},
            "nuc2": {"online": True, "status": {"counts": {"captures_saved": 45}}},
        },
        "receivers": {"nuc": {"photos_received": 10},
                      "fly": {"photos_received": 0},
                      "nuc2": {"photos_received": 15}},
        "video": video,
    }
    return {"running": True, "stale": False, "age_s": 1.0,
            "state": state, "error": None}


class TransferUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.fake = FakeTransfer()
        self.patches = contextlib.ExitStack()
        self.addCleanup(self.patches.close)
        self.patches.enter_context(patch.object(ui_app, "TransferController",
                                                lambda: self.fake))
        self.patches.enter_context(patch.object(
            ui_app, "build_link_specs", return_value=([], None)))
        self.patches.enter_context(patch.multiple(
            ros_adapter, rospy=object, ROS_AVAILABLE=False,
            BatteryState=object, PoseStamped=object, Bool=object,
        ))
        self.patches.enter_context(patch.object(UAVControlApp, "_init_log_file"))
        self.patches.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.window = UAVControlApp()
        self.addCleanup(self.window.close)

    def refresh(self):
        self.window._refresh_transfer_view()

    def wait_until(self, cond, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.app.processEvents()
            if cond():
                return True
            time.sleep(0.02)
        return False

    # ------------------------------------------------------------- 页面结构
    def test_transfer_tab_exists_next_to_link_monitor(self):
        self.assertEqual(self.window.tabs.count(), 3)
        self.assertEqual(self.window.tabs.tabText(1), "链路监控")
        self.assertEqual(self.window.tabs.tabText(2), "图传控制")

    def test_unavailable_page_keeps_everything_disabled(self):
        # 重新造一个不可用的窗口
        self.patches.enter_context(patch.object(
            ui_app, "TransferController", lambda: FakeTransfer(available=False)))
        window = UAVControlApp()
        self.addCleanup(window.close)
        page_text = window.transfer_photos_status.text()
        self.assertIn("照片传输", page_text)
        self.assertFalse(window.transfer_photos_start_button.isEnabled())
        self.assertFalse(window.transfer_photos_stop_button.isEnabled())
        for button in window._transfer_video_buttons.values():
            self.assertFalse(button.isEnabled())

    # ------------------------------------------------------------- 状态展示
    def test_not_running_shows_start_enabled_stop_disabled(self):
        self.refresh()
        self.assertIn("未运行", self.window.transfer_photos_status.text())
        self.assertTrue(self.window.transfer_photos_start_button.isEnabled())
        self.assertFalse(self.window.transfer_photos_stop_button.isEnabled())
        self.assertIn("视频预览：未运行", self.window.transfer_video_status.text())
        for button in self.window._transfer_video_buttons.values():
            self.assertFalse(button.isEnabled())

    def test_running_state_shows_counts_and_video_buttons(self):
        self.fake.status = running_status()
        self.refresh()
        self.assertIn("运行中", self.window.transfer_photos_status.text())
        self.assertIn("25 张", self.window.transfer_photos_status.text())
        self.assertIn("run_20261023_9999_ab12cd", self.window.transfer_photos_status.text())
        # 逐机状态：在线/离线 + 机载与地面站计数
        self.assertIn("在线", self.window._transfer_drone_labels["nuc"].text())
        self.assertIn("机载已存 30", self.window._transfer_drone_labels["nuc"].text())
        self.assertIn("收到 10", self.window._transfer_drone_labels["nuc"].text())
        self.assertIn("离线", self.window._transfer_drone_labels["fly"].text())
        self.assertIn("收到 15", self.window._transfer_drone_labels["nuc2"].text())
        # 视频未开：启动按钮可用、停止按钮不可用
        for name, button in self.window._transfer_video_buttons.items():
            self.assertTrue(button.isEnabled(), name)
        self.assertFalse(self.window.transfer_video_stop_button.isEnabled())

    def test_running_video_disables_start_and_enables_stop(self):
        self.fake.status = running_status(
            video={"name": "nuc2", "key": "run_x_v1", "output": "/tmp/out"})
        self.refresh()
        self.assertIn("运行中", self.window.transfer_video_status.text())
        for button in self.window._transfer_video_buttons.values():
            self.assertFalse(button.isEnabled())
        self.assertTrue(self.window.transfer_video_stop_button.isEnabled())

    def test_stale_state_allows_stop_but_not_start(self):
        self.fake.status = {"running": False, "stale": True, "age_s": 75.0,
                            "state": None, "error": None}
        self.refresh()
        self.assertIn("无响应", self.window.transfer_photos_status.text())
        self.assertFalse(self.window.transfer_photos_start_button.isEnabled())
        self.assertTrue(self.window.transfer_photos_stop_button.isEnabled())

    # ------------------------------------------------------------- 动作
    def test_start_photos_logs_and_flips_to_pending_then_running(self):
        self.window._transfer_start_photos()
        # worker 是异步的：等它把调用记到替身上再断言
        self.assertTrue(self.wait_until(
            lambda: self.fake.calls == ["start_photos"]))
        self.assertTrue(self.wait_until(
            lambda: "启动照片传输：照片传输已在后台启动"
                    in self.window.transfer_log_view.toPlainText()))
        # active.json 还没出现：pending=启动中，防重复点击
        self.refresh()
        self.assertIn("启动中", self.window.transfer_photos_status.text())
        self.assertFalse(self.window.transfer_photos_start_button.isEnabled())

        # 10 秒后主程序写出状态：pending 清除、显示运行中
        # （运行中时启动按钮应禁用——再启动没有意义；停止按钮可用）
        self.fake.status = running_status()
        self.refresh()
        self.assertIn("运行中", self.window.transfer_photos_status.text())
        self.assertFalse(self.window.transfer_photos_start_button.isEnabled())
        self.assertTrue(self.window.transfer_photos_stop_button.isEnabled())

    def test_start_photos_second_click_while_pending_is_ignored(self):
        self.window._transfer_start_photos()
        self.assertTrue(self.wait_until(
            lambda: self.fake.calls == ["start_photos"]))
        self.window._transfer_start_photos()
        self.assertTrue(self.wait_until(
            lambda: len(self.fake.calls) == 1))
        self.assertEqual(self.fake.calls, ["start_photos"])

    def test_stop_photos_reports_result(self):
        self.fake.status = running_status()
        self.window._transfer_stop_photos()
        self.assertTrue(self.wait_until(
            lambda: self.fake.calls == ["stop_photos"]))
        self.assertTrue(self.wait_until(
            lambda: "停止照片传输" in self.window.transfer_log_view.toPlainText()))

    def test_video_start_button_calls_controller_with_drone_name(self):
        self.fake.status = running_status()
        self.refresh()  # 先刷新启用按钮，禁用按钮的 click() 是空操作
        self.window._transfer_video_buttons["nuc2"].click()
        self.assertTrue(self.wait_until(
            lambda: self.fake.calls == [("start_video", "nuc2")]))
        self.assertTrue(self.wait_until(
            lambda: "启动 nuc2 视频：nuc2 视频已启动"
                    in self.window.transfer_log_view.toPlainText()))

    def test_stop_video_button_calls_controller(self):
        self.fake.status = running_status(video={"name": "nuc2", "key": "k"})
        self.refresh()
        self.window.transfer_video_stop_button.click()
        self.assertTrue(self.wait_until(
            lambda: self.fake.calls == ["stop_video"]))


if __name__ == "__main__":
    unittest.main()
