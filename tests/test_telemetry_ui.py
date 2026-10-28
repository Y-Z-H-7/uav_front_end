"""Exercise ROS callbacks -> Qt signals -> real UI labels without connecting to ROS."""
import contextlib
import io
import math
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtWidgets import QApplication
import ros_adapter
from ui_app import UAVControlApp


class FakeRos:
    core = SimpleNamespace(is_initialized=lambda: True)

    def __init__(self):
        self.subscriptions = {}

    def Publisher(self, *args, **kwargs):
        return SimpleNamespace(publish=lambda msg: None)

    def Subscriber(self, topic, message_type, callback, **kwargs):
        self.subscriptions[topic] = callback
        return SimpleNamespace(unregister=lambda: None)

    def get_master(self):
        return SimpleNamespace(getUri=lambda: "http://localhost:11311")


class TelemetryUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.fake_ros = FakeRos()
        self.patches = contextlib.ExitStack()
        self.addCleanup(self.patches.close)
        self.patches.enter_context(patch.multiple(
            ros_adapter, rospy=self.fake_ros, ROS_AVAILABLE=True,
            BatteryState=SimpleNamespace, PoseStamped=SimpleNamespace, Bool=SimpleNamespace,
        ))
        self.patches.enter_context(patch.object(UAVControlApp, "_init_log_file"))
        self.output = io.StringIO()
        self.patches.enter_context(contextlib.redirect_stdout(self.output))
        self.window = UAVControlApp()
        self.addCleanup(self.window.close)

    def deliver(self, suffix, message):
        errors = []
        def run_callback():
            try:
                self.fake_ros.subscriptions[f"/uav2/mavros/{suffix}"](message)
            except Exception as exc:
                errors.append(exc)
        thread = threading.Thread(target=run_callback)
        thread.start()
        thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])

    def test_background_callbacks_update_only_fly_after_qt_delivery(self):
        self.deliver("battery", SimpleNamespace(percentage=0.76, voltage=15.8))
        self.deliver("local_position/pose", SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=1.2, y=-3.4, z=5.6))))
        card = self.window._status_cards[2]
        # A worker must not write widgets directly; updates wait for the UI event loop.
        self.assertEqual(card["battery"].text(), "电量: --")
        self.app.processEvents()
        self.assertEqual(card["battery"].text(), "电量: 76%  (15.80V)")
        self.assertEqual(card["position"].text(), "位置: (1.2, -3.4, 5.6) m")
        for drone_id in (1, 3):
            self.assertEqual(self.window._status_cards[drone_id]["battery"].text(), "电量: --")
        self.assertIn("已收到 2 号机电量", self.window.log_view.toPlainText())
        self.assertIn("已收到 2 号机位置", self.window.log_view.toPlainText())

    def test_unknown_percentage_still_displays_voltage(self):
        self.deliver("battery", SimpleNamespace(percentage=math.nan, voltage=15.2))
        self.app.processEvents()
        self.assertEqual(self.window._status_cards[2]["battery"].text(), "电量: --  (15.20V)")

    def test_invalid_measurements_are_not_shown_as_valid_telemetry(self):
        self.deliver("battery", SimpleNamespace(percentage=math.inf, voltage=math.nan))
        self.deliver("local_position/pose", SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=math.nan, y=1.0, z=2.0))))
        self.app.processEvents()
        self.assertEqual(self.window._status_cards[2]["battery"].text(), "电量: --  (--V)")
        self.assertEqual(self.window._status_cards[2]["position"].text(), "位置: --")

    def test_px4_unknown_voltage_sentinel_is_not_displayed(self):
        # 飞控没测到电压时 PX4 填 65535 mV，MAVROS 转成 65.535 V；
        # 这是“无读数”而不是一块 65V 电池，必须显示成 --V。
        self.deliver("battery", SimpleNamespace(percentage=-0.01, voltage=65.535))
        self.app.processEvents()
        self.assertEqual(self.window._status_cards[2]["battery"].text(), "电量: --  (--V)")

    def test_receiving_telemetry_does_not_depend_on_selection(self):
        self.window._toggle_drone_selection(2)
        self.deliver("battery", SimpleNamespace(percentage=0.5, voltage=15.0))
        self.app.processEvents()
        self.assertEqual(self.window._status_cards[2]["battery"].text(), "电量: 50%  (15.00V)")


if __name__ == "__main__":
    unittest.main()
