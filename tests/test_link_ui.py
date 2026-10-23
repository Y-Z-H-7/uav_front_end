#!/usr/bin/env python3
"""链路监控页的界面测试：不连 ROS，用离屏 Qt 验证表格/汇总/清零。"""

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

from PySide6.QtWidgets import QApplication, QHeaderView

import ros_adapter
import ui_app
from link_monitor import STATUS_OK, STATUS_WAITING, LinkTopicSpec
from ui_app import (
    LINK_HEADERS,
    COL_AGE, COL_DESC, COL_HZ, COL_KBPS, COL_STATUS, COL_TOPIC, UAVControlApp,
)

SPECS = [
    LinkTopicSpec(1, "/uav1/mavros/battery", "sensor_msgs/BatteryState", "recv", True),
    LinkTopicSpec(1, "/uav1/mavros/local_position/pose", "geometry_msgs/PoseStamped", "recv", True),
    LinkTopicSpec(2, "/uav2/mavros/battery", "sensor_msgs/BatteryState", "recv", True),
    LinkTopicSpec(2, "/uav2/mavros/local_position/pose", "geometry_msgs/PoseStamped", "recv", True),
    LinkTopicSpec(3, "/uav3/mavros/battery", "sensor_msgs/BatteryState", "recv", True),
    LinkTopicSpec(3, "/uav3/mavros/local_position/pose", "geometry_msgs/PoseStamped", "recv", True),
]


class FakeRos:
    core = type("Core", (), {"is_initialized": lambda self=None: True})

    def Publisher(self, *args, **kwargs):
        return type("Pub", (), {"publish": lambda self, msg: None})()

    def Subscriber(self, topic, message_type, callback, **kwargs):
        return type("Sub", (), {"unregister": lambda self: None})()

    def get_master(self):
        return type("Master", (), {"getUri": lambda self: "http://localhost:11311"})()


class LinkUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.patches = contextlib.ExitStack()
        self.addCleanup(self.patches.close)
        self.patches.enter_context(patch.object(ui_app, "build_link_specs",
                                               return_value=(SPECS, None)))
        self.patches.enter_context(patch.multiple(
            ros_adapter, rospy=FakeRos(), ROS_AVAILABLE=True,
            BatteryState=object, PoseStamped=object, Bool=object,
        ))
        self.patches.enter_context(patch.object(UAVControlApp, "_init_log_file"))
        self.patches.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.window = UAVControlApp()
        self.addCleanup(self.window.close)

    def feed_battery(self, drone_id: int, rate_hz: float = 10.0, count: int = 12):
        """按固定频率喂入电量消息（时间用 monotonic 的当前值往回推）。"""
        now = time.monotonic()
        for i in range(count):
            self.window.link_monitor.record(
                drone_id, f"/uav{drone_id}/mavros/battery", 100,
                recv_time=now - 1.1 + i / rate_hz,
            )

    def test_link_tab_and_rows_are_built_from_specs(self):
        self.assertEqual(self.window.tabs.count(), 2)
        self.assertEqual(self.window.tabs.tabText(1), "链路监控")
        self.assertEqual(len(self.window._link_children), len(SPECS))
        self.assertEqual(self.window._link_children[0].text(COL_TOPIC), SPECS[0].topic)
        self.assertEqual(self.window._link_children[0].text(COL_STATUS), STATUS_WAITING)

    def test_tree_groups_topics_per_drone_and_can_collapse(self):
        tree = self.window.link_tree
        self.assertEqual(tree.topLevelItemCount(), 3)      # 3 架飞机
        first = tree.topLevelItem(0)
        self.assertEqual(first.childCount(), 2)            # 每架 2 条主题
        self.assertIn("2 条主题", first.text(COL_TOPIC))

        first.setExpanded(False)                           # 折叠后子节点收起
        self.assertFalse(first.isExpanded())
        first.setExpanded(True)
        self.assertTrue(first.isExpanded())

    def test_description_column_comes_from_spec(self):
        self.assertIn("电池", self.window._link_children[0].text(COL_DESC))
        self.assertIn("位置", self.window._link_children[1].text(COL_DESC))

    def test_parent_row_summarises_drone(self):
        self.feed_battery(1)
        self.window._refresh_link_view()
        parent = self.window.link_tree.topLevelItem(0)
        self.assertIn("在线 1", parent.text(COL_TOPIC))
        self.assertGreater(float(parent.text(COL_KBPS)), 0.0)

    def test_refresh_renders_rate_and_status(self):
        self.feed_battery(1)
        self.window._refresh_link_view()

        hz_text = self.window._link_children[0].text(COL_HZ)
        self.assertAlmostEqual(float(hz_text), 10.0, delta=0.3)
        self.assertGreater(float(self.window._link_children[0].text(COL_KBPS)), 0.0)
        self.assertEqual(self.window._link_children[0].text(COL_STATUS), STATUS_OK)
        # 位姿主题没有数据，仍显示 "--"
        self.assertEqual(self.window._link_children[1].text(COL_HZ), "--")
        self.assertIn("在线 1/2", self.window._link_summary_labels[1].text())

    def test_other_drones_stay_untouched(self):
        self.feed_battery(2)
        self.window._refresh_link_view()
        self.assertEqual(self.window._link_children[0].text(COL_HZ), "--")
        self.assertEqual(self.window._link_children[2].text(COL_STATUS), STATUS_OK)
        self.assertIn("等待数据", self.window._link_summary_labels[1].text())

    def test_reset_clears_table(self):
        self.feed_battery(1)
        self.window._refresh_link_view()
        self.assertEqual(self.window._link_children[0].text(COL_STATUS), STATUS_OK)

        self.window._reset_link_stats()
        self.window._refresh_link_view()
        self.assertEqual(self.window._link_children[0].text(COL_HZ), "--")
        self.assertEqual(self.window._link_children[0].text(COL_STATUS), STATUS_WAITING)
        self.assertIn("等待数据", self.window._link_summary_labels[1].text())

    def test_startup_log_reports_missing_bridge_config(self):
        text = self.window.log_view.toPlainText()
        self.assertIn("[LINK] 未找到桥接配置", text)

    def test_link_columns_are_draggable(self):
        # 之前除主题/说明外的列都是 ResizeToContents，Qt 不允许手动拖宽
        head = self.window.link_tree.header()
        for column in range(len(LINK_HEADERS)):
            with self.subTest(column=column):
                self.assertEqual(head.sectionResizeMode(column), QHeaderView.Interactive)
        self.assertGreater(head.sectionSize(COL_TOPIC), 0)

    def test_cells_expose_full_text_as_tooltip(self):
        child = self.window._link_children[0]
        self.assertEqual(child.toolTip(COL_TOPIC), SPECS[0].topic)
        self.assertTrue(child.toolTip(COL_DESC))

    def test_age_is_formatted_compactly(self):
        from ui_app import _format_age

        self.assertEqual(_format_age(None), "--")
        self.assertEqual(_format_age(3.24), "3.2")     # 刚超时：保留一位小数
        self.assertEqual(_format_age(27.4), "27")      # 十秒级：整数
        self.assertEqual(_format_age(100.0), "1:40")   # 一分钟以上：分:秒
        self.assertEqual(_format_age(3600.0), "60:00")  # 宽度仍然有界

    def test_stale_row_shows_compact_age_and_tooltip(self):
        # 让 1 号机电量在很久以前收到过一条，然后刷新：静默应显示"分:秒"
        self.window.link_monitor.record(
            1, "/uav1/mavros/battery", 60, recv_time=time.monotonic() - 125.0)
        self.window._refresh_link_view()
        self.assertEqual(self.window._link_children[0].text(COL_AGE), "2:05")
        self.assertIn("最后收到：", self.window._link_children[0].toolTip(COL_AGE))


if __name__ == "__main__":
    unittest.main()
