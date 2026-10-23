#!/usr/bin/env python3
"""link_monitor 的离线测试：不连 ROS、不建界面窗口。"""

import os
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import link_monitor
from link_monitor import (
    LinkMonitor,
    LinkTopicSpec,
    STATUS_COLLECTING,
    STATUS_OK,
    STATUS_SPARSE,
    STATUS_STALE,
    STATUS_WAITING,
    build_link_specs,
    describe_topic,
    load_specs_from_bridge_config,
    topic_priority,
)

BATTERY_1 = "/uav1/mavros/battery"
POSE_1 = "/uav1/mavros/local_position/pose"
CLOUD_1 = "/sdf_map/occupancy_all_1"

DRONE_IPS = {1: "192.168.0.121", 2: "192.168.0.195", 3: "192.168.0.243"}


def battery_spec(drone_id=1):
    return LinkTopicSpec(drone_id, BATTERY_1, "sensor_msgs/BatteryState", "recv", True)


class LinkMonitorTests(unittest.TestCase):
    def test_rate_and_bandwidth_from_window(self):
        monitor = LinkMonitor([battery_spec()])
        for i in range(20):  # 10 Hz, 每条 100 字节
            monitor.record(1, BATTERY_1, 100, recv_time=i * 0.1)

        row = monitor.snapshot(now=2.0)[0]
        self.assertAlmostEqual(row.hz, 10.0, delta=0.05)
        self.assertAlmostEqual(row.kbytes_per_s, 20 * 100 / 2.0 / 1024, delta=0.01)
        self.assertEqual(row.status, STATUS_OK)
        self.assertEqual(row.samples, 20)

    def test_latency_subtracts_clock_offset(self):
        monitor = LinkMonitor([battery_spec()])
        for i in range(10):
            t = i * 0.1
            # 地面站时钟比机载快 5 秒：直接相减是 5000 ms，扣掉基线后应为 0。
            # recv_time 是单调时钟（算间隔），wall_time 才是墙钟（算时延）。
            monitor.record(1, BATTERY_1, 60, recv_time=t, stamp_sec=t - 5.0, wall_time=t)

        row = monitor.snapshot(now=1.0)[0]
        self.assertAlmostEqual(row.latency_ms, 0.0, delta=1.0)
        self.assertAlmostEqual(row.offset_ms, 5000.0, delta=1.0)

        # 又晚到 200 ms：这才是链路排队造成的时延。
        monitor.record(1, BATTERY_1, 60, recv_time=1.05, stamp_sec=1.05 - 5.2,
                       wall_time=1.05)
        row = monitor.snapshot(now=1.05)[0]
        self.assertAlmostEqual(row.latency_ms, 200.0, delta=1.0)

    def test_monotonic_time_is_not_used_for_latency(self):
        """回归：以前拿 time.monotonic() 减 header.stamp，差值约 -1.8e9 被丢弃。"""
        monitor = LinkMonitor([battery_spec()])
        wall = time.time()
        for i in range(6):
            monitor.record(
                1, BATTERY_1, 60,
                recv_time=100.0 + i * 0.1,          # 假装是开机秒数
                stamp_sec=wall + i * 0.1 - 0.05,    # 墙钟，早 50 ms
                wall_time=wall + i * 0.1,
            )
        row = monitor.snapshot(now=100.5)[0]
        self.assertIsNotNone(row.latency_ms)
        self.assertAlmostEqual(row.latency_ms, 0.0, delta=5.0)

    def test_topic_without_stamp_reports_no_latency(self):
        monitor = LinkMonitor([LinkTopicSpec(1, CLOUD_1, "sensor_msgs/PointCloud2")])
        for i in range(5):
            monitor.record(1, CLOUD_1, 5000, recv_time=i * 0.5)
        row = monitor.snapshot(now=2.0)[0]
        self.assertIsNone(row.latency_ms)
        self.assertIsNone(row.offset_ms)
        self.assertGreater(row.kbytes_per_s, 0)

    def test_silent_topic_becomes_stale(self):
        monitor = LinkMonitor([battery_spec()])
        monitor.record(1, BATTERY_1, 60, recv_time=0.0)
        row = monitor.snapshot(now=10.0)[0]
        self.assertEqual(row.status, STATUS_STALE)
        self.assertAlmostEqual(row.age_s, 10.0, delta=0.01)

    def test_rate_drop_is_reported_as_sparse(self):
        monitor = LinkMonitor([battery_spec()])
        for i in range(20):           # 先跑满 10 Hz，建立峰值
            monitor.record(1, BATTERY_1, 60, recv_time=i * 0.1)
        self.assertEqual(monitor.snapshot(now=2.0)[0].status, STATUS_OK)

        for t in (2.5, 3.0, 3.5, 4.0, 4.5):   # 降到 2 Hz
            monitor.record(1, BATTERY_1, 60, recv_time=t)
        row = monitor.snapshot(now=4.5)[0]
        self.assertAlmostEqual(row.hz, 2.5, delta=0.1)   # 5 条 / 2 秒窗口
        self.assertEqual(row.status, STATUS_SPARSE)

    def test_burst_does_not_inflate_peak(self):
        """回归：桥接把积压消息一次性吐出时，峰值不能爆到几千 Hz。"""
        monitor = LinkMonitor([battery_spec()])
        for i in range(30):                    # 30 条挤在 30 ms 里
            monitor.record(1, BATTERY_1, 60, recv_time=i * 0.001)
        burst = monitor.snapshot(now=0.1)[0]
        self.assertLess(burst.peak_hz, 60.0)

        for i in range(60):                    # 之后稳定 30 Hz
            monitor.record(1, BATTERY_1, 60, recv_time=1.0 + i / 30.0)
        row = monitor.snapshot(now=3.0)[0]
        self.assertAlmostEqual(row.hz, 30.0, delta=1.0)
        self.assertLess(row.peak_hz, 60.0)
        self.assertEqual(row.status, STATUS_OK)   # 不该因为突发被判成“稀疏”

    def test_first_samples_are_marked_as_collecting(self):
        monitor = LinkMonitor([battery_spec()])
        monitor.record(1, BATTERY_1, 60, recv_time=0.0)
        monitor.record(1, BATTERY_1, 60, recv_time=0.1)
        self.assertEqual(monitor.snapshot(now=0.1)[0].status, STATUS_COLLECTING)

    def test_unconfigured_topic_is_ignored(self):
        monitor = LinkMonitor([battery_spec()])
        self.assertFalse(monitor.record(1, "/unknown", 10, recv_time=0.0))
        self.assertEqual(monitor.snapshot(now=0.0)[0].status, STATUS_WAITING)

    def test_slow_topic_keeps_rate_when_window_holds_one_sample(self):
        monitor = LinkMonitor([battery_spec()])
        for t in (0.0, 2.0, 4.0, 6.0, 8.0):   # 0.5 Hz
            monitor.record(1, BATTERY_1, 60, recv_time=t)
        row = monitor.snapshot(now=8.5)[0]    # 窗口 [6.5, 8.5] 只剩 8.0 这一条
        self.assertAlmostEqual(row.hz, 0.5, delta=0.02)
        self.assertEqual(row.status, STATUS_OK)

    def test_bandwidth_is_not_inflated_by_a_single_sample(self):
        monitor = LinkMonitor([battery_spec()])
        for t in (0.0, 2.0, 4.0):             # 每 2 秒一个 1 KB 的包
            monitor.record(1, BATTERY_1, 1024, recv_time=t)
        row = monitor.snapshot(now=5.0)[0]    # 窗口里只剩 4.0 那条
        # 应按 2 秒窗口算，而不是把它压进 0.2 秒 → 0.5 KB/s（旧实现会得 1.0）
        self.assertAlmostEqual(row.kbytes_per_s, 0.5, delta=0.05)

    def test_jitter_is_reported_in_milliseconds(self):
        monitor = LinkMonitor([battery_spec()])
        for t in (0.0, 0.1, 0.2, 0.3):
            monitor.record(1, BATTERY_1, 60, recv_time=t)
        row = monitor.snapshot(now=0.3)[0]
        self.assertAlmostEqual(row.jitter_ms, 0.0, delta=0.5)

    def test_summary_counts_online_waiting_and_interrupted(self):
        monitor = LinkMonitor([battery_spec(), LinkTopicSpec(1, POSE_1, "geometry_msgs/PoseStamped")])
        for i in range(10):
            monitor.record(1, BATTERY_1, 60, recv_time=i * 0.1)

        summary = monitor.summary(monitor.snapshot(now=1.0))
        self.assertEqual(summary[1]["total"], 2)
        self.assertEqual(summary[1]["online"], 1)        # 电量在线
        self.assertEqual(summary[1]["waiting"], 1)       # 位姿还没有任何消息

        summary = monitor.summary(monitor.snapshot(now=100.0))
        self.assertEqual(summary[1]["online"], 0)
        self.assertEqual(summary[1]["interrupted"], 1)   # 电量链路长时间静默

    def test_reset_clears_all_counters(self):
        monitor = LinkMonitor([battery_spec()])
        for i in range(10):
            monitor.record(1, BATTERY_1, 60, recv_time=i * 0.1)
        monitor.reset()
        row = monitor.snapshot(now=1.0)[0]
        self.assertEqual(row.samples, 0)
        self.assertEqual(row.status, STATUS_WAITING)

    def test_csv_export_roundtrip(self):
        monitor = LinkMonitor([battery_spec()])
        for i in range(10):
            monitor.record(1, BATTERY_1, 100, recv_time=i * 0.1)
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        path = os.path.join(scratch.name, "nested", "link.csv")
        monitor.write_csv(path, monitor.snapshot(now=1.0))
        with open(path, "r", encoding="utf-8-sig") as fh:
            lines = fh.read().strip().splitlines()
        self.assertEqual(lines[0].split(",")[0], "drone_id")
        self.assertEqual(len(lines), 2)
        self.assertIn(BATTERY_1, lines[1])
        self.assertIn(STATUS_OK, lines[1])


class LinkSpecTests(unittest.TestCase):
    def test_missing_config_falls_back_to_key_topics(self):
        specs, path = build_link_specs(DRONE_IPS, "C:/definitely/not/here.yaml")
        self.assertIsNone(path)
        self.assertEqual(len(specs), len(DRONE_IPS) * 2)
        self.assertTrue(all(spec.has_stamp for spec in specs))

    def test_key_state_topics_sort_before_map_and_cloud(self):
        # 电量/位姿是关键状态消息，应排在同一架飞机的最前面
        specs, _ = build_link_specs(DRONE_IPS, "C:/definitely/not/here.yaml")
        self.assertIn("/mavros/battery", specs[0].topic)
        self.assertIn("local_position/pose", specs[1].topic)

        self.assertLess(topic_priority("/uav1/mavros/battery"),
                        topic_priority("/multi_map_manager/chunk_data_recv_1"))
        self.assertLess(topic_priority("/swarm_expl/drone_state_recv_1"),
                        topic_priority("/sdf_map/occupancy_all_1"))
        self.assertEqual(topic_priority("/unknown/topic"), 9)

    def test_fake_yaml_maps_robot_ip_to_drone_id(self):
        payload = {
            "IP": {"self": "*", "robot1": "192.168.0.121", "robot2": "192.168.0.195"},
            "send_topics": [
                {"topic_name": "/uav1/mission/land_request",
                 "msg_type": "std_msgs/Bool", "srcIP": "gcs"},
            ],
            "recv_topics": [
                {"topic_name": BATTERY_1, "msg_type": "sensor_msgs/BatteryState",
                 "srcIP": "robot1"},
                {"topic_name": "/swarm_expl/drone_state_recv_1",
                 "msg_type": "exploration_manager/DroneState", "srcIP": "robot1"},
                {"topic_name": POSE_1, "msg_type": "geometry_msgs/PoseStamped",
                 "srcIP": "robot1"},
                {"topic_name": "/planning_vis/viewpoints_2",
                 "msg_type": "visualization_msgs/Marker", "srcIP": "robot2"},
                # 机不在本工程配置里，应当被忽略
                {"topic_name": "/lonely/topic", "msg_type": "std_msgs/Bool",
                 "srcIP": "robot9"},
            ],
        }
        fake = types.ModuleType("yaml")
        fake.safe_load = lambda fh: payload
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False,
                                         encoding="utf-8") as fh:
            fh.write("placeholder")
            path = fh.name
        self.addCleanup(os.unlink, path)

        original = sys.modules.get("yaml")
        sys.modules["yaml"] = fake
        try:
            specs = load_specs_from_bridge_config(path, DRONE_IPS)
        finally:
            if original is None:
                sys.modules.pop("yaml", None)
            else:
                sys.modules["yaml"] = original

        topics = {(spec.drone_id, spec.topic) for spec in specs}
        self.assertIn((1, "/swarm_expl/drone_state_recv_1"), topics)
        self.assertIn((2, "/planning_vis/viewpoints_2"), topics)
        self.assertNotIn((9, "/lonely/topic"), topics)
        self.assertEqual(len(specs), 4)

    def test_build_merges_config_without_duplicating_battery(self):
        payload = {
            "IP": {"robot1": "192.168.0.121"},
            "recv_topics": [
                {"topic_name": BATTERY_1, "msg_type": "sensor_msgs/BatteryState",
                 "srcIP": "robot1"},
                {"topic_name": CLOUD_1, "msg_type": "sensor_msgs/PointCloud2",
                 "srcIP": "robot1"},
            ],
        }
        fake = types.ModuleType("yaml")
        fake.safe_load = lambda fh: payload
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False,
                                         encoding="utf-8") as fh:
            fh.write("placeholder")
            path = fh.name
        self.addCleanup(os.unlink, path)

        original = sys.modules.get("yaml")
        sys.modules["yaml"] = fake
        try:
            specs, used = build_link_specs(DRONE_IPS, path)
        finally:
            if original is None:
                sys.modules.pop("yaml", None)
            else:
                sys.modules["yaml"] = original

        self.assertEqual(used, path)
        batteries = [s for s in specs if s.topic == BATTERY_1]
        self.assertEqual(len(batteries), 1)
        self.assertTrue(batteries[0].has_stamp)          # 仍由强类型回调记录
        self.assertEqual(len(specs), len(DRONE_IPS) * 2 + 1)

    def test_real_yaml_module_is_used_when_available(self):
        try:
            import yaml  # noqa: F401
        except Exception:
            self.skipTest("未安装 pyyaml")
        text = (
            "IP:\n"
            "  self: '*'\n"
            "  robot1: 192.168.0.121\n"
            "  robot2: 192.168.0.195\n"
            "recv_topics:\n"
            "  - topic_name: /swarm_expl/drone_state_recv_1\n"
            "    msg_type: exploration_manager/DroneState\n"
            "    srcIP: robot1\n"
            "  - topic_name: /swarm_expl/drone_state_recv_2\n"
            "    msg_type: exploration_manager/DroneState\n"
            "    srcIP: robot2\n"
        )
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False,
                                         encoding="utf-8") as fh:
            fh.write(text)
            path = fh.name
        self.addCleanup(os.unlink, path)

        specs = load_specs_from_bridge_config(path, DRONE_IPS)
        mapping = {spec.topic: spec.drone_id for spec in specs}
        self.assertEqual(mapping["/swarm_expl/drone_state_recv_1"], 1)
        self.assertEqual(mapping["/swarm_expl/drone_state_recv_2"], 2)


class TopicDescriptionTests(unittest.TestCase):
    """说明列：每个会出现在链路里的主题都要有中文说明。"""

    TOPICS = (
        "/swarm_expl/drone_state_recv_1",
        "/swarm_expl/pair_opt_recv_1",
        "/swarm_expl/pair_opt_res_recv_1",
        "/planning/swarm_traj_recv_1",
        "/swarm_expl/hgrid_recv_1",
        "/swarm_expl/grid_tour_recv_1",
        "/multi_map_manager/chunk_stamps_recv_1",
        "/multi_map_manager/chunk_data_recv_1",
        "/planning_vis/viewpoints_1",
        "/sdf_map/occupancy_all_1",
        "/planning/travel_traj_1",
        "/planning/position_cmd_vis_1",
        "/uav1/mavros/battery",
        "/uav1/mavros/local_position/pose",
    )

    def test_every_bridge_topic_has_a_description(self):
        for topic in self.TOPICS:
            with self.subTest(topic=topic):
                self.assertTrue(describe_topic(topic), f"{topic} 没有中文说明")

    def test_unknown_topic_falls_back_to_empty(self):
        self.assertEqual(describe_topic("/something/else"), "")

    def test_pair_opt_response_is_not_confused_with_request(self):
        # "pair_opt_res_recv" 必须排在 "pair_opt_recv" 前面匹配
        self.assertIn("应答", describe_topic("/swarm_expl/pair_opt_res_recv_1"))
        self.assertIn("请求", describe_topic("/swarm_expl/pair_opt_recv_1"))

    def test_spec_exposes_description(self):
        spec = LinkTopicSpec(1, "/sdf_map/occupancy_all_1", "sensor_msgs/PointCloud2")
        self.assertIn("点云", spec.description)


if __name__ == "__main__":
    unittest.main()
