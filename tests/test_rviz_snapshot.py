"""Offline checks for one-frame-per-UAV map snapshot saving."""

from pathlib import Path
from io import BytesIO
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rviz_snapshot import MapSnapshotCache, save_map_snapshot


class FakePointCloud2:
    def deserialize(self, raw):
        self.raw = raw
        return self


class FakeBag:
    writes = []

    def __init__(self, path, mode):
        assert mode == "w"
        self.path = Path(path)
        self.writes = []
        FakeBag.writes.append(self.writes)

    def __enter__(self):
        return self

    def write(self, topic, message, t):
        self.writes.append((topic, message.raw, t))

    def __exit__(self, *args):
        self.path.write_bytes(b"test bag")


class MapSnapshotTests(unittest.TestCase):
    def test_one_latest_frame_per_drone_and_missing_drone_reported(self):
        cache = MapSnapshotCache([1, 2, 3])
        cache.record("/sdf_map/occupancy_all_1", b"old")
        cache.record("/sdf_map/occupancy_all_1", b"new")
        cache.record("/sdf_map/occupancy_all_3", b"three")
        with tempfile.TemporaryDirectory() as temp:
            result = save_map_snapshot(cache, Path(temp) / "RVizlog",
                                       bag_factory=FakeBag,
                                       pointcloud_type=FakePointCloud2,
                                       time_factory=lambda seconds: seconds)
            self.assertTrue(result.path.exists())
            self.assertEqual(result.saved_ids, (1, 3))
            self.assertEqual(result.missing_ids, (2,))
            self.assertEqual([entry[:2] for entry in FakeBag.writes[-1]], [
                ("/sdf_map/occupancy_all_1", b"new"),
                ("/sdf_map/occupancy_all_3", b"three"),
            ])
            self.assertFalse(list(result.path.parent.glob("*.tmp")))

    def test_empty_cache_does_not_create_file_or_directory(self):
        cache = MapSnapshotCache([1, 2, 3])
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "RVizlog"
            with self.assertRaisesRegex(ValueError, "尚未收到"):
                save_map_snapshot(cache, output)
            self.assertFalse(output.exists())

    def test_new_exploration_clears_only_selected_drone_frames(self):
        cache = MapSnapshotCache([1, 2])
        cache.record("/sdf_map/occupancy_all_1", b"one")
        cache.record("/sdf_map/occupancy_all_2", b"two")
        cache.clear([1])
        self.assertEqual(set(cache.snapshot()), {2})

    def test_real_rosbag_round_trip_when_ros_is_available(self):
        try:
            import rosbag
            from sensor_msgs.msg import PointCloud2, PointField
        except ImportError:
            self.skipTest("ROS Noetic Python modules are unavailable")

        cloud = PointCloud2()
        cloud.header.frame_id = "world"
        cloud.height = 1
        cloud.width = 1
        cloud.fields = [PointField(name=name, offset=offset,
                                   datatype=PointField.FLOAT32, count=1)
                        for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        cloud.point_step = 12
        cloud.row_step = 12
        cloud.is_dense = True
        cloud.data = struct.pack("<fff", 1.0, 2.0, 3.0)
        buffer = BytesIO()
        cloud.serialize(buffer)

        cache = MapSnapshotCache([2])
        cache.record("/sdf_map/occupancy_all_2", buffer.getvalue())
        with tempfile.TemporaryDirectory() as temp:
            result = save_map_snapshot(cache, Path(temp) / "RVizlog")
            with rosbag.Bag(str(result.path), "r") as bag:
                received = list(bag.read_messages())
            self.assertEqual(len(received), 1)
            self.assertEqual(received[0].topic, "/sdf_map/occupancy_all_2")
            self.assertEqual(received[0].message.header.frame_id, "world")
            self.assertEqual(received[0].message.data, cloud.data)


if __name__ == "__main__":
    unittest.main()
