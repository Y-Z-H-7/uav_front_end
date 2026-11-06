"""The double-click viewer selects saved maps and opens an orbit view."""

import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from tools.rvizlog_viewer import build_config, load_maps


class FakeBag:
    def __init__(self, path, mode):
        self.path = path
        self.mode = mode

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def read_messages(self):
        def cloud(frame):
            return SimpleNamespace(_type="sensor_msgs/PointCloud2",
                                   header=SimpleNamespace(frame_id=frame))
        yield "/sdf_map/occupancy_all_2", cloud("world"), None
        yield "/unrelated", cloud("world"), None
        yield "/sdf_map/occupancy_all_2", cloud("world"), None
        yield "/sdf_map/occupancy_all_3", cloud("world"), None


class ViewerTests(unittest.TestCase):
    def test_only_saved_maps_are_selected_and_orbit_view_is_configured(self):
        fake_rosbag = ModuleType("rosbag")
        fake_rosbag.Bag = FakeBag
        with patch.dict(sys.modules, {"rosbag": fake_rosbag}):
            maps, frame = load_maps("example.bag")
        self.assertEqual(sorted(maps), [2, 3])
        self.assertEqual(frame, "world")
        config = build_config(frame, maps)
        self.assertIn("rviz/Orbit", config)
        self.assertIn("/rvizlog/occupancy_all_2", config)
        self.assertIn("/rvizlog/occupancy_all_3", config)
        self.assertNotIn("/rvizlog/occupancy_all_1", config)


if __name__ == "__main__":
    unittest.main()
