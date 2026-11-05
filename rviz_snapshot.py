"""Keep one received map frame per UAV and save the current RViz map as a bag."""

from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import threading
import time


@dataclass(frozen=True)
class MapFrame:
    raw: bytes
    received_at: float  # monotonic clock, used only for the age shown in the UI


@dataclass(frozen=True)
class SnapshotResult:
    path: Path
    saved_ids: tuple
    missing_ids: tuple
    ages_seconds: dict


class MapSnapshotCache:
    """Store serialized PointCloud2 frames without deserializing every update."""

    def __init__(self, drone_ids):
        self.topics = {
            f"/sdf_map/occupancy_all_{drone_id}": drone_id
            for drone_id in sorted(drone_ids)
        }
        self._frames = {}
        self._lock = threading.Lock()

    def record(self, topic, raw):
        if topic not in self.topics or not raw:
            return
        with self._lock:
            self._frames[self.topics[topic]] = MapFrame(bytes(raw), time.monotonic())

    def snapshot(self):
        with self._lock:
            return dict(self._frames)

    def clear(self, drone_ids):
        """Start a new exploration without retaining frames from the previous run."""
        with self._lock:
            for drone_id in drone_ids:
                self._frames.pop(drone_id, None)


def save_map_snapshot(cache, output_dir, *, bag_factory=None, pointcloud_type=None,
                      time_factory=None):
    """Write one latest full-map PointCloud2 per available UAV to one ROS bag."""
    frames = cache.snapshot()
    if not frames:
        raise ValueError("尚未收到任何 occupancy_all 地图；请先启动地面站桥接和机载探索。")

    if bag_factory is None:
        import rosbag
        bag_factory = rosbag.Bag
    if pointcloud_type is None:
        from sensor_msgs.msg import PointCloud2
        pointcloud_type = PointCloud2
    if time_factory is None:
        import rospy
        time_factory = rospy.Time.from_sec

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = output_dir / f"rviz_map_{stamp}.bag"
    temporary = output_dir / f".{path.name}.tmp"
    saved_at = time.time()
    saved_monotonic = time.monotonic()
    try:
        with bag_factory(str(temporary), "w") as bag:
            for drone_id in sorted(frames):
                topic = f"/sdf_map/occupancy_all_{drone_id}"
                message = pointcloud_type()
                message.deserialize(frames[drone_id].raw)
                bag.write(topic, message, t=time_factory(saved_at))
        os.replace(temporary, path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise

    return SnapshotResult(
        path=path,
        saved_ids=tuple(sorted(frames)),
        missing_ids=tuple(sorted(set(cache.topics.values()) - frames.keys())),
        ages_seconds={drone_id: max(0.0, saved_monotonic - frame.received_at)
                      for drone_id, frame in frames.items()},
    )
