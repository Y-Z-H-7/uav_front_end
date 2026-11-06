#!/usr/bin/env python3
"""Open a saved RViz map bag in an isolated, interactive RViz window."""

import argparse
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import tempfile
import time
from xmlrpc.client import ServerProxy


MAP_TOPIC = re.compile(r"^/sdf_map/occupancy_all_([1-9][0-9]*)$")


def load_maps(path):
    import rosbag

    latest = {}
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages():
            match = MAP_TOPIC.fullmatch(topic)
            if match and getattr(message, "_type", None) == "sensor_msgs/PointCloud2":
                latest[int(match.group(1))] = message
    if not latest:
        raise ValueError("文件中没有 /sdf_map/occupancy_all_N 点云快照")
    frames = {message.header.frame_id for message in latest.values()}
    if not all(frames) or len(frames) != 1:
        raise ValueError("各机地图的坐标系不一致或为空，无法在同一个 RViz 视图中可靠叠加")
    return latest, frames.pop()


def build_config(frame_id, drone_ids):
    displays = [
        "    - Class: rviz/Grid\n"
        "      Name: 地面网格\n"
        "      Enabled: true\n"
        "      Cell Size: 1\n"
        "      Plane Cell Count: 40"
    ]
    for drone_id in sorted(drone_ids):
        displays.append(
            "    - Class: rviz/PointCloud2\n"
            f"      Name: {drone_id}号机地图\n"
            "      Enabled: true\n"
            f"      Topic: /rvizlog/occupancy_all_{drone_id}\n"
            "      Queue Size: 1\n"
            "      Style: Flat Squares\n"
            "      Size (m): 0.12\n"
            "      Color Transformer: AxisColor\n"
            "      Axis: Z\n"
            "      Use rainbow: true\n"
            "      Decay Time: 0\n"
            "      Alpha: 1"
        )
    return (
        "Panels:\n"
        "  - Class: rviz/Displays\n"
        "    Name: Displays\n"
        "  - Class: rviz/Views\n"
        "    Name: Views\n"
        "Visualization Manager:\n"
        "  Class: \"\"\n"
        "  Name: root\n"
        "  Enabled: true\n"
        "  Global Options:\n"
        "    Background Color: 48; 48; 48\n"
        f"    Fixed Frame: {json.dumps(frame_id)}\n"
        "    Frame Rate: 30\n"
        "  Displays:\n"
        + "\n".join(displays) + "\n"
        "  Tools:\n"
        "    - Class: rviz/Interact\n"
        "    - Class: rviz/MoveCamera\n"
        "    - Class: rviz/FocusCamera\n"
        "  Views:\n"
        "    Current:\n"
        "      Class: rviz/Orbit\n"
        "      Name: 旋转视角\n"
        "      Target Frame: <Fixed Frame>\n"
        "      Distance: 30\n"
        "      Pitch: 0.7\n"
        "      Yaw: 0.7\n"
        "      Focal Point:\n"
        "        X: 0\n"
        "        Y: 0\n"
        "        Z: 0\n"
        "    Saved: []\n"
    )


def reserve_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for_master(uri, process, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("私有 roscore 启动失败")
        try:
            if ServerProxy(uri).getPid("/rvizlog_viewer")[0] == 1:
                return
        except (OSError, ConnectionError):
            pass
        time.sleep(0.2)
    raise TimeoutError("等待私有 roscore 超时")


def open_viewer(path):
    maps, frame_id = load_maps(path)
    port = reserve_port()
    uri = f"http://127.0.0.1:{port}/"
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = uri
    env["ROS_IP"] = "127.0.0.1"
    env.pop("ROS_HOSTNAME", None)
    with tempfile.TemporaryDirectory(prefix="rvizlog_") as temporary:
        config_path = Path(temporary) / "snapshot.rviz"
        config_path.write_text(build_config(frame_id, maps), encoding="utf-8")
        core_log = open(Path(temporary) / "roscore.log", "w", encoding="utf-8")
        core = subprocess.Popen(
            ["roscore", "-p", str(port)], env=env, stdout=core_log,
            stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            wait_for_master(uri, core)
            os.environ.update({"ROS_MASTER_URI": uri, "ROS_IP": "127.0.0.1"})
            os.environ.pop("ROS_HOSTNAME", None)
            import rospy
            from sensor_msgs.msg import PointCloud2

            rospy.init_node("rvizlog_viewer", anonymous=True, disable_signals=True)
            publishers = {}
            for drone_id, message in sorted(maps.items()):
                topic = f"/rvizlog/occupancy_all_{drone_id}"
                publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
                publisher.publish(message)
                publishers[drone_id] = publisher

            rviz = subprocess.Popen(["rviz", "-d", str(config_path)], env=env)
            print(f"已打开 {path.name}：{', '.join(str(i) for i in publishers)} 号机地图；"
                  "在 RViz 使用 Move Camera 工具拖动旋转。", flush=True)
            return rviz.wait()
        finally:
            os.killpg(core.pid, signal.SIGTERM)
            try:
                core.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(core.pid, signal.SIGKILL)
                core.wait()
            core_log.close()


def main():
    parser = argparse.ArgumentParser(description="在独立 RViz 窗口查看地图快照")
    parser.add_argument("bag", type=Path)
    parser.add_argument("--inspect", action="store_true", help="只检查内容，不打开窗口")
    args = parser.parse_args()
    path = args.bag.expanduser().resolve()
    if not path.is_file():
        parser.error(f"文件不存在：{path}")
    try:
        if args.inspect:
            maps, frame_id = load_maps(path)
            print(f"文件：{path}\n坐标系：{frame_id}\n包含飞机：{sorted(maps)}")
            return 0
        return open_viewer(path)
    except Exception as exc:
        print(f"打开 RViz 地图失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
