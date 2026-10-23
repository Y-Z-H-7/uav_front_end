#!/usr/bin/env python3
"""端到端自检：自己发 PoseStamped，走 RosAdapter → LinkMonitor，看时延算不算得出来。"""
import sys
import time

sys.path.insert(0, "/home/ubuntu20/uav_front_end")

import rospy
from geometry_msgs.msg import PoseStamped

from link_monitor import LinkMonitor, LinkTopicSpec
from ros_adapter import RosAdapter

TOPIC = "/uav2/mavros/local_position/pose"
DELAY = 0.08          # 假装消息在路上走了 80 ms

rospy.init_node("link_selftest", anonymous=True, disable_signals=True)
monitor = LinkMonitor([LinkTopicSpec(2, TOPIC, "geometry_msgs/PoseStamped", "recv", True)])
adapter = RosAdapter(lambda m: None, landing_topics={2: "/uav2/mission/land_request"},
                     link_monitor=monitor)
publisher = rospy.Publisher(TOPIC, PoseStamped, queue_size=10)
time.sleep(1.5)       # 等订阅连上

for _ in range(30):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now() - rospy.Duration(DELAY)
    msg.pose.position.x = 1.0
    publisher.publish(msg)
    time.sleep(0.05)

time.sleep(0.5)
row = monitor.snapshot()[0]
print(f"频率 {row.hz:.1f} Hz  时延 {row.latency_ms} ms  时钟偏差 {row.offset_ms} ms  "
      f"样本 {row.samples}  状态 {row.status}")
print("PASS" if (row.latency_ms is not None and row.latency_ms < 50) else "FAIL")
