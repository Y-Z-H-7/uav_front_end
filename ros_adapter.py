#!/usr/bin/env python3

"""
ROS 适配层：统一负责"发布控制请求、订阅状态（后续可扩展）"。

本版改进：
1. 关键控制指令（如降落）改为连发10次，间隔100ms，提高UDP传输可靠性。
2. 使用 QThread 异步发送，避免阻塞UI主线程。
3. 飞机端需要做防抖处理（首次收到Bool(True)后触发，忽略后续重复）。

说明：
- 本机 ROS 环境仍需可用（例如本机 roscore 已运行）。
- 这里不做远端 master 依赖探测，避免把跨机网络问题耦合到前端逻辑。
- 跨机数据转发由 swarm_ros_bridge 负责。
"""

import time
from typing import Callable, Dict, Iterable, Optional

from PySide6.QtCore import QThread, Signal

try:
    import rospy
    from std_msgs.msg import Bool

    ROS_AVAILABLE = True
except Exception:
    rospy = None
    Bool = None
    ROS_AVAILABLE = False


# 连发参数：保证UDP丢包场景下消息能到达飞机
LANDING_REPEAT_COUNT = 10        # 连发次数
LANDING_REPEAT_INTERVAL = 0.1    # 每次间隔（秒）


class LandingPublisher(QThread):
    """
    异步发送降落请求的工作线程。

    使用QThread的原因：
    - 主UI线程不能sleep，否则界面冻结
    - 连发10次×100ms=1秒，必须放后台
    - 通过Signal把日志反馈给主线程显示
    """

    log_signal = Signal(str)        # 普通日志
    finished_signal = Signal(int, bool)  # 完成信号 (uav_id, success)

    def __init__(self, publisher, topic, uav_id,
                 repeat=LANDING_REPEAT_COUNT,
                 interval=LANDING_REPEAT_INTERVAL):
        super().__init__()
        self.publisher = publisher
        self.topic = topic
        self.uav_id = uav_id
        self.repeat = repeat
        self.interval = interval

    def run(self):
        """线程主体：连续发布N次降落请求。"""
        try:
            for i in range(self.repeat):
                self.publisher.publish(Bool(data=True))
                # 每3次反馈一次进度，避免日志刷屏
                if (i + 1) % 3 == 0 or i == self.repeat - 1:
                    self.log_signal.emit(
                        f"  → {self.uav_id}号机 已发送 {i + 1}/{self.repeat} 次"
                    )
                time.sleep(self.interval)
            self.log_signal.emit(
                f"✓ {self.uav_id}号机降落请求发送完成（共{self.repeat}次，{self.topic}）"
            )
            self.finished_signal.emit(self.uav_id, True)
        except Exception as exc:
            self.log_signal.emit(f"✗ {self.uav_id}号机发送异常：{exc}")
            self.finished_signal.emit(self.uav_id, False)


class RosAdapter:
    """地面站与 ROS 之间的轻量桥接层。"""

    def __init__(
        self,
        log_callback: Optional[Callable[[str], None]] = None,
        landing_topics: Optional[Dict[int, str]] = None,
    ):
        self.log_callback = log_callback or (lambda message: None)

        # 这些 topic 可由 swarm_ros_bridge 转发到目标无人机。
        self.landing_topics = landing_topics or {
            1: "/uav1/mission/land_request",
            2: "/uav2/mission/land_request",
            3: "/uav3/mission/land_request",
        }

        self.publishers = {}
        self.ros_ready = False

        # 保存活动的发送线程，避免被GC回收
        self._active_threads = []

        if not ROS_AVAILABLE:
            self._log("ROS 环境不可用，当前以占位模式运行。")
            return

        try:
            # 不要求配置远端 master 地址，按本机 ROS 环境初始化。
            if not rospy.core.is_initialized():
                rospy.init_node(
                    "ground_station_ros_adapter",
                    anonymous=True,
                    disable_signals=True,
                )

            for uav_id, topic in self.landing_topics.items():
                self.publishers[uav_id] = rospy.Publisher(topic, Bool, queue_size=10)

            self.ros_ready = True
            self._log("ROS 适配层已初始化（Bool 触发模式，连发提高可靠性）。")
        except Exception as exc:
            self.ros_ready = False
            self._log(f"ROS 适配层初始化失败：{exc}")

    def _log(self, message: str) -> None:
        self.log_callback(message)

    def _on_thread_finished(self, thread: LandingPublisher) -> None:
        """线程结束后从活动列表移除，让Python GC回收。"""
        if thread in self._active_threads:
            self._active_threads.remove(thread)

    def request_landing(self, uav_ids: Iterable[int]) -> bool:
        """
        向目标无人机发布 Bool(True) 触发降落。

        改进点：
        - 连发10次（间隔100ms）抵抗UDP丢包
        - 使用QThread异步发送，UI不卡顿
        - 多机并发：各机同时下发指令
        """
        requested_ids = list(uav_ids)
        if not requested_ids:
            self._log("降落请求为空，已忽略。")
            return False

        target_text = "、".join(str(u) for u in requested_ids)
        self._log(
            f"开始下发降落请求：{target_text}号机 "
            f"（每机连发{LANDING_REPEAT_COUNT}次，间隔{int(LANDING_REPEAT_INTERVAL*1000)}ms）"
        )

        if not self.ros_ready:
            self._log("ROS 未就绪，无法发布降落请求")
            return False

        success = True
        for uav_id in requested_ids:
            topic = self.landing_topics.get(uav_id)
            if topic is None:
                self._log(f"未配置 {uav_id} 号机的降落话题。")
                success = False
                continue

            publisher = self.publishers.get(uav_id)
            if publisher is None:
                self._log(f"未找到 {uav_id} 号机的发布器。")
                success = False
                continue

            # 创建并启动异步发送线程
            thread = LandingPublisher(publisher, topic, uav_id)
            thread.log_signal.connect(self._log)
            # 线程结束后从活动列表清理
            thread.finished.connect(lambda t=thread: self._on_thread_finished(t))
            self._active_threads.append(thread)
            thread.start()

        return success
