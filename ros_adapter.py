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

import math
import time

from typing import Callable, Dict, Iterable, Optional

from PySide6.QtCore import QThread, Signal

try:
    import rospy
    from std_msgs.msg import Bool
    from sensor_msgs.msg import BatteryState
    from geometry_msgs.msg import PoseStamped

    ROS_AVAILABLE = True
    ROS_IMPORT_ERROR = ""
except Exception as exc:
    rospy = None
    Bool = None
    BatteryState = None
    PoseStamped = None
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = str(exc)


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
        battery_callback: Optional[Callable[[int, float, float], None]] = None,
        pose_callback: Optional[Callable[[int, float, float, float], None]] = None,
        link_monitor=None,
    ):
        self.log_callback = log_callback or (lambda message: None)
        self.battery_callback = battery_callback or (lambda uav_id, pct, volt: None)
        self.pose_callback = pose_callback or (lambda uav_id, x, y, z: None)

        # 链路监测器（link_monitor.LinkMonitor）；为 None 时不做任何额外订阅。
        # 电量/位姿的回调里顺手上报一次到达，其余主题用 AnyMsg 订阅后上报。
        self.link_monitor = link_monitor

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
        # 保存 Subscriber，防止被 GC 回收
        self._subs = []

        if not ROS_AVAILABLE:
            self._log(f"ROS 环境不可用，无法接收遥测：{ROS_IMPORT_ERROR}")
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

            self._log("ROS 适配层已初始化（Bool 触发模式，连发提高可靠性）。")
            self._log(f"遥测使用 ROS master：{_master_uri()}")
            
            for uav_id in sorted(self.landing_topics.keys()):
                ns = f"uav{uav_id}"
                self._subs.append(rospy.Subscriber(
                    f"/{ns}/mavros/battery",
                    BatteryState,
                    self._make_battery_cb(uav_id, ns),
                    queue_size=1,
                ))
            
            for uav_id in sorted(self.landing_topics.keys()):
                ns = f"uav{uav_id}"
                self._subs.append(rospy.Subscriber(
                    f"/{ns}/mavros/local_position/pose",
                    PoseStamped,
                    self._make_pose_cb(uav_id),
                    queue_size=1,
                ))

            self.ros_ready = True
            self._subscribe_link_topics()
            for uav_id in sorted(self.landing_topics):
                self._log(
                    f"等待 {uav_id} 号机遥测：/uav{uav_id}/mavros/battery、"
                    f"/uav{uav_id}/mavros/local_position/pose"
                )

        except Exception as exc:
            self.ros_ready = False
            self._log(f"ROS 适配层初始化失败：{exc}")

    def _log(self, message: str) -> None:
        self.log_callback(message)

    # ------------------------------------------------------------------
    # 链路监测
    # ------------------------------------------------------------------
    def _subscribe_link_topics(self) -> None:
        """为链路监测订阅“其余”主题。

        电量/位姿已经有强类型订阅（能拿到 header 时间戳，可以算时延），
        这里只处理剩下那些不需要反序列化的主题：用 rospy.AnyMsg 订阅，
        回调里只取 ``len(msg._buff)``，既拿到了链路上真实的载荷字节数，
        又不用导入 exploration_manager / bspline / plan_env 这些自定义消息包
        （前端虚拟环境里往往没有 source 过 RACER 工作空间）。
        """
        monitor = self.link_monitor
        if monitor is None:
            return
        any_msg = getattr(rospy, "AnyMsg", None)
        if any_msg is None:
            self._log("当前 rospy 不支持 AnyMsg，链路监控只统计电量/位姿主题。")
            return
        subscribed = 0
        for spec in monitor.specs():
            if spec.has_stamp:
                continue
            try:
                sub = rospy.Subscriber(
                    spec.topic,
                    any_msg,
                    self._make_raw_link_cb(spec),
                    queue_size=1,
                )
            except Exception as exc:
                self._log(f"链路监控订阅失败：{spec.topic}（{exc}）")
                continue
            self._subs.append(sub)
            subscribed += 1
        if subscribed:
            self._log(f"链路监控已订阅 {subscribed} 条原始主题（AnyMsg，不反序列化）。")

    def _make_raw_link_cb(self, spec):
        monitor = self.link_monitor

        def cb(msg):
            try:
                monitor.record(spec.drone_id, spec.topic, len(getattr(msg, "_buff", b"") or b""))
            except Exception:
                # 监测永远不能影响主链路或 ROS 回调线程，异常直接吞掉。
                pass

        return cb

    def _record_link(self, drone_id: int, topic: str, msg) -> None:
        """强类型回调里上报一次到达（带时间戳时可以算时延）。"""
        monitor = self.link_monitor
        if monitor is None:
            return
        try:
            monitor.record(drone_id, topic, _serialized_size(msg), stamp_sec=_header_stamp(msg))
        except Exception:
            pass

    def _make_battery_cb(self, uav_id, ns):
        topic = f"/{ns}/mavros/battery"
        last = [None]
        def cb(msg):
            # 先上报链路统计，再按 1Hz 节流刷新界面：节流只该影响显示，
            # 不该让监测少统计消息。
            self._record_link(uav_id, topic, msg)
            now = time.monotonic()
            if last[0] is not None and now - last[0] < 1.0:   # 1Hz 节流
                return
            if last[0] is None:
                self._log(f"已收到 {uav_id} 号机电量：{topic}")
            last[0] = now
            pct = msg.percentage
            if not math.isfinite(pct) or not 0.0 <= pct <= 1.0:
                pct = -1.0
            self.battery_callback(uav_id, pct, msg.voltage)
        return cb

    def _make_pose_cb(self, uav_id):
        topic = f"/uav{uav_id}/mavros/local_position/pose"
        last = [None]
        def cb(msg):
            self._record_link(uav_id, topic, msg)
            now = time.monotonic()
            if last[0] is not None and now - last[0] < 0.2:   # 5Hz
                return
            p = msg.pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                return
            if last[0] is None:
                self._log(f"已收到 {uav_id} 号机位置：{topic}")
            last[0] = now
            self.pose_callback(uav_id, p.x, p.y, p.z)
        return cb

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


# ----------------------------------------------------------------------
# 链路监测用到的小工具
# ----------------------------------------------------------------------
def _serialized_size(msg) -> int:
    """消息序列化后的字节数，即链路上实际传输的载荷大小。

    genpy 的 ``serialize(buff)`` **必须**传入 buffer（没有默认值），直接调
    ``msg.serialize()`` 会抛 TypeError；以前就是这样一路 fallback 到 0，
    导致电量/位姿的带宽恒为 0.00。
    """
    try:
        from io import BytesIO

        buff = BytesIO()
        msg.serialize(buff)
        return len(buff.getvalue())
    except Exception:
        pass
    try:
        data = msg.serialize()          # 某些自定义消息允许不传 buffer
        if data is not None:
            return len(data)
    except Exception:
        pass
    try:
        return len(getattr(msg, "_buff", b"") or b"")
    except Exception:
        return 0


def _header_stamp(msg) -> Optional[float]:
    """取消息 header.stamp 的秒数；没有 header 时返回 None。"""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    try:
        return float(stamp.to_sec())
    except Exception:
        pass
    try:
        return float(getattr(stamp, "secs")) + float(getattr(stamp, "nsecs")) * 1e-9
    except Exception:
        return None


def _master_uri() -> str:
    """取 master 地址。

    rosgraph 的 ``getUri()`` 实际返回的是 XML-RPC 响应三元组
    ``[code, statusMessage, uri]``，直接格式化会打出 ``[1, '', 'http://...']``，
    这里把它拆成纯地址。
    """
    try:
        result = rospy.get_master().getUri()
    except Exception:
        return "未知（无法连接 master）"
    if isinstance(result, (list, tuple)) and len(result) == 3:
        return str(result[2])
    return str(result)

