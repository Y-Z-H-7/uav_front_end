#!/usr/bin/env python3
"""地面站侧的通信链路状态测量（纯 Python，不依赖 ROS 和 Qt）。

职责边界
--------
- 这里只做"测量"：ROS 回调里调用 :meth:`LinkMonitor.record` 记录一次到达，
  UI 用定时器调用 :meth:`LinkMonitor.snapshot` 取一份快照渲染。
- 线程安全：rospy 回调运行在 ROS 自身的线程里，UI 在主线程，所有状态用
  ``threading.Lock`` 保护，快照返回的是普通 Python 对象，可以放心跨线程传。

三个指标怎么算
--------------
1. 频率 Hz：滑动窗口内到达条数 / 窗口实际时长。窗口没攒满时用
   ``now - 首条到达时刻`` 作为时长，避免刚启动时数值虚高。
2. 带宽 KB/s：滑动窗口内消息序列化字节数 / 同一时长。字节数来自
   ``len(msg._buff)``（AnyMsg 原始缓冲，就是链路上真实的载荷大小）。
3. 时延 ms：只对带 ``header.stamp`` 的消息有效。地面站和机载电脑的时钟一般
   不同步，直接相减会得到一个很大的常数偏差（例如 5 分钟 = 300000 ms）。
   这里采用"窗口内最小单向时延"估计时钟偏差：把窗口里最小的
   ``到达时刻 - 消息时间戳`` 当作基线扣掉，剩下的就是排队/拥塞带来的额外
   时延。两端时钟已同步（NTP/PTP）时该基线约为 0，结果即真实单向时延。

另外提供抖动（到达间隔的标准差）和静默时长（距最后一条消息的秒数），
后者可以直接判断"这条链路是断了还是只是发得慢"。
"""

from __future__ import annotations

import csv
import os
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

# 频率/带宽统计窗口（秒）。取 2 秒是为了在 10 Hz 左右的探索话题上
# 也能稳定算出小数位，同时不至于让界面反应太迟钝。
DEFAULT_WINDOW_SECONDS = 2.0

# 时钟偏差估计窗口（秒）。窗口越长基线越稳，但机上时钟漂移/重启后收敛越慢。
DEFAULT_OFFSET_WINDOW_SECONDS = 60.0

# 超过该秒数没有新消息，判定该主题中断（实际阈值还会按峰值频率放宽）。
DEFAULT_STALE_SECONDS = 3.0

# 少于该样本数时只显示“采集中”，不急着给频率结论。
MIN_SAMPLES_FOR_RATE = 3

# 窗口内样本数达到该值后，频率改用“条数 / 窗口时长”（有界），
# 否则用间隔倒数（对 0.5~2 Hz 的慢主题更准）。
RATE_MIN_SAMPLES = 5

# 频率低于峰值该比例时判定“稀疏”（链路时断时续、丢包严重）。
DEGRADED_RATIO = 0.5

STATUS_WAITING = "等待"
STATUS_COLLECTING = "采集中"
STATUS_OK = "正常"
STATUS_SPARSE = "稀疏"
STATUS_STALE = "中断"


# 主题名片段 -> 中文说明（按顺序匹配，先具体后宽泛）。
TOPIC_DESCRIPTIONS: Tuple[Tuple[str, str], ...] = (
    ("drone_state_recv", "集群探索状态：各机探索进度与当前目标，供协同决策"),
    ("pair_opt_res_recv", "配对优化结果：两机协同配对请求的应答"),
    ("pair_opt_recv", "配对优化请求：决定哪两架机协同探索"),
    ("swarm_traj_recv", "B 样条轨迹：其他机共享过来的飞行轨迹"),
    ("hgrid_recv", "分层栅格地图：探索规划用的网格地图"),
    ("grid_tour_recv", "栅格遍历顺序：探索目标的访问序列"),
    ("chunk_stamps_recv", "地图块时间戳：多机地图同步的版本信息，避免重复传块"),
    ("chunk_data_recv", "地图块数据：多机地图同步的体素内容"),
    ("viewpoints", "探索视点可视化（RViz Marker）"),
    ("occupancy_all", "全局占据栅格点云：建图输出，本链路里流量最大的主题"),
    ("travel_traj", "已飞轨迹可视化（RViz Marker）"),
    ("position_cmd_vis", "位置指令可视化（RViz Marker）"),
    ("mavros/battery", "电池状态：电量与电压（来自 MAVROS）"),
    ("mavros/local_position/pose", "本地位置估计：x/y/z（来自 MAVROS）"),
)


def describe_topic(topic: str) -> str:
    """按主题名给出中文说明；匹配不到时返回空串。"""
    for fragment, text in TOPIC_DESCRIPTIONS:
        if fragment in topic:
            return text
    return ""


@dataclass(frozen=True)
class LinkTopicSpec:
    """一条被监控的链路主题。

    ``has_stamp`` 为 True 表示这条主题用带 ``header`` 的强类型消息订阅，
    因而能算时延；其余主题用 ``rospy.AnyMsg`` 订阅，只算频率和带宽。
    """

    drone_id: int
    topic: str
    msg_type: str
    direction: str = "recv"
    has_stamp: bool = False

    @property
    def key(self) -> Tuple[int, str]:
        return (self.drone_id, self.topic)

    @property
    def description(self) -> str:
        """这条主题承载什么内容（中文，用于界面的说明列）。"""
        return describe_topic(self.topic)


@dataclass
class LinkRow:
    """一次快照里的一行，字段与界面表格列一一对应。"""

    drone_id: int
    topic: str
    msg_type: str
    direction: str
    hz: float
    kbytes_per_s: float
    latency_ms: Optional[float]
    jitter_ms: Optional[float]
    age_s: Optional[float]
    offset_ms: Optional[float]
    peak_hz: float
    samples: int
    status: str
    has_stamp: bool = False


class _TopicState:
    """单个主题的累计量。全部时间都用 time.monotonic() 的秒数。"""

    __slots__ = (
        "window", "samples", "total_samples",
        "last_t", "last_delay", "delays", "peak_hz", "prev_t", "first_t",
    )

    def __init__(self, window: float) -> None:
        self.window = window
        self.samples: Deque[Tuple[float, int]] = deque()
        self.total_samples = 0
        self.last_t: Optional[float] = None
        self.last_delay: Optional[float] = None
        self.delays: Deque[Tuple[float, float]] = deque()
        self.peak_hz = 0.0
        # 被窗口挤出去的最后一条消息时刻，用来给“窗口里只剩一条”的慢主题
        # 估算频率；first_t 是这个主题第一次收到消息的时刻。
        self.prev_t: Optional[float] = None
        self.first_t: Optional[float] = None

    def prune(self, now: float, offset_window: float) -> None:
        cut = now - self.window
        while self.samples and self.samples[0][0] < cut:
            self.prev_t = self.samples.popleft()[0]
        cut = now - offset_window
        while self.delays and self.delays[0][0] < cut:
            self.delays.popleft()

    def offset(self) -> Optional[float]:
        """窗口内最小单向时延，作为时钟偏差基线（秒）。"""
        if not self.delays:
            return None
        return min(delay for _, delay in self.delays)


class LinkMonitor:
    """链路指标采集器。一个实例服务整块界面，按 (drone_id, topic) 分桶。"""

    def __init__(
        self,
        specs: Iterable[LinkTopicSpec],
        window: float = DEFAULT_WINDOW_SECONDS,
        offset_window: float = DEFAULT_OFFSET_WINDOW_SECONDS,
        stale_seconds: float = DEFAULT_STALE_SECONDS,
    ) -> None:
        self._specs: List[LinkTopicSpec] = list(specs)
        self._by_key: Dict[Tuple[int, str], LinkTopicSpec] = {s.key: s for s in self._specs}
        self._order: List[Tuple[int, str]] = [s.key for s in self._specs]
        self._states: Dict[Tuple[int, str], _TopicState] = {
            s.key: _TopicState(window) for s in self._specs
        }
        self._window = window
        self._offset_window = offset_window
        self._stale_seconds = stale_seconds
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ 配置
    def specs(self) -> Sequence[LinkTopicSpec]:
        return tuple(self._specs)

    def spec_for(self, drone_id: int, topic: str) -> Optional[LinkTopicSpec]:
        return self._by_key.get((drone_id, topic))

    # ------------------------------------------------------------------ 记录
    def record(
        self,
        drone_id: int,
        topic: str,
        nbytes: int,
        recv_time: Optional[float] = None,
        stamp_sec: Optional[float] = None,
        wall_time: Optional[float] = None,
    ) -> bool:
        """记录一次消息到达。返回 False 表示该主题不在监控清单里。

        ``stamp_sec`` 是消息自带的时间戳（ROS 时间，epoch 秒）；只有
        ``has_stamp`` 的主题才会用它算时延。

        注意：``recv_time`` 用 ``time.monotonic()``（算间隔、频率），而时延必须
        用**墙钟** ``time.time()`` 和消息时间戳相减——两者混用会差出整个 epoch
        （约 1.8e9 秒），算出来的时延会被有效性检查直接丢掉。
        """
        key = (drone_id, topic)
        spec = self._by_key.get(key)
        if spec is None:
            return False
        now = time.monotonic() if recv_time is None else recv_time
        delay: Optional[float] = None
        if spec.has_stamp and stamp_sec is not None:
            wall = time.time() if wall_time is None else wall_time
            delay = wall - stamp_sec
            # 时间戳为 0 或明显异常（例如未填充 header）时不计入时延统计，
            # 否则会把“时钟偏差基线”污染成负的几十万秒。
            if not (-1e6 < delay < 1e6):
                delay = None
        with self._lock:
            st = self._states[key]
            if st.first_t is None:
                st.first_t = now
            st.samples.append((now, max(0, int(nbytes))))
            st.total_samples += 1
            st.last_t = now
            if delay is not None:
                st.last_delay = delay
                st.delays.append((now, delay))
            st.prune(now, self._offset_window)
        return True

    def reset(self) -> None:
        """清零统计（做前后对照实验前调用）。"""
        with self._lock:
            for key in list(self._states):
                self._states[key] = _TopicState(self._window)

    # ------------------------------------------------------------------ 快照
    def snapshot(self, now: Optional[float] = None) -> List[LinkRow]:
        """按配置顺序返回所有主题的当前指标。"""
        now = time.monotonic() if now is None else now
        rows: List[LinkRow] = []
        with self._lock:
            for key in self._order:
                spec = self._by_key[key]
                st = self._states[key]
                st.prune(now, self._offset_window)

                n = len(st.samples)
                # 频率有两种算法，按“窗口是否攒满”切换：
                # - 窗口已满且样本足够多（>=5）：用“窗口内条数 / 窗口时长”。
                #   它天然有界，桥接一次性吐出积压消息（几十条挤在几毫秒里）
                #   也不会算出几千 Hz；峰值也只由这种估计更新。
                # - 否则（刚启动、慢主题）：用“间隔倒数”，否则 0.5 Hz 的电量
                #   会被量化成 0.5/1.0 这种跳变，刚启动的话题也会被低估。
                window_full = (st.first_t is not None
                               and (now - st.first_t) >= self._window)
                if n >= RATE_MIN_SAMPLES and window_full:
                    hz = n / self._window
                    if hz > st.peak_hz:
                        st.peak_hz = hz
                elif n >= 2:
                    span = st.samples[-1][0] - st.samples[0][0]
                    hz = (n - 1) / span if span > 1e-6 else 0.0
                elif n == 1 and st.prev_t is not None and st.samples[0][0] > st.prev_t:
                    hz = 1.0 / (st.samples[0][0] - st.prev_t)
                else:
                    hz = 0.0
                # 带宽 = 窗口内字节数 / 窗口覆盖时长。用“首条消息以来”的时长
                # 而不是窗口内首条消息的时长：后者在窗口刚滚过一条消息时会把
                # 一个包放大成几 KB/s。
                if n >= 1:
                    since_first = now - (st.first_t if st.first_t is not None
                                         else st.samples[0][0])
                    elapsed = min(self._window, max(since_first, 0.2))
                    kbps = sum(b for _, b in st.samples) / elapsed / 1024.0
                else:
                    kbps = 0.0

                age = None if st.last_t is None else max(0.0, now - st.last_t)
                jitter = _gap_stdev_ms(st.samples)
                offset = st.offset()
                latency = None
                if st.last_delay is not None:
                    base = offset if offset is not None else 0.0
                    latency = max(0.0, (st.last_delay - base) * 1000.0)

                rows.append(
                    LinkRow(
                        drone_id=spec.drone_id,
                        topic=spec.topic,
                        msg_type=spec.msg_type,
                        direction=spec.direction,
                        hz=hz,
                        kbytes_per_s=kbps,
                        latency_ms=latency,
                        jitter_ms=jitter,
                        age_s=age,
                        offset_ms=None if offset is None else offset * 1000.0,
                        peak_hz=st.peak_hz,
                        samples=st.total_samples,
                        status=self._status(st, age, hz, n),
                        has_stamp=spec.has_stamp,
                    )
                )
        return rows

    def _status(self, st: _TopicState, age: Optional[float], hz: float, n: int) -> str:
        if st.last_t is None:
            return STATUS_WAITING
        peak = max(st.peak_hz, 0.5)
        # 发得越慢的主题，允许的静默时间越长（至少 3 秒，通常 4 个周期）。
        stale_after = max(self._stale_seconds, 4.0 / peak)
        if age is not None and age > stale_after:
            return STATUS_STALE
        # “采集中”按累计条数判断，而不是窗口内条数：1 Hz 的电量在 2 秒窗口里
        # 永远凑不够 3 条，否则会一直显示“采集中”。
        if st.total_samples < MIN_SAMPLES_FOR_RATE:
            return STATUS_COLLECTING
        # 窗口里样本太少时算出的频率不可靠，不做“稀疏”判断。
        if n >= 3 and st.peak_hz > 0 and hz < st.peak_hz * DEGRADED_RATIO:
            return STATUS_SPARSE
        return STATUS_OK

    # ------------------------------------------------------------------ 汇总
    def summary(self, rows: Optional[Sequence[LinkRow]] = None) -> Dict[int, Dict[str, object]]:
        """按无人机汇总：在线主题数、总带宽、时钟偏差。"""
        rows = self.snapshot() if rows is None else rows
        out: Dict[int, Dict[str, object]] = {}
        for row in rows:
            item = out.setdefault(
                row.drone_id,
                {"total": 0, "online": 0, "kbytes_per_s": 0.0,
                 "offset_ms": None, "interrupted": 0, "waiting": 0},
            )
            item["total"] = int(item["total"]) + 1
            if row.status in (STATUS_OK, STATUS_SPARSE, STATUS_COLLECTING):
                item["online"] = int(item["online"]) + 1
            elif row.status == STATUS_STALE:
                item["interrupted"] = int(item["interrupted"]) + 1
            elif row.status == STATUS_WAITING:
                item["waiting"] = int(item["waiting"]) + 1
            item["kbytes_per_s"] = float(item["kbytes_per_s"]) + row.kbytes_per_s
            if row.offset_ms is not None:
                cur = item["offset_ms"]
                item["offset_ms"] = row.offset_ms if cur is None else min(float(cur), row.offset_ms)
        return out

    # ------------------------------------------------------------------ 导出
    def write_csv(self, path: str, rows: Optional[Sequence[LinkRow]] = None) -> str:
        """把当前快照写成 CSV（UTF-8 BOM，Excel 可直接打开）。"""
        rows = self.snapshot() if rows is None else rows
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8-sig") as fh:
            writer = csv.writer(fh)
            writer.writerow([
                "drone_id", "topic", "msg_type", "direction",
                "hz", "kbytes_per_s", "latency_ms", "jitter_ms",
                "age_s", "offset_ms", "peak_hz", "samples", "status",
            ])
            for row in rows:
                writer.writerow([
                    row.drone_id, row.topic, row.msg_type, row.direction,
                    f"{row.hz:.2f}", f"{row.kbytes_per_s:.2f}",
                    "" if row.latency_ms is None else f"{row.latency_ms:.1f}",
                    "" if row.jitter_ms is None else f"{row.jitter_ms:.1f}",
                    "" if row.age_s is None else f"{row.age_s:.2f}",
                    "" if row.offset_ms is None else f"{row.offset_ms:.1f}",
                    f"{row.peak_hz:.2f}", row.samples, row.status,
                ])
        return path


def _gap_stdev_ms(samples: Deque[Tuple[float, int]]) -> Optional[float]:
    """到达间隔的标准差（毫秒）。样本不足时返回 None。"""
    if len(samples) < 3:
        return None
    times = [t for t, _ in samples]
    gaps = [later - earlier for earlier, later in zip(times, times[1:])]
    if not gaps:
        return None
    return statistics.pstdev(gaps) * 1000.0


# ---------------------------------------------------------------------- 配置
def default_config_candidates() -> List[str]:
    """桥接 GCS 配置的常见位置（可用环境变量覆盖）。"""
    env = os.environ.get("SWARM_BRIDGE_GCS_CONFIG")
    candidates = [env] if env else []
    candidates.extend([
        "~/swarm_ros_bridge_ws/src/swarm_ros_bridge/config/ros_topics_gcs.yaml",
        "~/catkin_ws/src/swarm_ros_bridge/config/ros_topics_gcs.yaml",
        "~/racer/src/swarm_ros_bridge/config/ros_topics_gcs.yaml",
    ])
    return [os.path.expanduser(path) for path in candidates if path]


def _base_specs(drone_ids: Iterable[int]) -> List[LinkTopicSpec]:
    """始终监控的两条关键状态主题（ros_adapter 用强类型订阅，可算时延）。"""
    specs: List[LinkTopicSpec] = []
    for drone_id in sorted(drone_ids):
        specs.append(LinkTopicSpec(
            drone_id, f"/uav{drone_id}/mavros/battery",
            "sensor_msgs/BatteryState", "recv", has_stamp=True,
        ))
        specs.append(LinkTopicSpec(
            drone_id, f"/uav{drone_id}/mavros/local_position/pose",
            "geometry_msgs/PoseStamped", "recv", has_stamp=True,
        ))
    return specs


def load_specs_from_bridge_config(
    path: str,
    drone_ips: Dict[int, str],
) -> List[LinkTopicSpec]:
    """从 swarm_ros_bridge 的 ros_topics_gcs.yaml 读出 recv_topics。

    用 srcIP 名字（robot1/robot2/robot3）在 YAML 的 IP 表里查到实际地址，
    再拿地址反查本工程的 DRONES 配置，得到无人机编号。这样主题清单和
    桥接配置保持同一份来源，改配置后前端不用跟着改代码。
    """
    try:
        import yaml  # 可选依赖：没装 pyyaml 时退化为只监控电量/位姿
    except Exception:
        return []
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except Exception:
        return []

    if not isinstance(data, dict):
        return []
    ip_by_name = {
        str(name): str(value)
        for name, value in (data.get("IP") or {}).items()
        if str(name) != "self"
    }
    drone_by_ip = {str(ip): drone_id for drone_id, ip in drone_ips.items()}

    specs: List[LinkTopicSpec] = []
    for entry in data.get("recv_topics") or []:
        if not isinstance(entry, dict):
            continue
        topic = entry.get("topic_name")
        msg_type = entry.get("msg_type") or ""
        if not topic:
            continue
        src_name = str(entry.get("srcIP") or "")
        drone_id = drone_by_ip.get(ip_by_name.get(src_name, ""))
        if drone_id is None and src_name.startswith("robot"):
            # IP 表被改过时的兜底：robotN 直接对应 N 号机
            try:
                candidate = int(src_name[len("robot"):])
            except ValueError:
                candidate = None
            if candidate in drone_ips:
                drone_id = candidate
        if drone_id is None:
            continue
        specs.append(LinkTopicSpec(drone_id, str(topic), str(msg_type), "recv"))
    return specs


def build_link_specs(
    drone_ips: Dict[int, str],
    config_path: Optional[str] = None,
) -> Tuple[List[LinkTopicSpec], Optional[str]]:
    """组装最终监控清单。

    返回 (specs, 实际使用的配置文件路径)。配置文件不存在或解析失败时，
    只保留电量/位姿两条关键主题——界面仍然可用，只是看不到全部主题。
    """
    base = _base_specs(drone_ips)
    known = {spec.key for spec in base}

    candidates = [config_path] if config_path else default_config_candidates()
    for path in candidates:
        if not path or not os.path.isfile(path):
            continue
        extra = load_specs_from_bridge_config(path, drone_ips)
        if not extra:
            continue
        for spec in extra:
            if spec.key in known:
                continue
            known.add(spec.key)
            base.append(spec)
        return _sort_specs(base), path

    return _sort_specs(base), None


# 主题优先级：数字越小越靠上。监测面板首先要看到的是关键状态消息，
# 而不是按字母序排在最前的 /multi_map_manager/chunk_data。
PRIORITY_FRAGMENTS: Tuple[Tuple[str, int], ...] = (
    ("mavros/battery", 0),
    ("mavros/local_position/pose", 0),
    ("drone_state_recv", 1),
    ("pair_opt", 1),
    ("hgrid", 2),
    ("grid_tour", 2),
    ("swarm_traj", 2),
    ("chunk_", 2),
    ("occupancy_all", 3),
    ("viewpoints", 3),
    ("travel_traj", 3),
    ("position_cmd_vis", 3),
)


def topic_priority(topic: str) -> int:
    """主题的展示优先级（0=最关键）。匹配不到时排到最后。"""
    for fragment, rank in PRIORITY_FRAGMENTS:
        if fragment in topic:
            return rank
    return 9


def _sort_specs(specs: List[LinkTopicSpec]) -> List[LinkTopicSpec]:
    """按 飞机 -> 优先级 -> 主题名 排序，保证界面与 CSV 顺序一致。"""
    return sorted(specs, key=lambda s: (s.drone_id, topic_priority(s.topic), s.topic))

