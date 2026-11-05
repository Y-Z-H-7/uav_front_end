"""Keep bounded snapshots around each topic's latest session bandwidth peak."""

import csv
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
import os

from link_monitor import STATUS_COLLECTING, STATUS_OK, STATUS_SPARSE, STATUS_STALE


@dataclass
class _Frame:
    at: float
    wall_time: float
    rows: tuple


@dataclass
class _PeakEvent:
    at: float
    value: float
    frames: list = field(default_factory=list)


class PeakContextRecorder:
    def __init__(self, radius_seconds=5.0):
        self.radius = radius_seconds
        self._recent = deque()
        self._events = {}

    def reset(self):
        self._recent.clear()
        self._events.clear()

    def observe(self, at, wall_time, rows):
        """Called on each UI refresh; a newer peak replaces the old context."""
        frame = _Frame(at, wall_time, tuple(rows))
        self._recent.append(frame)
        while self._recent and self._recent[0].at < at - self.radius:
            self._recent.popleft()
        for row in rows:
            if row.peak_at is None:
                continue
            key = (row.drone_id, row.topic)
            event = self._events.get(key)
            if event is None or event.at != row.peak_at:
                event = _PeakEvent(row.peak_at, row.peak_kbytes_per_s)
                event.frames = [item for item in self._recent
                                if item.at >= event.at - self.radius]
                self._events[key] = event
            elif at <= event.at + self.radius and (
                not event.frames or event.frames[-1].at != at
            ):
                event.frames.append(frame)

    @property
    def peak_count(self):
        return len(self._events)

    def write_csv(self, path):
        """Export each peak's ±5 s context, including all topics on that UAV."""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8-sig") as output:
            writer = csv.writer(output)
            writer.writerow((
                "peak_drone_id", "peak_topic", "peak_kbytes_per_s",
                "relative_seconds", "sample_time_local", "observed_topic",
                "hz", "kbytes_per_s", "latency_ms", "jitter_ms", "age_s",
                "status", "drone_total_kbytes_per_s", "drone_online",
                "drone_interrupted",
            ))
            for (drone_id, peak_topic), event in sorted(self._events.items()):
                for frame in event.frames:
                    same_drone = [row for row in frame.rows if row.drone_id == drone_id]
                    total = sum(row.kbytes_per_s for row in same_drone)
                    online = sum(row.status in (STATUS_OK, STATUS_SPARSE, STATUS_COLLECTING)
                                 for row in same_drone)
                    interrupted = sum(row.status == STATUS_STALE for row in same_drone)
                    sample_time = datetime.fromtimestamp(frame.wall_time).isoformat(
                        timespec="milliseconds")
                    for row in same_drone:
                        writer.writerow((
                            drone_id, peak_topic, f"{event.value:.2f}",
                            f"{frame.at - event.at:+.1f}", sample_time, row.topic,
                            f"{row.hz:.2f}", f"{row.kbytes_per_s:.2f}",
                            "" if row.latency_ms is None else f"{row.latency_ms:.1f}",
                            "" if row.jitter_ms is None else f"{row.jitter_ms:.1f}",
                            "" if row.age_s is None else f"{row.age_s:.2f}",
                            row.status, f"{total:.2f}", online, interrupted,
                        ))
        return path
