"""Persist observed battery depletion for later analysis.

These records contain telemetry observation time, not verified airborne time.
The current bridge does not forward an armed/landed state to the ground station.
"""

import csv
from datetime import datetime, timedelta
import os


FIELDS = (
    "uav_id", "start_local", "end_local", "duration_seconds",
    "start_percent", "end_percent", "consumed_percent", "percent_per_minute",
    "flight_confirmed",
)


class BatteryHistoryRecorder:
    def __init__(self, path):
        self.path = path
        self.checkpoints = {}

    def record(self, uav_id, monitor, now):
        """Append one nonoverlapping interval after >=60 s and >=2% drain."""
        if not monitor.samples:
            self.checkpoints.pop(uav_id, None)
            return
        first_time, first_pct = monitor.samples[0]
        checkpoint = self.checkpoints.get(uav_id)
        if checkpoint is None or checkpoint[0] < first_time:
            checkpoint = (first_time, first_pct)
            self.checkpoints[uav_id] = checkpoint
        start_time, start_pct = checkpoint
        end_time, end_pct = monitor.samples[-1]
        duration = end_time - start_time
        consumed = start_pct - end_pct
        if duration < 60 or consumed < 0.02 - 1e-6:
            return
        end_local = datetime.now().astimezone()
        start_local = end_local - timedelta(seconds=duration)
        row = {
            "uav_id": uav_id,
            "start_local": start_local.isoformat(timespec="seconds"),
            "end_local": end_local.isoformat(timespec="seconds"),
            "duration_seconds": round(duration, 1),
            "start_percent": round(start_pct * 100, 1),
            "end_percent": round(end_pct * 100, 1),
            "consumed_percent": round(consumed * 100, 1),
            "percent_per_minute": round(consumed * 6000 / duration, 2),
            "flight_confirmed": False,
        }
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        new_file = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
        with open(self.path, "a", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=FIELDS)
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        self.checkpoints[uav_id] = (end_time, end_pct)
