"""Ground-station battery display policy and conservative trend estimate.

The estimate is time to a 30% reserve at the *observed* recent drain rate.
It is not a flight-time guarantee: load, wind and cell health are unknown.
"""

from collections import deque
import math


STALE_SECONDS = 10.0
RESERVE = 0.30
# UI prompts intentionally precede PX4's own low-battery failsafe thresholds.
THRESHOLDS = (0.80, 0.60, 0.30, 0.20)
HYSTERESIS = 0.02


class BatteryMonitor:
    def __init__(self):
        self.samples = deque()
        self.last_seen = None
        self.pct = None
        self.level = 0

    def update(self, pct: float, now: float) -> None:
        valid = math.isfinite(pct) and 0 <= pct <= 1
        if not valid:
            self.pct = None
            self.last_seen = now
            self.level = 0
            self.samples.clear()
            return

        # A gap or significant upward jump starts a fresh trend. A charging or
        # changed battery must not inherit the old discharge rate.
        if (self.last_seen is not None and now - self.last_seen > STALE_SECONDS) or (
            self.pct is not None and pct > self.pct + 0.03
        ):
            self.samples.clear()
        self.pct = pct
        self.last_seen = now
        self.samples.append((now, pct))
        while self.samples and now - self.samples[0][0] > 600:
            self.samples.popleft()

        candidate = sum(pct <= threshold for threshold in THRESHOLDS)
        if candidate < self.level and self.level:
            # Keep a warning until the reading clears its own threshold by 2%.
            previous_threshold = THRESHOLDS[self.level - 1]
            if pct <= previous_threshold + HYSTERESIS:
                candidate = self.level
        self.level = candidate

    def status(self, now: float) -> str:
        if self.last_seen is None:
            return "waiting"
        if now - self.last_seen > STALE_SECONDS:
            return "stale"
        if self.pct is None:
            return "invalid"
        return "valid"

    def estimate_minutes(self, now: float):
        if self.status(now) != "valid" or self.pct <= RESERVE or len(self.samples) < 2:
            return None
        start_time, start_pct = self.samples[0]
        elapsed = now - start_time
        drop = start_pct - self.pct
        # A short or one-percent change is dominated by percentage rounding.
        if elapsed < 60 or drop < 0.02:
            return None
        rate_per_minute = drop * 60 / elapsed
        if rate_per_minute <= 0 or rate_per_minute > 0.20:
            return None
        remaining = (self.pct - RESERVE) / rate_per_minute
        return remaining if 0 < remaining <= 120 else None
