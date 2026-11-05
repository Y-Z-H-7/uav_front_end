import csv
from pathlib import Path
import tempfile
import unittest

from battery_history import BatteryHistoryRecorder
from battery_monitor import BatteryMonitor


class BatteryHistoryTests(unittest.TestCase):
    def test_nonoverlapping_telemetry_intervals_and_no_false_flight_claim(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "logs" / "battery_discharge.csv"
            recorder = BatteryHistoryRecorder(str(path))
            monitor = BatteryMonitor()
            for second in range(0, 181, 5):
                monitor.update(0.80 - 0.06 * second / 180, second)
                recorder.record(3, monitor, second)
            with path.open(newline="", encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(len(rows), 3)
            self.assertEqual([float(row["consumed_percent"]) for row in rows], [2, 2, 2])
            self.assertTrue(all(row["flight_confirmed"] == "False" for row in rows))
            self.assertEqual(float(rows[-1]["end_percent"]), 74)


if __name__ == "__main__":
    unittest.main()
