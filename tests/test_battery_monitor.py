"""Battery warning hysteresis and trend estimation without ROS or Qt."""
import unittest

from battery_monitor import BatteryMonitor


class BatteryMonitorTests(unittest.TestCase):
    def test_warning_levels_and_hysteresis(self):
        monitor = BatteryMonitor()
        for pct, expected in ((0.90, 0), (0.80, 1), (0.60, 2),
                              (0.30, 3), (0.20, 4)):
            monitor.update(pct, 0)
            self.assertEqual(monitor.level, expected)
        monitor.update(0.21, 1)
        self.assertEqual(monitor.level, 4)
        monitor.update(0.23, 2)
        self.assertEqual(monitor.level, 3)

    def test_estimate_requires_sustained_drop_and_resets(self):
        monitor = BatteryMonitor()
        for second in range(0, 31, 5):
            monitor.update(0.80 - 0.01 * second / 30, second)
        self.assertIsNone(monitor.estimate_minutes(30))
        for second in range(35, 91, 5):
            monitor.update(0.79 - 0.02 * (second - 30) / 60, second)
        self.assertAlmostEqual(monitor.estimate_minutes(90), 23.5)
        monitor.update(0.82, 91)  # charging or changed pack
        self.assertIsNone(monitor.estimate_minutes(91))
        monitor.update(0.78, 101)
        self.assertIsNone(monitor.estimate_minutes(101))

    def test_stale_and_invalid_data_never_produce_eta(self):
        monitor = BatteryMonitor()
        monitor.update(0.70, 0)
        monitor.update(0.66, 90)  # long gap starts a new trend
        self.assertIsNone(monitor.estimate_minutes(90))
        self.assertEqual(monitor.status(101), "stale")
        self.assertIsNone(monitor.estimate_minutes(101))
        monitor.update(-1, 102)
        self.assertEqual(monitor.status(102), "invalid")
        self.assertIsNone(monitor.estimate_minutes(102))


if __name__ == "__main__":
    unittest.main()
