#!/usr/bin/env python3
"""图传控制层的测试：active.json 状态解析、每机摘要、ctl.py 调用。纯逻辑，不用 Qt。"""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import transfer_controller
from transfer_controller import TransferController


class TransferControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.home = Path(self.tmp.name) / "UAVLink"
        (self.home / "runtime").mkdir(parents=True)
        (self.home / "start_photos.sh").write_text("#!/usr/bin/env bash\n")
        (self.home / "runtime" / "ctl.py").write_text("# fake\n")
        self.ctl = TransferController(home=str(self.home))

    # ------------------------------------------------------------- 可用性
    def test_available_when_entry_scripts_exist(self):
        self.assertTrue(self.ctl.available)

    def test_unavailable_when_home_is_empty(self):
        missing = TransferController(home=str(Path(self.tmp.name) / "nope"))
        self.assertFalse(missing.available)
        self.assertIn("UAVLINK_HOME", missing.hint)

    # ------------------------------------------------------------- 状态读取
    def test_missing_active_file_means_not_running(self):
        status = self.ctl.read_status()
        self.assertFalse(status["running"])
        self.assertFalse(status["stale"])
        self.assertIsNone(status["state"])

    def test_fresh_active_file_means_running(self):
        self.ctl.active_path.write_text(json.dumps({"session": "run_x", "nodes": {}}))
        status = self.ctl.read_status()
        self.assertTrue(status["running"])
        self.assertFalse(status["stale"])
        self.assertEqual(status["state"]["session"], "run_x")

    def test_old_active_file_means_stale_not_running(self):
        self.ctl.active_path.write_text(json.dumps({"nodes": {}}))
        old = time.time() - transfer_controller.STALE_SECONDS - 5
        os.utime(self.ctl.active_path, (old, old))
        status = self.ctl.read_status()
        self.assertFalse(status["running"])
        self.assertTrue(status["stale"])
        self.assertGreater(status["age_s"], transfer_controller.STALE_SECONDS)

    def test_unparseable_active_file_is_stale_with_error(self):
        self.ctl.active_path.write_text("{不是 json")
        status = self.ctl.read_status()
        self.assertFalse(status["running"])
        self.assertTrue(status["stale"])
        self.assertIn("无法解析", status["error"])

    # ------------------------------------------------------------- 每机摘要
    def test_summarize_drones_extracts_counts(self):
        state = {
            "nodes": {
                "nuc": {"online": True, "status": {"counts": {"captures_saved": 12}}},
                "fly": {"online": False},
            },
            "receivers": {"nuc": {"photos_received": 4}},
        }
        items = {item["name"]: item for item in self.ctl.summarize_drones(state)}
        self.assertTrue(items["nuc"]["online"])
        self.assertEqual(items["nuc"]["saved"], 12)
        self.assertEqual(items["nuc"]["received"], 4)
        self.assertFalse(items["fly"]["online"])
        self.assertIsNone(items["fly"]["saved"])
        self.assertIsNone(items["fly"]["received"])

    def test_summarize_drones_tolerates_missing_sections(self):
        self.assertEqual(self.ctl.summarize_drones(None), [])
        self.assertEqual(self.ctl.summarize_drones({"nodes": None, "receivers": {}}), [])

    # ------------------------------------------------------------- 启动照片
    def test_start_photos_spawns_detached_process(self):
        recorded = {}

        def fake_popen(args, **kwargs):
            recorded["args"] = args
            recorded["kwargs"] = kwargs
            return object()

        with patch.object(transfer_controller.subprocess, "Popen", fake_popen):
            ok, message = self.ctl.start_photos()
        self.assertTrue(ok)
        self.assertEqual(recorded["args"][0], "bash")
        self.assertTrue(str(recorded["args"][1]).endswith("start_photos.sh"))
        # 必须脱离本进程生命周期：前端退出不能带走图传。
        self.assertTrue(recorded["kwargs"]["start_new_session"])
        self.assertIn("后台启动", message)

    def test_start_photos_reports_missing_install(self):
        missing = TransferController(home=str(Path(self.tmp.name) / "nope"))
        ok, message = missing.start_photos()
        self.assertFalse(ok)
        self.assertIn("未找到", message)

    # ------------------------------------------------------------- ctl 调用
    def _write_real_ctl(self, code, output="hello"):
        """写一个真的可执行 ctl.py 替身，走真实 subprocess 路径。"""
        script = (
            "import sys, time\n"
            f"print({output!r})\n"
            "if len(sys.argv) > 1 and sys.argv[1] == 'slow': time.sleep(5)\n"
            f"sys.exit({code})\n"
        )
        (self.home / "runtime" / "ctl.py").write_text(script)

    def setUpRealPython(self):
        # SYSTEM_PYTHON 写死为地面站的 /usr/bin/python3；测试里换成本地解释器。
        patcher = patch.object(
            transfer_controller, "SYSTEM_PYTHON", sys.executable)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_ctl_success_returns_stdout(self):
        self.setUpRealPython()
        self._write_real_ctl(0, "操作已完成")
        ok, message = self.ctl._ctl("stop", timeout=10)
        self.assertTrue(ok)
        self.assertIn("操作已完成", message)

    def test_ctl_failure_returns_output_with_ok_false(self):
        self.setUpRealPython()
        self._write_real_ctl(1, "请先运行start_photos.sh。")
        ok, message = self.ctl._ctl("stop", timeout=10)
        self.assertFalse(ok)
        self.assertIn("请先运行", message)

    def test_ctl_timeout_reports_instead_of_hanging(self):
        self.setUpRealPython()
        self._write_real_ctl(0)
        ok, message = self.ctl._ctl("slow", timeout=1)
        self.assertFalse(ok)
        self.assertIn("超时", message)

    def test_stop_photos_translates_success_message(self):
        self.setUpRealPython()
        self._write_real_ctl(0, "ok")
        ok, message = self.ctl.stop_photos()
        self.assertTrue(ok)
        self.assertIn("收尾", message)

    def test_video_start_passes_drone_name(self):
        self.setUpRealPython()
        seen = {}

        class FakeCompleted:
            returncode = 0
            stdout = "视频已启动"
            stderr = ""

        def fake_run(args, **kwargs):
            seen["args"] = args
            return FakeCompleted()

        with patch.object(transfer_controller.subprocess, "run", fake_run):
            ok, message = self.ctl.start_video("nuc2")
        self.assertTrue(ok)
        self.assertEqual(seen["args"][2:4], ["video-start", "nuc2"])
        self.assertIn("视频已启动", message)


if __name__ == "__main__":
    unittest.main()
