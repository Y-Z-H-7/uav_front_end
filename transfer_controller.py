#!/usr/bin/env python3
"""图传（UAVLink）控制层：照片/视频传输的启动、停止与状态读取，纯逻辑无 Qt。

后台事实
--------
- 地面站 ``~/Workspace/UAVLink``（uav-image-link-v2）是一套独立的图像采集回传系统，
  入口 ``start_photos.sh`` / ``start_{nuc,fly,nuc2}_video.sh`` 最终都调用
  ``runtime/ctl.py``，并把每次入口调用写入 ``logs/命令审计.jsonl``。
- ``ctl.py photos`` 会 exec 成 ``ground_manager.py``（照片主程序，常驻）：持续探测
  在线飞机，机载每秒存一张照片、每 3 秒回传一张，断网时机载继续保存、恢复后补传。
  程序化停止用 ``ctl.py stop``——与在主终端按 Ctrl+C 等价的优雅收尾：等在途照片
  落盘、撤销机载 tc 限速规则后才退出。
- 视频是单会话的：主程序一次只持有一个视频接收进程（桌面预览窗口），目前仅 nuc2
  开放，nuc/fly 的入口会返回明确的拒绝信息；关闭预览窗口或 ``ctl.py video-stop``
  只停视频，照片不受影响。
- 状态轮询不打控制 socket：主程序运行期间每 5 秒把完整状态写进
  ``runtime/active.json``，退出时删除该文件。这里只读该文件——存在、可解析且
  mtime 新鲜即认为在运行。三架飞机全部离线时探测循环最坏会阻塞约 1 分钟，
  所以过期阈值取 60 秒，避免把"正在探测"误报成"没在跑"。
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# 与 start_*.sh 保持一致：ctl.py 只依赖标准库，固定用系统 python。
SYSTEM_PYTHON = "/usr/bin/python3"

# 主程序每 5 秒写一次 active.json；全部飞机离线时探测循环最坏约 1 分钟。
STALE_SECONDS = 60.0

# 各操作超时：ctl.py 内部对控制 socket 的等待是 45 秒；video-start 还要等接收端
# 就绪（<=18s）与源端应答（<=28s），再留探测余量。
PHOTOS_STOP_TIMEOUT = 55
VIDEO_START_TIMEOUT = 90
VIDEO_STOP_TIMEOUT = 55


class TransferController:
    """UAVLink 图传系统的启动 / 停止 / 状态查询。

    所有方法都不抛异常，统一返回 ``(ok, message)``，消息直接可显示。
    """

    def __init__(self, home: Optional[str] = None) -> None:
        base = Path(home or os.environ.get("UAVLINK_HOME") or "~/Workspace/UAVLink")
        self.home = base.expanduser()
        self.ctl_path = self.home / "runtime" / "ctl.py"
        self.active_path = self.home / "runtime" / "active.json"
        self.photos_script = self.home / "start_photos.sh"

    # ---------------------------------------------------------------- 可用性
    @property
    def available(self) -> bool:
        """UAVLink 是否安装在本机（入口脚本与 ctl.py 是否存在）。"""
        return self.ctl_path.is_file() and self.photos_script.is_file()

    @property
    def hint(self) -> str:
        """不可用时给用户的提示。"""
        return (
            f"未找到 UAVLink 图传系统（{self.home}），本页按钮不可用；"
            "可用环境变量 UAVLINK_HOME 指定其安装路径。"
        )

    # ---------------------------------------------------------------- 状态
    def read_status(self) -> Dict[str, object]:
        """读 active.json，返回 {running, stale, age_s, state, error}。

        - ``running``：照片主程序在运行（文件存在、可解析、未过期）。
        - ``stale``：文件还在但超过 STALE_SECONDS 没更新——收尾中或主程序无响应。
        - ``state``：主程序写入的完整状态字典（running 时才有值）。
        """
        result: Dict[str, object] = {
            "running": False, "stale": False, "age_s": None, "state": None, "error": None,
        }
        try:
            age = max(0.0, time.time() - self.active_path.stat().st_mtime)
        except OSError:
            return result  # 文件不存在：未运行
        result["age_s"] = age
        try:
            with open(self.active_path, "r", encoding="utf-8") as fh:
                state = json.load(fh)
        except (OSError, ValueError) as exc:
            result["stale"] = True
            result["error"] = f"active.json 无法解析：{exc}"
            return result
        if not isinstance(state, dict):
            result["stale"] = True
            result["error"] = "active.json 内容不是对象"
            return result
        result["state"] = state
        if age > STALE_SECONDS:
            result["stale"] = True
        else:
            result["running"] = True
        return result

    @staticmethod
    def summarize_drones(state: Optional[dict]) -> List[Dict[str, object]]:
        """从状态字典提取每机摘要（界面逐机一行用）。

        - ``saved``：机载端累计保存的照片数（离线时可能为 None）。
        - ``received``：地面站累计收到的照片数。
        """
        out: List[Dict[str, object]] = []
        nodes = (state or {}).get("nodes") or {}
        receivers = (state or {}).get("receivers") or {}
        for name, node in nodes.items():
            if not isinstance(node, dict):
                continue
            counts = ((node.get("status") or {}).get("counts")) or {}
            out.append({
                "name": str(name),
                "online": bool(node.get("online")),
                "saved": counts.get("captures_saved"),
                "received": ((receivers.get(name) or {}).get("photos_received")
                             if isinstance(receivers.get(name), dict) else None),
            })
        return out

    # ---------------------------------------------------------------- 操作
    def start_photos(self) -> Tuple[bool, str]:
        """后台启动照片主程序（start_photos.sh，脱离本进程的生命周期）。

        用 Popen + start_new_session：主程序常驻，前端退出/重启不影响图传，
        这与"在独立终端跑 start_photos.sh 且不要关闭终端"的用法等价。
        """
        if not self.available:
            return False, self.hint
        try:
            subprocess.Popen(
                ["bash", str(self.photos_script)],
                cwd=str(self.home),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            return False, f"照片传输启动失败：{exc}"
        return True, (
            "照片传输已在后台启动：首次在线探测约需 10 秒，"
            "运行日志在 ~/Workspace/UAVLink/logs/（前端退出不影响其运行）"
        )

    def stop_photos(self) -> Tuple[bool, str]:
        """请求照片主程序优雅收尾（在途照片落盘、撤销 tc 规则后自行退出）。"""
        ok, message = self._ctl("stop", timeout=PHOTOS_STOP_TIMEOUT)
        if ok:
            message = "已请求停止照片传输：收尾需等待在途照片与 tc 规则撤销（最长约 1 分钟）"
        return ok, message

    def start_video(self, name: str) -> Tuple[bool, str]:
        """启动某架飞机的视频预览（需要照片主程序已在运行）。"""
        return self._ctl("video-start", name, timeout=VIDEO_START_TIMEOUT)

    def stop_video(self) -> Tuple[bool, str]:
        """停止当前视频预览（照片采集与回传继续）。"""
        return self._ctl("video-stop", timeout=VIDEO_STOP_TIMEOUT)

    # ---------------------------------------------------------------- 内部
    def _ctl(self, *args: str, timeout: int) -> Tuple[bool, str]:
        """调用 ctl.py 的控制子命令，返回 (ok, 输出消息)。"""
        try:
            proc = subprocess.run(
                [SYSTEM_PYTHON, str(self.ctl_path), *args],
                capture_output=True, text=True, timeout=timeout, check=False,
            )
        except subprocess.TimeoutExpired:
            return False, f"ctl.py {' '.join(args)} 超时（>{timeout}s）"
        except Exception as exc:
            return False, f"调用失败：{exc}"
        message = (proc.stdout or "").strip() or (proc.stderr or "").strip()
        return proc.returncode == 0, message or f"exit={proc.returncode}"
