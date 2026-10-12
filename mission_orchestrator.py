#!/usr/bin/env python3
"""Mission orchestration for manual SSH-driven multi-UAV startup.

The front-end no longer stores SSH passwords or opens remote sessions itself.
Instead it runs the local ``ssh`` client against preconfigured host aliases
such as ``uav1``, ``uav2`` and ``uav3``. Window definitions are stored per-drone
in DroneConfig.windows, making it easy to add new drones with different commands.
1. Check basic SSH reachability and print a short status summary.
2. Dispatch the exact remote commands requested by the operator.
"""

from dataclasses import dataclass, field
import shlex
import subprocess
import time
import threading
from shutil import which
from typing import Callable, Dict, List, Optional, Tuple


LogFn = Callable[[str], None]


@dataclass(frozen=True)
class WindowDef:
    """Definition of a single tmux window inside a drone session."""

    name: str
    setups: Tuple[str, ...] = ()
    cd_cmd: str = ""
    main_cmd: str = ""
    execute: bool = True


@dataclass(frozen=True)
class DroneConfig:
    display_name: str
    ssh_target: str
    session_name: str
    windows: Tuple[WindowDef, ...] = ()


class MissionOrchestrator:
    """Run SSH-based checks and pre-defined remote commands."""

    # Shared PX4 command for nuc / fly (password-sudo).
    _PX4_CMD_NUC_FLY = (
        "echo 8888 | sudo -S bash -c "
        "'source /opt/ros/noetic/setup.bash && "
        "if [ -e /dev/ttyACM0 ]; then chmod 777 /dev/ttyACM0; fi && "
        "roslaunch mavros px4.launch'"
    )

    # PX4 command for nuc2 (passwordless sudo).
    _PX4_CMD_NUC2 = (
        "echo 8888 | sudo -S bash -c "
        "'source /opt/ros/noetic/setup.bash && "
        "if [ -e /dev/ttyACM0 ]; then chmod 777 /dev/ttyACM0; fi && "
        "roslaunch mavros px4.launch'"
    )

    # ===== 无人机配置 =====
    # 增减飞机：在此字典中添加或注释掉对应的 DroneConfig 条目即可。
    # UI (ui_app.py) 会自动根据此字典的 keys 动态生成状态卡和按钮，无需修改 UI 代码。
    # 示例：注释掉 1: DroneConfig(...) 即可移除 nuc；添加 4: DroneConfig(...) 即可新增第 4 架。
    DRONES: Dict[int, DroneConfig] = {
        1: DroneConfig(
            display_name="nuc",
            ssh_target="nuc@192.168.0.121",
            #ssh_target="nvidia@192.168.18.100",
            session_name="nuc_drone",
            windows=(
                WindowDef("px4", main_cmd=_PX4_CMD_NUC_FLY, execute=True),
                WindowDef("livox", main_cmd="roslaunch livox_ros_driver2 msg_MID360.launch", execute=True),
                WindowDef("fastlio", main_cmd="roslaunch fast_lio mapping_mid360.launch", execute=True),
                WindowDef("offb_node", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 offb_node", execute=True),
                WindowDef("ego_transfer", cd_cmd="cd ~/racer/src/scripts", main_cmd="python3 ego_transfer.py", execute=True),
                WindowDef("trans", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 trans.py", execute=True),
                WindowDef("bridge", setups=("source ~/swarm_ros_bridge_ws/devel/setup.bash",), main_cmd="roslaunch swarm_ros_bridge bridge_drone1.launch", execute=True),
                WindowDef("cxr_egoctrl", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 cxr_egoctrl_v2", execute=False),
                WindowDef("exploration", setups=("source ~/racer/devel/setup.bash",), main_cmd="roslaunch exploration_manager swarm_exploration_realworld.launch 2>&1 | tee \"/home/nuc/文档/swarm$(date +%Y%m%d%H%M%S).log\"", execute=False),
            ),
        ),
        2: DroneConfig(
            display_name="fly",
            ssh_target="fly@192.168.0.195",
            #ssh_target="nvidia@192.168.18.100",
            session_name="fly_drone",
            windows=(
                WindowDef("px4", main_cmd=_PX4_CMD_NUC_FLY, execute=True),
                WindowDef("livox", main_cmd="roslaunch livox_ros_driver2 msg_MID360.launch", execute=True),
                WindowDef("fastlio", main_cmd="roslaunch fast_lio mapping_mid360.launch", execute=True),
                WindowDef("offb_node", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 offb_node", execute=True),
                WindowDef("ego_transfer", cd_cmd="cd ~/racer/src/scripts", main_cmd="python3 ego_transfer.py", execute=True),
                WindowDef("trans", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 trans.py", execute=True),
                WindowDef("bridge", setups=("source ~/swarm_ros_bridge_ws/devel/setup.bash",), main_cmd="roslaunch swarm_ros_bridge bridge_drone2.launch", execute=True),
                WindowDef("cxr_egoctrl", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 cxr_egoctrl_v2", execute=False),
                WindowDef("exploration", setups=("source ~/racer/devel/setup.bash",), main_cmd="roslaunch exploration_manager swarm_exploration_realworld.launch 2>&1 | tee \"/home/fly/Music/swarm$(date +%Y%m%d%H%M%S).log\"", execute=False),
            ),
        ),
        3: DroneConfig(
            display_name="nuc2",
            ssh_target="nuc2@192.168.0.243",
            #ssh_target="nvidia@192.168.18.100",
            session_name="nuc2_drone",
            windows=(
                WindowDef("px4", main_cmd=_PX4_CMD_NUC2, execute=True),
                WindowDef("livox", setups=("cd ~/ws_livox", "source devel/setup.bash"), main_cmd="roslaunch livox_ros_driver2 msg_MID360s.launch", execute=True),
                WindowDef("fastlio", setups=("cd ~/fast_lio_ws", "source ~/fast_lio_ws/devel/setup.bash"), main_cmd="roslaunch fast_lio mapping_mid360s.launch", execute=True),
                WindowDef("offb_node", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 offb_node", execute=True),
                WindowDef("ego_transfer", cd_cmd="cd ~/racer/src/scripts", main_cmd="python3 ego_transfer.py", execute=True),
                WindowDef("trans", setups=("chmod +x /home/nuc2/racer/src/control/src/trans.py", "source ~/racer/devel/setup.bash"), main_cmd="rosrun cxr_egoctrl_v1 trans.py", execute=True),
                WindowDef("bridge", setups=("source ~/swarm_ros_bridge_ws/devel/setup.bash",), main_cmd="roslaunch swarm_ros_bridge bridge_drone3.launch", execute=True),
                WindowDef("cxr_egoctrl", setups=("source ~/racer/devel/setup.bash",), main_cmd="rosrun cxr_egoctrl_v1 cxr_egoctrl_v2", execute=False),
                WindowDef("exploration", setups=("source ~/racer/devel/setup.bash",), main_cmd="roslaunch exploration_manager swarm_exploration_realworld.launch 2>&1 | tee \"/home/nuc2/文档/swarm$(date +%Y%m%d%H%M%S).log\"", execute=False),
            ),
        ),
    }

    SSH_CONNECT_TIMEOUT_SEC = 8
    COMMAND_TIMEOUT_SEC = 45
    FIRST_WINDOW_DELAY_SEC = 7
    FOLLOWING_WINDOW_DELAY_SEC = 3

    def __init__(
        self,
        manual_stage_start_order: Tuple[int, ...] = (3, 2, 1),
        manual_stage_start_delay_sec: float = 1.0,
    ) -> None:
        # Only affects the manual stage buttons (control/exploration).
        self.manual_stage_start_order = manual_stage_start_order
        self.manual_stage_start_delay_sec = manual_stage_start_delay_sec

    def _format_exception(self, exc: Exception) -> str:
        return f"{type(exc).__name__}: {exc}"

    def _get_active_drones(self, drone_ids: Optional[List[int]] = None) -> List[Tuple[int, DroneConfig]]:
        """Return (drone_id, drone) pairs, optionally filtered by drone_ids."""
        if drone_ids is None:
            return list(self.DRONES.items())
        id_set = set(drone_ids)
        return [(did, d) for did, d in self.DRONES.items() if did in id_set]

    def _ssh_base_args(self, drone: DroneConfig) -> List[str]:
        return [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            f"ConnectTimeout={self.SSH_CONNECT_TIMEOUT_SEC}",
            drone.ssh_target,
        ]

    def _run_remote_command(
        self,
        drone: DroneConfig,
        remote_command: str,
        log: Optional[LogFn] = None,
        context: str = "SSH",
    ) -> Tuple[int, str, str]:
        start = time.monotonic()
        command_preview = " ".join(remote_command.split())
        if log is not None:
            log(
                f"[{context}] target={drone.ssh_target} timeout={self.COMMAND_TIMEOUT_SEC}s "
                f"cmd={command_preview[:160]}"
            )

        try:
            completed = subprocess.run(
                self._ssh_base_args(drone) + [remote_command],
                capture_output=True,
                text=True,
                timeout=self.COMMAND_TIMEOUT_SEC,
                check=False,
            )
        except FileNotFoundError as exc:
            if log is not None:
                log(f"[{context}] local ssh command not found: {self._format_exception(exc)}")
            return 127, "", str(exc)
        except subprocess.TimeoutExpired as exc:
            if log is not None:
                elapsed = time.monotonic() - start
                log(
                    f"[{context}] timeout after {elapsed:.2f}s target={drone.ssh_target} "
                    f"cmd={command_preview[:160]}"
                )
            stdout = exc.stdout.decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
            stderr = exc.stderr.decode("utf-8", errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
            return 124, stdout.strip(), stderr.strip()

        stdout = (completed.stdout or "").strip()
        stderr = (completed.stderr or "").strip()
        if log is not None:
            elapsed = time.monotonic() - start
            log(
                f"[{context}] done in {elapsed:.2f}s exit_code={completed.returncode} "
                f"stdout_len={len(stdout)} stderr_len={len(stderr)}"
            )
        return completed.returncode, stdout, stderr

    def _probe_summary(self, drone: DroneConfig) -> str:
        return (
            "printf 'user=%s\\nhost=%s\\npwd=%s\\n' "
            '"$(whoami)" "$(hostname)" "$(pwd)"; '
            f"if tmux has-session -t {shlex.quote(drone.session_name)} >/dev/null 2>&1; then "
            f"echo tmux_session={drone.session_name}:present; "
            "else echo tmux_session=missing; fi"
        )

    def _check_session(self, drone: DroneConfig, log: Optional[LogFn] = None) -> Tuple[bool, str]:
        code, out, err = self._run_remote_command(
            drone,
            f"tmux has-session -t {shlex.quote(drone.session_name)} >/dev/null 2>&1; echo $?",
            log=log,
            context=f"STATUS {drone.display_name}",
        )
        if code != 0:
            detail = err or out or "unknown"
            return False, f"tmux status check failed: {detail}"

        state = out.splitlines()[-1].strip() if out else "1"
        exists = state == "0"
        return exists, f"tmux session={drone.session_name} exists={exists}"

    def _cleanup_named_tmux_sessions(self, log: LogFn, drone_ids: Optional[List[int]] = None) -> int:
        """Best-effort cleanup for known tmux session names before startup."""
        cleared = 0
        for drone_id, drone in self._get_active_drones(drone_ids):
            remote_command = (
                f"if tmux has-session -t {shlex.quote(drone.session_name)} >/dev/null 2>&1; then "
                f"tmux kill-session -t {shlex.quote(drone.session_name)}; "
                "echo cleaned; "
                "else echo missing; fi"
            )
            code, out, err = self._run_remote_command(
                drone,
                remote_command,
                log=None,
                context=f"CLEANUP {drone.display_name}",
            )
            if code != 0:
                detail = err or out or "no output"
                log(
                    f"[TASK] {drone.display_name} 预清理失败 exit_code={code} "
                    f"detail={detail[:300]}"
                )
                continue

            state = out.splitlines()[-1].strip() if out else "missing"
            if state == "cleaned":
                cleared += 1
                log(f"[TASK] {drone.display_name} 预清理: 已清除旧会话 {drone.session_name}")
            else:
                log(f"[TASK] {drone.display_name} 预清理: 无旧会话 {drone.session_name}")

        return cleared

    def _launch_local_terminal(self, remote_command: str, log: Optional[LogFn] = None) -> Tuple[int, str, str]:
        launchers = [
            ["gnome-terminal", "--", "bash", "-lc", remote_command],
            ["konsole", "-e", "bash", "-lc", remote_command],
            ["xfce4-terminal", "--hold", "-e", f"bash -lc {shlex.quote(remote_command)}"],
            ["mate-terminal", "--", "bash", "-lc", remote_command],
            ["lxterminal", "-e", "bash", "-lc", remote_command],
            ["xterm", "-hold", "-e", "bash", "-lc", remote_command],
            ["x-terminal-emulator", "-e", "bash", "-lc", remote_command],
        ]
        for launcher in launchers:
            if which(launcher[0]):
                try:
                    subprocess.Popen(
                        launcher,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        start_new_session=True,
                    )
                    return 0, "", ""
                except FileNotFoundError as exc:
                    if log is not None:
                        log(f"[ATTACH] local terminal launcher not found: {self._format_exception(exc)}")
                    return 127, "", str(exc)
                except Exception as exc:
                    if log is not None:
                        log(f"[ATTACH] failed to launch terminal: {self._format_exception(exc)}")
                    return 1, "", str(exc)

        if log is not None:
            log("[ATTACH] no supported terminal emulator found")
        return 127, "", "no supported terminal emulator found"

    def collect_ssh_status_snapshot(self, drone_ids: Optional[List[int]] = None) -> Dict[int, Dict[str, str]]:
        """Collect lightweight SSH/session status for UI cards."""
        snapshot: Dict[int, Dict[str, str]] = {}
        for drone_id, drone in self._get_active_drones(drone_ids):
            status = {
                "display_name": drone.display_name,
                "ssh_target": drone.ssh_target,
                "reachable": "false",
                "user": "-",
                "host": "-",
                "session": "unknown",
                "detail": "",
            }

            code, out, err = self._run_remote_command(
                drone,
                self._probe_summary(drone),
                log=None,
                context=f"SSH-SNAPSHOT {drone.display_name}",
            )
            if code == 0:
                status["reachable"] = "true"
                for line in (out or "").splitlines():
                    line = line.strip()
                    if line.startswith("user="):
                        status["user"] = line.split("=", 1)[1] or "-"
                    elif line.startswith("host="):
                        status["host"] = line.split("=", 1)[1] or "-"
                    elif line.startswith("tmux_session="):
                        raw = line.split("=", 1)[1]
                        if raw.endswith(":present"):
                            status["session"] = "present"
                        elif raw == "missing":
                            status["session"] = "missing"
            else:
                status["detail"] = (err or out or "ssh_unreachable")[:300]

            snapshot[drone_id] = status

        return snapshot

    def check_ssh_connectivity(self, log: LogFn, drone_ids: Optional[List[int]] = None) -> bool:
        all_ok = True
        log("[SSH] 基础状况检查开始")
        log("[SSH] 当前使用手动 SSH 模式：请先在 VSCode 或 ~/.ssh/config 中配置好各无人机 SSH 别名")
        for drone_id, drone in self._get_active_drones(drone_ids):
            code, out, err = self._run_remote_command(
                drone,
                self._probe_summary(drone),
                log=log,
                context=f"SSH-CHECK {drone.display_name}",
            )
            log(f"[SSH] {drone.display_name} target={drone.ssh_target} exit_code={code}")
            if out:
                for line in out.splitlines():
                    log(f"[SSH] {drone.display_name} {line}")
            if err:
                log(f"[SSH] {drone.display_name} stderr: {err[:300]}")

            # 这里只把 SSH 可达性作为检查结果；tmux 状态仅用于提示信息。
            session_ok, detail = self._check_session(drone, log=log)
            log(f"[SSH] {drone.display_name} {detail}")
            if code != 0:
                all_ok = False
                log(f"[SSH] {drone.display_name} 基础状况异常，请先在终端确认能手动 ssh 到 {drone.ssh_target}")
            elif not session_ok:
                log(f"[SSH] {drone.display_name} tmux 会话暂未就绪，属于正常现象（如果还没启动基础链路）")

        return all_ok

    def auto_ssh_connect(self, log: LogFn, drone_ids: Optional[List[int]] = None) -> bool:
        """Lightweight auto-connect probe to selected drones.

        Uses ``StrictHostKeyChecking=accept-new`` so first-time connections
        are accepted without manual intervention.  Only a trivial ``echo ok``
        is executed — this is enough to warm SSH host-key cache and confirm
        reachability without the overhead of ``_probe_summary``.

        This method runs its own ``ssh`` subprocess (not through
        ``_run_remote_command``) so it can inject ``StrictHostKeyChecking``
        without affecting the standard command path.
        """
        all_ok = True
        log("[AUTO-SSH] 开始自动连接各无人机 …")
        for drone_id, drone in self._get_active_drones(drone_ids):
            args = [
                "ssh",
                "-o", "BatchMode=yes",
                "-o", "StrictHostKeyChecking=accept-new",
                "-o", f"ConnectTimeout={self.SSH_CONNECT_TIMEOUT_SEC}",
                drone.ssh_target,
                "echo ok",
            ]
            try:
                completed = subprocess.run(
                    args,
                    capture_output=True,
                    text=True,
                    timeout=self.SSH_CONNECT_TIMEOUT_SEC + 2,
                    check=False,
                )
                code = completed.returncode
                stderr = (completed.stderr or "").strip()
            except FileNotFoundError as exc:
                log(f"[AUTO-SSH] {drone.display_name} ssh 命令未找到: {exc}")
                all_ok = False
                continue
            except subprocess.TimeoutExpired as exc:
                log(f"[AUTO-SSH] {drone.display_name} 连接超时")
                all_ok = False
                continue

            if code != 0:
                all_ok = False
                detail = stderr[:200] if stderr else f"exit_code={code}"
                log(f"[AUTO-SSH] {drone.display_name} 连接失败: {detail}")
            else:
                log(f"[AUTO-SSH] {drone.display_name} 连接成功")
        log(f"[AUTO-SSH] 自动连接完成 all_ok={all_ok}")
        return all_ok

    def start_basic_chain(self, log: LogFn, drone_ids: Optional[List[int]] = None) -> bool:
        """Create tmux sessions on each drone and start windows 1-7 sequentially.

        Window 1 waits 7 seconds before window 2; all later windows wait 3 seconds.
        Windows 8 and 9 are prepared but not executed here.
        Windows definitions are read from each drone's ``windows`` tuple.
        """
        active_drones = self._get_active_drones(drone_ids)

        def build_full_command(setups, cd_cmd, main_cmd):
            parts = [*setups]
            if cd_cmd:
                parts.append(f"{cd_cmd}")
            parts.append(main_cmd)
            return " && ".join(parts)

        def create_session_command(session_name: str, term_name: str) -> str:
            return (
                f"if tmux has-session -t {shlex.quote(session_name)} >/dev/null 2>&1; then "
                f"tmux kill-session -t {shlex.quote(session_name)}; fi; "
                f"tmux new-session -d -s {shlex.quote(session_name)} -n {shlex.quote(term_name)}"
            )

        def create_window_command(session_name: str, window_name: str, command: str, execute: bool) -> str:
            target = f"{session_name}:{window_name}"
            quoted_target = shlex.quote(target)
            if execute:
                return (
                    f"tmux new-window -d -t {shlex.quote(session_name)} -n {shlex.quote(window_name)}; "
                    f"tmux send-keys -t {quoted_target} {shlex.quote(command)} Enter"
                )
            return (
                f"tmux new-window -d -t {shlex.quote(session_name)} -n {shlex.quote(window_name)}; "
                f"tmux send-keys -t {quoted_target} {shlex.quote(command)}"
            )

        log("[TASK] start_basic_chain 开始")
        log(f"[TASK] 基础链路将按 {len(active_drones)} 架无人机并行启动；单机内部窗口仍保持串行执行")

        cleared_count = self._cleanup_named_tmux_sessions(log, drone_ids)
        log(f"[TASK] 预清理完成：共清除 {cleared_count} 个同名 tmux 会话")

        results = {}
        results_lock = threading.Lock()

        def run_remote_step(drone: DroneConfig, step: str, command: str) -> bool:
            code, stdout, stderr = self._run_remote_command(
                drone,
                command,
                log=None,
                context=f"START {drone.display_name}",
            )
            if code != 0:
                detail = stderr or stdout or "no output"
                log(
                    f"[TASK] {drone.display_name} {step} 失败 exit_code={code} "
                    f"detail={detail[:300]}"
                )
                return False
            log(f"[TASK] {drone.display_name} {step} 完成")
            return True

        def run_drone_chain(drone: DroneConfig) -> bool:
            terms = drone.windows
            if not terms:
                log(
                    f"[TASK] {drone.display_name} 未找到窗口配置: "
                    f"session={drone.session_name}"
                )
                return False

            last_started_window = terms[0].name
            failures = 0
            log(f"[TASK] {drone.display_name} start_basic_chain 开始")
            log(f"[TASK] {drone.display_name} session={drone.session_name} target={drone.ssh_target}")

            first = terms[0]
            first_full = build_full_command(first.setups, first.cd_cmd, first.main_cmd)
            first_remote = create_session_command(drone.session_name, first.name)
            if not run_remote_step(drone, "创建 tmux session", first_remote):
                failures += 1
            else:
                start_cmd = f"tmux send-keys -t {shlex.quote(f'{drone.session_name}:0')} {shlex.quote(first_full)} Enter"
                log(f"[TASK] {drone.display_name} window={first.name} exec={first.execute} delay={self.FIRST_WINDOW_DELAY_SEC}s")
                if not run_remote_step(drone, f"启动窗口 {first.name}", start_cmd):
                    failures += 1

            time.sleep(self.FIRST_WINDOW_DELAY_SEC)

            for idx, w in enumerate(terms[1:7], start=1):
                last_started_window = w.name
                full_cmd = build_full_command(w.setups, w.cd_cmd, w.main_cmd)
                remote_cmd = create_window_command(drone.session_name, w.name, full_cmd, w.execute)
                delay_sec = self.FOLLOWING_WINDOW_DELAY_SEC if idx < 6 else 0
                log(f"[TASK] {drone.display_name} window={w.name} exec={w.execute} delay={delay_sec}s")
                if not run_remote_step(drone, f"准备窗口 {w.name}", remote_cmd):
                    failures += 1
                if idx < 6:
                    time.sleep(self.FOLLOWING_WINDOW_DELAY_SEC)

            for w in terms[7:9]:
                full_cmd = build_full_command(w.setups, w.cd_cmd, w.main_cmd)
                remote_cmd = create_window_command(drone.session_name, w.name, full_cmd, w.execute)
                log(f"[TASK] {drone.display_name} window={w.name} exec={w.execute} delay=0s")
                if not run_remote_step(drone, f"预置窗口 {w.name}", remote_cmd):
                    failures += 1

            session_ok, _ = self._check_session(drone, log=None)
            success = session_ok and failures == 0
            log(
                f"[TASK] {drone.display_name} start_basic_chain 结束 success={success} "
                f"window={last_started_window} failures={failures}"
            )
            return success

        def worker(drone: DroneConfig) -> None:
            result = run_drone_chain(drone)
            with results_lock:
                results[drone.session_name] = result

        threads = []
        for drone_id, drone in active_drones:
            thread = threading.Thread(target=worker, args=(drone,), daemon=True)
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        all_ok = all(results.get(drone.session_name, False) for drone_id, drone in active_drones)
        log(f"[TASK] start_basic_chain 结束 success={all_ok}")
        return all_ok

    def open_base_chain_terminals(self, log: LogFn, drone_ids: Optional[List[int]] = None) -> bool:
        """Open local attached terminals for selected drone tmux sessions."""
        all_ok = True
        log("[TASK] open_base_chain_terminals 开始")
        for drone_id, drone in self._get_active_drones(drone_ids):
            session_ok, _ = self._check_session(drone, log=None)
            if not session_ok:
                all_ok = False
                continue

            remote_command = f"ssh -tt {shlex.quote(drone.ssh_target)} {shlex.quote(f'tmux attach-session -t {drone.session_name}')}"
            code, _, _ = self._launch_local_terminal(remote_command, log=None)
            if code != 0:
                all_ok = False

        log(f"[TASK] open_base_chain_terminals 结束 success={all_ok}")
        return all_ok

    def trigger_manual_stage(self, stage: str, log: LogFn, drone_ids: Optional[List[int]] = None) -> bool:
        """Trigger manual tmux windows by sending Enter.

        stage:
          - control: window 8_cxr_egoctrl
          - exploration: window 9_exploration
        """
        stage_map = {
            "control": "cxr_egoctrl",
            "exploration": "exploration",
        }
        window_name = stage_map.get(stage)
        if window_name is None:
            log(f"[TASK] unknown stage: {stage}")
            return False

        all_ok = True
        log(f"[TASK] trigger_manual_stage({stage}) 开始")
        active_set = set(did for did, _ in self._get_active_drones(drone_ids))
        ordered_drone_ids = [drone_id for drone_id in self.manual_stage_start_order if drone_id in active_set]
        remaining_ids = [drone_id for drone_id in active_set if drone_id not in ordered_drone_ids]
        ordered_drone_ids.extend(remaining_ids)

        for index, drone_id in enumerate(ordered_drone_ids):
            drone = self.DRONES[drone_id]
            session_ok, _ = self._check_session(drone, log=None)
            if not session_ok:
                all_ok = False
            else:
                target = f"{drone.session_name}:{window_name}"
                cmd = f"tmux send-keys -t {shlex.quote(target)} Enter"
                code, _, _ = self._run_remote_command(drone, cmd, log=None, context=f"TASK {drone.display_name}")
                if code != 0:
                    all_ok = False

            if index < len(ordered_drone_ids) - 1 and self.manual_stage_start_delay_sec > 0:
                time.sleep(self.manual_stage_start_delay_sec)

        log(f"[TASK] trigger_manual_stage({stage}) 结束 success={all_ok}")
        return all_ok

    def terminate_tmux_session(self, drone_id: int, log: LogFn) -> bool:
        """Terminate one drone tmux session and all processes inside it."""
        drone = self.DRONES.get(drone_id)
        if drone is None:
            log(f"[TASK] unknown drone id: {drone_id}")
            return False

        log(f"[TASK] terminate_tmux_session({drone.display_name}) 开始")
        remote_command = (
            f"if tmux has-session -t {shlex.quote(drone.session_name)} >/dev/null 2>&1; then "
            f"tmux kill-session -t {shlex.quote(drone.session_name)} && echo result=terminated || echo result=kill_failed; "
            "else echo result=already_absent; fi; "
            f"if tmux has-session -t {shlex.quote(drone.session_name)} >/dev/null 2>&1; then "
            "echo verify=present; "
            "else echo verify=gone; fi"
        )
        code, stdout, stderr = self._run_remote_command(
            drone,
            remote_command,
            log=None,
            context=f"STOP {drone.display_name}",
        )
        if code != 0:
            detail = stderr or stdout or "no output"
            log(
                f"[TASK] {drone.display_name} 结束任务失败 exit_code={code} "
                f"detail={detail[:300]}"
            )
            return False

        result = "unknown"
        verify = "unknown"
        for line in (stdout or "").splitlines():
            line = line.strip()
            if line.startswith("result="):
                result = line.split("=", 1)[1]
            elif line.startswith("verify="):
                verify = line.split("=", 1)[1]

        if result == "terminated" and verify == "gone":
            log(f"[TASK] {drone.display_name} 结束任务结果: 已清理会话 {drone.session_name}")
            return True

        if result == "already_absent" and verify == "gone":
            log(f"[TASK] {drone.display_name} 结束任务结果: 会话原本不存在（视为已结束）")
            return True

        detail = (stderr or stdout or "no output")[:300]
        log(
            f"[TASK] {drone.display_name} 结束任务异常 result={result} verify={verify} "
            f"detail={detail}"
        )
        return False


    def launch_bridge_node_gcs(self, log: LogFn) -> bool:
        """Launch local terminal running bridge_node_gcs launch."""
        cmd = "source ~/swarm_ros_bridge_ws/devel/setup.bash && roslaunch swarm_ros_bridge bridge_node_gcs.launch"
        remote_command = f"bash -lc {shlex.quote(cmd)}"
        code, out, err = self._launch_local_terminal(remote_command, log=log)
        if code != 0:
            log(f"[LAUNCH] bridge_node_gcs 启动失败 code={code} err={err or out}")
            return False
        log("[LAUNCH] bridge_node_gcs 已在本地终端启动")
        return True

    def launch_exploration_rviz(self, log: LogFn) -> bool:
        """Launch local terminal running exploration RViz."""
        cmd = "source ~/racer/devel/setup.bash && roslaunch exploration_manager rviz.launch"
        remote_command = f"bash -lc {shlex.quote(cmd)}"
        code, out, err = self._launch_local_terminal(remote_command, log=log)
        if code != 0:
            log(f"[LAUNCH] exploration RViz 启动失败 code={code} err={err or out}")
            return False
        log("[LAUNCH] exploration RViz 已在本地终端启动")
        return True

    def run_takeoff_sequence_placeholder(self, log: LogFn) -> bool:
        """Placeholder for sequential takeoff flow logging.

        We only log phase transitions now because concrete takeoff ROS interfaces
        are not finalized in this iteration.
        """
        log("[PHASE] takeoff sequence requested")
        log("[PHASE] phase-1 drone1 precheck")
        log("[PHASE] phase-2 drone1 takeoff trigger pending implementation")
        log("[PHASE] phase-3 drone1 stable -> drone2 precheck")
        log("[PHASE] phase-4 drone2 takeoff trigger pending implementation")
        log("[PHASE] phase-5 both drones entered exploration-ready")
        return True
