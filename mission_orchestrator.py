#!/usr/bin/env python3
"""Mission orchestration for manual SSH-driven dual-UAV startup.

The front-end no longer stores SSH passwords or opens remote sessions itself.
Instead it runs the local ``ssh`` client against preconfigured host aliases
such as ``uav1`` and ``uav2``. That keeps the UI focused on two tasks only:
1. Check basic SSH reachability and print a short status summary.
2. Dispatch the exact remote commands requested by the operator.
"""

from dataclasses import dataclass
import shlex
import subprocess
import time
from shutil import which
from typing import Callable, Dict, List, Optional, Tuple


LogFn = Callable[[str], None]


@dataclass(frozen=True)
class DroneConfig:
    display_name: str
    ssh_target: str
    session_name: str


class MissionOrchestrator:
    """Run SSH-based checks and pre-defined remote commands."""

    DRONES: Dict[int, DroneConfig] = {
        # 1: DroneConfig(
        #     display_name="1号机",
        #     ssh_target="nvidia@192.168.18.100",
        #     session_name="nuc_drone",
        # ),
        # 2: DroneConfig(
        #     display_name="2号机",
        #     ssh_target="nvidia@192.168.18.100",
        #     session_name="fly_drone",
        # ),
        1: DroneConfig(
            display_name="nuc",
            ssh_target="nuc@192.168.0.121",
            session_name="nuc_drone",
        ),
        2: DroneConfig(
            display_name="fly",
            ssh_target="fly@192.168.0.195",
            session_name="fly_drone",
        ),
    }

    SSH_CONNECT_TIMEOUT_SEC = 8
    COMMAND_TIMEOUT_SEC = 45
    FIRST_WINDOW_DELAY_SEC = 7
    FOLLOWING_WINDOW_DELAY_SEC = 3

    def _format_exception(self, exc: Exception) -> str:
        return f"{type(exc).__name__}: {exc}"

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

    def check_ssh_connectivity(self, log: LogFn) -> bool:
        all_ok = True
        log("[SSH] 基础状况检查开始")
        log("[SSH] 当前使用手动 SSH 模式：请先在 VSCode 或 ~/.ssh/config 中准备好 uav1/uav2")
        for drone in self.DRONES.values():
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
    def start_basic_chain(self, log: LogFn) -> bool:
        """Create tmux sessions on each drone and start windows 1-7 sequentially.

        Window 1 waits 7 seconds before window 2; all later windows wait 3 seconds.
        Windows 8 and 9 are prepared but not executed here.
        """

        def px4_cmd() -> str:
            return (
                "echo 8888 | sudo -S bash -c "
                "'source /opt/ros/noetic/setup.bash && "
                "if [ -e /dev/ttyACM0 ]; then chmod 777 /dev/ttyACM0; fi && "
                "roslaunch mavros px4.launch'"
            )

        def build_full_command(setups, cd_cmd, main_cmd):
            parts = [*setups]
            if cd_cmd:
                parts.append(f"cd {cd_cmd}")
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

        per_drone_terms = {
            "nuc_drone": [
                ("1_px4", [], "", px4_cmd(), True),
                ("2_livox", [], "", "roslaunch livox_ros_driver2 msg_MID360.launch", True),
                ("3_fastlio", [], "", "roslaunch fast_lio mapping_mid360.launch", True),
                ("4_offb_node", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 offb_node", True),
                ("5_ego_transfer", [], "cd ~/racer/src/scripts", "python3 ego_transfer.py", True),
                ("6_trans", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 trans.py", True),
                ("7_bridge", ["source ~/swarm_ros_bridge_ws/devel/setup.bash"], "", "roslaunch swarm_ros_bridge bridge_drone1.launch", True),
                ("8_cxr_egoctrl", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 cxr_egoctrl_v2", False),
                ("9_exploration", ["source ~/racer/devel/setup.bash"], "", "roslaunch exploration_manager swarm_exploration_realworld.launch 2>&1 | tee '/home/nuc/文档/swarm$(date +%Y%m%d%H%M%S).log'", False),
            ],
            "fly_drone": [
                ("1_px4", [], "", px4_cmd(), True),
                ("2_livox", [], "", "roslaunch livox_ros_driver2 msg_MID360.launch", True),
                ("3_fastlio", [], "", "roslaunch fast_lio mapping_mid360.launch", True),
                ("4_offb_node", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 offb_node", True),
                ("5_ego_transfer", [], "cd ~/racer/src/scripts", "python3 ego_transfer.py", True),
                ("6_trans", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 trans.py", True),
                ("7_bridge", ["source ~/swarm_ros_bridge_ws/devel/setup.bash"], "", "roslaunch swarm_ros_bridge bridge_drone2.launch", True),
                ("8_cxr_egoctrl", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 cxr_egoctrl_v2", False),
                ("9_exploration", ["source ~/racer/devel/setup.bash"], "", "roslaunch exploration_manager swarm_exploration_realworld.launch 2>&1 | tee '/home/fly/Music/swarm$(date +%Y%m%d%H%M%S).log'", False),
            ],
        }

        all_ok = True
        log("[TASK] start_basic_chain 开始")
        log("[TASK] 基础链路窗口 1-7 自动执行，8-9 仅预置等待手动触发")

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

        for drone in self.DRONES.values():
            terms = per_drone_terms.get(drone.session_name)
            if not terms:
                log(
                    f"[TASK] {drone.display_name} 未找到 session 配置: "
                    f"session={drone.session_name}"
                )
                all_ok = False
                continue

            last_started_window = terms[0][0]
            failures = 0
            log(f"[TASK] {drone.display_name} start_basic_chain 开始")
            log(f"[TASK] {drone.display_name} session={drone.session_name} target={drone.ssh_target}")

            first_name, first_setups, first_cd, first_cmd, first_exec = terms[0]
            first_full = build_full_command(first_setups, first_cd, first_cmd)
            first_remote = create_session_command(drone.session_name, first_name)
            if not run_remote_step(drone, "创建 tmux session", first_remote):
                all_ok = False
                failures += 1
            else:
                start_cmd = f"tmux send-keys -t {shlex.quote(f'{drone.session_name}:0')} {shlex.quote(first_full)} Enter"
                log(f"[TASK] {drone.display_name} window={first_name} exec={first_exec} delay={self.FIRST_WINDOW_DELAY_SEC}s")
                if not run_remote_step(drone, f"启动窗口 {first_name}", start_cmd):
                    all_ok = False
                    failures += 1

            time.sleep(self.FIRST_WINDOW_DELAY_SEC)

            for idx, (window_name, setups, cd_cmd, main_cmd, execute) in enumerate(terms[1:7], start=1):
                last_started_window = window_name
                full_cmd = build_full_command(setups, cd_cmd, main_cmd)
                remote_cmd = create_window_command(drone.session_name, window_name, full_cmd, execute)
                delay_sec = self.FOLLOWING_WINDOW_DELAY_SEC if idx < 6 else 0
                log(f"[TASK] {drone.display_name} window={window_name} exec={execute} delay={delay_sec}s")
                if not run_remote_step(drone, f"准备窗口 {window_name}", remote_cmd):
                    all_ok = False
                    failures += 1
                if idx < 6:
                    time.sleep(self.FOLLOWING_WINDOW_DELAY_SEC)

            for window_name, setups, cd_cmd, main_cmd, execute in terms[7:9]:
                full_cmd = build_full_command(setups, cd_cmd, main_cmd)
                remote_cmd = create_window_command(drone.session_name, window_name, full_cmd, execute)
                log(f"[TASK] {drone.display_name} window={window_name} exec={execute} delay=0s")
                if not run_remote_step(drone, f"预置窗口 {window_name}", remote_cmd):
                    all_ok = False
                    failures += 1

            session_ok, _ = self._check_session(drone, log=None)
            all_ok = all_ok and session_ok
            log(
                f"[TASK] {drone.display_name} start_basic_chain 结束 success={session_ok} "
                f"window={last_started_window} failures={failures}"
            )

        log(f"[TASK] start_basic_chain 结束 success={all_ok}")
        return all_ok

    def open_base_chain_terminals(self, log: LogFn) -> bool:
        """Open local attached terminals for both drone tmux sessions."""
        all_ok = True
        log("[TASK] open_base_chain_terminals 开始")
        for drone in self.DRONES.values():
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

    def trigger_manual_stage(self, stage: str, log: LogFn) -> bool:
        """Trigger manual tmux windows by sending Enter.

        stage:
          - control: window 8_cxr_egoctrl
          - exploration: window 9_exploration
        """
        stage_map = {
            "control": "8_cxr_egoctrl",
            "exploration": "9_exploration",
        }
        window_name = stage_map.get(stage)
        if window_name is None:
            log(f"[TASK] unknown stage: {stage}")
            return False

        all_ok = True
        log(f"[TASK] trigger_manual_stage({stage}) 开始")
        for drone in self.DRONES.values():
            session_ok, _ = self._check_session(drone, log=None)
            if not session_ok:
                all_ok = False
                continue

            target = f"{drone.session_name}:{window_name}"
            cmd = f"tmux send-keys -t {shlex.quote(target)} Enter"
            code, _, _ = self._run_remote_command(drone, cmd, log=None, context=f"TASK {drone.display_name}")
            if code != 0:
                all_ok = False

        log(f"[TASK] trigger_manual_stage({stage}) 结束 success={all_ok}")
        return all_ok

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
