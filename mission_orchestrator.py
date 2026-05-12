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
    start_cmd: str
    session_name: str


class MissionOrchestrator:
    """Run SSH-based checks and pre-defined remote commands."""

    DRONES: Dict[int, DroneConfig] = {
        1: DroneConfig(
            display_name="1号机",
            ssh_target="nvidia@192.168.18.100",
            start_cmd="python3 /home/hua/双机启动/nuc.py start",
            session_name="nuc_drone",
        ),
        2: DroneConfig(
            display_name="2号机",
            ssh_target="nvidia@192.168.18.100",
            start_cmd="python3 /home/hua/双机启动/fly.py start",
            session_name="fly_drone",
        ),
    }

    SSH_CONNECT_TIMEOUT_SEC = 8
    COMMAND_TIMEOUT_SEC = 45

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
        """Create tmux sessions on each drone and start windows 1-7 automatically.

        For windows 8 and 9 we create the windows and populate the command but do not execute it;
        those are triggered by the "启动程控" and "启动探索" buttons which will send Enter.
        """
        all_ok = True
        log("[START] 基础链路启动开始 (在远端创建 tmux session 并按 1-7 启动)")

        # per-drone terminal definitions (window name and components)
        per_drone_terms = {
            'nuc_drone': [
                ("1_px4", [], "", "echo 8888 | sudo -S bash -c 'source /opt/ros/noetic/setup.bash && chmod 777 /dev/ttyACM0 && roslaunch mavros px4.launch'", True),
                ("2_livox", [], "", "roslaunch livox_ros_driver2 msg_MID360.launch", True),
                ("3_fastlio", [], "", "roslaunch fast_lio mapping_mid360.launch", True),
                ("4_offb_node", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 offb_node", True),
                ("5_ego_transfer", [], "cd ~/racer/src/scripts", "python3 ego_transfer.py", True),
                ("6_trans", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 trans.py", True),
                ("7_bridge", ["source ~/swarm_ros_bridge_ws/devel/setup.bash"], "", "roslaunch swarm_ros_bridge bridge_drone1.launch", True),
                ("8_cxr_egoctrl", ["source ~/racer/devel/setup.bash"], "", "rosrun cxr_egoctrl_v1 cxr_egoctrl_v2", False),
                ("9_exploration", ["source ~/racer/devel/setup.bash"], "", "roslaunch exploration_manager swarm_exploration_realworld.launch 2>&1 | tee '/home/nuc/文档/swarm$(date +%Y%m%d%H%M%S).log'", False),
            ],
            'fly_drone': [
                ("1_px4", [], "", "echo 8888 | sudo -S bash -c 'source /opt/ros/noetic/setup.bash && chmod 777 /dev/ttyACM0 && roslaunch mavros px4.launch'", True),
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

        for drone in self.DRONES.values():
            terms = per_drone_terms.get(drone.session_name)
            if not terms:
                log(f"[START] 未知 session 配置: {drone.session_name}")
                all_ok = False
                continue

            # Build a remote shell script that kills old session and recreates windows
            parts = []
            parts.append(f"if tmux has-session -t {shlex.quote(drone.session_name)} >/dev/null 2>&1; then tmux kill-session -t {shlex.quote(drone.session_name)}; fi")
            # create first window (new-session)
            name0, setups0, cd0, cmd0, auto0 = terms[0]
            cmd_chain0 = ' && '.join([f"{s}" for s in setups0] + ([f"cd {cd0}"] if cd0 else []) + [cmd0]) if (setups0 or cd0) else cmd0
            parts.append(f"tmux new-session -d -s {shlex.quote(drone.session_name)} -n {shlex.quote(name0)} '{cmd_chain0}'")
            # remaining windows
            for idx, (wname, setups, cd, main_cmd, auto_exec) in enumerate(terms[1:], start=1):
                # create window
                parts.append(f"tmux new-window -t {shlex.quote(drone.session_name)} -n {shlex.quote(wname)}")
                # prepare full command string with setups and cd
                full_cmd = ' && '.join([f"{s}" for s in setups] + ([f"cd {cd}"] if cd else []) + [main_cmd]) if (setups or cd) else main_cmd
                if auto_exec:
                    # send keys and press Enter to execute
                    parts.append(f"tmux send-keys -t {shlex.quote(drone.session_name)}:{idx} {shlex.quote(full_cmd)} Enter")
                else:
                    # populate the command but do not execute (no Enter)
                    parts.append(f"tmux send-keys -t {shlex.quote(drone.session_name)}:{idx} {shlex.quote(full_cmd)}")

            remote_script = '; '.join(parts)
            code, out, err = self._run_remote_command(drone, remote_script, log=log, context=f"START {drone.display_name}")
            if out:
                log(f"[START] {drone.display_name} stdout: {out[:400]}")
            if err:
                log(f"[START] {drone.display_name} stderr: {err[:400]}")

            session_ok, detail = self._check_session(drone, log=log)
            log(f"[STATUS] {drone.display_name} {detail}")
            if code != 0 or not session_ok:
                all_ok = False
                log(f"[START] {drone.display_name} 启动未完全确认，请检查远端 tmux 会话")

        if all_ok:
            log("[START] 基础链路已完成，tmux session 已重建并配置窗口 1-9（8/9 为未执行命令）")

        return all_ok

    def open_base_chain_terminals(self, log: LogFn) -> bool:
        """Open local attached terminals for both drone tmux sessions."""
        all_ok = True
        log("[ATTACH] 开始打开两台无人机的 tmux 附着窗口")
        for drone in self.DRONES.values():
            session_ok, detail = self._check_session(drone, log=log)
            log(f"[ATTACH] {drone.display_name} {detail}")
            if not session_ok:
                all_ok = False
                log(f"[ATTACH] {drone.display_name} session 不存在，请先启动基础链路")
                continue

            remote_command = f"ssh -tt {shlex.quote(drone.ssh_target)} {shlex.quote(f'tmux attach-session -t {drone.session_name}')}"
            code, _, err = self._launch_local_terminal(remote_command, log=log)
            if code != 0:
                all_ok = False
                log(f"[ATTACH] {drone.display_name} 打开失败：{err[:200]}")
            else:
                log(f"[ATTACH] {drone.display_name} 已打开附着窗口")

        return all_ok

    def trigger_manual_stage(self, stage: str, log: LogFn) -> bool:
        """Trigger manual tmux windows by sending Enter.

        stage:
          - control: window 8 (cxr_egoctrl)
          - exploration: window 9 (swarm exploration)
        """
        stage_map = {
            "control": 8,
            "exploration": 9,
        }
        window_idx = stage_map.get(stage)
        if window_idx is None:
            log(f"[TASK] unknown stage: {stage}")
            return False

        all_ok = True
        for drone in self.DRONES.values():
            session_ok, detail = self._check_session(drone, log=log)
            log(f"[TASK] {drone.display_name} {detail}")
            if not session_ok:
                all_ok = False
                log(f"[TASK] {drone.display_name} stage={stage} 失败：tmux session 不存在")
                continue

            cmd = f"tmux send-keys -t {shlex.quote(drone.session_name)}:{window_idx} Enter"
            code, out, err = self._run_remote_command(
                drone,
                cmd,
                log=log,
                context=f"TASK {drone.display_name}",
            )
            log(
                f"[TASK] {drone.display_name} stage={stage} window={window_idx} exit_code={code}"
            )
            if out:
                log(f"[TASK] {drone.display_name} stdout: {out[:200]}")
            if err:
                log(f"[TASK] {drone.display_name} stderr: {err[:200]}")

            if code != 0:
                all_ok = False
                log(f"[TASK] {drone.display_name} stage={stage} 执行失败")

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
