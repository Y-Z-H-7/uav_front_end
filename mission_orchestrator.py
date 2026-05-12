# #!/usr/bin/env python3

# """Mission orchestration for dual-UAV startup over SSH.

# This module intentionally keeps SSH targets hardcoded for quick field testing.
# Later we can migrate these values to a settings file.
# """

# from dataclasses import dataclass
# from typing import Callable, Dict, Tuple

# import paramiko


# LogFn = Callable[[str], None]


# @dataclass
# class DroneConfig:
#     name: str
#     host: str
#     username: str
#     password: str
#     port: int
#     start_cmd: str
#     session_name: str


# class MissionOrchestrator:
#     """Handles SSH connectivity checks and remote startup actions."""

#     # TODO: replace host/username/password with real values for your drones.
#     DRONES: Dict[int, DroneConfig] = {
#         # 1: DroneConfig(
#         #     name="NUC",
#         #     host="192.168.1.101",
#         #     username="hua",
#         #     password="8888",
#         #     port=22,
#         #     start_cmd="python3 /home/hua/双机启动/nuc.py start",
#         #     session_name="nuc_drone",
#         # ),
#         # 2: DroneConfig(
#         #     name="FLY",
#         #     host="192.168.1.102",
#         #     username="hua",
#         #     password="8888",
#         #     port=22,
#         #     start_cmd="python3 /home/hua/双机启动/fly.py start",
#         #     session_name="fly_drone",
#         # ),
#         1: DroneConfig(
#             name="mu-VMware-Virtual-Platform",
#             host="192.168.6.128",
#             username="mu",
#             password="2005",
#             port=22,
#             start_cmd="python3 /home/hua/双机启动/nuc.py start",
#             session_name="nuc_drone",
#         ),
#         2: DroneConfig(
#             name="mu-VMware-Virtual-Platform",
#             host="192.168.6.131",
#             username="mu",
#             password="2005",
#             port=22,
#             start_cmd="python3 /home/hua/双机启动/fly.py start",
#             session_name="fly_drone",
#         ),
#     }

#     CONNECT_TIMEOUT_SEC = 8
#     COMMAND_TIMEOUT_SEC = 45

#     def _build_client(self, drone: DroneConfig) -> paramiko.SSHClient:
#         client = paramiko.SSHClient()
#         client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
#         client.connect(
#             hostname=drone.host,
#             port=drone.port,
#             username=drone.username,
#             password=drone.password,
#             timeout=self.CONNECT_TIMEOUT_SEC,
#         )
#         return client

#     def _run_command(self, client: paramiko.SSHClient, command: str) -> Tuple[int, str, str]:
#         stdin, stdout, stderr = client.exec_command(command, timeout=self.COMMAND_TIMEOUT_SEC)
#         _ = stdin
#         exit_code = stdout.channel.recv_exit_status()
#         out = stdout.read().decode("utf-8", errors="replace").strip()
#         err = stderr.read().decode("utf-8", errors="replace").strip()
#         return exit_code, out, err

#     def _check_session(self, client: paramiko.SSHClient, session_name: str) -> Tuple[bool, str]:
#         code, out, err = self._run_command(
#             client,
#             f"tmux has-session -t {session_name} >/dev/null 2>&1; echo $?",
#         )
#         if code != 0:
#             return False, f"status check command failed: {err or out or 'unknown'}"
#         state = out.splitlines()[-1].strip() if out else "1"
#         return state == "0", f"tmux session={session_name} exists={state == '0'}"

#     def check_ssh_connectivity(self, log: LogFn) -> bool:
#         all_ok = True
#         log("[SSH] connectivity check started")
#         for drone_id, drone in self.DRONES.items():
#             client = None
#             try:
#                 client = self._build_client(drone)
#                 log(f"[SSH] drone{drone_id} {drone.name} connected")
#             except Exception as exc:
#                 all_ok = False
#                 log(f"[SSH] drone{drone_id} {drone.name} connect failed: {exc}")
#             finally:
#                 if client is not None:
#                     client.close()
#         return all_ok

#     def start_basic_chain(self, log: LogFn) -> bool:
#         """Run the requested remote start scripts on both drones."""
#         all_ok = True
#         for drone_id, drone in self.DRONES.items():
#             client = None
#             try:
#                 log(f"[START] drone{drone_id} running: {drone.start_cmd}")
#                 client = self._build_client(drone)
#                 protected_cmd = f"timeout 35s {drone.start_cmd}"
#                 code, out, err = self._run_command(client, protected_cmd)
#                 log(f"[START] drone{drone_id} exit_code={code}")
#                 if out:
#                     log(f"[START] drone{drone_id} stdout: {out[:300]}")
#                 if err:
#                     log(f"[START] drone{drone_id} stderr: {err[:300]}")

#                 session_ok, detail = self._check_session(client, drone.session_name)
#                 log(f"[STATUS] drone{drone_id} {detail}")
#                 if not session_ok:
#                     all_ok = False
#             except Exception as exc:
#                 all_ok = False
#                 log(f"[START] drone{drone_id} failed: {exc}")
#             finally:
#                 if client is not None:
#                     client.close()
#         return all_ok

#     def trigger_manual_stage(self, stage: str, log: LogFn) -> bool:
#         """Trigger manual tmux windows by sending Enter.

#         stage:
#           - control: window 7 (cxr_egoctrl)
#           - exploration: window 9 (swarm exploration)
#         """
#         stage_map = {
#             "control": 7,
#             "exploration": 9,
#         }
#         window_idx = stage_map.get(stage)
#         if window_idx is None:
#             log(f"[TASK] unknown stage: {stage}")
#             return False

#         all_ok = True
#         for drone_id, drone in self.DRONES.items():
#             client = None
#             try:
#                 client = self._build_client(drone)
#                 cmd = f"tmux send-keys -t {drone.session_name}:{window_idx} Enter"
#                 code, out, err = self._run_command(client, cmd)
#                 log(
#                     f"[TASK] drone{drone_id} stage={stage} window={window_idx} "
#                     f"exit_code={code}"
#                 )
#                 if out:
#                     log(f"[TASK] drone{drone_id} stdout: {out[:200]}")
#                 if err:
#                     log(f"[TASK] drone{drone_id} stderr: {err[:200]}")

#                 session_ok, detail = self._check_session(client, drone.session_name)
#                 log(f"[STATUS] drone{drone_id} {detail}")
#                 if code != 0 or not session_ok:
#                     all_ok = False
#             except Exception as exc:
#                 all_ok = False
#                 log(f"[TASK] drone{drone_id} stage={stage} failed: {exc}")
#             finally:
#                 if client is not None:
#                     client.close()
#         return all_ok

#     def run_takeoff_sequence_placeholder(self, log: LogFn) -> bool:
#         """Placeholder for sequential takeoff flow logging.

#         We only log phase transitions now because concrete takeoff ROS interfaces
#         are not finalized in this iteration.
#         """
#         log("[PHASE] takeoff sequence requested")
#         log("[PHASE] phase-1 drone1 precheck")
#         log("[PHASE] phase-2 drone1 takeoff trigger pending implementation")
#         log("[PHASE] phase-3 drone1 stable -> drone2 precheck")
#         log("[PHASE] phase-4 drone2 takeoff trigger pending implementation")
#         log("[PHASE] phase-5 both drones entered exploration-ready")
#         return True

#!/usr/bin/env python3

"""Mission orchestration for dual-UAV startup over SSH.

This module intentionally keeps SSH targets hardcoded for quick field testing.
Later we can migrate these values to a settings file.
"""

from dataclasses import dataclass
import time
from typing import Callable, Dict, Optional, Tuple

import paramiko
from paramiko.ssh_exception import AuthenticationException, BadHostKeyException, SSHException


LogFn = Callable[[str], None]


@dataclass
class DroneConfig:
    name: str
    host: str
    username: str
    password: str
    port: int
    start_cmd: str
    session_name: str


class MissionOrchestrator:
    """Handles SSH connectivity checks and remote startup actions."""

    # TODO: replace host/username/password with real values for your drones.
    DRONES: Dict[int, DroneConfig] = {
        # 1: DroneConfig(
        #     name="NUC",
        #     host="192.168.1.101",
        #     username="hua",
        #     password="PLEASE_SET_PASSWORD",
        #     port=22,
        #     start_cmd="python3 /home/hua/双机启动/nuc.py start",
        #     session_name="nuc_drone",
        # ),
        # 2: DroneConfig(
        #     name="FLY",
        #     host="192.168.1.102",
        #     username="hua",
        #     password="PLEASE_SET_PASSWORD",
        #     port=22,
        #     start_cmd="python3 /home/hua/双机启动/fly.py start",
        #     session_name="fly_drone",
        # ),
        1: DroneConfig(
            name="mu-VMware-Virtual-Platform",
            host="192.168.6.128",
            username="fly",
            password="8888",
            port=22,
            start_cmd="python3 /home/hua/双机启动/nuc.py start",
            session_name="nuc_drone",
        ),
        2: DroneConfig(
            name="mu-VMware-Virtual-Platform",
            host="192.168.6.131",
            username="nuc",
            password="8888",
            port=22,
            start_cmd="python3 /home/hua/双机启动/fly.py start",
            session_name="fly_drone",
        ),
    }

    CONNECT_TIMEOUT_SEC = 8
    COMMAND_TIMEOUT_SEC = 45

    def _format_exception(self, exc: Exception) -> str:
        return f"{type(exc).__name__}: {exc}"

    def _build_client(
        self,
        drone: DroneConfig,
        log: Optional[LogFn] = None,
        context: str = "SSH",
    ) -> paramiko.SSHClient:
        start = time.monotonic()
        if log is not None:
            log(
                f"[{context}] connecting target={drone.username}@{drone.host}:{drone.port} "
                f"timeout={self.CONNECT_TIMEOUT_SEC}s"
            )

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                hostname=drone.host,
                port=drone.port,
                username=drone.username,
                password=drone.password,
                timeout=self.CONNECT_TIMEOUT_SEC,
            )
        except AuthenticationException as exc:
            if log is not None:
                elapsed = time.monotonic() - start
                log(f"[{context}] authentication failed after {elapsed:.2f}s: {self._format_exception(exc)}")
            raise
        except BadHostKeyException as exc:
            if log is not None:
                elapsed = time.monotonic() - start
                log(f"[{context}] host key check failed after {elapsed:.2f}s: {self._format_exception(exc)}")
            raise
        except SSHException as exc:
            if log is not None:
                elapsed = time.monotonic() - start
                log(f"[{context}] ssh protocol error after {elapsed:.2f}s: {self._format_exception(exc)}")
            raise
        except Exception as exc:
            if log is not None:
                elapsed = time.monotonic() - start
                log(f"[{context}] connect failed after {elapsed:.2f}s: {self._format_exception(exc)}")
            raise

        if log is not None:
            elapsed = time.monotonic() - start
            log(f"[{context}] connection established in {elapsed:.2f}s")
        return client

    def _run_command(
        self,
        client: paramiko.SSHClient,
        command: str,
        log: Optional[LogFn] = None,
        context: str = "CMD",
    ) -> Tuple[int, str, str]:
        start = time.monotonic()
        if log is not None:
            compact_cmd = " ".join(command.split())
            log(
                f"[{context}] exec start timeout={self.COMMAND_TIMEOUT_SEC}s cmd={compact_cmd[:160]}"
            )

        stdin, stdout, stderr = client.exec_command(command, timeout=self.COMMAND_TIMEOUT_SEC)
        _ = stdin
        exit_code = stdout.channel.recv_exit_status()
        out = stdout.read().decode("utf-8", errors="replace").strip()
        err = stderr.read().decode("utf-8", errors="replace").strip()
        if log is not None:
            elapsed = time.monotonic() - start
            log(
                f"[{context}] exec done in {elapsed:.2f}s exit_code={exit_code} "
                f"stdout_len={len(out)} stderr_len={len(err)}"
            )
        return exit_code, out, err

    def _check_session(
        self,
        client: paramiko.SSHClient,
        session_name: str,
        log: Optional[LogFn] = None,
        context: str = "STATUS",
    ) -> Tuple[bool, str]:
        code, out, err = self._run_command(
            client,
            f"tmux has-session -t {session_name} >/dev/null 2>&1; echo $?",
            log=log,
            context=context,
        )
        if code != 0:
            return False, f"status check command failed: {err or out or 'unknown'}"
        state = out.splitlines()[-1].strip() if out else "1"
        return state == "0", f"tmux session={session_name} exists={state == '0'}"

    def check_ssh_connectivity(self, log: LogFn) -> bool:
        all_ok = True
        log("[SSH] connectivity check started")
        for drone_id, drone in self.DRONES.items():
            client = None
            try:
                client = self._build_client(drone, log=log, context=f"SSH-CHECK drone{drone_id}")
                log(f"[SSH] drone{drone_id} {drone.name} connected")
                code, out, err = self._run_command(
                    client,
                    "echo CONNECT_OK && whoami && hostname",
                    log=log,
                    context=f"SSH-CHECK drone{drone_id}",
                )
                if code == 0:
                    probe = " | ".join(out.splitlines()[:3]) if out else ""
                    log(f"[SSH] drone{drone_id} probe ok: {probe or 'no-output'}")
                else:
                    all_ok = False
                    log(f"[SSH] drone{drone_id} probe failed: {err or out or 'unknown'}")
            except Exception as exc:
                all_ok = False
                log(
                    f"[SSH] drone{drone_id} {drone.name} connect failed: "
                    f"{self._format_exception(exc)}"
                )
            finally:
                if client is not None:
                    client.close()
        return all_ok

    def start_basic_chain(self, log: LogFn) -> bool:
        """Run the requested remote start scripts on both drones."""
        all_ok = True
        for drone_id, drone in self.DRONES.items():
            client = None
            try:
                log(f"[START] drone{drone_id} running: {drone.start_cmd}")
                client = self._build_client(drone, log=log, context=f"START drone{drone_id}")
                # Start command as requested by user. It may still return non-zero
                # in non-interactive shells because script attempts tmux attach.
                code, out, err = self._run_command(
                    client,
                    drone.start_cmd,
                    log=log,
                    context=f"START drone{drone_id}",
                )
                log(f"[START] drone{drone_id} exit_code={code}")
                if out:
                    log(f"[START] drone{drone_id} stdout: {out[:300]}")
                if err:
                    log(f"[START] drone{drone_id} stderr: {err[:300]}")

                session_ok, detail = self._check_session(
                    client,
                    drone.session_name,
                    log=log,
                    context=f"STATUS drone{drone_id}",
                )
                log(f"[STATUS] drone{drone_id} {detail}")
                if not session_ok:
                    all_ok = False
            except Exception as exc:
                all_ok = False
                log(f"[START] drone{drone_id} failed: {self._format_exception(exc)}")
            finally:
                if client is not None:
                    client.close()
        return all_ok

    def trigger_manual_stage(self, stage: str, log: LogFn) -> bool:
        """Trigger manual tmux windows by sending Enter.

        stage:
          - control: window 7 (cxr_egoctrl)
          - exploration: window 9 (swarm exploration)
        """
        stage_map = {
            "control": 7,
            "exploration": 9,
        }
        window_idx = stage_map.get(stage)
        if window_idx is None:
            log(f"[TASK] unknown stage: {stage}")
            return False

        all_ok = True
        for drone_id, drone in self.DRONES.items():
            client = None
            try:
                client = self._build_client(drone, log=log, context=f"TASK drone{drone_id}")
                cmd = f"tmux send-keys -t {drone.session_name}:{window_idx} Enter"
                code, out, err = self._run_command(
                    client,
                    cmd,
                    log=log,
                    context=f"TASK drone{drone_id}",
                )
                log(
                    f"[TASK] drone{drone_id} stage={stage} window={window_idx} "
                    f"exit_code={code}"
                )
                if out:
                    log(f"[TASK] drone{drone_id} stdout: {out[:200]}")
                if err:
                    log(f"[TASK] drone{drone_id} stderr: {err[:200]}")

                session_ok, detail = self._check_session(
                    client,
                    drone.session_name,
                    log=log,
                    context=f"STATUS drone{drone_id}",
                )
                log(f"[STATUS] drone{drone_id} {detail}")
                if code != 0 or not session_ok:
                    all_ok = False
            except Exception as exc:
                all_ok = False
                log(
                    f"[TASK] drone{drone_id} stage={stage} failed: "
                    f"{self._format_exception(exc)}"
                )
            finally:
                if client is not None:
                    client.close()
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
