"""
VECTOR NAV — Mode Manager Node
==============================
Owns the NAV ↔ SLAM lifecycle. Spawns and tears down the appropriate
launch (`hw_nav.launch.py` or `hw_slam.launch.py`) as a child process,
logging every transition so `journalctl -u vector-navigation -f` is the
single place to monitor mode changes.

Services
--------
  /mode_manager/set_mode   (vector_interfaces/srv/SetMode)
  /mode_manager/get_mode   (vector_interfaces/srv/GetMode)

State
-----
  Persists current mode + map to ~/.vector_nav/state.yaml so a service
  restart resumes whatever mode was running before.
"""

import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vector_interfaces.srv import SetMode, GetMode


HOME = Path(os.path.expanduser('~'))
STATE_DIR = HOME / '.vector_nav'
STATE_FILE = STATE_DIR / 'state.yaml'
MAPS_DIR = HOME / 'vector_nav' / 'maps'
DEFAULT_MAP_STEM = 'custom_map'

NAV_PKG = 'vector_navigation'
NAV_LAUNCH = 'hw_nav.launch.py'
SLAM_LAUNCH = 'hw_slam.launch.py'


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('mode_manager_node')

        cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self._mode: str = 'nav'
        self._map: str = DEFAULT_MAP_STEM

        # Services
        self.create_service(SetMode, '/mode_manager/set_mode',
                            self._on_set_mode, callback_group=cb)
        self.create_service(GetMode, '/mode_manager/get_mode',
                            self._on_get_mode, callback_group=cb)

        # Restore state and bring up initial launch
        self._restore_state()
        self.get_logger().info(
            f'mode_manager_node ready — initial mode={self._mode}'
            + (f' map={self._map}' if self._mode == 'nav' else '')
        )
        self._spawn_current_mode()

    # ── State persistence ─────────────────────────────────────────
    def _restore_state(self):
        try:
            if STATE_FILE.exists():
                data = yaml.safe_load(STATE_FILE.read_text()) or {}
                m = data.get('mode')
                if m in ('nav', 'slam'):
                    self._mode = m
                mp = data.get('map')
                if isinstance(mp, str) and mp:
                    self._map = mp
                self.get_logger().info(f'restored state from {STATE_FILE}: mode={self._mode} map={self._map}')
                return
        except Exception as e:
            self.get_logger().warn(f'failed to read state file: {e}')
        self.get_logger().info(f'no state file — defaulting to mode={self._mode} map={self._map}')

    def _persist_state(self):
        try:
            STATE_DIR.mkdir(parents=True, exist_ok=True)
            STATE_FILE.write_text(yaml.safe_dump({'mode': self._mode, 'map': self._map}))
        except Exception as e:
            self.get_logger().warn(f'failed to write state file: {e}')

    # ── Child process management ──────────────────────────────────
    def _spawn_current_mode(self):
        if self._mode == 'slam':
            cmd = ['ros2', 'launch', NAV_PKG, SLAM_LAUNCH]
        else:
            map_yaml = str(MAPS_DIR / f'{self._map}.yaml')
            cmd = ['ros2', 'launch', NAV_PKG, NAV_LAUNCH, f'map:={map_yaml}']
        self.get_logger().info(f'spawning: {" ".join(cmd)}')
        self._proc = subprocess.Popen(
            cmd, start_new_session=True,
            stdout=None, stderr=None,  # inherit -> systemd journal
        )
        self.get_logger().info(f'child started (pid={self._proc.pid}, pgid={os.getpgid(self._proc.pid)})')

    def _kill_current(self, timeout_term: float = 8.0):
        proc = self._proc
        if proc is None or proc.poll() is not None:
            self._proc = None
            return
        try:
            pgid = os.getpgid(proc.pid)
            self.get_logger().info(f'killing child (pid={proc.pid}, pgid={pgid}) with SIGTERM')
            t0 = time.monotonic()
            os.killpg(pgid, signal.SIGTERM)
            try:
                proc.wait(timeout=timeout_term)
                self.get_logger().info(f'child exited cleanly in {time.monotonic() - t0:.1f}s')
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f'child did not exit in {timeout_term}s; escalating to SIGKILL')
                os.killpg(pgid, signal.SIGKILL)
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().error('child still alive after SIGKILL')
        except ProcessLookupError:
            self.get_logger().info('child already gone')
        except Exception as e:
            self.get_logger().error(f'error killing child: {e}')
        self._proc = None

    # ── Service handlers ──────────────────────────────────────────
    def _on_set_mode(self, req: SetMode.Request, res: SetMode.Response):
        mode = (req.mode or '').strip().lower()
        if mode not in ('nav', 'slam'):
            res.success = False
            res.message = "mode must be 'nav' or 'slam'"
            res.mode = self._mode
            res.map = self._map
            return res

        # Resolve map for nav mode
        target_map = self._map
        if mode == 'nav':
            stem = (req.map or '').strip() or self._map or DEFAULT_MAP_STEM
            map_path = MAPS_DIR / f'{stem}.yaml'
            if not map_path.exists():
                res.success = False
                res.message = f'map not found: {stem}.yaml'
                res.mode = self._mode
                res.map = self._map
                return res
            target_map = stem

        # Idempotent: same mode + same map → no-op
        with self._lock:
            if mode == self._mode:
                if mode == 'slam' or target_map == self._map:
                    if self._proc and self._proc.poll() is None:
                        self.get_logger().info(f'set_mode no-op (already {mode}'
                                               + (f' on map={target_map})' if mode == 'nav' else ')'))
                        res.success = True
                        res.message = 'already in requested mode'
                        res.mode = self._mode
                        res.map = self._map
                        return res

            self.get_logger().info(
                f'set_mode requested: {self._mode} -> {mode}'
                + (f' map={target_map}' if mode == 'nav' else '')
            )
            self._kill_current()
            # brief grace period for ROS graph to settle
            time.sleep(1.5)
            self._mode = mode
            if mode == 'nav':
                self._map = target_map
            self._persist_state()
            self._spawn_current_mode()

        res.success = True
        res.message = f'mode={self._mode}' + (f' map={self._map}' if self._mode == 'nav' else '')
        res.mode = self._mode
        res.map = self._map if self._mode == 'nav' else ''
        return res

    def _on_get_mode(self, req: GetMode.Request, res: GetMode.Response):
        res.mode = self._mode
        res.map = self._map if self._mode == 'nav' else ''
        return res

    # ── Shutdown ─────────────────────────────────────────────────
    def shutdown(self):
        self.get_logger().info('mode_manager_node shutting down — killing child')
        self._kill_current(timeout_term=5.0)


def main():
    rclpy.init()
    node = ModeManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
