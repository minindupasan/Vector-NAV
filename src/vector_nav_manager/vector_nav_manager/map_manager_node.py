"""
VECTOR NAV — Map Manager Node
=============================
Handles saving the live SLAM map and enumerating saved maps on disk.
Saves are delegated to nav2_map_server's `map_saver_cli`.

Services
--------
  /map_manager/save_map    (vector_interfaces/srv/SaveMap)
  /map_manager/list_maps   (vector_interfaces/srv/ListMaps)
"""

import os
import re
import subprocess
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vector_interfaces.srv import SaveMap, ListMaps


MAPS_DIR = Path(os.path.expanduser('~')) / 'vector_nav' / 'maps'
STATE_FILE = Path(os.path.expanduser('~')) / '.vector_nav' / 'state.yaml'
NAME_RE = re.compile(r'^[a-zA-Z0-9_-]+$')


class MapManagerNode(Node):
    def __init__(self):
        super().__init__('map_manager_node')
        cb = ReentrantCallbackGroup()
        self.create_service(SaveMap, '/map_manager/save_map',
                            self._on_save_map, callback_group=cb)
        self.create_service(ListMaps, '/map_manager/list_maps',
                            self._on_list_maps, callback_group=cb)
        self.get_logger().info(f'map_manager_node ready — maps dir: {MAPS_DIR}')

    def _current_map(self) -> str:
        try:
            if STATE_FILE.exists():
                import yaml
                data = yaml.safe_load(STATE_FILE.read_text()) or {}
                if data.get('mode') == 'nav':
                    return str(data.get('map') or '')
        except Exception:
            pass
        return ''

    def _on_save_map(self, req: SaveMap.Request, res: SaveMap.Response):
        name = (req.name or '').strip()
        if not name or not NAME_RE.match(name):
            res.success = False
            res.message = 'invalid map name (allowed: letters, digits, _ -)'
            res.path = ''
            self.get_logger().warn(f'save_map rejected: {name!r}')
            return res

        MAPS_DIR.mkdir(parents=True, exist_ok=True)
        out_stem = str(MAPS_DIR / name)
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', out_stem]
        self.get_logger().info(f'save_map: running {" ".join(cmd)}')
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out')
            res.success = False
            res.message = 'map_saver_cli timed out (no /map topic?)'
            res.path = ''
            return res

        self.get_logger().info(f'map_saver_cli rc={result.returncode}')
        if result.stdout:
            self.get_logger().info(f'stdout: {result.stdout.strip()}')
        if result.stderr:
            self.get_logger().info(f'stderr: {result.stderr.strip()}')

        if result.returncode == 0 and Path(f'{out_stem}.yaml').exists():
            res.success = True
            res.message = f'saved {name}'
            res.path = f'{out_stem}.yaml'
            self.get_logger().info(f'map saved: {res.path}')
        else:
            res.success = False
            res.message = (result.stderr or result.stdout or 'map_saver_cli failed').strip()[:500]
            res.path = ''
        return res

    def _on_list_maps(self, req: ListMaps.Request, res: ListMaps.Response):
        try:
            if MAPS_DIR.exists():
                yamls = sorted(MAPS_DIR.glob('*.yaml'),
                               key=lambda p: p.stat().st_mtime, reverse=True)
                res.maps = [p.stem for p in yamls]
            else:
                res.maps = []
        except Exception as e:
            self.get_logger().error(f'list_maps error: {e}')
            res.maps = []
        res.current = self._current_map()
        return res


def main():
    rclpy.init()
    node = MapManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
