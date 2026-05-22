"""
VECTOR NAV — Navigation Manager Node
=====================================
Manages named locations and drives Nav2 waypoint navigation.

Pose source  : /amcl_pose  (written to /tmp/vector_current_pose.json for web UI)
LLM bridge   : /llm/tool_call  (handles navigate_to / stop_navigation)

Services
--------
  /navigate_to_location  (vector_interfaces/srv/NavigateToLocation)
  /set_location          (vector_interfaces/srv/SetLocation)
  /get_locations         (vector_interfaces/srv/GetLocations)
  /delete_location       (vector_interfaces/srv/DeleteLocation)

Persistence
-----------
  Locations stored in config/locations.yaml relative to the workspace root.
  Override with parameter locations_file.
"""

import json
import math
import threading
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Duration  # noqa: F401 — kept for Nav2 goal stamping
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header, String

from vector_interfaces.msg import LLMToolCall, SttResult
from vector_interfaces.srv import DeleteLocation, GetLocations, NavigateToLocation, SetLocation, RenameLocation

CURRENT_POSE_FILE = Path('/tmp/vector_current_pose.json')


def _find_config(filename: str) -> Path | None:
    candidate = Path(__file__).resolve()
    for _ in range(8):
        candidate = candidate.parent
        p = candidate / 'config' / filename
        if p.exists():
            return p
    return None


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class NavManagerNode(Node):

    def __init__(self):
        super().__init__('nav_manager')

        self.declare_parameter('locations_file', '')
        loc_param = self.get_parameter('locations_file').get_parameter_value().string_value
        if loc_param:
            self._locations_path = Path(loc_param)
        else:
            found = _find_config('locations.yaml')
            self._locations_path = found or Path('/home/admin/vector_nav/config/locations.yaml')

        self._locations_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._locations: dict = self._load_locations()
        self._current_pose: tuple | None = None  # (x, y, yaw)
        self._current_cmd_vel: tuple = (0.0, 0.0)  # (linear, angular)

        cb = ReentrantCallbackGroup()

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._goal_handle = None
        self._nav_lock = threading.Lock()
        self._active_goal: tuple | None = None   # (x, y, yaw) of in-flight goal
        self._paused_goal: tuple | None = None   # cached goal across pause/resume
        self._nav_state = 'idle'

        # ── Publishers ────────────────────────────────────────────────────────
        self._stt_pub = self.create_publisher(SttResult, '/stt/text', 10)
        self._nav_state_pub = self.create_publisher(String, '/nav/state', 10)
        self._nav_loc_pub = self.create_publisher(String, '/nav/current_location', 10)
        self.create_timer(1.0, self._heartbeat, callback_group=cb)
        # Emit initial state so late subscribers see it.
        self._emit_nav_state('idle')

        # ── Subscriptions ─────────────────────────────────────────────────────
        # AMCL publishes /amcl_pose with TRANSIENT_LOCAL durability and only on
        # pose updates — match it so the last pose is delivered on startup
        # (otherwise a stationary robot leaves us with no pose forever).
        amcl_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._on_amcl_pose, amcl_qos, callback_group=cb)
        self.create_subscription(
            LLMToolCall, '/llm/tool_call',
            self._on_tool_call, 10, callback_group=cb)
        self.create_subscription(
            Twist, '/cmd_vel',
            self._on_cmd_vel, 10, callback_group=cb)

        # ── Services ──────────────────────────────────────────────────────────
        self.create_service(NavigateToLocation, '/navigate_to_location',
                            self._svc_navigate, callback_group=cb)
        self.create_service(SetLocation, '/set_location',
                            self._svc_set_location, callback_group=cb)
        self.create_service(GetLocations, '/get_locations',
                            self._svc_get_locations, callback_group=cb)
        self.create_service(DeleteLocation, '/delete_location',
                            self._svc_delete_location, callback_group=cb)
        self.create_service(RenameLocation, '/rename_location',
                            self._svc_rename_location, callback_group=cb)

        self.get_logger().info(
            f'NavManager ready  locations={self._locations_path}  '
            f'loaded={len(self._locations)} location(s)'
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._current_cmd_vel = (msg.linear.x, msg.angular.z)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = _quat_to_yaw(p.orientation)
        self._current_pose = (p.position.x, p.position.y, yaw)
        try:
            CURRENT_POSE_FILE.write_text(json.dumps({
                'x': round(p.position.x, 4),
                'y': round(p.position.y, 4),
                'yaw': round(yaw, 4),
            }))
        except Exception as e:
            self.get_logger().warn(f'pose file write failed: {e}', once=True)

    def _heartbeat(self) -> None:
        """Periodically republish nav state + nearest location for late subscribers."""
        self._nav_state_pub.publish(String(data=self._nav_state))
        self._nav_loc_pub.publish(String(data=self._nearest_location()))

    def _nearest_location(self) -> str:
        if not self._current_pose:
            return 'unknown'
        x, y, _ = self._current_pose
        nearest = 'unknown'
        min_dist = 0.5  # metres
        with self._lock:
            for name, loc in self._locations.items():
                dist = math.sqrt((x - loc['x']) ** 2 + (y - loc['y']) ** 2)
                if dist < min_dist:
                    nearest = name
                    min_dist = dist
        return nearest

    def _emit_nav_state(self, name: str) -> None:
        if name != self._nav_state:
            self.get_logger().info(f'nav state: {self._nav_state} → {name}')
        self._nav_state = name
        self._nav_state_pub.publish(String(data=name))

    def _on_tool_call(self, msg: LLMToolCall) -> None:
        try:
            args = json.loads(msg.arguments_json) if msg.arguments_json else {}
        except json.JSONDecodeError:
            args = {}

        if msg.name in ('navigate', 'navigate_to'):
            location = args.get('location', '').strip()
            if location:
                threading.Thread(
                    target=self._navigate_by_name, args=(location,), daemon=True
                ).start()
        elif msg.name == 'stop_navigation':
            self._cancel_nav()
        elif msg.name == 'pause_navigation':
            self._pause_nav()
        elif msg.name == 'resume_navigation':
            self._resume_nav()
        elif msg.name == 'get_location':
            if self._current_pose:
                x, y, yaw = self._current_pose
                self.get_logger().info(
                    f'Current pose: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°'
                )

    # ── Services ──────────────────────────────────────────────────────────────

    def _svc_navigate(self, req: NavigateToLocation.Request,
                      res: NavigateToLocation.Response) -> NavigateToLocation.Response:
        name = req.name.strip()
        loc = self._lookup(name)
        if not loc:
            res.success = False
            res.message = f"Location '{name}' not found."
            return res
        threading.Thread(target=self._navigate_by_name, args=(name,), daemon=True).start()
        res.success = True
        res.message = f"Navigating to '{name}'."
        return res

    def _svc_set_location(self, req: SetLocation.Request,
                          res: SetLocation.Response) -> SetLocation.Response:
        name = req.name.strip()
        if not name:
            res.success = False
            res.message = 'Name cannot be empty.'
            return res
        if self._current_pose is None:
            res.success = False
            res.message = 'No AMCL pose available — is the navigation stack running?'
            return res

        x, y, yaw = self._current_pose
        with self._lock:
            self._locations[name] = {
                'x': round(x, 4),
                'y': round(y, 4),
                'yaw': round(yaw, 4),
            }
            self._save_locations()

        res.success = True
        res.message = (f"Saved '{name}' at "
                       f"x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°.")
        self.get_logger().info(res.message)
        return res

    def _svc_get_locations(self, _req: GetLocations.Request,
                           res: GetLocations.Response) -> GetLocations.Response:
        with self._lock:
            locs = dict(self._locations)
        res.names = list(locs.keys())
        res.xs    = [float(v['x'])   for v in locs.values()]
        res.ys    = [float(v['y'])   for v in locs.values()]
        res.yaws  = [float(v.get('yaw', 0.0)) for v in locs.values()]
        return res

    def _svc_delete_location(self, req: DeleteLocation.Request,
                             res: DeleteLocation.Response) -> DeleteLocation.Response:
        name = req.name.strip()
        with self._lock:
            if name in self._locations:
                del self._locations[name]
                self._save_locations()
                res.success = True
                res.message = f"Deleted '{name}'."
                self.get_logger().info(res.message)
            else:
                res.success = False
                res.message = f"Location '{name}' not found."
        return res

    def _svc_rename_location(self, req: RenameLocation.Request,
                             res: RenameLocation.Response) -> RenameLocation.Response:
        old = req.old_name.strip()
        new = req.new_name.strip()
        if not new:
            res.success = False
            res.message = "New name cannot be empty."
            return res
        
        with self._lock:
            if old not in self._locations:
                res.success = False
                res.message = f"Location '{old}' not found."
                return res
            if new in self._locations:
                res.success = False
                res.message = f"Location '{new}' already exists."
                return res
            
            # Transfer coordinates
            self._locations[new] = self._locations.pop(old)
            self._save_locations()
            res.success = True
            res.message = f"Renamed '{old}' to '{new}'."
            self.get_logger().info(res.message)
        return res

    # ── Navigation ────────────────────────────────────────────────────────────

    def _lookup(self, name: str) -> dict | None:
        with self._lock:
            # Exact match first, then case-insensitive
            if name in self._locations:
                return self._locations[name]
            name_lower = name.lower()
            for k, v in self._locations.items():
                if k.lower() == name_lower:
                    return v
        return None

    def _navigate_by_name(self, name: str) -> None:
        loc = self._lookup(name)
        if not loc:
            self.get_logger().warn(f"navigate_to: unknown location '{name}'")
            return
        x   = float(loc['x'])
        y   = float(loc['y'])
        yaw = float(loc.get('yaw', 0.0))
        self.get_logger().info(
            f"Navigating to '{name}' → x={x} y={y} yaw={math.degrees(yaw):.1f}°"
        )
        self._send_goal(x, y, yaw)

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/navigate_to_pose action server not available')
            self._emit_nav_state('halted')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header = Header(frame_id='map')
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = _yaw_to_quat(yaw)

        with self._nav_lock:
            if self._goal_handle:
                try:
                    self._goal_handle.cancel_goal()
                except Exception:
                    pass
            self._active_goal = (x, y, yaw)
            future = self._nav_client.send_goal_async(goal)
            future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Navigation goal rejected by Nav2')
            with self._nav_lock:
                self._active_goal = None
            self._emit_nav_state('halted')
            return
        with self._nav_lock:
            self._goal_handle = handle
        self._emit_nav_state('navigating')
        handle.get_result_async().add_done_callback(self._on_nav_done)

    def _on_nav_done(self, future) -> None:
        status = future.result().status
        with self._nav_lock:
            self._goal_handle = None
            self._active_goal = None
        self.get_logger().info(f'Navigation complete, status={status}')
        if status == 4:  # SUCCEEDED
            self._emit_nav_state('idle')
            msg = SttResult()
            msg.text = 'Navigated to the goal successfully.'
            msg.confidence = -1.0
            self._stt_pub.publish(msg)
        else:
            # Anything other than SUCCEEDED is treated as a failure/abort,
            # *unless* the cancellation was driven by stop/pause (which already
            # emitted 'stopped' / 'paused').
            if self._nav_state not in ('stopped', 'paused'):
                self._emit_nav_state('halted')

    def _cancel_nav(self) -> None:
        with self._nav_lock:
            if self._goal_handle:
                try:
                    self._goal_handle.cancel_goal()
                    self.get_logger().info('Navigation cancelled')
                except Exception as e:
                    self.get_logger().warn(f'Cancel error: {e}')
                self._goal_handle = None
            self._active_goal = None
            self._paused_goal = None
        self._emit_nav_state('stopped')

    def _pause_nav(self) -> None:
        with self._nav_lock:
            if not self._goal_handle or not self._active_goal:
                self.get_logger().info('pause_navigation: no active goal to pause')
                return
            self._paused_goal = self._active_goal
            try:
                self._goal_handle.cancel_goal()
            except Exception as e:
                self.get_logger().warn(f'Pause cancel error: {e}')
            self._goal_handle = None
            self._active_goal = None
        self._emit_nav_state('paused')

    def _resume_nav(self) -> None:
        with self._nav_lock:
            cached = self._paused_goal
            self._paused_goal = None
        if not cached:
            self.get_logger().info('resume_navigation: nothing to resume')
            return
        self._emit_nav_state('resumed')
        # _send_goal will transition to 'navigating' once Nav2 accepts the goal.
        threading.Thread(target=self._send_goal, args=cached, daemon=True).start()

    # ── Persistence ───────────────────────────────────────────────────────────

    def _load_locations(self) -> dict:
        if not self._locations_path.exists():
            return {}
        try:
            data = yaml.safe_load(self._locations_path.read_text()) or {}
            return data.get('locations') or {}
        except Exception as e:
            self.get_logger().warn(f'Failed to load locations.yaml: {e}')
            return {}

    def _save_locations(self) -> None:
        try:
            self._locations_path.write_text(
                yaml.dump({'locations': self._locations}, default_flow_style=False)
            )
        except Exception as e:
            self.get_logger().error(f'Failed to save locations.yaml: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = NavManagerNode()
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
