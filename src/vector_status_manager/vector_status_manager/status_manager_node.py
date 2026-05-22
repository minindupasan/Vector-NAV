"""
VECTOR STATUS MANAGER — Unified Robot Status Aggregator
========================================================
Subscribes to every state/telemetry topic produced across the robot and
publishes a single RobotStatus message on /robot_status at 2 Hz.

Producer topics
---------------
  /nav/state              std_msgs/String     (nav_manager)
  /nav/current_location   std_msgs/String     (nav_manager)
  /llm/state              std_msgs/String     (llm_node)
  /stt/state              std_msgs/String     (stt_node)
  /tts/speaking           std_msgs/Bool       (tts_node)
  /battery                vector_interfaces/BatteryStats
  /system_stats/pi        vector_interfaces/SystemStats
  /system_stats/jetson    vector_interfaces/SystemStats
  /amcl_pose              geometry_msgs/PoseWithCovarianceStamped
  /cmd_vel                geometry_msgs/Twist

Outputs
-------
  /robot_status                       vector_interfaces/RobotStatus  (2 Hz timer)
  /tmp/vector_robot_status.json       JSON snapshot for legacy/fallback consumers
"""

import json
import math
import threading
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Bool, String

from vector_interfaces.msg import BatteryStats, RobotStatus, SystemStats


STATUS_FILE = Path('/tmp/vector_robot_status.json')


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StatusManagerNode(Node):

    def __init__(self):
        super().__init__('status_manager_node')

        self._lock = threading.Lock()

        # Producer states — populated by subscriber callbacks.
        self._nav_state = 'unknown'
        self._tts_state = 'unknown'
        self._stt_state = 'unknown'
        self._llm_state = 'unknown'
        self._current_location = 'unknown'

        # Numeric telemetry.
        self._battery_voltage = 0.0
        self._battery_percentage = 0.0
        self._linear_velocity = 0.0
        self._angular_velocity = 0.0
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._pi_cpu = 0.0
        self._pi_temp = 0.0
        self._jetson_cpu = 0.0
        self._jetson_gpu = 0.0
        self._jetson_temp = 0.0

        cb = ReentrantCallbackGroup()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String, '/nav/state', self._on_nav_state, 10, callback_group=cb)
        self.create_subscription(String, '/nav/current_location', self._on_nav_location, 10, callback_group=cb)
        self.create_subscription(String, '/llm/state', self._on_llm_state, 10, callback_group=cb)
        self.create_subscription(String, '/stt/state', self._on_stt_state, 10, callback_group=cb)
        self.create_subscription(Bool, '/tts/speaking', self._on_tts_speaking, 10, callback_group=cb)
        # /battery and /system_stats/pi are published from the RPi with
        # TRANSIENT_LOCAL — match QoS so late startup gets the cached value.
        rpi_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(BatteryStats, '/battery', self._on_battery, rpi_qos, callback_group=cb)
        self.create_subscription(SystemStats, '/system_stats/pi', self._on_pi_stats, rpi_qos, callback_group=cb)
        self.create_subscription(SystemStats, '/system_stats/jetson', self._on_jetson_stats, 10, callback_group=cb)
        # /amcl_pose is published with TRANSIENT_LOCAL durability and only on
        # pose updates — match it so we get the latched last value at startup.
        amcl_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, amcl_qos, callback_group=cb)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10, callback_group=cb)

        # ── Publisher + timer ─────────────────────────────────────────────────
        self._status_pub = self.create_publisher(RobotStatus, '/robot_status', 10)
        self.create_timer(0.5, self._publish_status, callback_group=cb)

        self.get_logger().info('StatusManager ready  publishing /robot_status @ 2 Hz')

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _on_nav_state(self, msg: String) -> None:
        with self._lock:
            self._nav_state = msg.data.strip() or 'unknown'

    def _on_nav_location(self, msg: String) -> None:
        with self._lock:
            self._current_location = msg.data.strip() or 'unknown'

    def _on_llm_state(self, msg: String) -> None:
        with self._lock:
            self._llm_state = msg.data.strip() or 'unknown'

    def _on_stt_state(self, msg: String) -> None:
        with self._lock:
            self._stt_state = msg.data.strip() or 'unknown'

    def _on_tts_speaking(self, msg: Bool) -> None:
        with self._lock:
            self._tts_state = 'speaking' if msg.data else 'idle'

    def _on_battery(self, msg: BatteryStats) -> None:
        with self._lock:
            self._battery_voltage = float(msg.voltage)
            self._battery_percentage = float(msg.percentage)

    def _on_pi_stats(self, msg: SystemStats) -> None:
        with self._lock:
            self._pi_cpu = float(msg.cpu_usage)
            self._pi_temp = float(msg.temperature)

    def _on_jetson_stats(self, msg: SystemStats) -> None:
        with self._lock:
            self._jetson_cpu = float(msg.cpu_usage)
            self._jetson_gpu = float(msg.gpu_usage)
            self._jetson_temp = float(msg.temperature)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = _quat_to_yaw(p.orientation)
        with self._lock:
            self._pose_x = float(p.position.x)
            self._pose_y = float(p.position.y)
            self._pose_yaw = float(yaw)

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._linear_velocity = float(msg.linear.x)
            self._angular_velocity = float(msg.angular.z)

    # ── Derivation & publication ──────────────────────────────────────────────

    def _derive_system_status(self) -> tuple[str, str]:
        """Return (system_status, status_message)."""
        battery = self._battery_percentage
        temp_max = max(self._pi_temp, self._jetson_temp)
        cpu_max = max(self._pi_cpu, self._jetson_cpu)

        if (0.0 < battery < 15.0) or temp_max > 85.0:
            if 0.0 < battery < 15.0:
                return 'critical', f'Battery critical — {battery:.0f}%'
            return 'critical', f'Temperature critical — {temp_max:.0f}°C'

        if (0.0 < battery < 30.0) or temp_max > 75.0 or cpu_max > 90.0:
            if 0.0 < battery < 30.0:
                return 'warning', f'Battery low — {battery:.0f}%'
            if temp_max > 75.0:
                return 'warning', f'Temperature high — {temp_max:.0f}°C'
            return 'warning', f'CPU under heavy load — {cpu_max:.0f}%'

        return 'ok', 'Robot System Ready'

    def _publish_status(self) -> None:
        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()

        with self._lock:
            msg.navigation_status = self._nav_state
            msg.tts_status = self._tts_state
            msg.stt_status = self._stt_state
            msg.llm_status = self._llm_state
            msg.current_location = self._current_location
            msg.linear_velocity = round(self._linear_velocity, 3)
            msg.angular_velocity = round(self._angular_velocity, 3)
            msg.pose_x = round(self._pose_x, 4)
            msg.pose_y = round(self._pose_y, 4)
            msg.pose_yaw = round(self._pose_yaw, 4)
            msg.battery_voltage = round(self._battery_voltage, 2)
            msg.battery_percentage = round(self._battery_percentage, 1)
            msg.pi_cpu_percent = round(self._pi_cpu, 1)
            msg.pi_temp_c = round(self._pi_temp, 1)
            msg.jetson_cpu_percent = round(self._jetson_cpu, 1)
            msg.jetson_gpu_percent = round(self._jetson_gpu, 1)
            msg.jetson_temp_c = round(self._jetson_temp, 1)
            system_status, status_message = self._derive_system_status()

        msg.system_status = system_status
        msg.status_message = status_message
        self._status_pub.publish(msg)

        try:
            STATUS_FILE.write_text(json.dumps({
                'navigation_status': msg.navigation_status,
                'tts_status': msg.tts_status,
                'stt_status': msg.stt_status,
                'llm_status': msg.llm_status,
                'system_status': msg.system_status,
                'current_location': msg.current_location,
                'linear_velocity': msg.linear_velocity,
                'angular_velocity': msg.angular_velocity,
                'pose': {'x': msg.pose_x, 'y': msg.pose_y, 'yaw': msg.pose_yaw},
                'battery_voltage': msg.battery_voltage,
                'battery_percentage': msg.battery_percentage,
                'pi_cpu_percent': msg.pi_cpu_percent,
                'pi_temp_c': msg.pi_temp_c,
                'jetson_cpu_percent': msg.jetson_cpu_percent,
                'jetson_gpu_percent': msg.jetson_gpu_percent,
                'jetson_temp_c': msg.jetson_temp_c,
                'status_message': msg.status_message,
            }))
        except Exception as e:
            self.get_logger().warn(f'status file write failed: {e}', once=True)


def main(args=None):
    rclpy.init(args=args)
    node = StatusManagerNode()
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
