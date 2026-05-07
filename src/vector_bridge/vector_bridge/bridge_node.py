"""
VECTOR BRIDGE — Robot Control Web Dashboard
============================================
FastAPI server (port 8081) that bridges the web control dashboard to ROS2.

Endpoints
---------
  GET  /                  → control.html
  GET  /static/*          → static assets
  WS   /ws/control        → real-time telemetry + teleop commands
  GET  /api/stats         → latest RobotStats snapshot (JSON)
  GET  /api/locations     → list saved locations
  POST /api/locations     → save current pose as named location
  DEL  /api/locations/{n} → delete a location
  POST /api/navigate      → navigate to a named location
  POST /api/navigate/cancel → cancel active navigation

WebSocket protocol (client → server):
  { "type": "cmd_vel", "linear": 0.3, "angular": -0.1 }
  { "type": "stop" }
  { "type": "navigate", "name": "kitchen" }
  { "type": "cancel_nav" }

WebSocket protocol (server → client, pushed ~1 Hz):
  { "type": "stats", "battery_percentage": 73.2, ... }
  { "type": "ack", "cmd": "navigate", "success": true, "message": "..." }
"""

import asyncio
import json
import logging
import math
import os
import subprocess
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from vector_interfaces.msg import RobotStats
from vector_interfaces.srv import (
    DeleteLocation, GetLocations, NavigateToLocation, SetLocation,
)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
)
logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / 'static'
CERT_DIR = Path(__file__).parent.parent.parent.parent.parent / 'src' / 'vector_web' / 'certs'


# ── Pydantic request models ───────────────────────────────────────────────────

class SaveLocationRequest(BaseModel):
    name: str


class NavigateRequest(BaseModel):
    name: str


# ── BridgeNode ────────────────────────────────────────────────────────────────

class BridgeNode(Node):

    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('vector_bridge')

        self.declare_parameter('port', 8081)
        self._port = self.get_parameter('port').get_parameter_value().integer_value

        self._loop = loop
        self._ws_clients: set[WebSocket] = set()
        self._ws_lock = threading.Lock()
        self._latest_stats: dict = {
            'type': 'stats',
            'battery_voltage': 0.0,
            'battery_percentage': 0.0,
            'navigation_status': 'Unknown',
            'current_location': 'Unknown',
            'status_message': 'Connecting...',
            'linear_velocity': 0.0,
            'angular_velocity': 0.0,
            'pose': None,
        }
        self._latest_pose: Optional[dict] = None

        cb = ReentrantCallbackGroup()

        # ── Publisher ─────────────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            RobotStats, '/robot_stats', self._on_stats, 10, callback_group=cb)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10, callback_group=cb)

        # ── Service clients ───────────────────────────────────────────────────
        self._svc_navigate = self.create_client(
            NavigateToLocation, '/navigate_to_location', callback_group=cb)
        self._svc_set_loc = self.create_client(
            SetLocation, '/set_location', callback_group=cb)
        self._svc_get_locs = self.create_client(
            GetLocations, '/get_locations', callback_group=cb)
        self._svc_del_loc = self.create_client(
            DeleteLocation, '/delete_location', callback_group=cb)

        self.get_logger().info(f'VectorBridge ready — dashboard at port {self._port}')

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _on_stats(self, msg: RobotStats) -> None:
        self._latest_stats = {
            'type': 'stats',
            'battery_voltage': round(msg.battery_voltage, 2),
            'battery_percentage': round(msg.battery_percentage, 1),
            'navigation_status': msg.navigation_status,
            'current_location': msg.current_location,
            'status_message': msg.status_message,
            'linear_velocity': round(msg.linear_velocity, 3),
            'angular_velocity': round(msg.angular_velocity, 3),
            'pose': self._latest_pose,
        }
        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._latest_stats), self._loop
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        yaw = math.atan2(
            2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
            1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2)
        )
        self._latest_pose = {
            'x': round(p.position.x, 4),
            'y': round(p.position.y, 4),
            'yaw': round(yaw, 4),
        }

    # ── WebSocket broadcast ───────────────────────────────────────────────────

    async def _broadcast(self, data: dict) -> None:
        dead: set[WebSocket] = set()
        with self._ws_lock:
            clients = set(self._ws_clients)
        for ws in clients:
            try:
                await ws.send_json(data)
            except Exception:
                dead.add(ws)
        if dead:
            with self._ws_lock:
                self._ws_clients -= dead

    async def _send_ack(self, ws: WebSocket, cmd: str, success: bool, message: str) -> None:
        try:
            await ws.send_json({'type': 'ack', 'cmd': cmd, 'success': success, 'message': message})
        except Exception:
            pass

    # ── Service call helpers (blocking, run in thread pool) ───────────────────

    def _call_navigate(self, name: str) -> tuple[bool, str]:
        if not self._svc_navigate.wait_for_service(timeout_sec=2.0):
            return False, 'Navigation service unavailable'
        req = NavigateToLocation.Request()
        req.name = name
        future = self._svc_navigate.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return False, 'Service call timed out'
        r = future.result()
        return r.success, r.message

    def _call_set_location(self, name: str) -> tuple[bool, str]:
        if not self._svc_set_loc.wait_for_service(timeout_sec=2.0):
            return False, 'Set location service unavailable'
        req = SetLocation.Request()
        req.name = name
        future = self._svc_set_loc.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return False, 'Service call timed out'
        r = future.result()
        return r.success, r.message

    def _call_get_locations(self) -> dict:
        if not self._svc_get_locs.wait_for_service(timeout_sec=2.0):
            return {'names': [], 'xs': [], 'ys': [], 'yaws': []}
        req = GetLocations.Request()
        future = self._svc_get_locs.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return {'names': [], 'xs': [], 'ys': [], 'yaws': []}
        r = future.result()
        return {
            'names': list(r.names),
            'xs': list(r.xs),
            'ys': list(r.ys),
            'yaws': list(r.yaws),
        }

    def _call_delete_location(self, name: str) -> tuple[bool, str]:
        if not self._svc_del_loc.wait_for_service(timeout_sec=2.0):
            return False, 'Delete location service unavailable'
        req = DeleteLocation.Request()
        req.name = name
        future = self._svc_del_loc.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return False, 'Service call timed out'
        r = future.result()
        return r.success, r.message

    # ── FastAPI app builder ───────────────────────────────────────────────────

    def build_app(self) -> FastAPI:
        app = FastAPI(title='Vector Bridge ROS2 Bridge')

        @app.get('/api/stats')
        async def get_stats():
            return JSONResponse(self._latest_stats)

        @app.get('/api/locations')
        async def get_locations():
            loop = asyncio.get_event_loop()
            data = await loop.run_in_executor(None, self._call_get_locations)
            # Standardize locations to a dict {name: {x, y, yaw}}
            result = {}
            for i, name in enumerate(data['names']):
                result[name] = {
                    'x': data['xs'][i],
                    'y': data['ys'][i],
                    'yaw': data['yaws'][i]
                }
            return JSONResponse(result)

        @app.post('/api/locations')
        async def save_location(body: SaveLocationRequest):
            loop = asyncio.get_event_loop()
            success, message = await loop.run_in_executor(
                None, self._call_set_location, body.name.strip())
            return JSONResponse({'success': success, 'message': message})

        @app.delete('/api/locations/{name}')
        async def delete_location(name: str):
            loop = asyncio.get_event_loop()
            success, message = await loop.run_in_executor(
                None, self._call_delete_location, name)
            return JSONResponse({'success': success, 'message': message})

        @app.post('/api/navigate')
        async def navigate(body: NavigateRequest):
            loop = asyncio.get_event_loop()
            success, message = await loop.run_in_executor(
                None, self._call_navigate, body.name.strip())
            return JSONResponse({'success': success, 'message': message})

        @app.post('/api/navigate/cancel')
        async def cancel_nav():
            # Publish zero velocity to halt the robot
            msg = Twist()
            self._cmd_vel_pub.publish(msg)
            return JSONResponse({'success': True, 'message': 'Stop command sent.'})

        @app.websocket('/ws/control')
        async def ws_control(websocket: WebSocket):
            await websocket.accept()
            with self._ws_lock:
                self._ws_clients.add(websocket)
            logger.info('Dashboard client connected')

            # Send current stats immediately on connect
            try:
                await websocket.send_json(self._latest_stats)
            except Exception:
                pass

            try:
                while True:
                    raw = await websocket.receive_text()
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue

                    msg_type = msg.get('type')

                    if msg_type == 'cmd_vel':
                        if self._latest_stats.get('navigation_status') == 'Navigating':
                            await self._send_ack(
                                websocket, 'cmd_vel', False,
                                'Teleop blocked — navigation in progress')
                        else:
                            twist = Twist()
                            twist.linear.x = float(msg.get('linear', 0.0))
                            twist.angular.z = float(msg.get('angular', 0.0))
                            self._cmd_vel_pub.publish(twist)

                    elif msg_type == 'stop':
                        self._cmd_vel_pub.publish(Twist())

                    elif msg_type == 'navigate':
                        name = msg.get('name', '').strip()
                        if name:
                            loop = asyncio.get_event_loop()
                            success, message = await loop.run_in_executor(
                                None, self._call_navigate, name)
                            await self._send_ack(websocket, 'navigate', success, message)

                    elif msg_type == 'cancel_nav':
                        self._cmd_vel_pub.publish(Twist())
                        await self._send_ack(
                            websocket, 'cancel_nav', True, 'Stop command sent.')

            except WebSocketDisconnect:
                logger.info('Dashboard client disconnected')
            finally:
                with self._ws_lock:
                    self._ws_clients.discard(websocket)

        return app


# ── SSL helpers (mirrors vector_web/app.py) ───────────────────────────────────

def _ensure_self_signed_cert() -> tuple[str, str]:
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    cert_file = CERT_DIR / 'cert.pem'
    key_file = CERT_DIR / 'key.pem'
    if cert_file.exists() and key_file.exists():
        return str(cert_file), str(key_file)
    logger.info('Generating self-signed SSL certificate...')
    subprocess.run([
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', str(key_file), '-out', str(cert_file),
        '-days', '365', '-nodes', '-subj', '/CN=vector-nav',
    ], check=True, capture_output=True)
    return str(cert_file), str(key_file)


def _free_port(port: int) -> None:
    try:
        out = subprocess.run(
            ['lsof', f'-tiTCP:{port}', '-sTCP:LISTEN'],
            capture_output=True, text=True, check=False,
        )
        own_pid = os.getpid()
        pids = [int(p) for p in out.stdout.split()
                if p.strip().isdigit() and int(p) != own_pid]
        for pid in pids:
            logger.warning(f'Port {port} held by PID {pid} — terminating')
            try:
                os.kill(pid, 15)
            except ProcessLookupError:
                continue
        if pids:
            import time
            time.sleep(1)
            for pid in pids:
                try:
                    os.kill(pid, 9)
                except ProcessLookupError:
                    pass
    except FileNotFoundError:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    import uvicorn

    rclpy.init(args=args)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    node = BridgeNode(loop)
    app = node.build_app()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    port = node._port
    _free_port(port)
    cert_file, key_file = _ensure_self_signed_cert()

    logger.info(f'Vector Bridge dashboard → https://0.0.0.0:{port}')

    try:
        uvicorn.run(
            app,
            host='0.0.0.0',
            port=port,
            ssl_keyfile=key_file,
            ssl_certfile=cert_file,
            log_level='warning',
            loop='none',
        )
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
