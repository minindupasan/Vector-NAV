"""
VECTOR NAV — Web Voice Assistant & ROS2 Bridge
=============================================
Unified FastAPI server on port 8080.
Handles Voice interaction, Telemetry, and Teleop Bridge.
"""

import os
import io
import base64
import ssl
import asyncio
import logging
import subprocess
import threading
import json
import math
import time
from pathlib import Path
from typing import Optional
import numpy as np
from PIL import Image

import httpx
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse, Response
from pydantic import BaseModel

import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros
from std_msgs.msg import String, Float32MultiArray, Float32, Bool
from vector_interfaces.msg import RobotStatus, SttResult
from vector_interfaces.srv import (
    DeleteLocation, GetLocations, NavigateToLocation, SetLocation, RenameLocation,
    SetMode, GetMode, SaveMap, ListMaps,
)
from nav2_msgs.srv import SetInitialPose as SetInitialPoseSrv

from .audio_utils import pcm_to_wav

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
)
logger = logging.getLogger(__name__)

# ── Configuration from environment ───────────────────────────────────────────

HOST = os.environ.get('HOST', '0.0.0.0')
PORT = int(os.environ.get('PORT', '8080'))

STATIC_DIR = Path(__file__).parent / 'static'
CERT_DIR = Path(__file__).parent.parent / 'certs'

PI_HOST      = os.environ.get('PI_HOST', '192.168.10.2')
_SSH_BASE    = ['ssh', '-i', '/home/admin/.ssh/id_ed25519',
                '-o', 'StrictHostKeyChecking=no',
                '-o', 'ConnectTimeout=5', f'admin@{PI_HOST}']
JETSON_SERVICES = ['vector-web', 'vector-llm', 'vector-assistant',
                   'vector-navigation', 'vector-status']
PI_SERVICES     = ['sllidar', 'camera', 'battery', 'shutdown', 'oled-display', 'vector-control']


async def _run_cmd(args: list, timeout: float = 10.0):
    proc = await asyncio.create_subprocess_exec(
        *args, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    try:
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout)
    except asyncio.TimeoutError:
        proc.kill()
        return -1, '', 'timeout'
    return proc.returncode, stdout.decode(errors='replace'), stderr.decode(errors='replace')


# ── Pydantic request models ───────────────────────────────────────────────────

class SaveLocationRequest(BaseModel):
    name: str

class NavigateRequest(BaseModel):
    name: str

class RenameLocationRequest(BaseModel):
    old_name: str
    new_name: str

class AudioConfigRequest(BaseModel):
    volume: Optional[float] = None
    target: Optional[str] = None  # 'robot' or 'browser'

class ModeRequest(BaseModel):
    mode: str           # 'nav' | 'slam'
    map: Optional[str] = None  # map stem for nav mode

class SaveMapRequest(BaseModel):
    name: str

class RenameMapRequest(BaseModel):
    new_name: str

class ServiceActionBody(BaseModel):
    action: str   # 'restart' | 'stop'

class PowerActionBody(BaseModel):
    action: str   # 'shutdown' | 'reboot'
    target: str   # 'jetson' | 'pi'

# ── Maps directory (read-only views still served by the web layer) ───────────

MAPS_DIR = Path(os.path.expanduser('~')) / 'vector_nav' / 'maps'


# ── ROS2 Bridge Node ─────────────────────────────────────────────────────────

class UnifiedBridgeNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('vector_web_bridge')
        self._loop = loop
        self._ws_clients: set[WebSocket] = set()
        self._voice_clients: set[WebSocket] = set()
        self._ws_lock = threading.Lock()
        
        self._speaker_volume = 3.0
        self._speaker_target = 'robot'
        
        self._latest_stats: dict = {
            'type': 'stats',
            'battery_voltage': 0.0,
            'battery_percentage': 0.0,
            'navigation_status': 'unknown',
            'tts_status': 'unknown',
            'stt_status': 'unknown',
            'llm_status': 'unknown',
            'system_status': 'unknown',
            'current_location': 'unknown',
            'status_message': 'Connecting...',
            'linear_velocity': 0.0,
            'angular_velocity': 0.0,
            'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'pi_cpu': 0.0,
            'pi_temp': 0.0,
            'jetson_cpu': 0.0,
            'jetson_gpu': 0.0,
            'jetson_temp': 0.0,
        }
        
        cb = ReentrantCallbackGroup()
        
        # Publishers
        self._stt_pub = self.create_publisher(SttResult, '/stt/text', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._volume_pub = self.create_publisher(Float32, '/tts/volume', 10)
        self._clear_pub = self.create_publisher(Bool, '/llm/clear', 10)
        self._interrupt_pub = self.create_publisher(Bool, '/llm/interrupt', 10)
        self._tts_pub = self.create_publisher(String, '/tts/input', 10)
        self._target_pub = self.create_publisher(String, '/tts/target', 10)
        self._goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribers
        self.create_subscription(RobotStatus, '/robot_status', self._on_status, 10, callback_group=cb)
        # /amcl_pose uses TRANSIENT_LOCAL — match it so we get the last pose at startup.
        amcl_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, amcl_qos, callback_group=cb)
        self.create_subscription(SttResult, '/stt/text', self._on_stt, 10, callback_group=cb)
        self.create_subscription(String, '/tts/input', self._on_tts_input, 10, callback_group=cb)
        self.create_subscription(Float32MultiArray, '/tts/audio/stream', self._on_tts_audio, 10, callback_group=cb)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10, callback_group=cb)
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos, callback_group=cb)

        # TF buffer for robot pose in map frame
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._latest_map_msg: Optional[OccupancyGrid] = None
        self._map_lock = threading.Lock()

        # Services
        self._svc_navigate = self.create_client(NavigateToLocation, '/navigate_to_location', callback_group=cb)
        self._svc_set_loc   = self.create_client(SetLocation, '/set_location', callback_group=cb)
        self._svc_get_locs  = self.create_client(GetLocations, '/get_locations', callback_group=cb)
        self._svc_del_loc   = self.create_client(DeleteLocation, '/delete_location', callback_group=cb)
        self._svc_rename_loc = self.create_client(RenameLocation, '/rename_location', callback_group=cb)
        self._svc_set_mode  = self.create_client(SetMode, '/mode_manager/set_mode', callback_group=cb)
        self._svc_get_mode  = self.create_client(GetMode, '/mode_manager/get_mode', callback_group=cb)
        self._svc_save_map  = self.create_client(SaveMap, '/map_manager/save_map', callback_group=cb)
        self._svc_list_maps = self.create_client(ListMaps, '/map_manager/list_maps', callback_group=cb)
        self._svc_set_initial_pose = self.create_client(SetInitialPoseSrv, '/set_initial_pose', callback_group=cb)

        # Teleop Watchdog
        self._target_twist = Twist()
        self._last_cmd_time = 0
        self.create_timer(0.05, self._on_teleop_timer)

    def _on_teleop_timer(self):
        # Only publish if we've had a command in the last 500ms
        if time.time() - self._last_cmd_time < 0.5:
            self._cmd_vel_pub.publish(self._target_twist)

    def set_twist(self, linear, angular):
        self._target_twist.linear.x = linear
        self._target_twist.angular.z = angular
        self._last_cmd_time = time.time()
        self._cmd_vel_pub.publish(self._target_twist)

    def stop_robot(self):
        self._target_twist = Twist()
        self._last_cmd_time = 0
        self._cmd_vel_pub.publish(self._target_twist)

    def _on_status(self, msg: RobotStatus):
        self._latest_stats.update({
            'navigation_status': msg.navigation_status,
            'tts_status': msg.tts_status,
            'stt_status': msg.stt_status,
            'llm_status': msg.llm_status,
            'system_status': msg.system_status,
            'current_location': msg.current_location,
            'status_message': msg.status_message,
            'linear_velocity': round(msg.linear_velocity, 3),
            'angular_velocity': round(msg.angular_velocity, 3),
            'battery_voltage': round(msg.battery_voltage, 2),
            'battery_percentage': round(msg.battery_percentage, 1),
            'pi_cpu': round(msg.pi_cpu_percent, 1),
            'pi_temp': round(msg.pi_temp_c, 1),
            'pi_mem': round(msg.pi_memory_usage, 1),
            'jetson_cpu': round(msg.jetson_cpu_percent, 1),
            'jetson_gpu': round(msg.jetson_gpu_percent, 1),
            'jetson_temp': round(msg.jetson_temp_c, 1),
            'jetson_mem': round(msg.jetson_memory_usage, 1),
            'jetson_mem_used_gb': round(msg.jetson_memory_used_gb, 2),
            'jetson_mem_total_gb': round(msg.jetson_memory_total_gb, 2),
        })
        asyncio.run_coroutine_threadsafe(self._broadcast(self._latest_stats), self._loop)

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        # Convert quaternion to yaw
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        self._latest_stats['pose'] = {
            'x': round(p.position.x, 3),
            'y': round(p.position.y, 3),
            'yaw': round(yaw, 3),
        }
        asyncio.run_coroutine_threadsafe(self._broadcast(self._latest_stats), self._loop)

    def _on_scan(self, msg: LaserScan):
        # Downsample/Interpolate to 360 points for the UI if needed
        # Most RPLidars provide ~360-400 points
        ranges = list(msg.ranges)
        # Replace inf/nan with 0 for JSON serialization
        clean_ranges = [r if (math.isfinite(r) and r > 0) else 0.0 for r in ranges]
        
        # If we have significantly more than 360 pts, downsample for bandwidth
        if len(clean_ranges) > 450:
            step = len(clean_ranges) // 360
            clean_ranges = clean_ranges[::step][:360]
            
        asyncio.run_coroutine_threadsafe(
            self._broadcast({'type': 'scan', 'ranges': clean_ranges}),
            self._loop
        )
        
    def _on_map(self, msg: OccupancyGrid):
        with self._map_lock:
            self._latest_map_msg = msg
        payload = self._encode_map(msg)
        if payload:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _encode_map(self, msg: OccupancyGrid) -> Optional[dict]:
        try:
            w, h = msg.info.width, msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y

            arr = np.array(msg.data, dtype=np.int8)
            rgba = np.zeros((len(arr), 4), dtype=np.uint8)
            free     = arr == 0
            occupied = arr > 0
            unknown  = ~(free | occupied)
            rgba[free]     = [230, 232, 235, 255]  # light warm white
            rgba[occupied] = [ 22,  26,  40, 255]  # dark navy
            rgba[unknown]  = [ 48,  52,  68, 255]  # muted slate, fully opaque

            grid = rgba.reshape((h, w, 4))
            grid = np.flipud(grid)

            img = Image.fromarray(grid, mode='RGBA')
            buf = io.BytesIO()
            img.save(buf, format='PNG', compress_level=1)
            b64 = base64.b64encode(buf.getvalue()).decode('ascii')

            return {
                'type': 'slam_map',
                'img': b64,
                'width': w,
                'height': h,
                'resolution': res,
                'origin_x': ox,
                'origin_y': oy,
            }
        except Exception as e:
            logger.warning(f'Map encode error: {e}')
            return None

    async def send_map_to_client(self, ws: WebSocket):
        with self._map_lock:
            msg = self._latest_map_msg
        if msg:
            payload = self._encode_map(msg)
            if payload:
                try:
                    await ws.send_json(payload)
                except Exception:
                    pass

    def _on_stt(self, msg: SttResult):
        asyncio.run_coroutine_threadsafe(
            self._broadcast_voice({'type': 'stt', 'text': msg.text, 'confidence': msg.confidence}),
            self._loop
        )

    def _on_tts_input(self, msg: String):
        text = msg.data.strip()
        if text == '[end]':
            asyncio.run_coroutine_threadsafe(
                self._broadcast_voice({'type': 'done'}),
                self._loop
            )
        elif text == '[interrupt]':
            asyncio.run_coroutine_threadsafe(
                self._broadcast_voice({'type': 'interrupt'}),
                self._loop
            )
        elif text and not text.startswith('['):
            asyncio.run_coroutine_threadsafe(
                self._broadcast_voice({'type': 'tts_chunk', 'text': text}),
                self._loop
            )

    def _on_tts_audio(self, msg: Float32MultiArray):
        if self._speaker_target == 'browser':
            pcm = np.array(msg.data, dtype=np.float32)
            wav_bytes = pcm_to_wav(pcm, sample_rate=24000)
            asyncio.run_coroutine_threadsafe(
                self._broadcast_voice_bytes(wav_bytes),
                self._loop
            )

    async def _broadcast(self, data: dict):
        with self._ws_lock:
            clients = list(self._ws_clients)
        for ws in clients:
            try: await ws.send_json(data)
            except: pass
            
    async def _broadcast_voice(self, data: dict):
        with self._ws_lock:
            clients = list(self._voice_clients)
        for ws in clients:
            try: await ws.send_json(data)
            except: pass

    async def _broadcast_voice_bytes(self, data: bytes):
        with self._ws_lock:
            clients = list(self._voice_clients)
        for ws in clients:
            try: await ws.send_bytes(data)
            except: pass

    def call_srv(self, client, req, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=2.0): return None
        future = client.call_async(req)
        import time
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        return future.result()

# ── App ──────────────────────────────────────────────────────────────────────

app = FastAPI(title='Vector Nav Unified App')
app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')

bridge: UnifiedBridgeNode | None = None

@app.on_event('startup')
async def startup():
    global bridge
    logger.info('Loading web bridge components...')
    
    loop = asyncio.get_event_loop()
    
    # Init ROS2 Bridge
    if not rclpy.ok(): rclpy.init()
    bridge = UnifiedBridgeNode(loop)
    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()
    
    logger.info('Web bridge ready')

CAMERA_WEBRTC_URL = os.environ.get('CAMERA_WEBRTC_URL', 'https://localhost:8443')

@app.get('/')
async def index():
    return FileResponse(str(STATIC_DIR / 'index.html'))

@app.post('/camera/offer')
async def camera_offer(request: Request):
    body = await request.body()
    async with httpx.AsyncClient(verify=False) as client:
        try:
            resp = await client.post(
                f'{CAMERA_WEBRTC_URL}/offer',
                content=body,
                headers={'Content-Type': 'application/json'},
                timeout=10.0,
            )
            return Response(content=resp.content, media_type='application/json', status_code=resp.status_code)
        except httpx.ConnectError:
            return JSONResponse({'error': 'camera service unavailable'}, status_code=503)

# ── API Endpoints (Proxied to ROS2) ──────────────────────────────────────────

@app.get('/api/locations')
async def get_locations():
    req = GetLocations.Request()
    res = bridge.call_srv(bridge._svc_get_locs, req)
    if not res: return JSONResponse({})
    out = {}
    for i, name in enumerate(res.names):
        out[name] = {'x': res.xs[i], 'y': res.ys[i], 'yaw': res.yaws[i]}
    return JSONResponse(out)

@app.post('/api/locations')
async def save_location(body: SaveLocationRequest):
    req = SetLocation.Request()
    req.name = body.name
    res = bridge.call_srv(bridge._svc_set_loc, req)
    return JSONResponse({'success': res.success if res else False})

@app.delete('/api/locations/{name}')
async def delete_location(name: str):
    req = DeleteLocation.Request()
    req.name = name
    res = bridge.call_srv(bridge._svc_del_loc, req)
    return JSONResponse({'success': res.success if res else False})

@app.put('/api/locations')
async def rename_location(body: RenameLocationRequest):
    req = RenameLocation.Request()
    req.old_name = body.old_name
    req.new_name = body.new_name
    res = bridge.call_srv(bridge._svc_rename_loc, req)
    return JSONResponse({'success': res.success if res else False})

@app.post('/api/set_pose')
async def set_pose_raw(request: Request):
    """RViz-style pose estimation. Accepts {x, y, yaw} in map frame."""
    try:
        body = await request.json()
        x = float(body['x'])
        y = float(body['y'])
        yaw = float(body['yaw'])
    except (KeyError, ValueError, TypeError):
        return JSONResponse({'success': False, 'error': 'expected {x, y, yaw} numbers'}, status_code=400)

    half = yaw / 2.0
    qz = math.sin(half)
    qw = math.cos(half)

    req = SetInitialPoseSrv.Request()
    req.pose.header.frame_id = 'map'
    req.pose.pose.pose.position.x = x
    req.pose.pose.pose.position.y = y
    req.pose.pose.pose.position.z = 0.0
    req.pose.pose.pose.orientation.x = 0.0
    req.pose.pose.pose.orientation.y = 0.0
    req.pose.pose.pose.orientation.z = qz
    req.pose.pose.pose.orientation.w = qw
    cov = [0.0] * 36
    cov[0]  = 0.25
    cov[7]  = 0.25
    cov[35] = 0.0685
    req.pose.pose.covariance = cov

    loop = asyncio.get_event_loop()
    res = await loop.run_in_executor(None, lambda: bridge.call_srv(bridge._svc_set_initial_pose, req, timeout_sec=5.0))
    if res is None:
        return JSONResponse({'success': False, 'error': 'set_initial_pose service unavailable'}, status_code=503)
    logger.info(f'Set initial pose (manual) x={x:.3f} y={y:.3f} yaw={yaw:.3f}')

    bridge._latest_stats['pose'] = {'x': x, 'y': y, 'yaw': yaw}
    asyncio.run_coroutine_threadsafe(bridge._broadcast(bridge._latest_stats), bridge._loop)
    return JSONResponse({'success': True})

@app.post('/api/set_goal')
async def set_goal(request: Request):
    """RViz-style 2D goal. Accepts {x, y, yaw} in map frame and publishes to /goal_pose."""
    try:
        body = await request.json()
        x = float(body['x'])
        y = float(body['y'])
        yaw = float(body['yaw'])
    except (KeyError, ValueError, TypeError):
        return JSONResponse({'success': False, 'error': 'expected {x, y, yaw} numbers'}, status_code=400)

    half = yaw / 2.0
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = bridge.get_clock().now().to_msg()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)
    bridge._goal_pose_pub.publish(msg)
    logger.info(f'Published goal pose x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
    return JSONResponse({'success': True})

@app.post('/api/locations/{name}/set_pose')
async def set_pose_from_location(name: str):
    """Tell AMCL the robot is at the saved location (pose estimation / relocalization)."""
    locs_req = GetLocations.Request()
    locs_res = bridge.call_srv(bridge._svc_get_locs, locs_req)
    if not locs_res or name not in locs_res.names:
        return JSONResponse({'success': False, 'error': f"Location '{name}' not found"}, status_code=404)

    idx = list(locs_res.names).index(name)
    x   = locs_res.xs[idx]
    y   = locs_res.ys[idx]
    yaw = locs_res.yaws[idx]

    half = yaw / 2.0
    qz = math.sin(half)
    qw = math.cos(half)

    req = SetInitialPoseSrv.Request()
    req.pose.header.frame_id = 'map'
    req.pose.pose.pose.position.x = x
    req.pose.pose.pose.position.y = y
    req.pose.pose.pose.position.z = 0.0
    req.pose.pose.pose.orientation.x = 0.0
    req.pose.pose.pose.orientation.y = 0.0
    req.pose.pose.pose.orientation.z = qz
    req.pose.pose.pose.orientation.w = qw
    # Moderate covariance — AMCL will refine from scan
    cov = [0.0] * 36
    cov[0]  = 0.25   # x
    cov[7]  = 0.25   # y
    cov[35] = 0.0685 # yaw (~15 deg std)
    req.pose.pose.covariance = cov

    loop = asyncio.get_event_loop()
    res = await loop.run_in_executor(None, lambda: bridge.call_srv(bridge._svc_set_initial_pose, req, timeout_sec=5.0))
    if res is None:
        return JSONResponse({'success': False, 'error': 'set_initial_pose service unavailable'}, status_code=503)
    logger.info(f'Set initial pose to location "{name}" x={x:.3f} y={y:.3f} yaw={yaw:.3f}')
    
    bridge._latest_stats['pose'] = {'x': float(x), 'y': float(y), 'yaw': float(yaw)}
    asyncio.run_coroutine_threadsafe(bridge._broadcast(bridge._latest_stats), bridge._loop)

    return JSONResponse({'success': True})

@app.post('/api/navigate')
async def navigate(body: NavigateRequest):
    req = NavigateToLocation.Request()
    req.name = body.name
    res = bridge.call_srv(bridge._svc_navigate, req)
    return JSONResponse({'success': res.success if res else False})

@app.get('/api/audio')
async def get_audio_config():
    return JSONResponse({
        'volume': (bridge._speaker_volume / 5.0) if bridge else 1.0,
        'target': bridge._speaker_target if bridge else 'robot',
    })

@app.post('/api/audio')
async def set_audio_config(body: AudioConfigRequest):
    if bridge is None:
        return JSONResponse({'success': False, 'message': 'bridge not ready'})
    if body.volume is not None:
        # Scale by 5.0 so that 50% on the web slider (0.5) maps to a 2.5x physical volume multiplier
        scaled_volume = float(body.volume) * 5.0
        bridge._speaker_volume = max(0.0, min(12.5, scaled_volume))
        msg = Float32()
        msg.data = float(bridge._speaker_volume)
        bridge._volume_pub.publish(msg)
    if body.target is not None:
        if body.target in ('robot', 'browser'):
            bridge._speaker_target = body.target
            msg = String()
            msg.data = bridge._speaker_target
            bridge._target_pub.publish(msg)
    return JSONResponse({
        'success': True,
        'volume': bridge._speaker_volume,
        'target': bridge._speaker_target,
    })

@app.get('/api/mode')
async def get_mode():
    if bridge is None:
        return JSONResponse({'mode': 'unknown', 'map': None, 'maps': []})
    loop = asyncio.get_event_loop()
    mode_res = await loop.run_in_executor(None, lambda: bridge.call_srv(bridge._svc_get_mode, GetMode.Request()))
    list_res = await loop.run_in_executor(None, lambda: bridge.call_srv(bridge._svc_list_maps, ListMaps.Request()))
    mode = mode_res.mode if mode_res else 'unknown'
    cur_map = (mode_res.map or None) if mode_res else None
    maps = list(list_res.maps) if list_res else []
    return JSONResponse({'mode': mode, 'map': cur_map, 'maps': maps})

@app.post('/api/mode')
async def set_mode(body: ModeRequest):
    if body.mode not in ('nav', 'slam'):
        return JSONResponse({'success': False, 'error': 'mode must be nav or slam'}, status_code=400)
    if bridge is None:
        return JSONResponse({'success': False, 'error': 'bridge not ready'}, status_code=503)

    req = SetMode.Request()
    req.mode = body.mode
    req.map = body.map or ''
    loop = asyncio.get_event_loop()
    res = await loop.run_in_executor(None, lambda: bridge.call_srv(bridge._svc_set_mode, req, timeout_sec=60.0))
    if res is None:
        return JSONResponse({'success': False, 'error': 'mode_manager unavailable'}, status_code=503)
    if not res.success:
        return JSONResponse({'success': False, 'error': res.message}, status_code=400)
    logger.info(f'Mode → {res.mode}' + (f' map={res.map}' if res.map else ''))
    # Clear stale cached map so clients show "waiting" until the new map publishes
    with bridge._map_lock:
        bridge._latest_map_msg = None
    return JSONResponse({'success': True, 'mode': res.mode, 'map': (res.map or None)})

@app.get('/api/maps/{name}/image')
async def map_image(name: str):
    if '/' in name or '..' in name:
        return JSONResponse({'error': 'invalid'}, status_code=400)
    pgm_path = MAPS_DIR / f'{name}.pgm'
    if not pgm_path.exists():
        return JSONResponse({'error': 'not found'}, status_code=404)
    try:
        img = Image.open(str(pgm_path)).convert('L')
        buf = io.BytesIO()
        img.save(buf, format='PNG', compress_level=1)
        return Response(content=buf.getvalue(), media_type='image/png')
    except Exception as e:
        return JSONResponse({'error': str(e)}, status_code=500)

@app.get('/api/maps')
async def list_maps():
    if bridge is None:
        return JSONResponse({'maps': []})
    loop = asyncio.get_event_loop()
    res = await loop.run_in_executor(None, lambda: bridge.call_srv(bridge._svc_list_maps, ListMaps.Request()))
    if res is None:
        return JSONResponse({'maps': []})
    return JSONResponse({'maps': [{'name': n} for n in res.maps]})

@app.post('/api/maps/save')
async def save_map(body: SaveMapRequest):
    if bridge is None:
        return JSONResponse({'success': False, 'error': 'bridge not ready'}, status_code=503)
    req = SaveMap.Request()
    req.name = body.name.strip()
    loop = asyncio.get_event_loop()
    res = await loop.run_in_executor(None, lambda: bridge.call_srv(bridge._svc_save_map, req, timeout_sec=30.0))
    if res is None:
        return JSONResponse({'success': False, 'error': 'map_manager unavailable'}, status_code=503)
    if not res.success:
        logger.error(f'save_map failed: {res.message}')
        return JSONResponse({'success': False, 'error': res.message}, status_code=500)
    logger.info(f'Map saved: {res.path}')
    return JSONResponse({'success': True, 'name': req.name, 'path': res.path})

@app.delete('/api/maps/{name}')
async def delete_map(name: str):
    if '/' in name or '..' in name:
        return JSONResponse({'error': 'invalid name'}, status_code=400)
    deleted = []
    for ext in ('.pgm', '.yaml'):
        p = MAPS_DIR / f'{name}{ext}'
        if p.exists():
            p.unlink()
            deleted.append(str(p))
    if not deleted:
        return JSONResponse({'error': 'not found'}, status_code=404)
    logger.info(f'Deleted map: {name}')
    return JSONResponse({'success': True, 'name': name})

@app.patch('/api/maps/{name}/rename')
async def rename_map(name: str, body: RenameMapRequest):
    new_name = body.new_name.strip()
    if '/' in name or '..' in name or '/' in new_name or '..' in new_name or not new_name:
        return JSONResponse({'error': 'invalid name'}, status_code=400)
    if not (MAPS_DIR / f'{name}.pgm').exists():
        return JSONResponse({'error': 'not found'}, status_code=404)
    if (MAPS_DIR / f'{new_name}.pgm').exists():
        return JSONResponse({'error': 'name already exists'}, status_code=409)
    for ext in ('.pgm', '.yaml'):
        src = MAPS_DIR / f'{name}{ext}'
        dst = MAPS_DIR / f'{new_name}{ext}'
        if src.exists():
            src.rename(dst)
    logger.info(f'Renamed map: {name} -> {new_name}')
    return JSONResponse({'success': True, 'old_name': name, 'new_name': new_name})

@app.post('/api/navigate/cancel')
async def cancel_nav():
    bridge._cmd_vel_pub.publish(Twist())
    return JSONResponse({'success': True})

# ── System management ─────────────────────────────────────────────────────────

@app.get('/api/system/services')
async def get_system_services():
    async def _status(args: list, name: str) -> dict:
        rc, out, _ = await _run_cmd(args + [
            'systemctl', 'show', name, '--no-pager',
            '--property=ActiveState,SubState'], timeout=6.0)
        props = {}
        for line in out.splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                props[k] = v
        return {'name': name,
                'active_state': props.get('ActiveState', 'unknown'),
                'sub_state':    props.get('SubState', '')}

    jetson_results, pi_results = await asyncio.gather(
        asyncio.gather(*[_status([], s) for s in JETSON_SERVICES]),
        asyncio.gather(*[_status(_SSH_BASE, s) for s in PI_SERVICES]),
    )
    return JSONResponse({'jetson': list(jetson_results), 'pi': list(pi_results)})


@app.get('/api/system/services/{target}/{name}/logs')
async def get_service_logs(target: str, name: str):
    if target == 'jetson':
        if name not in JETSON_SERVICES:
            return JSONResponse({'error': 'unknown service'}, status_code=400)
        rc, out, err = await _run_cmd(
            ['journalctl', '-u', name, '-n', '60', '--no-pager', '--output=short-iso'],
            timeout=8.0)
    elif target == 'pi':
        if name not in PI_SERVICES:
            return JSONResponse({'error': 'unknown service'}, status_code=400)
        rc, out, err = await _run_cmd(
            _SSH_BASE + ['journalctl', '-u', name, '-n', '60',
                         '--no-pager', '--output=short-iso'],
            timeout=14.0)
    else:
        return JSONResponse({'error': 'invalid target'}, status_code=400)
    return JSONResponse({'lines': out if out else err, 'ok': rc == 0})


@app.post('/api/system/services/{target}/{name}/action')
async def service_action(target: str, name: str, body: ServiceActionBody):
    if body.action not in ('restart', 'stop'):
        return JSONResponse({'error': 'invalid action'}, status_code=400)
    if target == 'jetson':
        if name not in JETSON_SERVICES:
            return JSONResponse({'error': 'unknown service'}, status_code=400)
        rc, _, err = await _run_cmd(
            ['sudo', 'systemctl', body.action, name], timeout=15.0)
    elif target == 'pi':
        if name not in PI_SERVICES:
            return JSONResponse({'error': 'unknown service'}, status_code=400)
        rc, _, err = await _run_cmd(
            _SSH_BASE + ['sudo', 'systemctl', body.action, name], timeout=20.0)
    else:
        return JSONResponse({'error': 'invalid target'}, status_code=400)
    return JSONResponse({'success': rc == 0, 'error': err if rc != 0 else None})


@app.post('/api/system/power')
async def system_power(body: PowerActionBody):
    if body.action not in ('shutdown', 'reboot'):
        return JSONResponse({'error': 'invalid action'}, status_code=400)
    cmd = 'poweroff' if body.action == 'shutdown' else 'reboot'
    if body.target == 'jetson':
        asyncio.create_task(_run_cmd(['sudo', cmd], timeout=5.0))
    elif body.target == 'pi':
        asyncio.create_task(_run_cmd(_SSH_BASE + ['sudo', cmd], timeout=8.0))
    else:
        return JSONResponse({'error': 'invalid target'}, status_code=400)
    return JSONResponse({'success': True})

# ── WebSockets ────────────────────────────────────────────────────────────────

@app.websocket('/ws/voice')
async def voice_ws(websocket: WebSocket):
    await websocket.accept()
    with bridge._ws_lock: bridge._voice_clients.add(websocket)
    try:
        while True:
            msg = await websocket.receive()
            if msg['type'] == 'websocket.disconnect':
                break

            if 'text' in msg and msg['text'] is not None:
                payload = json.loads(msg['text'])
                mtype = payload.get('type')
                if mtype == 'text':
                    text = (payload.get('text') or '').strip()
                    if text:
                        res = SttResult()
                        res.text = text
                        res.confidence = 1.0
                        bridge._stt_pub.publish(res)
                elif mtype == 'clear_chat':
                    logger.info("RECEIVED clear_chat command from browser")
                    bridge._clear_pub.publish(Bool(data=True))
                elif mtype == 'interrupt':
                    logger.info("Voice: Interrupting response")
                    bridge._interrupt_pub.publish(Bool(data=True))
                    bridge._tts_pub.publish(String(data='[interrupt]'))
    except WebSocketDisconnect: pass
    finally:
        with bridge._ws_lock: bridge._voice_clients.discard(websocket)

@app.websocket('/ws/control')
async def control_ws(websocket: WebSocket):
    await websocket.accept()
    with bridge._ws_lock: bridge._ws_clients.add(websocket)
    try:
        await websocket.send_json(bridge._latest_stats)
        await bridge.send_map_to_client(websocket)
        while True:
            raw = await websocket.receive_text()
            msg = json.loads(raw)
            mtype = msg.get('type')
            if mtype == 'cmd_vel':
                lin = float(msg.get('linear', 0.0))
                ang = float(msg.get('angular', 0.0))
                if abs(lin) > 0.001 or abs(ang) > 0.001:
                    logger.debug(f"Teleop: lin={lin:.3f}, ang={ang:.3f}")
                bridge.set_twist(lin, ang)
            elif mtype == 'stop':
                logger.debug("Teleop: STOP")
                bridge.stop_robot()
            elif mtype == 'get_map':
                await bridge.send_map_to_client(websocket)
    except WebSocketDisconnect: pass
    finally:
        with bridge._ws_lock: bridge._ws_clients.discard(websocket)

# ── Utils & Entry ───────────────────────────────────────────────────────────

def _ensure_self_signed_cert():
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    c, k = CERT_DIR / 'cert.pem', CERT_DIR / 'key.pem'
    if not (c.exists() and k.exists()):
        subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-keyout', str(k), '-out', str(c), '-days', '365', '-nodes', '-subj', '/CN=vector-nav'], check=True, capture_output=True)
    return str(c), str(k)

def _free_port(port: int):
    try:
        out = subprocess.run(['lsof', '-tiTCP:%d' % port, '-sTCP:LISTEN'], capture_output=True, text=True)
        for pid in out.stdout.split():
            if int(pid) != os.getpid(): os.kill(int(pid), 9)
    except: pass

def main():
    import uvicorn
    _free_port(PORT)
    c, k = _ensure_self_signed_cert()
    uvicorn.run('vector_web.app:app', host=HOST, port=PORT, ssl_keyfile=k, ssl_certfile=c, log_level='info')

if __name__ == '__main__': main()
