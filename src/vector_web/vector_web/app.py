"""
VECTOR NAV — Web Voice Assistant & ROS2 Bridge
=============================================
Unified FastAPI server on port 8080.
Handles Voice interaction, Telemetry, and Teleop Bridge.
"""

import os
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

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray, Float32, Bool
from vector_interfaces.msg import RobotStats, SttResult, SystemStats, BatteryStats
from vector_interfaces.srv import (
    DeleteLocation, GetLocations, NavigateToLocation, SetLocation, RenameLocation
)

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
            'navigation_status': 'Unknown',
            'current_location': 'Unknown',
            'status_message': 'Connecting...',
            'linear_velocity': 0.0,
            'angular_velocity': 0.0,
            'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
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
        
        # Subscribers
        self.create_subscription(RobotStats, '/robot_stats', self._on_stats, 10, callback_group=cb)
        self.create_subscription(SystemStats, '/system_stats/pi', self._on_system_stats_pi, 10, callback_group=cb)
        self.create_subscription(SystemStats, '/system_stats/jetson', self._on_system_stats_jetson, 10, callback_group=cb)
        self.create_subscription(BatteryStats, '/battery', self._on_battery, 10, callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10, callback_group=cb)
        self.create_subscription(SttResult, '/stt/text', self._on_stt, 10, callback_group=cb)
        self.create_subscription(String, '/tts/input', self._on_tts_input, 10, callback_group=cb)
        self.create_subscription(Float32MultiArray, '/tts/audio/stream', self._on_tts_audio, 10, callback_group=cb)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10, callback_group=cb)
        
        # Services
        self._svc_navigate = self.create_client(NavigateToLocation, '/navigate_to_location', callback_group=cb)
        self._svc_set_loc   = self.create_client(SetLocation, '/set_location', callback_group=cb)
        self._svc_get_locs  = self.create_client(GetLocations, '/get_locations', callback_group=cb)
        self._svc_del_loc   = self.create_client(DeleteLocation, '/delete_location', callback_group=cb)
        self._svc_rename_loc = self.create_client(RenameLocation, '/rename_location', callback_group=cb)

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

    def _on_stats(self, msg: RobotStats):
        update_data = {
            'navigation_status': msg.navigation_status,
            'current_location': msg.current_location,
            'status_message': msg.status_message,
            'linear_velocity': round(msg.linear_velocity, 3),
            'angular_velocity': round(msg.angular_velocity, 3),
        }
        # Only update battery if it's non-zero (avoid clobbering real battery data with defaults)
        if msg.battery_voltage > 0.01:
            update_data['battery_voltage'] = round(msg.battery_voltage, 2)
            update_data['battery_percentage'] = round(msg.battery_percentage, 1)
            
        self._latest_stats.update(update_data)
        asyncio.run_coroutine_threadsafe(self._broadcast(self._latest_stats), self._loop)

    def _on_system_stats_pi(self, msg: SystemStats):
        # Update specific fields for Pi
        self._latest_stats.update({
            'pi_cpu': round(msg.cpu_usage, 1),
            'pi_mem': round(msg.memory_usage, 1),
            'pi_temp': round(msg.temperature, 1),
            'pi_ip': msg.ip_address,
        })
        # logger.debug(f"RPi Stats: CPU={msg.cpu_usage}% TEMP={msg.temperature}C")
        asyncio.run_coroutine_threadsafe(self._broadcast(self._latest_stats), self._loop)

    def _on_battery(self, msg: BatteryStats):
        # Update battery from dedicated message
        self._latest_stats.update({
            'battery_voltage': round(msg.voltage, 2),
            'battery_percentage': round(msg.percentage, 1),
        })
        # logger.debug(f"Battery: {msg.voltage}V ({msg.percentage}%)")
        asyncio.run_coroutine_threadsafe(self._broadcast(self._latest_stats), self._loop)

    def _on_system_stats_jetson(self, msg: SystemStats):
        # Update specific fields for Jetson
        self._latest_stats.update({
            'jetson_cpu': round(msg.cpu_usage, 1),
            'jetson_gpu': round(msg.gpu_usage, 1),
            'jetson_mem': round(msg.memory_usage, 1),
            'jetson_mem_used': round(msg.memory_used_gb, 2),
            'jetson_mem_total': round(msg.memory_total_gb, 2),
            'jetson_temp': round(msg.temperature, 1),
            'jetson_ip': msg.ip_address,
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

    def call_srv(self, client, req):
        if not client.wait_for_service(timeout_sec=2.0): return None
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
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

@app.get('/')
async def index():
    return FileResponse(str(STATIC_DIR / 'index.html'))

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

@app.post('/api/navigate/cancel')
async def cancel_nav():
    bridge._cmd_vel_pub.publish(Twist())
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
