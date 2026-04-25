"""
VECTOR NAV — Web Voice Assistant
=================================
FastAPI server with WebSocket endpoint for browser-based voice interaction.

Run:
    cd /home/admin/vector_nav/src/vector_web
    uvicorn vector_web.app:app --host 0.0.0.0 --port 8080
"""

import os
import ssl
import asyncio
import logging
import subprocess
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from .audio_utils import decode_webm_to_pcm
from .pipeline import VoicePipeline

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
)
logger = logging.getLogger(__name__)

# ── Configuration from environment ───────────────────────────────────────────

WS_URL = os.environ.get('WS_URL', 'wss://localhost:49000')
STT_MODEL = os.environ.get('STT_MODEL', 'base.en')
TTS_VOICE = os.environ.get('TTS_VOICE', 'af_heart')
TTS_SPEED = float(os.environ.get('TTS_SPEED', '1.0'))
TTS_ENGINE_DIR = os.environ.get('TTS_ENGINE_DIR', '/home/admin/models/kokoro_trt')
USE_TRT = os.environ.get('USE_TRT', 'true').lower() in ('true', '1', 'yes')
HOST = os.environ.get('HOST', '0.0.0.0')
PORT = int(os.environ.get('PORT', '8080'))

STATIC_DIR = Path(__file__).parent / 'static'
CERT_DIR = Path(__file__).parent.parent / 'certs'

# ── App ──────────────────────────────────────────────────────────────────────

app = FastAPI(title='Vector Nav Voice Assistant')
app.mount('/static', StaticFiles(directory=str(STATIC_DIR)), name='static')

pipeline: VoicePipeline | None = None


@app.on_event('startup')
async def startup():
    global pipeline
    logger.info('Loading voice pipeline...')
    pipeline = VoicePipeline(
        ws_url=WS_URL,
        stt_model=STT_MODEL,
        tts_voice=TTS_VOICE,
        tts_speed=TTS_SPEED,
        tts_engine_dir=TTS_ENGINE_DIR,
        use_trt=USE_TRT,
    )
    # Load engines in a thread so we don't block the event loop
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, pipeline.load)
    logger.info('Voice pipeline ready — server accepting connections')


@app.get('/')
async def index():
    return FileResponse(str(STATIC_DIR / 'index.html'))


@app.websocket('/ws/voice')
async def voice_ws(websocket: WebSocket):
    await websocket.accept()
    logger.info('WebSocket client connected')

    try:
        while True:
            # Receive binary audio (WebM/Opus from browser)
            data = await websocket.receive_bytes()
            logger.info(f'Received audio: {len(data)} bytes')

            loop = asyncio.get_event_loop()

            # Decode audio in executor
            try:
                pcm = await loop.run_in_executor(
                    None, decode_webm_to_pcm, data
                )
            except RuntimeError as e:
                await websocket.send_json({
                    'type': 'error',
                    'message': f'Audio decode failed: {e}',
                })
                continue

            duration = len(pcm) / 16000
            logger.info(f'Decoded to {duration:.1f}s of audio')

            if duration < 0.3:
                await websocket.send_json({
                    'type': 'error',
                    'message': 'Audio too short — hold the button longer',
                })
                continue

            # Run the full pipeline in executor with streaming callbacks
            def run_pipeline():
                chunks_sent = []

                def on_tts_chunk(sentence: str, wav_bytes: bytes):
                    # Schedule sends from the executor thread
                    asyncio.run_coroutine_threadsafe(
                        _send_tts_chunk(websocket, sentence, wav_bytes),
                        loop,
                    ).result(timeout=10)  # Wait so chunks stay ordered
                    chunks_sent.append(sentence)

                result = pipeline.process(pcm, on_tts_chunk=on_tts_chunk)
                return result

            try:
                result = await loop.run_in_executor(None, run_pipeline)
            except Exception as e:
                logger.exception('Pipeline error')
                await websocket.send_json({
                    'type': 'error',
                    'message': f'Processing failed: {e}',
                })
                continue

            # Send STT result
            await websocket.send_json({
                'type': 'stt',
                'text': result['stt_text'],
                'confidence': result['confidence'],
            })

            # Signal completion
            await websocket.send_json({'type': 'done'})

    except WebSocketDisconnect:
        logger.info('WebSocket client disconnected')


async def _send_tts_chunk(websocket: WebSocket, text: str, wav_bytes: bytes):
    """Send a TTS chunk (JSON metadata + binary WAV) over the WebSocket."""
    await websocket.send_json({
        'type': 'tts_chunk',
        'text': text,
    })
    await websocket.send_bytes(wav_bytes)


# ── SSL cert generation ──────────────────────────────────────────────────────

def _ensure_self_signed_cert() -> tuple[str, str]:
    """Generate a self-signed cert if one doesn't exist. Returns (certfile, keyfile)."""
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    cert_file = CERT_DIR / 'cert.pem'
    key_file = CERT_DIR / 'key.pem'

    if cert_file.exists() and key_file.exists():
        logger.info(f'Using existing SSL cert from {CERT_DIR}')
        return str(cert_file), str(key_file)

    logger.info('Generating self-signed SSL certificate for HTTPS...')
    subprocess.run([
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', str(key_file),
        '-out', str(cert_file),
        '-days', '365', '-nodes',
        '-subj', '/CN=vector-nav',
    ], check=True, capture_output=True)
    logger.info(f'SSL cert generated at {CERT_DIR}')
    return str(cert_file), str(key_file)


# ── CLI entry point ──────────────────────────────────────────────────────────

def main():
    import uvicorn

    cert_file, key_file = _ensure_self_signed_cert()

    print(f'\n  Open https://{HOST}:{PORT} in your browser')
    print(f'  (Note: You will need to accept the self-signed certificate warning)\n')

    uvicorn.run(
        'vector_web.app:app',
        host=HOST,
        port=PORT,
        ssl_keyfile=key_file,
        ssl_certfile=cert_file,
        log_level='info',
    )


if __name__ == '__main__':
    main()
