/**
 * Vector Nav — Web Voice Assistant Client
 *
 * Push-to-talk mic capture, WebSocket transport, and gapless audio playback.
 */

// ── State ───────────────────────────────────────────────────────────────────

let ws = null;
let mediaRecorder = null;
let audioChunks = [];
let micStream = null;  // Persistent mic stream after permission is granted
let state = 'idle'; // idle | recording | processing | playing

const chat = document.getElementById('chat');
const micBtn = document.getElementById('micBtn');
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const stateLabel = document.getElementById('stateLabel');
const stateDetail = document.getElementById('stateDetail');
const spectrumCanvas = document.getElementById('spectrum');

// ── Spectrum visualizer ─────────────────────────────────────────────────────

class SpectrumVisualizer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.analyser = null;          // active analyser (mic or tts)
        this.source = 'idle';          // 'idle' | 'mic' | 'tts'
        this.data = null;
        this.smoothed = null;          // for smooth bar transitions
        this.raf = null;
        this.dpr = Math.max(1, window.devicePixelRatio || 1);
        this._resize();
        window.addEventListener('resize', () => this._resize());
        this._loop = this._loop.bind(this);
        this._loop();
    }

    _resize() {
        const rect = this.canvas.getBoundingClientRect();
        this.canvas.width = Math.max(1, Math.floor(rect.width * this.dpr));
        this.canvas.height = Math.max(1, Math.floor(rect.height * this.dpr));
    }

    setAnalyser(analyser, source) {
        this.analyser = analyser;
        this.source = source;
        this.env = 0;
        if (analyser) {
            this.data = new Uint8Array(analyser.frequencyBinCount);
            this.smoothed = null;
        }
    }

    clear() {
        this.analyser = null;
        this.source = 'idle';
    }

    _loop() {
        this.raf = requestAnimationFrame(this._loop);
        const { ctx, canvas } = this;
        const W = canvas.width, H = canvas.height;
        ctx.clearRect(0, 0, W, H);

        if (!this.analyser) {
            this._drawIdle(W, H);
            return;
        }

        // Time-domain waveform → "signal" line
        if (!this.data || this.data.length !== this.analyser.fftSize) {
            this.data = new Uint8Array(this.analyser.fftSize);
        }
        this.analyser.getByteTimeDomainData(this.data);

        const centerY = H / 2;
        const N = this.data.length;
        const stepX = W / (N - 1);

        // Subtle center reference line
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.06)';
        ctx.lineWidth = 1 * this.dpr;
        ctx.beginPath();
        ctx.moveTo(0, centerY);
        ctx.lineTo(W, centerY);
        ctx.stroke();

        // Compute RMS of current frame → drives dynamic amplification so
        // quiet voice still produces visible motion, and louder voice pushes
        // the line toward full height.
        let sumSq = 0;
        for (let i = 0; i < N; i++) {
            const s = (this.data[i] - 128) / 128;
            sumSq += s * s;
        }
        const rms = Math.sqrt(sumSq / N); // 0..~0.7 for speech

        // Envelope follower (fast attack, slower release) for pulsing thickness/glow
        if (typeof this.env !== 'number') this.env = 0;
        const target = Math.min(1, rms * 6);   // 0..1
        const coef = target > this.env ? 0.5 : 0.08;
        this.env += (target - this.env) * coef;

        // Base + dynamic gain. Mic hits hard; TTS still reacts noticeably.
        const baseGain = this.source === 'mic' ? 6.0 : 4.0;
        const dynGain = 1 + this.env * 2.5;   // 1x..3.5x extra when loud
        const gain = baseGain * dynGain;

        // Glow pulses with envelope
        const glow = 4 + this.env * 14;
        const lineW = (1.4 + this.env * 1.6) * this.dpr;

        ctx.shadowColor = 'rgba(255, 255, 255, 0.55)';
        ctx.shadowBlur = glow * this.dpr;
        ctx.lineWidth = lineW;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.98)';
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.beginPath();
        for (let i = 0; i < N; i++) {
            const v = (this.data[i] - 128) / 128; // -1..1
            const y = centerY + Math.max(-1, Math.min(1, v * gain)) * (H * 0.48);
            const x = i * stepX;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.shadowBlur = 0;
    }

    _drawIdle(W, H) {
        const { ctx } = this;
        const t = performance.now() / 1000;
        const centerY = H / 2;
        const amp = H * 0.06;

        // Center reference line
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
        ctx.lineWidth = 1 * this.dpr;
        ctx.beginPath();
        ctx.moveTo(0, centerY);
        ctx.lineTo(W, centerY);
        ctx.stroke();

        // Soft breathing sine line
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
        ctx.lineWidth = 1.4 * this.dpr;
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        ctx.beginPath();
        const steps = 180;
        for (let i = 0; i <= steps; i++) {
            const x = (i / steps) * W;
            const phase = (i / steps) * Math.PI * 4 + t * 1.2;
            const y = centerY + Math.sin(phase) * amp * (0.6 + 0.4 * Math.sin(t * 0.7));
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        }
        ctx.stroke();
    }
}

const spectrum = new SpectrumVisualizer(spectrumCanvas);

// Mic analyser (attached while recording)
let micAnalyserCtx = null;
let micAnalyser = null;
let micSourceNode = null;

function attachMicAnalyser(stream) {
    try {
        if (!micAnalyserCtx) {
            micAnalyserCtx = new (window.AudioContext || window.webkitAudioContext)();
        }
        if (micAnalyserCtx.state === 'suspended') micAnalyserCtx.resume();
        // Tear down prior source if any
        if (micSourceNode) {
            try { micSourceNode.disconnect(); } catch (_) {}
        }
        micSourceNode = micAnalyserCtx.createMediaStreamSource(stream);
        micAnalyser = micAnalyserCtx.createAnalyser();
        micAnalyser.fftSize = 512;
        micAnalyser.smoothingTimeConstant = 0.75;
        micSourceNode.connect(micAnalyser);
        spectrum.setAnalyser(micAnalyser, 'mic');
    } catch (e) {
        console.warn('Mic analyser attach failed', e);
    }
}

function detachMicAnalyser() {
    if (spectrum.source === 'mic') spectrum.clear();
    if (micSourceNode) {
        try { micSourceNode.disconnect(); } catch (_) {}
        micSourceNode = null;
    }
    micAnalyser = null;
}

function releaseMicStream() {
    detachMicAnalyser();
    if (micStream) {
        micStream.getTracks().forEach(t => {
            try { t.stop(); } catch (_) {}
        });
        micStream = null;
    }
}

// ── Audio playback queue ────────────────────────────────────────────────────

class AudioQueue {
    constructor() {
        this.ctx = null;
        this.analyser = null;
        this.gain = null;
        this.nextStart = 0;
        this.playing = 0;
        this.onFinish = null;
        // Boost TTS playback so it's clearly audible (browsers cap at ~1.0 per node
        // but chained gain + compressor can push perceived loudness higher).
        this.volume = 3.0;
    }

    _buildGraph() {
        this.analyser = this.ctx.createAnalyser();
        this.analyser.fftSize = 1024;
        this.analyser.smoothingTimeConstant = 0.75;

        this.gain = this.ctx.createGain();
        this.gain.gain.value = this.volume;

        const comp = this.ctx.createDynamicsCompressor();
        comp.threshold.value = -18;
        comp.knee.value = 12;
        comp.ratio.value = 3;
        comp.attack.value = 0.003;
        comp.release.value = 0.2;

        // source → analyser → gain → compressor → destination
        this.analyser.connect(this.gain);
        this.gain.connect(comp);
        comp.connect(this.ctx.destination);
    }

    _createContext() {
        this.ctx = new AudioContext({ sampleRate: 24000, latencyHint: 'interactive' });
        this._buildGraph();

        // Watch for the context being auto-suspended by the browser (happens
        // on Safari/iOS after silence, or after a mic session ends). When it
        // flips to 'suspended', try to bring it back right away.
        this.ctx.onstatechange = () => {
            console.log('[audio] ctx.state =', this.ctx.state);
            if (this.ctx.state === 'suspended') {
                this.ctx.resume().catch(() => {});
            }
        };

        // Prime with a 1-sample silent buffer — a well-known iOS trick that
        // fully unlocks the audio output route on first activation.
        try {
            const silent = this.ctx.createBuffer(1, 1, this.ctx.sampleRate);
            const src = this.ctx.createBufferSource();
            src.buffer = silent;
            src.connect(this.ctx.destination);
            src.start(0);
        } catch (_) {}
    }

    async _ensureContext() {
        // Recreate if the context was closed/broken
        if (this.ctx && this.ctx.state === 'closed') {
            this.ctx = null;
        }
        if (!this.ctx) {
            this._createContext();
        }
        if (this.ctx.state === 'suspended') {
            try {
                await this.ctx.resume();
            } catch (e) {
                console.warn('[audio] resume failed:', e);
            }
        }
        return this.ctx.state === 'running';
    }

    setVolume(v) {
        this.volume = v;
        if (this.gain) this.gain.gain.value = v;
    }

    async enqueue(wavArrayBuffer) {
        const running = await this._ensureContext();
        if (!running) {
            console.warn('[audio] context not running, state=', this.ctx && this.ctx.state);
        }

        let buffer;
        try {
            buffer = await this.ctx.decodeAudioData(wavArrayBuffer);
        } catch (e) {
            console.error('[audio] decode failed:', e);
            return;
        }

        const source = this.ctx.createBufferSource();
        source.buffer = buffer;
        source.connect(this.analyser);

        const now = this.ctx.currentTime;
        // If scheduling has drifted behind the clock (context was suspended
        // then resumed), snap back to "now" so audio doesn't get scheduled
        // in the past and silently dropped.
        if (this.nextStart < now) {
            this.nextStart = now + 0.02;
        }
        const start = Math.max(now + 0.02, this.nextStart);
        source.start(start);
        this.nextStart = start + buffer.duration;
        this.playing++;

        // Switch spectrum to TTS output while playback is active
        spectrum.setAnalyser(this.analyser, 'tts');

        source.onended = () => {
            this.playing--;
            if (this.playing <= 0) {
                this.playing = 0;
                if (spectrum.source === 'tts') spectrum.clear();
                if (this.onFinish) this.onFinish();
            }
        };
    }

    reset() {
        this.nextStart = 0;
        this.playing = 0;
    }
}

const audioQueue = new AudioQueue();

// ── WebSocket ───────────────────────────────────────────────────────────────

function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${proto}://${location.host}/ws/voice`);

    ws.binaryType = 'arraybuffer';

    ws.onopen = async () => {
        statusDot.className = 'status-dot connected';
        statusText.textContent = 'Connected';
        micBtn.disabled = false;
        setState('idle');
        // NOTE: We intentionally do NOT request the mic here.
        // On iOS/Safari, holding an active getUserMedia stream forces the
        // audio session into "play-and-record" mode, which routes TTS output
        // through the earpiece instead of the loudspeaker. We only request
        // the mic on button press and release all tracks on stop.
    };

    ws.onclose = () => {
        statusDot.className = 'status-dot';
        statusText.textContent = 'Disconnected';
        micBtn.disabled = true;
        // Reconnect after a delay
        setTimeout(connect, 3000);
    };

    ws.onerror = () => {
        statusDot.className = 'status-dot';
        statusText.textContent = 'Connection error';
    };

    // Track whether we're expecting a binary WAV after a tts_chunk JSON
    let expectingAudio = false;
    let currentBotMsg = null;

    ws.onmessage = (event) => {
        // Binary message = WAV audio
        if (event.data instanceof ArrayBuffer) {
            if (state !== 'playing') {
                setState('playing');
            }
            audioQueue.enqueue(event.data.slice(0));  // copy for decodeAudioData
            return;
        }

        // JSON message
        const msg = JSON.parse(event.data);

        switch (msg.type) {
            case 'stt':
                // Update user message with actual transcription
                updateLastUserMessage(msg.text);
                break;

            case 'tts_chunk':
                // Append bot text
                if (!currentBotMsg) {
                    currentBotMsg = addMessage('bot', msg.text);
                } else {
                    appendToMessage(currentBotMsg, ' ' + msg.text);
                }
                break;

            case 'done':
                currentBotMsg = null;
                audioQueue.onFinish = () => {
                    setState('idle');
                    audioQueue.onFinish = null;
                };
                // If no audio was queued, go idle immediately
                if (audioQueue.playing <= 0) {
                    setState('idle');
                }
                break;

            case 'error':
                addMessage('error', msg.message);
                setState('idle');
                currentBotMsg = null;
                break;
        }
    };
}

// ── UI helpers ──────────────────────────────────────────────────────────────

function setState(newState) {
    state = newState;
    micBtn.classList.remove('recording');

    switch (newState) {
        case 'idle':
            stateLabel.textContent = 'Hold to speak';
            stateDetail.textContent = '';
            micBtn.disabled = false;
            break;
        case 'recording':
            stateLabel.textContent = 'Listening...';
            stateDetail.textContent = '';
            micBtn.classList.add('recording');
            break;
        case 'processing':
            stateLabel.textContent = 'Processing...';
            stateDetail.textContent = '';
            micBtn.disabled = true;
            break;
        case 'playing':
            stateLabel.textContent = 'Speaking...';
            stateDetail.textContent = '';
            micBtn.disabled = true;
            break;
    }
}

function addMessage(role, text) {
    const div = document.createElement('div');
    div.className = `message ${role}`;

    if (role !== 'error') {
        const label = document.createElement('div');
        label.className = 'label';
        label.textContent = role === 'user' ? 'You' : 'Vector Nav';
        div.appendChild(label);
    }

    const textEl = document.createElement('div');
    textEl.className = 'text';
    textEl.textContent = text;
    div.appendChild(textEl);

    chat.appendChild(div);
    chat.scrollTop = chat.scrollHeight;
    return div;
}

function appendToMessage(msgEl, text) {
    const textEl = msgEl.querySelector('.text');
    if (textEl) {
        textEl.textContent += text;
        chat.scrollTop = chat.scrollHeight;
    }
}

function updateLastUserMessage(text) {
    const msgs = chat.querySelectorAll('.message.user');
    if (msgs.length > 0) {
        const last = msgs[msgs.length - 1];
        const textEl = last.querySelector('.text');
        if (textEl) {
            textEl.textContent = text;
        }
    }
}

// ── Microphone permission ───────────────────────────────────────────────────

async function requestMicPermission() {
    // Check if getUserMedia is available (requires secure context)
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        const isSecure = window.isSecureContext;
        addMessage('error',
            `Microphone API not available (Secure Context: ${isSecure ? 'Yes' : 'No'}).\n\n` +
            '1. Ensure you are using https://' + location.host + '\n' +
            '2. If using a self-signed certificate, you must "Accept Risk" in your browser.\n' +
            '3. On some mobile devices, the mic is only allowed over HTTPS.');
        return false;
    }

    try {
        // Disable processing flags that push iOS into voice-call audio mode.
        // This keeps the audio session as "playback" once tracks are released,
        // so TTS output routes to the loudspeaker, not the earpiece.
        micStream = await navigator.mediaDevices.getUserMedia({
            audio: {
                echoCancellation: false,
                noiseSuppression: false,
                autoGainControl: false,
            },
        });
        console.log('Microphone permission granted');
        return true;
    } catch (err) {
        if (err.name === 'NotAllowedError') {
            addMessage('error',
                'Microphone permission denied. Please click the lock/site-settings icon ' +
                'in your browser address bar and allow microphone access, then reload.');
        } else if (err.name === 'NotFoundError') {
            addMessage('error', 'No microphone found. Please connect a microphone and reload.');
        } else {
            addMessage('error', `Microphone error: ${err.message}`);
        }
        console.error('Mic permission error:', err);
        return false;
    }
}

// ── Push-to-talk ────────────────────────────────────────────────────────────

async function startRecording() {
    if (state !== 'idle') return;

    // Unlock the playback AudioContext on this user gesture (required by
    // iOS Safari & some Android browsers). There's no separate "speaker
    // permission" in browsers — audio output just needs a user-initiated
    // resume() of the AudioContext. Awaited so playback is guaranteed to
    // be unlocked before any TTS audio is scheduled.
    try {
        await audioQueue._ensureContext();
    } catch (_) { /* will retry on first chunk */ }

    // Ensure we have mic access
    if (!micStream || !micStream.active) {
        const granted = await requestMicPermission();
        if (!granted) return;
    }

    try {
        // Prefer webm/opus, fall back to whatever the browser supports
        const mimeType = MediaRecorder.isTypeSupported('audio/webm;codecs=opus')
            ? 'audio/webm;codecs=opus'
            : 'audio/webm';

        mediaRecorder = new MediaRecorder(micStream, { mimeType });
        audioChunks = [];

        mediaRecorder.ondataavailable = (e) => {
            if (e.data.size > 0) {
                audioChunks.push(e.data);
            }
        };

        mediaRecorder.onstop = () => {
            // Fully release mic tracks so iOS switches the audio session
            // back from "play-and-record" (earpiece) to "playback" (loudspeaker).
            releaseMicStream();

            if (audioChunks.length === 0) {
                setState('idle');
                return;
            }

            const blob = new Blob(audioChunks, { type: mimeType });
            sendAudio(blob);
        };

        mediaRecorder.start();
        attachMicAnalyser(micStream);
        setState('recording');
        addMessage('user', '...');  // Placeholder

    } catch (err) {
        addMessage('error', `Recording failed: ${err.message}`);
        console.error('Recording error:', err);
    }
}

function stopRecording() {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
        mediaRecorder.stop();
        detachMicAnalyser();
        setState('processing');
    }
}

function sendAudio(blob) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addMessage('error', 'Not connected to server');
        setState('idle');
        return;
    }

    blob.arrayBuffer().then(buffer => {
        ws.send(buffer);
        // Reset audio queue for new response
        audioQueue.reset();
    });
}

// ── Event listeners ─────────────────────────────────────────────────────────

// Mouse events
micBtn.addEventListener('mousedown', (e) => {
    e.preventDefault();
    startRecording();
});
micBtn.addEventListener('mouseup', (e) => {
    e.preventDefault();
    stopRecording();
});
micBtn.addEventListener('mouseleave', (e) => {
    if (state === 'recording') {
        stopRecording();
    }
});

// Touch events (mobile)
micBtn.addEventListener('touchstart', (e) => {
    e.preventDefault();
    startRecording();
});
micBtn.addEventListener('touchend', (e) => {
    e.preventDefault();
    stopRecording();
});

// Keyboard: hold Space to talk
document.addEventListener('keydown', (e) => {
    if (e.code === 'Space' && !e.repeat && state === 'idle') {
        e.preventDefault();
        startRecording();
    }
});
document.addEventListener('keyup', (e) => {
    if (e.code === 'Space' && state === 'recording') {
        e.preventDefault();
        stopRecording();
    }
});

// ── Init ────────────────────────────────────────────────────────────────────

connect();
