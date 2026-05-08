/**
 * Vector Nav — Unified Control & Voice Assistant (Split Layout)
 */

// ── Configuration ────────────────────────────────────────────────────────────
const APP_PORT = location.port || (location.protocol === 'https:' ? '443' : '80');

// ── State ────────────────────────────────────────────────────────────────────
let voiceWs = null;
let bridgeWs = null;
let mediaRecorder = null;
let audioChunks = [];
let micStream = null;
let state = 'idle'; // idle | recording | processing | playing

// Control State
let joyLinearActive = false;
let joyAngularActive = false;
let joyLinear = 0.0;
let joyAngular = 0.0;
let maxLinear = 0.50;
let maxAngular = 1.00;
let cmdInterval = null;

const PRESETS = {
    ECO:     { linear: 0.30, angular: 1.00 },
    CLASSIC: { linear: 0.60, angular: 1.80 },
    SPORT:   { linear: 0.90, angular: 2.50 },
};

// ── DOM Refs ──────────────────────────────────────────────────────────────────
const chat = document.getElementById('chat');
const micBtn = document.getElementById('micBtn');
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const stateLabel = document.getElementById('stateLabel');
const spectrumCanvas = document.getElementById('spectrum');

// Drive Mode Refs
const driveOverlay = document.getElementById('drive-overlay');
const enterDriveBtn = document.getElementById('enter-drive-btn');
const closeDriveBtn = document.getElementById('close-drive');
const maxSpeedSlider = document.getElementById('max-speed-slider');
const maxSpeedVal = document.getElementById('max-speed-val');
const driveBattery = document.getElementById('drive-stat-battery');
const driveVel = document.getElementById('drive-stat-vel');

// Stats Refs
const headerBattery = document.getElementById('header-battery-fill');
const headerBatteryText = document.getElementById('header-battery-text');
const statNav = document.getElementById('stat-nav');
const statVel = document.getElementById('stat-vel');
const statPose = document.getElementById('stat-pose');

// Waypoints Refs
const locInput = document.getElementById('loc-input');
const saveLocBtn = document.getElementById('save-loc-btn');
const locList = document.getElementById('locations-list');
const stopNavBtn = document.getElementById('stop-nav-btn');
const toast = document.getElementById('toast');

// ── Spectrum Visualizer ───────────────────────────────────────────────────────
class SpectrumVisualizer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.analyser = null;
        this.data = null;
        this.dpr = window.devicePixelRatio || 1;
        this._resize();
        window.addEventListener('resize', () => this._resize());
        this._loop = this._loop.bind(this);
        this._loop();
    }
    _resize() {
        const rect = this.canvas.getBoundingClientRect();
        this.canvas.width = rect.width * this.dpr;
        this.canvas.height = rect.height * this.dpr;
    }
    setAnalyser(analyser) {
        this.analyser = analyser;
        if (analyser) this.data = new Uint8Array(analyser.frequencyBinCount);
    }
    _loop() {
        requestAnimationFrame(this._loop);
        const { ctx, canvas } = this;
        const W = canvas.width, H = canvas.height;
        ctx.clearRect(0, 0, W, H);
        if (!this.analyser) return;
        this.analyser.getByteTimeDomainData(this.data);
        ctx.strokeStyle = 'rgba(255,255,255,0.8)';
        ctx.lineWidth = 3 * this.dpr;
        ctx.beginPath();
        const sliceWidth = W / this.data.length;
        let x = 0;
        for (let i = 0; i < this.data.length; i++) {
            const v = this.data[i] / 128.0;
            const y = v * (H / 2);
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
            x += sliceWidth;
        }
        ctx.lineTo(W, H / 2);
        ctx.stroke();
    }
}
const visualizer = new SpectrumVisualizer(spectrumCanvas);

// ── Connections ───────────────────────────────────────────────────────────────
function connectVoice() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    voiceWs = new WebSocket(`${protocol}//${location.hostname}:${APP_PORT}/ws/voice`);
    voiceWs.onopen = () => {
        statusDot.className = 'status-dot connected';
        statusText.textContent = 'Ready';
        micBtn.disabled = false;
    };
    voiceWs.onclose = () => {
        statusDot.className = 'status-dot';
        statusText.textContent = 'Offline';
        micBtn.disabled = true;
        setTimeout(connectVoice, 2000);
    };
    voiceWs.onmessage = handleVoiceMessage;
}

function connectBridge() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    bridgeWs = new WebSocket(`${protocol}//${location.hostname}:${APP_PORT}/ws/control`);
    bridgeWs.onopen = () => {
        console.log('Control Bridge connected');
        fetchLocations();
    };
    bridgeWs.onclose = () => {
        console.log('Control Bridge disconnected');
        setTimeout(connectBridge, 2000);
    };
    bridgeWs.onmessage = (evt) => {
        const msg = JSON.parse(evt.data);
        if (msg.type === 'stats') updateStats(msg);
    };
}

// ── Voice Logic ───────────────────────────────────────────────────────────────
async function startRecording() {
    if (state !== 'idle') return;
    try {
        if (!micStream) micStream = await navigator.mediaDevices.getUserMedia({ audio: true });
        const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
        const source = audioCtx.createMediaStreamSource(micStream);
        const analyser = audioCtx.createAnalyser();
        source.connect(analyser);
        visualizer.setAnalyser(analyser);

        mediaRecorder = new MediaRecorder(micStream);
        audioChunks = [];
        mediaRecorder.ondataavailable = (e) => audioChunks.push(e.data);
        mediaRecorder.onstop = () => {
            const blob = new Blob(audioChunks, { type: 'audio/webm' });
            sendAudio(blob);
            visualizer.setAnalyser(null);
        };
        mediaRecorder.start();
        setState('recording');
    } catch (err) {
        showToast('Mic access denied', false);
    }
}

function stopRecording() {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
        mediaRecorder.stop();
        setState('processing');
    }
}

function sendAudio(blob) {
    if (voiceWs && voiceWs.readyState === WebSocket.OPEN) {
        blob.arrayBuffer().then(buf => voiceWs.send(buf));
    }
}

function handleVoiceMessage(evt) {
    if (typeof evt.data === 'string') {
        const msg = JSON.parse(evt.data);
        if (msg.type === 'stt') addMessage('user', msg.text);
        else if (msg.type === 'tts_chunk') addMessage('bot', msg.text);
        else if (msg.type === 'done') setState('idle');
    }
}

function setState(s) {
    state = s;
    micBtn.className = 'mic-btn' + (s === 'recording' ? ' recording' : '');
    stateLabel.textContent = s.toUpperCase();
}

function addMessage(who, text) {
    if (!text) return;
    const div = document.createElement('div');
    div.className = `message ${who}`;
    div.innerHTML = `<div class="label">${who}</div><div class="text">${text}</div>`;
    chat.appendChild(div);
    chat.scrollTop = chat.scrollHeight;
}

// ── Control Logic ─────────────────────────────────────────────────────────────
function updateStats(msg) {
    const pct = msg.battery_percentage || 0;
    const batStr = `${pct.toFixed(0)}%`;
    
    headerBatteryText.textContent = batStr;
    headerBattery.style.width = batStr;
    headerBattery.style.background = pct > 50 ? '#34c759' : pct > 20 ? '#ff9800' : '#ff3b30';
    if (driveBattery) driveBattery.textContent = batStr;

    statNav.textContent = msg.navigation_status || 'Idle';
    
    if (msg.linear_velocity !== undefined) {
        const velStr = `${msg.linear_velocity.toFixed(2)} m/s`;
        if (statVel) statVel.textContent = velStr;
        if (driveVel) driveVel.textContent = velStr;
    }

    if (msg.pose) {
        const p = msg.pose;
        statPose.textContent = `X: ${p.x.toFixed(2)} Y: ${p.y.toFixed(2)} θ: ${(p.yaw * 180 / Math.PI).toFixed(1)}°`;
    }
}

function fetchLocations() {
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/locations`)
        .then(r => r.json())
        .then(renderLocations)
        .catch(err => console.error('Failed to fetch locations:', err));
}

function renderLocations(locs) {
    if (!locs || Object.keys(locs).length === 0) {
        locList.innerHTML = '<div class="empty-msg">No waypoints saved</div>';
        return;
    }
    locList.innerHTML = '';
    for (const [name, data] of Object.entries(locs)) {
        const item = document.createElement('div');
        item.className = 'location-item';
        item.innerHTML = `
            <div class="loc-main">
                <span class="location-name">${name}</span>
                <span class="location-coords">X: ${data.x.toFixed(2)} Y: ${data.y.toFixed(2)}</span>
            </div>
            <div class="loc-actions">
                <span class="edit-btn" data-name="${name}">✎</span>
                <span class="delete-btn" data-name="${name}">&times;</span>
            </div>
        `;
        item.querySelector('.loc-main').onclick = () => navigateTo(name);
        item.querySelector('.delete-btn').onclick = (e) => deleteLocation(e.target.dataset.name);
        item.querySelector('.edit-btn').onclick = (e) => editLocationName(e.target.dataset.name);
        locList.appendChild(item);
    }
}

function saveLocation() {
    const name = locInput.value.trim();
    if (!name) return;
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/locations`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name })
    }).then(r => r.json()).then(res => {
        if (res.success) {
            locInput.value = '';
            fetchLocations();
            showToast(`Saved '${name}'`, true);
        } else {
            showToast(res.message || 'Failed to save', false);
        }
    });
}

function deleteLocation(name) {
    if (!confirm(`Delete '${name}'?`)) return;
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/locations/${encodeURIComponent(name)}`, {
        method: 'DELETE'
    }).then(() => {
        fetchLocations();
        showToast(`Deleted '${name}'`, true);
    });
}

function editLocationName(oldName) {
    const newName = prompt(`Rename '${oldName}' to:`, oldName);
    if (!newName || newName === oldName) return;
    
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/locations`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ old_name: oldName, new_name: newName })
    }).then(r => r.json()).then(res => {
        if (res.success) {
            fetchLocations();
            showToast(`Renamed to '${newName}'`, true);
        } else {
            showToast(res.message || 'Failed to rename', false);
        }
    });
}

function navigateTo(name) {
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/navigate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name })
    }).then(() => showToast(`Navigating to ${name}`, true));
}

let linearManager = null;
let angularManager = null;

// ── Teleop ──
function initJoysticks() {
    if (linearManager) linearManager.destroy();
    if (angularManager) angularManager.destroy();
    document.getElementById('joystick-linear').innerHTML = '';
    document.getElementById('joystick-angular').innerHTML = '';

    linearManager = nipplejs.create({
        zone: document.getElementById('joystick-linear'),
        mode: 'static',
        position: { left: '100px', top: '100px' },
        color: '#ffffff',
        size: 150,
        threshold: 0.1,
        lockY: true 
    });

    linearManager.on('move', (evt, data) => {
        if (!data.vector) return;
        joyLinearActive = true;
        joyLinear = data.vector.y * 2.0 * maxLinear;
        joyLinear = Math.max(-maxLinear, Math.min(maxLinear, joyLinear));
    });

    linearManager.on('end', () => {
        joyLinearActive = false;
        joyLinear = 0;
        if (!joyAngularActive) sendWS({ type: 'stop' });
    });

    angularManager = nipplejs.create({
        zone: document.getElementById('joystick-angular'),
        mode: 'static',
        position: { left: '100px', top: '100px' },
        color: '#ffffff',
        size: 150,
        threshold: 0.1,
        lockX: true
    });

    angularManager.on('move', (evt, data) => {
        if (!data.vector) return;
        joyAngularActive = true;
        joyAngular = -data.vector.x * 2.0 * maxAngular;
        joyAngular = Math.max(-maxAngular, Math.min(maxAngular, joyAngular));
    });

    angularManager.on('end', () => {
        joyAngularActive = false;
        joyAngular = 0;
        if (!joyLinearActive) sendWS({ type: 'stop' });
    });

    if (cmdInterval) clearInterval(cmdInterval);
    cmdInterval = setInterval(() => {
        if (joyLinearActive || joyAngularActive) {
            sendWS({ type: 'cmd_vel', linear: joyLinear, angular: joyAngular });
        }
    }, 50);
}

function sendWS(obj) {
    if (bridgeWs && bridgeWs.readyState === WebSocket.OPEN) {
        bridgeWs.send(JSON.stringify(obj));
    }
}

function showToast(msg, success) {
    toast.textContent = msg;
    toast.style.borderColor = success ? 'var(--success)' : 'var(--danger)';
    toast.classList.add('active');
    setTimeout(() => toast.classList.remove('active'), 3000);
}

// ── Listeners ────────────────────────────────────────────────────────────────
enterDriveBtn.onclick = () => {
    driveOverlay.classList.add('active');
    setTimeout(initJoysticks, 200);
};

closeDriveBtn.onclick = () => {
    driveOverlay.classList.remove('active');
    if (linearManager) linearManager.destroy();
    if (angularManager) angularManager.destroy();
};

maxSpeedSlider.oninput = () => {
    maxLinear = parseFloat(maxSpeedSlider.value);
    maxSpeedVal.textContent = maxLinear.toFixed(2);
};

micBtn.onmousedown = micBtn.ontouchstart = (e) => { e.preventDefault(); startRecording(); };
micBtn.onmouseup = micBtn.onmouseleave = micBtn.ontouchend = (e) => { e.preventDefault(); stopRecording(); };

saveLocBtn.onclick = saveLocation;
stopNavBtn.onclick = () => fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/navigate/cancel`, { method: 'POST' });

document.querySelectorAll('.preset-btn').forEach(btn => {
    btn.onclick = () => {
        document.querySelectorAll('.preset-btn.active').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        const p = PRESETS[btn.dataset.preset];
        maxLinear = p.linear;
        maxAngular = p.angular;
        maxSpeedSlider.value = maxLinear;
        maxSpeedVal.textContent = maxLinear.toFixed(2);
    };
});

// ── Init ─────────────────────────────────────────────────────────────────────
connectVoice();
connectBridge();
