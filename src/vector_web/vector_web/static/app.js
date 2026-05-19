/**
 * Vector Nav — Unified Control & Voice Assistant (Split Layout)
 *
 * Voice input: Web Speech API (browser-native STT, zero server load).
 * Only the transcript text travels over the WebSocket to the Jetson.
 */

// ── Configuration ────────────────────────────────────────────────────────────
const APP_PORT = location.port || (location.protocol === 'https:' ? '443' : '80');

// ── Speech Recognition setup ─────────────────────────────────────────────────
const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
let recognition = null;
let _recognizing = false;
let _finalTranscript = '';
let _interimTranscript = '';
let _micStream = null;       // kept open for spectrum visualisation
let _audioCtx = null;
let _analyser = null;

if (SpeechRecognition) {
    recognition = new SpeechRecognition();
    recognition.lang = 'en-US';
    recognition.continuous = false;
    recognition.interimResults = true;
    recognition.maxAlternatives = 1;

    recognition.onstart = () => {
        _recognizing = true;
        _finalTranscript = '';
        _interimTranscript = '';
        setState('recording');
        // Bind spectrum analyser if we have a mic stream
        if (_analyser && window.__vnSpectrum) {
            window.__vnSpectrum.bind(_analyser);
            window.__vnSpectrum.start();
        }
        document.querySelector('.mic-wrap')?.classList.add('live');
        document.getElementById('spectrum-stage')?.classList.add('live');
        addLog('VOX', 'Listening...');
        console.debug('[STT] recognition started');
    };

    recognition.onresult = (evt) => {
        let interim = '';
        for (let i = evt.resultIndex; i < evt.results.length; i++) {
            const t = evt.results[i][0].transcript;
            if (evt.results[i].isFinal) _finalTranscript += t;
            else interim += t;
        }
        _interimTranscript = interim;
        if (interim) stateLabel.textContent = interim.slice(0, 60);
        console.debug('[STT] interim:', interim, 'final:', _finalTranscript);
    };

    recognition.onerror = (evt) => {
        console.warn('[STT] error:', evt.error);
        if (evt.error === 'no-speech') {
            addLog('VOX', 'No speech detected');
        } else if (evt.error === 'not-allowed') {
            addLog('WARN', 'Mic permission denied');
        } else if (evt.error !== 'aborted') {
            addLog('WARN', `STT error: ${evt.error}`);
        }
        _stopRecognition();
    };

    recognition.onend = () => {
        console.debug('[STT] ended, final:', _finalTranscript, 'interim:', _interimTranscript);
        _recognizing = false;
        _stopRecognitionUI();
        
        // Fallback to interim transcript if we stopped before it became final
        const textToSend = (_finalTranscript || _interimTranscript).trim();
        
        if (textToSend) {
            sendText(textToSend);
        } else {
            addLog('VOX', 'Nothing heard');
        }
    };
} else {
    console.warn('Web Speech API not supported. Use Chrome/Edge.');
    addLog('WARN', 'Speech recognition not supported — use Chrome/Edge');
}

function _stopRecognition() {
    if (_recognizing && recognition) recognition.stop();
}

function _stopRecognitionUI() {
    if (window.__vnSpectrum) window.__vnSpectrum.stop();
    document.querySelector('.mic-wrap')?.classList.remove('live');
    document.getElementById('spectrum-stage')?.classList.remove('live');
    setState(_finalTranscript.trim() ? 'processing' : 'idle');
}

function sendText(text) {
    if (!text) return;
    if (voiceWs && voiceWs.readyState === WebSocket.OPEN) {
        voiceWs.send(JSON.stringify({ type: 'text', text }));
        addLog('VOX', `"${text}"`);
    }
}

function handleManualSend() {
    if (state === 'recording') return; // Don't interrupt recording
    const text = chatInput.value.trim();
    if (!text) return;

    if (state !== 'idle') {
        if (voiceWs && voiceWs.readyState === WebSocket.OPEN) {
            voiceWs.send(JSON.stringify({ type: 'interrupt' }));
        }
        browserAudio.queue = [];
        browserAudio.playing = false;
        if (browserAudio.currentAudio) {
            browserAudio.currentAudio.pause();
            browserAudio.currentAudio = null;
        }
    }

    // Add to local chat immediately
    addMessage('user', text);
    sendText(text);

    chatInput.value = '';
    setState('processing');
}

// ── State ────────────────────────────────────────────────────────────────────
let voiceWs = null;
let bridgeWs = null;
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
const chatInput = document.getElementById('chat-input');
const chatSendBtn = document.getElementById('chat-send-btn');
// Spectrum canvas is owned by effects.js (id "spectrum-bars"); we just bind the analyser via __vnSpectrum.

// Drive Mode Refs
const driveOverlay = document.getElementById('drive-overlay');
const enterDriveBtn = document.getElementById('enter-drive-btn');
const closeDriveBtn = document.getElementById('close-drive');
const maxSpeedSlider = document.getElementById('max-speed-slider');
const maxSpeedVal = document.getElementById('max-speed-val');
const driveBattery = document.getElementById('drive-stat-battery');
const driveVel = document.getElementById('drive-stat-vel');

// Stats Refs
const statNav  = document.getElementById('stat-nav');
const statVel  = document.getElementById('stat-vel');
const statAng  = document.getElementById('stat-ang');
const statHead = document.getElementById('stat-heading');
const headNeedle = document.getElementById('heading-needle');
const velBar  = document.getElementById('vel-bar');
const angBar  = document.getElementById('ang-bar');
const poseX   = document.getElementById('pose-x');
const poseY   = document.getElementById('pose-y');
const poseTh  = document.getElementById('pose-th');
const logList = document.getElementById('log-list');
const chatEmpty = document.getElementById('chat-empty');
const voxBadge = document.getElementById('vox-badge');
const estopBtn = document.getElementById('estop-btn');
const resumeBtn = document.getElementById('resume-btn');
const stopTopBtn = document.getElementById('stop-btn-top');
const clearChatBtn = document.getElementById('clear-chat-btn');

// Waypoints Refs
const locInput = document.getElementById('loc-input');
const saveLocBtn = document.getElementById('save-loc-btn');
const locList = document.getElementById('locations-list');
const stopNavBtn = document.getElementById('stop-nav-btn');
const toast = document.getElementById('toast');

// Modal Refs
const pillJetson = document.getElementById('pill-jetson');
const pillPi = document.getElementById('pill-pi');

if (pillJetson) pillJetson.onclick = () => openModal('modal-jetson');
if (pillPi) pillPi.onclick = () => openModal('modal-pi');

function openModal(id) {
    const modal = document.getElementById(id);
    if (modal) modal.classList.add('active');
}

function closeModal(id) {
    const modal = document.getElementById(id);
    if (modal) modal.classList.remove('active');
}

// Close modals on overlay click
window.onclick = (event) => {
    if (event.target.classList.contains('modal-overlay')) {
        event.target.classList.remove('active');
    }
};

// Spectrum is fully handled by effects.js (window.__vnSpectrum).

// ── Connections ───────────────────────────────────────────────────────────────
function connectVoice() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    voiceWs = new WebSocket(`${protocol}//${location.hostname}:${APP_PORT}/ws/voice`);
    voiceWs.onopen = () => {
        statusDot.className = 'status-dot connected';
        statusText.textContent = 'Ready';
        micBtn.disabled = false;
        addLog('OK', 'Voice channel ready');
    };
    voiceWs.onclose = () => {
        statusDot.className = 'status-dot';
        statusText.textContent = 'Offline';
        micBtn.disabled = true;
        setTimeout(connectVoice, 2000);
    };
    voiceWs.onmessage = handleVoiceMessage;
}

// ── Map polling — requests latest map from server until one arrives ───────────
let _mapPollTimer = null;

function startMapPoll() {
    stopMapPoll();
    let attempts = 0;
    _mapPollTimer = setInterval(() => {
        if (++attempts > 15) { stopMapPoll(); return; }
        if (bridgeWs && bridgeWs.readyState === WebSocket.OPEN)
            bridgeWs.send(JSON.stringify({ type: 'get_map' }));
    }, 3000);
}

function stopMapPoll() {
    if (_mapPollTimer) { clearInterval(_mapPollTimer); _mapPollTimer = null; }
}

function connectBridge() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    bridgeWs = new WebSocket(`${protocol}//${location.hostname}:${APP_PORT}/ws/control`);
    bridgeWs.onopen = () => {
        console.log('Control Bridge connected');
        fetchLocations();
        startMapPoll();
    };
    bridgeWs.onclose = () => {
        console.log('Control Bridge disconnected');
        stopMapPoll();
        setTimeout(connectBridge, 2000);
    };
    bridgeWs.onmessage = (evt) => {
        const msg = JSON.parse(evt.data);
        if (msg.type === 'stats') updateStats(msg);
        else if (msg.type === 'scan') {
            if (window.__vnLidar) window.__vnLidar.push(msg.ranges);
        } else if (msg.type === 'slam_map') {
            if (window.__vnMap) window.__vnMap.update(msg);
        }
    };
}

// ── Voice Logic ───────────────────────────────────────────────────────────────
async function startRecording() {
    if (state === 'recording') return;
    
    if (voiceWs && voiceWs.readyState === WebSocket.OPEN) {
        voiceWs.send(JSON.stringify({ type: 'interrupt' }));
    }
    browserAudio.queue = [];
    browserAudio.playing = false;
    if (browserAudio.currentAudio) {
        browserAudio.currentAudio.pause();
        browserAudio.currentAudio = null;
    }

    if (!recognition) {
        showToast('Speech recognition not supported — use Chrome/Edge', false);
        return;
    }
    // Get mic stream for spectrum visualisation (separate from Web Speech API)
    try {
        if (!_micStream) {
            _micStream = await navigator.mediaDevices.getUserMedia({ audio: true, video: false });
            _audioCtx = new (window.AudioContext || window.webkitAudioContext)();
            const source = _audioCtx.createMediaStreamSource(_micStream);
            _analyser = _audioCtx.createAnalyser();
            _analyser.fftSize = 256;
            source.connect(_analyser);
        }
    } catch (e) {
        console.warn('[MIC] getUserMedia failed:', e);
        addLog('WARN', 'Mic access denied — check browser permissions');
    }
    try {
        recognition.start();
    } catch (e) {
        // Already started (rapid double-tap guard)
        console.debug('[STT] start guard:', e);
    }
}

function stopRecording() {
    // Stop recognition — fires onend which sends the transcript
    _stopRecognition();
}

function handleVoiceMessage(evt) {
    if (typeof evt.data === 'string') {
        const msg = JSON.parse(evt.data);
        if (msg.type === 'stt') {
            addMessage('user', msg.text);
            addLog('VOX', `"${msg.text}" · conf ${(msg.confidence ?? 1).toFixed(2)}`);
        } else if (msg.type === 'tts_chunk') {
            addMessage('bot', msg.text);
        } else if (msg.type === 'done') {
            setState('idle');
        }
    } else {
        // Binary WAV chunk — play in browser
        playBrowserWav(evt.data);
    }
}

// ── Browser TTS playback queue ──────────────────────────────────────────────
const browserAudio = { queue: [], playing: false, volume: 1.0, currentAudio: null };
function playBrowserWav(blob) {
    const url = URL.createObjectURL(blob);
    browserAudio.queue.push(url);
    if (!browserAudio.playing) _drainBrowserAudio();
}
function _drainBrowserAudio() {
    const url = browserAudio.queue.shift();
    if (!url) { browserAudio.playing = false; browserAudio.currentAudio = null; return; }
    browserAudio.playing = true;
    const a = new Audio(url);
    browserAudio.currentAudio = a;
    a.volume = Math.max(0, Math.min(1, browserAudio.volume));
    a.onended = a.onerror = () => { URL.revokeObjectURL(url); _drainBrowserAudio(); };
    a.play().catch(() => { URL.revokeObjectURL(url); _drainBrowserAudio(); });
}

function setState(s) {
    state = s;
    micBtn.className = 'mic-btn' + (s === 'recording' ? ' recording' : '');
    stateLabel.textContent = ({ idle: 'Standby', recording: 'Listening', processing: 'Thinking', playing: 'Speaking' }[s]) || s.toUpperCase();
    if (voxBadge) voxBadge.textContent = s === 'idle' ? 'IDLE' : s.toUpperCase();
}

function addMessage(who, text) {
    if (!text) return;
    if (chatEmpty && chatEmpty.parentNode) chatEmpty.remove();
    const div = document.createElement('div');
    div.className = `msg ${who === 'user' ? 'msg-user' : 'msg-bot'}`;
    const safe = String(text).replace(/[&<>]/g, c => ({ '&':'&amp;', '<':'&lt;', '>':'&gt;' }[c]));
    if (who === 'user') {
        div.innerHTML = `<div class="msg-bubble"><div class="msg-label">You</div><div class="msg-text">${safe}</div></div>`;
    } else {
        div.innerHTML = `<div class="msg-avatar"><svg class="ico"><use href="#i-bot"/></svg></div><div class="msg-bubble"><div class="msg-label">Vector</div><div class="msg-text">${safe}</div></div>`;
    }
    chat.appendChild(div);
    chat.scrollTop = chat.scrollHeight;
}

// ── Mission log ─────────────────────────────────────────────────────────────
const LOG_TAGS = { NAV: 'tag-info', VOX: 'tag-vox', OK: 'tag-ok', WARN: 'tag-warn', SYS: 'tag-info', ERR: 'tag-warn' };
const MAX_LOG_ROWS = 40;
let _lastLogKey = '';
function addLog(tag, msg) {
    if (!logList) return;
    const key = `${tag}|${msg}`;
    if (key === _lastLogKey) return;
    _lastLogKey = key;
    const t = new Date();
    const hh = String(t.getHours()).padStart(2, '0');
    const mm = String(t.getMinutes()).padStart(2, '0');
    const ss = String(t.getSeconds()).padStart(2, '0');
    const li = document.createElement('li');
    const cls = LOG_TAGS[tag] || 'tag-info';
    const safe = String(msg).replace(/[&<>]/g, c => ({ '&':'&amp;', '<':'&lt;', '>':'&gt;' }[c]));
    li.innerHTML = `<span class="log-time">${hh}:${mm}:${ss}</span><span class="log-tag ${cls}">${tag}</span><span class="log-msg">${safe}</span>`;
    logList.prepend(li);
    while (logList.children.length > MAX_LOG_ROWS) logList.lastElementChild.remove();
}

// ── Control Logic ─────────────────────────────────────────────────────────────
let _lastNav = null;
function updateStats(msg) {
    const pct = msg.battery_percentage;
    const volt = msg.battery_voltage;
    if (typeof pct === 'number') {
        if (window.__vnSetBattery) window.__vnSetBattery(pct, volt);
        if (driveBattery) driveBattery.textContent = `${pct.toFixed(0)}%`;
    }

    if (msg.navigation_status !== undefined) {
        const nav = msg.navigation_status || 'Idle';
        if (statNav) statNav.textContent = nav;
        if (nav !== _lastNav) {
            _lastNav = nav;
            addLog('NAV', nav);
        }
    }

    if (typeof msg.linear_velocity === 'number') {
        const v = msg.linear_velocity;
        if (statVel) statVel.textContent = v.toFixed(2);
        if (velBar) velBar.style.width = `${Math.min(100, Math.abs(v) / Math.max(0.1, maxLinear) * 100)}%`;
        if (driveVel) driveVel.textContent = `${v.toFixed(2)} m/s`;
    }

    if (typeof msg.angular_velocity === 'number') {
        const a = msg.angular_velocity;
        if (statAng) statAng.textContent = a.toFixed(2);
        if (angBar) angBar.style.width = `${Math.min(100, Math.abs(a) / Math.max(0.1, maxAngular) * 100)}%`;
    }

    if (msg.pose) {
        const p = msg.pose;
        const yawDeg = (p.yaw * 180 / Math.PI);
        const yawNorm = ((yawDeg % 360) + 360) % 360;
        if (poseX)  poseX.textContent  = (p.x >= 0 ? '+' : '') + p.x.toFixed(3);
        if (poseY)  poseY.textContent  = (p.y >= 0 ? '+' : '') + p.y.toFixed(3);
        if (poseTh) poseTh.textContent = `${yawDeg.toFixed(1)}°`;
        if (statHead) statHead.textContent = yawNorm.toFixed(1);
        // 0° heading = North (up). CSS rotate is clockwise so the needle
        // wrapper rotates directly by yaw. The translate(-50%,-50%) keeps
        // the pivot at the compass centre.
        if (headNeedle) headNeedle.style.transform = `translate(-50%, -50%) rotate(${yawNorm}deg)`;
    }

    // Pi Metrics
    if (msg.pi_cpu !== undefined) {
        const val = `${msg.pi_cpu.toFixed(0)}%`;
        const pillVal = document.getElementById('pill-pi-val');
        if (pillVal) pillVal.textContent = val;
        const modalVal = document.getElementById('modal-pi-cpu');
        if (modalVal) modalVal.textContent = val;
    }
    if (msg.pi_mem !== undefined) {
        const modalVal = document.getElementById('modal-pi-mem');
        if (modalVal) modalVal.textContent = `${msg.pi_mem.toFixed(0)}%`;
    }
    if (msg.pi_temp !== undefined) {
        const modalVal = document.getElementById('modal-pi-temp');
        if (modalVal) modalVal.textContent = `${msg.pi_temp.toFixed(0)}°C`;
    }
    if (msg.pi_ip !== undefined) {
        const modalVal = document.getElementById('modal-pi-ip');
        if (modalVal) modalVal.textContent = msg.pi_ip;
    }

    // Jetson Metrics
    if (msg.jetson_cpu !== undefined) {
        const val = `${msg.jetson_cpu.toFixed(0)}%`;
        const pillVal = document.getElementById('pill-jetson-val');
        if (pillVal) pillVal.textContent = val;
        const modalVal = document.getElementById('modal-jetson-cpu');
        if (modalVal) modalVal.textContent = val;
    }
    if (msg.jetson_gpu !== undefined) {
        const modalVal = document.getElementById('modal-jetson-gpu');
        if (modalVal) modalVal.textContent = `${msg.jetson_gpu.toFixed(0)}%`;
    }
    if (msg.jetson_mem !== undefined) {
        const modalVal = document.getElementById('modal-jetson-mem');
        if (modalVal) modalVal.textContent = `${msg.jetson_mem.toFixed(0)}%`;
        const modalGb = document.getElementById('modal-jetson-mem-gb');
        if (modalGb && msg.jetson_mem_used !== undefined && msg.jetson_mem_total !== undefined) {
            modalGb.textContent = `${msg.jetson_mem_used.toFixed(1)} / ${msg.jetson_mem_total.toFixed(1)} GB`;
        }
    }
    if (msg.jetson_temp !== undefined) {
        const modalVal = document.getElementById('modal-jetson-temp');
        if (modalVal) modalVal.textContent = `${msg.jetson_temp.toFixed(0)}°C`;
    }
    if (msg.jetson_ip !== undefined) {
        const modalVal = document.getElementById('modal-jetson-ip');
        if (modalVal) modalVal.textContent = msg.jetson_ip;
    }

    // Battery (from Pi)
    if (msg.battery_percentage !== undefined) {
        const modalVal = document.getElementById('modal-pi-batt');
        if (modalVal) modalVal.textContent = `${msg.battery_percentage.toFixed(0)}%`;
    }
    if (msg.battery_voltage !== undefined) {
        const modalVal = document.getElementById('modal-pi-volt');
        if (modalVal) modalVal.textContent = `${msg.battery_voltage.toFixed(2)}V`;
    }
}

function fetchLocations() {
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/locations`)
        .then(r => r.json())
        .then(renderLocations)
        .catch(err => console.error('Failed to fetch locations:', err));
}

function renderLocations(locs) {
    const wpCount = document.getElementById('wp-count');
    const entries = Object.entries(locs || {});
    if (wpCount) wpCount.textContent = entries.length;
    if (!entries.length) {
        locList.innerHTML = '<div class="empty-msg" style="color:var(--vn-fg-muted);font-size:var(--vn-fs-small);text-align:center;padding:20px;">No waypoints saved</div>';
        return;
    }
    locList.innerHTML = '';
    for (const [name, data] of entries) {
        const yawDeg = ((((data.yaw ?? 0) * 180 / Math.PI) % 360) + 360) % 360;
        const safe = String(name).replace(/[&<>"]/g, c => ({ '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;' }[c]));
        const row = document.createElement('div');
        row.className = 'loc-row';
        row.innerHTML = `
            <div class="loc-icon"><svg class="ico"><use href="#i-pin"/></svg></div>
            <div class="loc-main">
                <span class="loc-name">${safe}</span>
                <span class="loc-coords text-mono">X ${data.x.toFixed(2)} · Y ${data.y.toFixed(2)} · θ ${yawDeg.toFixed(0).padStart(3,'0')}°</span>
            </div>
            <button class="icon-btn icon-btn-go" title="Go" data-act="go"><svg class="ico"><use href="#i-chevron"/></svg></button>
            <button class="icon-btn" title="Rename" data-act="edit"><svg class="ico"><use href="#i-pencil"/></svg></button>
            <button class="icon-btn icon-btn-danger" title="Delete" data-act="del"><svg class="ico"><use href="#i-trash"/></svg></button>
        `;
        row.querySelector('.loc-main').onclick = () => navigateTo(name);
        row.querySelector('[data-act="go"]').onclick = () => navigateTo(name);
        row.querySelector('[data-act="edit"]').onclick = () => editLocationName(name);
        row.querySelector('[data-act="del"]').onclick = () => deleteLocation(name);
        locList.appendChild(row);
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

function joystickGeom(zone) {
    // Use offsetWidth/Height — these are layout-only and ignore any CSS
    // transform (scale/rotate) applied by the entry animation. Using
    // getBoundingClientRect() here returns the *visually* transformed size
    // and produces an off-center handle.
    const w = zone.offsetWidth  || 200;
    const h = zone.offsetHeight || 200;
    return {
        size: Math.max(120, Math.min(240, Math.round(Math.min(w, h) * 0.78))),
        cx: `${Math.round(w / 2)}px`,
        cy: `${Math.round(h / 2)}px`,
    };
}

// ── Teleop ──
function initJoysticks() {
    if (linearManager) linearManager.destroy();
    if (angularManager) angularManager.destroy();
    const lz = document.getElementById('joystick-linear');
    const az = document.getElementById('joystick-angular');
    lz.innerHTML = '';
    az.innerHTML = '';
    const lg = joystickGeom(lz);
    const ag = joystickGeom(az);

    linearManager = nipplejs.create({
        zone: lz,
        mode: 'static',
        position: { left: lg.cx, top: lg.cy },
        color: '#ffffff',
        size: lg.size,
        threshold: 0.1
        // No lockY: handle rotates 360°. Velocity below uses vector.y only.
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
        zone: az,
        mode: 'static',
        position: { left: ag.cx, top: ag.cy },
        color: '#ffffff',
        size: ag.size,
        threshold: 0.1
        // No lockX: handle rotates 360°. Velocity below uses vector.x only.
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
    toast.style.borderColor = success ? 'var(--vn-success)' : 'var(--vn-danger)';
    toast.classList.add('active');
    setTimeout(() => toast.classList.remove('active'), 3000);
}

// ── Listeners ────────────────────────────────────────────────────────────────
let driveResizeTimer = null;
function reinitJoysticksDeferred() {
    if (!driveOverlay.classList.contains('active')) return;
    clearTimeout(driveResizeTimer);
    driveResizeTimer = setTimeout(initJoysticks, 180);
}
window.addEventListener('resize', reinitJoysticksDeferred);
window.addEventListener('orientationchange', reinitJoysticksDeferred);

enterDriveBtn.onclick = () => {
    driveOverlay.classList.add('active');
    // Wait for joystick entry animation (460ms delay + 340ms = ~800ms) so
    // nipplejs reads the final layout, not a mid-animation transformed one.
    setTimeout(initJoysticks, 820);
};

closeDriveBtn.onclick = () => {
    driveOverlay.classList.remove('active');
    if (linearManager) { linearManager.destroy(); linearManager = null; }
    if (angularManager) { angularManager.destroy(); angularManager = null; }
    if (cmdInterval) { clearInterval(cmdInterval); cmdInterval = null; }
};

maxSpeedSlider.oninput = () => {
    maxLinear = parseFloat(maxSpeedSlider.value);
    maxSpeedVal.textContent = maxLinear.toFixed(2);
};

micBtn.onmousedown = micBtn.ontouchstart = (e) => { e.preventDefault(); startRecording(); };
micBtn.onmouseup = micBtn.onmouseleave = micBtn.ontouchend = (e) => { e.preventDefault(); stopRecording(); };

if (chatSendBtn) {
    chatSendBtn.onclick = (e) => {
        e.preventDefault();
        handleManualSend();
    };
}
if (chatInput) {
    chatInput.onkeydown = (e) => {
        if (e.key === 'Enter') {
            e.preventDefault();
            handleManualSend();
        }
    };
}

window.addEventListener('keydown', (e) => {
    if (e.code === 'Space' && !e.repeat && document.activeElement.tagName !== 'INPUT') {
        e.preventDefault();
        if (!micBtn.disabled) startRecording();
    }
});

window.addEventListener('keyup', (e) => {
    if (e.code === 'Space' && document.activeElement.tagName !== 'INPUT') {
        e.preventDefault();
        stopRecording();
    }
});

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

// ── Top safety buttons ──────────────────────────────────────────────────────
let estopActive = false;
function setEstop(on) {
    estopActive = on;
    estopBtn?.classList.toggle('active', on);
    if (resumeBtn) resumeBtn.disabled = !on;
}
estopBtn?.addEventListener('click', () => {
    sendWS({ type: 'stop' });
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/navigate/cancel`, { method: 'POST' }).catch(() => {});
    setEstop(true);
    addLog('WARN', 'E-STOP engaged');
    showToast('E-STOP engaged', false);
});
resumeBtn?.addEventListener('click', () => {
    if (!estopActive) return;
    setEstop(false);
    addLog('OK', 'E-STOP cleared · resume');
    showToast('Resumed', true);
});
stopTopBtn?.addEventListener('click', () => {
    sendWS({ type: 'stop' });
    fetch(`${location.protocol}//${location.hostname}:${APP_PORT}/api/navigate/cancel`, { method: 'POST' }).catch(() => {});
    addLog('NAV', 'Motion halted');
});

clearChatBtn?.addEventListener('click', () => {
    if (confirm('Clear conversation history?')) {
        if (voiceWs && voiceWs.readyState === WebSocket.OPEN) {
            voiceWs.send(JSON.stringify({ type: 'clear_chat' }));
        }
        chat.innerHTML = '';
        if (chatEmpty) chat.appendChild(chatEmpty);
        addLog('SYS', 'Conversation history cleared');
    }
});

// ── Map / Scan tabs ─────────────────────────────────────────────────────────
document.querySelectorAll('.tab').forEach(btn => {
    btn.onclick = () => {
        const t = btn.dataset.tab;
        document.querySelectorAll('.tab').forEach(b => b.classList.toggle('active', b === btn));
        document.querySelectorAll('.tab-pane').forEach(p => p.classList.toggle('active', p.dataset.pane === t));
        document.querySelectorAll('.tab-meta').forEach(m => { m.hidden = (m.dataset.tab !== t); });
    };
});

// ── Audio config popover ────────────────────────────────────────────────────
const audioCfgBtn = document.getElementById('audio-cfg-btn');
const audioPop    = document.getElementById('audio-cfg-pop');
const volSlider   = document.getElementById('volume-slider');
const volVal      = document.getElementById('vol-val');
const targetBtns  = document.querySelectorAll('.target-btn');
const apiBase     = `${location.protocol}//${location.hostname}:${APP_PORT}`;

function postAudioConfig(patch) {
    return fetch(`${apiBase}/api/audio`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(patch),
    }).then(r => r.json()).catch(() => null);
}
function applyAudioConfig({ volume, target } = {}) {
    if (typeof volume === 'number') {
        const pct = Math.round(volume * 100);
        if (volSlider) volSlider.value = pct;
        if (volVal) volVal.textContent = `${pct}%`;
        browserAudio.volume = Math.min(1.0, volume);
    }
    if (target) targetBtns.forEach(b => b.classList.toggle('active', b.dataset.target === target));
}
fetch(`${apiBase}/api/audio`).then(r => r.json()).then(applyAudioConfig).catch(() => {});

audioCfgBtn?.addEventListener('click', (e) => {
    e.stopPropagation();
    audioPop.hidden = !audioPop.hidden;
});
document.addEventListener('click', (e) => {
    if (!audioPop || audioPop.hidden) return;
    if (!audioPop.contains(e.target) && e.target !== audioCfgBtn) audioPop.hidden = true;
});

let _volTimer = null;
volSlider?.addEventListener('input', () => {
    const pct = parseInt(volSlider.value, 10);
    volVal.textContent = `${pct}%`;
    browserAudio.volume = Math.min(1.0, pct / 100);
    clearTimeout(_volTimer);
    _volTimer = setTimeout(() => postAudioConfig({ volume: pct / 100 }), 150);
});
targetBtns.forEach(btn => {
    btn.onclick = () => {
        targetBtns.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        postAudioConfig({ target: btn.dataset.target });
    };
});

// ── Live RTT ────────────────────────────────────────────────────────────────
const rttEl = document.getElementById('dock-rtt');
async function pingRTT() {
    if (document.hidden) return;
    const t0 = performance.now();
    try {
        const r = await fetch(`${apiBase}/api/audio`, { cache: 'no-store' });
        if (!r.ok) throw 0;
        await r.json();
        const dt = Math.round(performance.now() - t0);
        if (rttEl) rttEl.textContent = dt;
    } catch {
        if (rttEl) rttEl.textContent = '—';
    }
}
pingRTT();
setInterval(pingRTT, 3000);

// Initial seed log
addLog('SYS', 'Console online');

// ── Live Map Renderer ─────────────────────────────────────────────────────────
(function () {
    const canvas  = document.getElementById('map-canvas');
    const noData  = document.getElementById('map-no-data');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = 'high';
    let _img = null;
    let _meta = null;
    let _retryTimer = null;

    function getSize() {
        let el = canvas.parentElement;
        while (el) {
            if (el.clientWidth > 0 && el.clientHeight > 0) return [el.clientWidth, el.clientHeight];
            el = el.parentElement;
        }
        return [0, 0];
    }

    function resize() {
        const [w, h] = getSize();
        if (w === 0 || h === 0) return;
        canvas.width  = w;
        canvas.height = h;
        // Re-apply smoothing after resize (canvas reset clears context state)
        ctx.imageSmoothingEnabled = true;
        ctx.imageSmoothingQuality = 'high';
        if (_img) draw();
    }

    function draw() {
        if (!_img || !_meta) return;
        const [w, h] = getSize();
        if (w === 0 || h === 0) return;
        if (canvas.width !== w || canvas.height !== h) {
            canvas.width = w; canvas.height = h;
            ctx.imageSmoothingEnabled = true;
            ctx.imageSmoothingQuality = 'high';
        }
        const { width: mw, height: mh, resolution, origin_x, origin_y, robot_x, robot_y, robot_yaw } = _meta;
        const cw = canvas.width, ch = canvas.height;

        const scale = Math.min(cw / mw, ch / mh);
        const dx = (cw - mw * scale) / 2;
        const dy = (ch - mh * scale) / 2;

        // Dark background for unknown/empty areas outside map bounds
        ctx.fillStyle = '#10121a';
        ctx.fillRect(0, 0, cw, ch);
        ctx.drawImage(_img, dx, dy, mw * scale, mh * scale);

        // Robot marker — fixed pixel size so it's always visible
        if (resolution > 0) {
            // image is 2× upscaled: each image pixel = resolution/2 metres
            const pxPerMetre = 2 / resolution;
            const px = dx + (robot_x - origin_x) * pxPerMetre * scale;
            const py = dy + (mh - (robot_y - origin_y) * pxPerMetre) * scale;

            // Direction arrow (fixed 12px)
            ctx.save();
            ctx.translate(px, py);
            ctx.rotate(-robot_yaw);
            ctx.beginPath();
            ctx.moveTo(0, -12);
            ctx.lineTo(6, 6);
            ctx.lineTo(-6, 6);
            ctx.closePath();
            ctx.fillStyle = '#00e5ff';
            ctx.shadowColor = '#00e5ff';
            ctx.shadowBlur = 6;
            ctx.fill();
            ctx.restore();

            // Centre dot
            ctx.beginPath();
            ctx.arc(px, py, 5, 0, Math.PI * 2);
            ctx.fillStyle = '#ffffff';
            ctx.shadowColor = '#00e5ff';
            ctx.shadowBlur = 8;
            ctx.fill();
            ctx.shadowBlur = 0;
        }
    }

    window.__vnMap = {
        update(msg) {
            if (noData) noData.hidden = true;
            _meta = msg;
            stopMapPoll();
            const image = new Image();
            image.onload = () => {
                _img = image;
                draw();
                if (_retryTimer) clearInterval(_retryTimer);
                const [w] = getSize();
                if (w === 0) {
                    _retryTimer = setInterval(() => {
                        const [w2] = getSize();
                        if (w2 > 0) { clearInterval(_retryTimer); _retryTimer = null; resize(); draw(); }
                    }, 200);
                }
            };
            image.src = 'data:image/png;base64,' + msg.img;
        },
        clear() {
            _img = null; _meta = null;
            if (_retryTimer) { clearInterval(_retryTimer); _retryTimer = null; }
            ctx.fillStyle = '#10121a';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            if (noData) noData.hidden = false;
        }
    };

    new IntersectionObserver((entries) => {
        if (entries[0].isIntersecting && _img) { resize(); draw(); }
    }, { threshold: 0.1 }).observe(canvas);

    new ResizeObserver(resize).observe(canvas.parentElement);
    resize();
})();

// ── Mode Manager ─────────────────────────────────────────────────────────────

let _currentMode = 'nav';
let _modeSwitching = false;
let _selectedMap = 'custom_map';

const _modeBtnNav    = document.getElementById('mode-btn-nav');
const _modeBtnSlam   = document.getElementById('mode-btn-slam');
const _mapCarouselWrap = document.getElementById('map-carousel-wrap');
const _mapCarousel   = document.getElementById('map-carousel');
const _carouselPrev  = document.getElementById('carousel-prev');
const _carouselNext  = document.getElementById('carousel-next');
const _slamControls  = document.getElementById('slam-controls');
const _slamMapName   = document.getElementById('slam-map-name');
const _saveMapBtn    = document.getElementById('save-map-btn');
const _loadMapBtn    = document.getElementById('load-map-btn');
const _modeSwitchEl  = document.getElementById('mode-switching');
const _mapModeBadge  = document.getElementById('map-mode-badge');
const _mapTabBadge   = document.getElementById('map-tab-badge');

function renderMapCarousel(maps) {
    if (!_mapCarousel || !maps || !maps.length) return;
    _mapCarousel.innerHTML = '';
    maps.forEach(m => {
        const name = m.name || m;
        const card = document.createElement('div');
        card.className = 'map-card' + (name === _selectedMap ? ' active' : '');
        card.dataset.map = name;
        card.innerHTML = `<img class="map-card-img" src="${apiBase}/api/maps/${encodeURIComponent(name)}/image" loading="lazy" alt="${name}"><span class="map-card-name">${name}</span>`;
        card.addEventListener('click', () => selectMap(name));
        _mapCarousel.appendChild(card);
    });
    updateCarouselArrows();
}

function selectMap(name) {
    _selectedMap = name;
    _mapCarousel?.querySelectorAll('.map-card').forEach(c => {
        c.classList.toggle('active', c.dataset.map === name);
    });
}

function updateCarouselArrows() {
    if (!_mapCarousel || !_carouselPrev || !_carouselNext) return;
    _carouselPrev.disabled = _mapCarousel.scrollLeft <= 0;
    _carouselNext.disabled = _mapCarousel.scrollLeft + _mapCarousel.clientWidth >= _mapCarousel.scrollWidth - 2;
}

_carouselPrev?.addEventListener('click', () => {
    _mapCarousel?.scrollBy({ left: -100, behavior: 'smooth' });
});
_carouselNext?.addEventListener('click', () => {
    _mapCarousel?.scrollBy({ left: 100, behavior: 'smooth' });
});
_mapCarousel?.addEventListener('scroll', updateCarouselArrows);

function applyModeUI(mode, maps, activeMap) {
    _currentMode = mode;
    _modeBtnNav?.classList.toggle('active', mode === 'nav');
    _modeBtnSlam?.classList.toggle('active', mode === 'slam');
    if (_mapCarouselWrap) _mapCarouselWrap.hidden = (mode !== 'nav');
    if (_slamControls) _slamControls.hidden = (mode !== 'slam');
    if (_mapModeBadge) _mapModeBadge.textContent = mode.toUpperCase();
    if (_mapTabBadge) _mapTabBadge.textContent = mode === 'slam' ? 'SLAM · MAPPING' : 'NAV · ACTIVE';
    if (maps) renderMapCarousel(maps);
    const nameEl = document.getElementById('current-map-name');
    if (nameEl) nameEl.textContent = activeMap || _selectedMap || 'no map';
}

function setModeSwitching(on) {
    _modeSwitching = on;
    if (_modeSwitchEl) _modeSwitchEl.hidden = !on;
    if (_modeBtnNav)  _modeBtnNav.disabled  = on;
    if (_modeBtnSlam) _modeBtnSlam.disabled  = on;
}

async function switchMode(mode) {
    if (_modeSwitching || mode === _currentMode) return;
    setModeSwitching(true);
    addLog('SYS', `Switching to ${mode.toUpperCase()}…`);
    if (window.__vnMap) window.__vnMap.clear();
    const body = { mode };
    if (mode === 'nav' && _selectedMap) body.map = _selectedMap;
    try {
        const res = await fetch(`${apiBase}/api/mode`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
        }).then(r => r.json());
        if (res.success) {
            applyModeUI(res.mode, null, res.map || null);
            showToast(`Switched to ${res.mode.toUpperCase()} mode`, true);
            addLog('OK', `Mode: ${res.mode.toUpperCase()}`);
            startMapPoll();
        } else {
            showToast(res.error || 'Mode switch failed', false);
        }
    } catch (e) {
        showToast('Mode switch error', false);
    } finally {
        setModeSwitching(false);
    }
}

async function loadMap() {
    if (!_selectedMap) { showToast('Select a map first', false); return; }
    if (_modeSwitching) return;
    setModeSwitching(true);
    addLog('SYS', `Loading map: ${_selectedMap}…`);
    if (window.__vnMap) window.__vnMap.clear();
    try {
        const res = await fetch(`${apiBase}/api/mode`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ mode: 'nav', map: _selectedMap }),
        }).then(r => r.json());
        if (res.success) {
            applyModeUI(res.mode, null, res.map || _selectedMap);
            showToast(`Map loaded: ${res.map || _selectedMap}`, true);
            addLog('OK', `Map loaded: ${res.map || _selectedMap}`);
            startMapPoll();
        } else {
            showToast(res.error || 'Load failed', false);
        }
    } catch (e) {
        showToast('Load map error', false);
    } finally {
        setModeSwitching(false);
    }
}

_loadMapBtn?.addEventListener('click', loadMap);

async function saveMap() {
    const name = (_slamMapName?.value || '').trim();
    if (!name) { showToast('Enter a map name', false); return; }
    if (!/^[a-zA-Z0-9_-]+$/.test(name)) { showToast('Name: letters, digits, _ - only', false); return; }
    if (_saveMapBtn) _saveMapBtn.disabled = true;
    addLog('SYS', `Saving map '${name}'…`);
    try {
        const res = await fetch(`${apiBase}/api/maps/save`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name }),
        }).then(r => r.json());
        if (res.success) {
            showToast(`Map '${name}' saved`, true);
            addLog('OK', `Map saved: ${name}`);
            if (_slamMapName) _slamMapName.value = '';
            fetchMapsAndMode();
        } else {
            showToast(res.error || 'Save failed', false);
        }
    } catch (e) {
        showToast('Map save error', false);
    } finally {
        if (_saveMapBtn) _saveMapBtn.disabled = false;
    }
}

async function fetchMapsAndMode() {
    try {
        const res = await fetch(`${apiBase}/api/mode`).then(r => r.json());
        if (res.map) _selectedMap = res.map;
        applyModeUI(res.mode, res.maps.map(n => ({ name: n })), res.map);
    } catch (e) { /* silent */ }
}

_modeBtnNav?.addEventListener('click', () => switchMode('nav'));
_modeBtnSlam?.addEventListener('click', () => switchMode('slam'));
_saveMapBtn?.addEventListener('click', saveMap);
_slamMapName?.addEventListener('keydown', e => { if (e.key === 'Enter') saveMap(); });

document.getElementById('map-refresh-btn')?.addEventListener('click', () => {
    const ico = document.getElementById('map-refresh-ico');
    if (ico) { ico.style.transition = 'transform 0.6s'; ico.style.transform = 'rotate(360deg)'; setTimeout(() => { ico.style.transition = ''; ico.style.transform = ''; }, 650); }
    if (bridgeWs && bridgeWs.readyState === WebSocket.OPEN) {
        bridgeWs.send(JSON.stringify({ type: 'get_map' }));
    }
});

// ── Init ─────────────────────────────────────────────────────────────────────
connectVoice();
connectBridge();
fetchMapsAndMode();
