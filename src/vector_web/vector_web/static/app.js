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
    if (typeof pct === 'number') {
        if (window.__vnSetBattery) window.__vnSetBattery(pct);
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
        if (headNeedle) headNeedle.style.transform = `rotate(${yawNorm}deg)`;
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
    const r = zone.getBoundingClientRect();
    const w = r.width || 200, h = r.height || 200;
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
        zone: az,
        mode: 'static',
        position: { left: ag.cx, top: ag.cy },
        color: '#ffffff',
        size: ag.size,
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
    setTimeout(initJoysticks, 220);
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

// ── Init ─────────────────────────────────────────────────────────────────────
connectVoice();
connectBridge();
