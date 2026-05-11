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
    } else {
        console.warn('[VOX] WebSocket not open, cannot send:', text);
        addLog('ERR', 'Voice channel disconnected');
    }
}

function handleManualSend() {
    if (state === 'recording') return; // Don't interrupt recording
    const text = chatInput.value.trim();
    if (!text) return;
    
    // Add to local chat immediately
    addMessage('user', text);
    sendText(text);
    
    chatInput.value = '';
    setState('processing');
}

chatSendBtn.onclick = (e) => {
    e.preventDefault();
    handleManualSend();
};

chatInput.onkeydown = (e) => {
    if (e.key === 'Enter') {
        e.preventDefault();
        handleManualSend();
    }
};

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
