/**
 * Vector Nav — FPV / HUD overlay glue.
 * - Reparents the existing lidar + map canvases into the HUD mini-window
 *   on entry, restores on exit. No data pipelines change.
 * - Wires the HUD voice button to the same start/stopRecording flow.
 * - Wires HUD halt, tab switcher, clock, and per-frame telemetry mirror.
 *
 * Loads AFTER app.js so it can attach extra listeners (vs overriding .onclick).
 */
(() => {
  const overlay   = document.getElementById('drive-overlay');
  if (!overlay) return;

  const enterBtn  = document.getElementById('enter-drive-btn');
  const exitBtn   = document.getElementById('close-drive');
  const miniStage = document.getElementById('hud-mini-stage');
  const camVideo   = document.getElementById('drive-camera-video');
  const camFallback = document.querySelector('.drive-camera-fallback');
  const hudMic    = document.getElementById('hud-mic-btn');
  const hudHalt   = document.getElementById('hud-halt');
  const dashMic   = document.getElementById('micBtn');
  const hudTabs   = overlay.querySelectorAll('.hud-tab');

  // Telemetry / control mirror destinations
  const hudVel       = document.getElementById('hud-vel');
  const hudHeading   = document.getElementById('hud-heading');
  const hudPoseX     = document.getElementById('hud-pose-x');
  const hudPoseY     = document.getElementById('hud-pose-y');
  const hudPoseTh    = document.getElementById('hud-pose-th');
  const hudBattPct   = document.getElementById('hud-batt-pct');
  const hudBattFill  = document.getElementById('hud-batt-fill');
  const hudBattTrack = hudBattFill?.parentElement;
  const hudThr       = document.getElementById('hud-thr-fill');
  const hudYaw       = document.getElementById('hud-yaw-fill');
  const hudStickL    = document.getElementById('hud-stick-l-val');
  const hudStickR    = document.getElementById('hud-stick-r-val');
  const hudDriveMode = document.getElementById('hud-drive-mode');
  const hudClock     = document.getElementById('hud-clock');
  const hudSession   = document.getElementById('hud-session');
  const hudLogList   = document.getElementById('hud-log-list');
  const hudMiniMeta  = document.getElementById('hud-mini-meta');

  // Where canvases live in the dashboard, so we can put them back.
  const lidarCanvas = document.getElementById('lidar-canvas');
  const mapCanvas   = document.getElementById('map-canvas');
  const lidarHome   = lidarCanvas?.parentElement;
  const mapHome     = mapCanvas?.parentElement;

  // ── WebRTC camera ─────────────────────────────────────────────────
  let _cameraPc = null;

  async function cameraConnect() {
    if (_cameraPc) return;
    _cameraPc = new RTCPeerConnection({ iceServers: [], iceTransportPolicy: 'all' });

    _cameraPc.ontrack = (e) => {
      if (e.track.kind === 'video' && camVideo) {
        camVideo.srcObject = e.streams[0];
        const show = () => {
          camVideo.classList.add('active');
          if (camFallback) camFallback.style.display = 'none';
        };
        camVideo.onloadedmetadata = show;
        camVideo.oncanplay = show;
      }
    };

    _cameraPc.onconnectionstatechange = () => {
      if (_cameraPc?.connectionState === 'connected') {
        _cameraPc.getReceivers().forEach(r => {
          if (r.track.kind === 'video') r.jitterBufferTarget = 0;
        });
      }
      if (_cameraPc && (_cameraPc.connectionState === 'failed' || _cameraPc.connectionState === 'disconnected')) {
        cameraDisconnect();
        setTimeout(cameraConnect, 3000);
      }
    };

    _cameraPc.addTransceiver('video', { direction: 'recvonly' });
    const offer = await _cameraPc.createOffer();
    await _cameraPc.setLocalDescription(offer);

    await new Promise(resolve => {
      if (_cameraPc.iceGatheringState === 'complete') return resolve();
      _cameraPc.onicegatheringstatechange = () => { if (_cameraPc?.iceGatheringState === 'complete') resolve(); };
      setTimeout(resolve, 2000);
    });

    try {
      const res = await fetch('/camera/offer', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ sdp: _cameraPc.localDescription.sdp, type: _cameraPc.localDescription.type }),
      });
      if (!res.ok) throw new Error('offer failed');
      const answer = await res.json();
      await _cameraPc.setRemoteDescription(answer);
    } catch {
      cameraDisconnect();
      setTimeout(cameraConnect, 3000);
    }
  }

  function cameraDisconnect() {
    if (_cameraPc) { _cameraPc.close(); _cameraPc = null; }
    if (camVideo)  { camVideo.srcObject = null; camVideo.classList.remove('active'); }
    if (camFallback) camFallback.style.display = '';
  }

  // ── State ─────────────────────────────────────────────────────────
  let active = false;
  let rafId = null;
  let sessionStart = 0;
  let activeMini = 'scan';

  let _hudMode = 'unknown';
  let _hudSwitching = false;

  function setActiveMini(which) {
    activeMini = which;
    hudTabs.forEach(t => t.classList.toggle('active', t.dataset.hudTab === which));
    if (lidarCanvas) lidarCanvas.style.display = (which === 'scan') ? 'block' : 'none';
    if (mapCanvas)   mapCanvas.style.display   = (which === 'map')  ? 'block' : 'none';
    if (hudMiniMeta) hudMiniMeta.textContent = (which === 'scan')
        ? 'RPLIDAR · 12m'
        : (_hudMode === 'slam' ? 'SLAM · MAPPING' : (_hudMode === 'nav' ? 'NAV · ACTIVE' : 'NO MAP'));
    // Force map renderer to re-measure after visibility flip
    if (which === 'map' && window.__vnMap?.resize) {
      requestAnimationFrame(() => window.__vnMap.resize());
    }
  }
  hudTabs.forEach(t => t.addEventListener('click', () => setActiveMini(t.dataset.hudTab)));

  // ── HUD NAV/SLAM mode buttons ────────────────────────────────────
  const hudModeNav  = document.getElementById('hud-mode-nav');
  const hudModeSlam = document.getElementById('hud-mode-slam');
  const hudHaltBtn  = document.getElementById('hud-halt');

  // ── HUD save-map UI ──────────────────────────────────────────────
  const hudSavemapPill  = document.getElementById('hud-savemap-pill');
  const hudSavemapInput = document.getElementById('hud-savemap-input');
  const hudSavemapBtn   = document.getElementById('hud-savemap-btn');

  function applyHudUiForMode() {
    // NAV/SLAM toggle visual state + disabled
    hudModeNav?.classList.toggle('active', _hudMode === 'nav');
    hudModeSlam?.classList.toggle('active', _hudMode === 'slam');
    if (hudModeNav)  hudModeNav.disabled  = _hudSwitching;
    if (hudModeSlam) hudModeSlam.disabled = _hudSwitching;
    // Save-map pill visibility — only in SLAM, not while switching
    if (hudSavemapPill) hudSavemapPill.hidden = !(_hudMode === 'slam' && !_hudSwitching);
    // Halt only meaningful in NAV; disable in SLAM/switching
    if (hudHaltBtn) {
      hudHaltBtn.disabled = (_hudMode !== 'nav') || _hudSwitching;
      hudHaltBtn.style.opacity = hudHaltBtn.disabled ? '0.45' : '';
      hudHaltBtn.style.cursor  = hudHaltBtn.disabled ? 'not-allowed' : '';
    }
    // Update mini meta if map tab is showing
    if (activeMini === 'map' && hudMiniMeta) {
      hudMiniMeta.textContent = _hudMode === 'slam' ? 'SLAM · MAPPING' : (_hudMode === 'nav' ? 'NAV · ACTIVE' : 'NO MAP');
    }
  }

  hudModeNav?.addEventListener('click', () => {
    if (_hudSwitching) return;
    if (typeof window.switchMode === 'function') window.switchMode('nav');
  });
  hudModeSlam?.addEventListener('click', () => {
    if (_hudSwitching) return;
    if (typeof window.switchMode === 'function') window.switchMode('slam');
  });

  // Save-map handler — proxies to app.js saveMap() with HUD input
  hudSavemapBtn?.addEventListener('click', async () => {
    const name = (hudSavemapInput?.value || '').trim();
    if (!name) return;
    if (typeof window.saveMap === 'function') {
      const ok = await window.saveMap(name, {
        onBusy: (busy) => { if (hudSavemapBtn) hudSavemapBtn.disabled = busy; },
      });
      if (ok && hudSavemapInput) hudSavemapInput.value = '';
    }
  });
  hudSavemapInput?.addEventListener('keydown', e => {
    if (e.key === 'Enter') hudSavemapBtn?.click();
  });

  window.addEventListener('vectorModeChange', e => {
    _hudMode = e.detail.mode;
    applyHudUiForMode();
  });
  window.addEventListener('vectorModeSwitching', e => {
    _hudSwitching = e.detail.switching;
    applyHudUiForMode();
  });

  // ── Canvas reparenting ───────────────────────────────────────────
  function moveCanvasesToHud() {
    if (lidarCanvas && miniStage) miniStage.appendChild(lidarCanvas);
    if (mapCanvas   && miniStage) miniStage.appendChild(mapCanvas);
    setActiveMini(activeMini);
    // Two animation frames so layout settles, then force renderers to re-measure
    requestAnimationFrame(() => requestAnimationFrame(() => {
      window.dispatchEvent(new Event('resize'));
      if (window.__vnMap?.resize) window.__vnMap.resize();
    }));
  }
  function moveCanvasesHome() {
    if (lidarCanvas && lidarHome) {
      lidarCanvas.style.display = '';
      lidarHome.appendChild(lidarCanvas);
    }
    if (mapCanvas && mapHome) {
      mapCanvas.style.display = '';
      mapHome.appendChild(mapCanvas);
    }
    requestAnimationFrame(() => requestAnimationFrame(() => {
      window.dispatchEvent(new Event('resize'));
      if (window.__vnMap?.resize) window.__vnMap.resize();
    }));
  }

  // ── Telemetry mirror — read dashboard's already-updated DOM ──────
  function tick() {
    rafId = active ? requestAnimationFrame(tick) : null;
    // Read source DOM
    const vEl  = document.getElementById('stat-vel');
    const aEl  = document.getElementById('stat-ang');
    const pxEl = document.getElementById('pose-x');
    const pyEl = document.getElementById('pose-y');
    const pthEl= document.getElementById('pose-th');
    const battEl = document.getElementById('batt-pct');
    const battCell = document.getElementById('battery-cell');
    const velBarEl = document.getElementById('vel-bar');
    const angBarEl = document.getElementById('ang-bar');

    if (vEl  && hudVel)     hudVel.textContent     = vEl.textContent;
    if (pthEl && hudHeading) hudHeading.textContent = Math.round(parseFloat(pthEl.textContent) || 0).toString().padStart(3, '0');
    if (pxEl && hudPoseX)   hudPoseX.textContent   = pxEl.textContent;
    if (pyEl && hudPoseY)   hudPoseY.textContent   = pyEl.textContent;
    if (pthEl&& hudPoseTh)  hudPoseTh.textContent  = pthEl.textContent;
    if (battEl && hudBattPct) hudBattPct.textContent = battEl.textContent;
    if (hudBattFill && battEl) {
      const pct = Math.max(0, Math.min(100, parseFloat(battEl.textContent) || 0));
      hudBattFill.style.width = pct + '%';
      if (hudBattTrack) hudBattTrack.dataset.state = pct < 15 ? 'low' : pct < 35 ? 'warn' : 'ok';
    }
    if (velBarEl && hudThr) hudThr.style.width = velBarEl.style.width || '0%';
    if (angBarEl && hudYaw) {
      const w = parseFloat(angBarEl.style.width) || 0;
      // detect sign from numeric text (already signed)
      const a = parseFloat(aEl?.textContent || '0');
      hudYaw.classList.toggle('neg', a < 0);
      hudYaw.style.width = Math.min(50, w / 2) + '%';
    }
    if (hudStickL && vEl) hudStickL.textContent = parseFloat(vEl.textContent).toFixed(2);
    if (hudStickR && aEl) hudStickR.textContent = parseFloat(aEl.textContent).toFixed(2);

    // Clock + session
    if (hudClock) {
      const d = new Date();
      hudClock.textContent =
        String(d.getHours()).padStart(2,'0') + ':' +
        String(d.getMinutes()).padStart(2,'0') + ':' +
        String(d.getSeconds()).padStart(2,'0');
    }
    if (hudSession && sessionStart) {
      const s = Math.floor((Date.now() - sessionStart) / 1000);
      hudSession.textContent = String(Math.floor(s/60)).padStart(2,'0') + ':' + String(s%60).padStart(2,'0');
    }

    // Log mirror — copy the top N rows from dashboard's #log-list
    if (hudLogList) {
      const src = document.getElementById('log-list');
      if (src && src.children.length) {
        const sig = src.firstElementChild?.textContent;
        if (sig !== hudLogList.dataset.sig) {
          hudLogList.dataset.sig = sig || '';
          hudLogList.innerHTML = '';
          for (let i = 0; i < Math.min(4, src.children.length); i++) {
            hudLogList.appendChild(src.children[i].cloneNode(true));
          }
        }
      }
    }

    // Mode text — mirror nav badge
    if (hudDriveMode) {
      const nav = document.getElementById('stat-nav');
      if (nav) hudDriveMode.textContent = (nav.textContent || 'MANUAL').toUpperCase();
    }
  }

  // ── Voice mirror — when HUD voice button is pressed, dispatch the
  // same gestures to the dashboard mic button to keep one code path.
  function pressDashMic(down) {
    if (!dashMic) return;
    if (down && !dashMic.disabled) {
      dashMic.dispatchEvent(new MouseEvent('mousedown', { bubbles: true }));
    } else {
      dashMic.dispatchEvent(new MouseEvent('mouseup',   { bubbles: true }));
    }
  }
  let hudMicHeld = false;
  hudMic?.addEventListener('mousedown', (e) => { e.preventDefault(); hudMicHeld = true; hudMic.classList.add('recording'); pressDashMic(true); });
  hudMic?.addEventListener('touchstart', (e) => { e.preventDefault(); hudMicHeld = true; hudMic.classList.add('recording'); pressDashMic(true); }, { passive: false });
  const release = () => { if (!hudMicHeld) return; hudMicHeld = false; hudMic.classList.remove('recording'); pressDashMic(false); };
  hudMic?.addEventListener('mouseup', release);
  hudMic?.addEventListener('mouseleave', release);
  hudMic?.addEventListener('touchend', release);
  hudMic?.addEventListener('touchcancel', release);

  // Mirror dashboard mic .recording state → HUD mic + voice rings.
  const voiceWrap = document.querySelector('.hud-voice-wrap');
  const micObserver = new MutationObserver(() => {
    if (!dashMic) return;
    const recording = dashMic.classList.contains('recording');
    if (!hudMicHeld) hudMic?.classList.toggle('recording', recording);
    voiceWrap?.classList.toggle('live', recording);
  });
  if (dashMic) micObserver.observe(dashMic, { attributes: true, attributeFilter: ['class'] });

  // Halt button — proxy through dashboard's halt
  hudHalt?.addEventListener('click', () => {
    document.getElementById('stop-btn-top')?.click();
    document.getElementById('stop-nav-btn')?.click();
  });

  // ── Enter / Exit ─────────────────────────────────────────────────
  enterBtn?.addEventListener('click', () => {
    if (active) return;
    active = true;
    sessionStart = Date.now();
    // Call play() synchronously within the tap gesture so iOS Safari allows autoplay.
    if (camVideo) camVideo.play().catch(() => {});
    cameraConnect();
    // Defer reparent so the .active animation can read initial layout.
    requestAnimationFrame(() => requestAnimationFrame(moveCanvasesToHud));
    if (!rafId) tick();
  });

  exitBtn?.addEventListener('click', () => {
    if (!active) return;
    active = false;
    if (rafId) { cancelAnimationFrame(rafId); rafId = null; }
    moveCanvasesHome();
    cameraDisconnect();
  });

  // Esc to exit
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && active) exitBtn?.click();
  });
})();
