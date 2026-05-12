/**
 * Vector Nav — Visual effects layer.
 * - Animated battery (level + ETA, smooth interp)
 * - Beautiful spectrum: 64-bar mirrored bar visualizer with peak hold,
 *   driven by mic AnalyserNode when available, otherwise demo synth.
 * - Mic ring activation (.live class on .mic-wrap and .spectrum-stage)
 */
(() => {
  // ── Battery ───────────────────────────────────────────────
  const cell = document.getElementById('battery-cell');
  const pctEl = document.getElementById('batt-pct');
  const voltEl = document.getElementById('batt-voltage');
  const glowEl = document.getElementById('bat-glow');
  const baseFillEl = document.getElementById('bat-base-fill');
  const path1 = document.getElementById('bat-path-1');
  const path2 = document.getElementById('bat-path-2');
  const path3 = document.getElementById('bat-path-3');
  const path4 = document.getElementById('bat-path-4');

  let battTarget = parseFloat(cell?.dataset.pct || '87');
  let battShown = battTarget;
  let voltShown = 12.6;

  const innerW = 32, innerH = 76;
  const PERIOD = 36, AMP = 6;
  const REPS = Math.ceil(innerW / PERIOD) + 5;

  function makePath(yLevel, amp) {
    let d = `M 0,${yLevel}`;
    for (let i = 0; i < REPS; i++) {
      const x = i * PERIOD;
      d += ` Q ${x + PERIOD * 0.25},${yLevel - amp} ${x + PERIOD * 0.5},${yLevel}`;
      d += ` Q ${x + PERIOD * 0.75},${yLevel + amp} ${x + PERIOD},${yLevel}`;
    }
    d += ` V ${innerH + 4} H 0 Z`;
    return d;
  }

  function setBattery(pct, voltage) {
    battTarget = Math.max(0, Math.min(100, pct));
    if (voltage !== undefined) voltShown = voltage;
    cell.dataset.state = pct <= 20 ? 'low' : pct < 50 ? 'warn' : 'ok';
    if (pct <= 20) cell.classList.add('animate-pulse');
    else cell.classList.remove('animate-pulse');
  }

  function tickBattery() {
    battShown += (battTarget - battShown) * 0.08;
    if (pctEl) pctEl.textContent = battShown.toFixed(0);
    if (voltEl) voltEl.textContent = `${voltShown.toFixed(1)}V`;

    const pct = battShown;
    const fillY = innerH * (1 - Math.max(0, Math.min(100, pct)) / 100);
    const fillColor = pct > 50 ? "#22c55e" : pct > 20 ? "#eab308" : "#ef4444";

    if (glowEl) glowEl.setAttribute('fill', fillColor);
    if (baseFillEl) {
      baseFillEl.setAttribute('fill', fillColor);
      baseFillEl.setAttribute('y', 9 + fillY);
      baseFillEl.setAttribute('height', innerH - fillY + 4);
    }

    if (path1) { path1.setAttribute('d', makePath(fillY, AMP * 0.35)); path1.setAttribute('fill', fillColor); }
    if (path2) { path2.setAttribute('d', makePath(fillY, AMP * 0.60)); path2.setAttribute('fill', fillColor); }
    if (path3) { path3.setAttribute('d', makePath(fillY, AMP * 0.82)); path3.setAttribute('fill', fillColor); }
    if (path4) { path4.setAttribute('d', makePath(fillY, AMP));        path4.setAttribute('fill', fillColor); }

    requestAnimationFrame(tickBattery);
  }
  if (cell) {
    setBattery(battTarget, 12.6);
    tickBattery();
    window.__vnSetBattery = setBattery;
  }

  // ── Spectrum ──────────────────────────────────────────────
  const stage  = document.getElementById('spectrum-stage');
  const canvas = document.getElementById('spectrum-bars');
  const timeEl = document.getElementById('spec-time');
  const lvlEl  = document.getElementById('spec-level');
  if (!canvas) return;

  const ctx = canvas.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const N_BARS = 64;
  const peaks = new Array(N_BARS).fill(0);
  const targets = new Array(N_BARS).fill(0);
  const values = new Array(N_BARS).fill(0);

  let analyser = null;
  let analyserData = null;
  let live = false;
  let recStart = 0;

  function resize() {
    const r = canvas.getBoundingClientRect();
    canvas.width  = r.width  * dpr;
    canvas.height = r.height * dpr;
  }
  resize();
  window.addEventListener('resize', resize);

  // Demo signal — perlin-ish per-bin noise + gentle wave
  function demoFrame(t) {
    for (let i = 0; i < N_BARS; i++) {
      const k = i / N_BARS;
      const base = 0.10 + 0.06 * Math.sin(t * 0.6 + i * 0.18);
      const wob  = 0.04 * Math.sin(t * 1.7 + i * 0.4);
      targets[i] = Math.max(0, base + wob);
    }
  }

  function liveFrame() {
    analyser.getByteFrequencyData(analyserData);
    const len = analyserData.length;
    // Map FFT bins to N_BARS with log-ish spacing
    for (let i = 0; i < N_BARS; i++) {
      const lo = Math.floor(Math.pow(i / N_BARS, 1.6) * len);
      const hi = Math.floor(Math.pow((i + 1) / N_BARS, 1.6) * len);
      let sum = 0, n = 0;
      for (let j = lo; j < Math.max(hi, lo + 1) && j < len; j++) { sum += analyserData[j]; n++; }
      const v = (n ? sum / n : 0) / 255;
      targets[i] = Math.pow(v, 1.4);
    }
  }

  function draw(now) {
    requestAnimationFrame(draw);
    const t = now * 0.001;
    if (live && analyser) liveFrame(); else demoFrame(t);

    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    const gap = 3 * dpr;
    const bw  = (W - gap * (N_BARS - 1)) / N_BARS;
    const cy  = H / 2;

    // ambient grid line
    ctx.strokeStyle = 'rgba(255,255,255,0.04)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke();

    let totalLevel = 0;
    for (let i = 0; i < N_BARS; i++) {
      values[i] += (targets[i] - values[i]) * (live ? 0.35 : 0.18);
      const v = values[i];
      totalLevel += v;
      // peak hold
      if (v > peaks[i]) peaks[i] = v;
      else peaks[i] = Math.max(0, peaks[i] - 0.006);

      const h = Math.max(2 * dpr, v * (H * 0.46));
      const x = i * (bw + gap);

      const col = live
        ? `rgba(${250 + (i*0.05|0)}, ${183 - i * 0.6}, ${183 - i * 0.4}, ${0.55 + v * 0.4})`
        : `rgba(255, 255, 255, ${0.10 + v * 0.45})`;
      ctx.fillStyle = col;

      // mirror up + down with rounded caps
      const r = Math.min(bw / 2, 2 * dpr);
      roundRect(ctx, x, cy - h, bw, h, r); ctx.fill();
      roundRect(ctx, x, cy,     bw, h, r);
      ctx.globalAlpha = 0.45; ctx.fill(); ctx.globalAlpha = 1;

      // peak cap
      const ph = Math.max(2 * dpr, peaks[i] * (H * 0.46));
      ctx.fillStyle = live ? 'rgba(255,255,255,0.85)' : 'rgba(255,255,255,0.35)';
      ctx.fillRect(x, cy - ph - 2 * dpr, bw, 1.5 * dpr);
    }

    // readout
    const avg = totalLevel / N_BARS;
    if (live) {
      const elapsed = (performance.now() - recStart) / 1000;
      const mm = String(Math.floor(elapsed / 60)).padStart(2,'0');
      const ss = String(Math.floor(elapsed % 60)).padStart(2,'0');
      timeEl && (timeEl.textContent = `${mm}:${ss}`);
      const db = avg < 0.001 ? -60 : Math.max(-60, 20 * Math.log10(avg));
      lvlEl  && (lvlEl.textContent  = `${db > 0 ? '+' : ''}${db.toFixed(1)} dB`);
    } else {
      timeEl && (timeEl.textContent = '00:00');
      lvlEl  && (lvlEl.textContent  = '−∞ dB');
    }
  }

  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y,     x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x,     y + h, r);
    ctx.arcTo(x,     y + h, x,     y,     r);
    ctx.arcTo(x,     y,     x + w, y,     r);
    ctx.closePath();
  }

  requestAnimationFrame(draw);

  // ── Public API for app.js ─────────────────────────────────
  window.__vnSpectrum = {
    bind(an) {
      analyser = an;
      if (an) analyserData = new Uint8Array(an.frequencyBinCount);
    },
    start() {
      live = true;
      recStart = performance.now();
      stage?.classList.add('live');
      document.querySelector('.mic-wrap')?.classList.add('live');
    },
    stop() {
      live = false; analyser = null; analyserData = null;
      stage?.classList.remove('live');
      document.querySelector('.mic-wrap')?.classList.remove('live');
    },
  };

  // ── Tabs ──────────────────────────────────────────────────
  const tabs = document.querySelectorAll('.tab');
  tabs.forEach(t => t.addEventListener('click', () => {
    const id = t.dataset.tab;
    document.querySelectorAll('.tab').forEach(x => x.classList.toggle('active', x.dataset.tab === id));
    document.querySelectorAll('.tab-pane').forEach(p => p.classList.toggle('active', p.dataset.pane === id));
    document.querySelectorAll('.tab-meta').forEach(m => m.hidden = m.dataset.tab !== id);
  }));

  // ── RPLidar A1M8 polar scan visualizer ────────────────────
  const lc = document.getElementById('lidar-canvas');
  if (lc) {
    const lctx = lc.getContext('2d');
    const ldpr = window.devicePixelRatio || 1;
    const NRAYS = 360;          // A1M8 ≈ 1°/sample @ 5.5Hz
    const ranges = new Float32Array(NRAYS);     // current
    const rangesDraw = new Float32Array(NRAYS); // smoothed for render
    const intensity = new Float32Array(NRAYS);
    let MAX_R = 6;               // Variable range (zoomable)
    let sweepDeg = 0;            // rotating sensor head
    let scanFrames = 0;
    let _hasRealData = false;
    let lastScanTime = 0;

    // Zoom controls
    const zin = document.getElementById('zoom-in');
    const zout = document.getElementById('zoom-out');
    zin?.addEventListener('click', () => {
      MAX_R = Math.max(2, MAX_R - 1);
    });
    zout?.addEventListener('click', () => {
      MAX_R = Math.min(24, MAX_R + 2);
    });

    function lresize() {
      const r = lc.getBoundingClientRect();
      lc.width = r.width * ldpr;
      lc.height = r.height * ldpr;
    }
    lresize();
    window.addEventListener('resize', lresize);

    function ldraw(now) {
      requestAnimationFrame(ldraw);
      const t = now * 0.001;
      
      // Watchdog: If no data for > 1s, clear everything
      if (_hasRealData && (now - lastScanTime > 1000)) {
        ranges.fill(0);
      }

      scanFrames++; // Always increment for HUD/animation timing
      sweepDeg = (sweepDeg + 6) % 360;

      const W = lc.width, H = lc.height;
      const cx = W / 2, cy = H / 2;
      const radius = Math.min(W, H) * 0.48;
      const pxPerM = radius / MAX_R;

      lctx.clearRect(0, 0, W, H);

      // Range rings (dynamic steps)
      const rStep = MAX_R <= 4 ? 1 : MAX_R <= 8 ? 2 : MAX_R <= 15 ? 3 : 5;
      lctx.strokeStyle = 'rgba(255,255,255,0.06)';
      lctx.lineWidth = 1 * ldpr;
      lctx.font = `${10 * ldpr}px ui-monospace, JetBrains Mono, monospace`;
      lctx.fillStyle = 'rgba(255,255,255,0.25)';
      for (let m = rStep; m <= MAX_R; m += rStep) {
        const rr = m * pxPerM;
        lctx.beginPath(); lctx.arc(cx, cy, rr, 0, Math.PI * 2); lctx.stroke();
        lctx.fillText(`${m}m`, cx + 3 * ldpr, cy - rr - 3 * ldpr);
      }
      // Crosshair
      lctx.strokeStyle = 'rgba(255,255,255,0.08)';
      lctx.beginPath(); lctx.moveTo(cx - radius, cy); lctx.lineTo(cx + radius, cy); lctx.stroke();
      lctx.beginPath(); lctx.moveTo(cx, cy - radius); lctx.lineTo(cx, cy + radius); lctx.stroke();
      // Heading marker (forward)
      lctx.strokeStyle = 'rgba(250,183,183,0.5)';
      lctx.lineWidth = 2 * ldpr;
      lctx.beginPath(); lctx.moveTo(cx, cy); lctx.lineTo(cx, cy - radius); lctx.stroke();

      // Sweep beam
      const sweepRad = (sweepDeg * Math.PI) / 180;
      const grad = lctx.createConicGradient(sweepRad - Math.PI / 2, cx, cy);
      grad.addColorStop(0,    'rgba(250,183,183,0.0)');
      grad.addColorStop(0.04, 'rgba(250,183,183,0.18)');
      grad.addColorStop(0.10, 'rgba(250,183,183,0.0)');
      grad.addColorStop(1,    'rgba(250,183,183,0.0)');
      lctx.fillStyle = grad;
      lctx.beginPath(); lctx.arc(cx, cy, radius, 0, Math.PI * 2); lctx.fill();

      // Polygon hull (filled cone, soft)
      lctx.beginPath();
      let started = false;
      for (let i = 0; i < NRAYS; i++) {
        // smooth rendered range
        rangesDraw[i] += (ranges[i] - rangesDraw[i]) * 0.35;
        const r = rangesDraw[i];
        if (r <= 0) { started = false; continue; }
        const a = (i / NRAYS) * Math.PI * 2 - Math.PI / 2; // 0deg = up
        const x = cx + Math.cos(a) * r * pxPerM;
        const y = cy + Math.sin(a) * r * pxPerM;
        if (!started) { lctx.moveTo(x, y); started = true; }
        else lctx.lineTo(x, y);
      }
      lctx.closePath();
      lctx.fillStyle = 'rgba(250,183,183,0.04)';
      lctx.fill();
      lctx.strokeStyle = 'rgba(250,183,183,0.20)';
      lctx.lineWidth = 1.2 * ldpr;
      lctx.stroke();

      // Points
      let ptCount = 0, minR = Infinity, maxR = 0, obs = 0;
      let lastObsBin = -1;
      for (let i = 0; i < NRAYS; i++) {
        const r = rangesDraw[i];
        if (r <= 0) continue;
        const a = (i / NRAYS) * Math.PI * 2 - Math.PI / 2;
        const x = cx + Math.cos(a) * r * pxPerM;
        const y = cy + Math.sin(a) * r * pxPerM;

        // Color by distance
        let col;
        if (r < 1)      col = `rgba(255,91,80,${0.65 + intensity[i] * 0.35})`;
        else if (r < 4) col = `rgba(250,183,183,${0.55 + intensity[i] * 0.4})`;
        else            col = `rgba(255,255,255,${0.45 + intensity[i] * 0.4})`;

        // Highlight currently swept ray
        const sweepDelta = Math.abs(((i - sweepDeg) + NRAYS / 2 + NRAYS) % NRAYS - NRAYS / 2);
        const sweepBoost = sweepDelta < 18 ? (1 - sweepDelta / 18) : 0;
        const sz = (1.8 + 1.2 * sweepBoost) * ldpr; // Smaller points

        lctx.fillStyle = col;
        lctx.beginPath(); lctx.arc(x, y, sz, 0, Math.PI * 2); lctx.fill();
        if (sweepBoost > 0.5) {
          lctx.fillStyle = `rgba(255,255,255,${sweepBoost * 0.8})`;
          lctx.beginPath(); lctx.arc(x, y, sz * 1.6, 0, Math.PI * 2); lctx.fill();
        }
        ptCount++;
        if (r < minR) minR = r;
        if (r > maxR) maxR = r;
        if (r < 1.5) {
          // count contiguous near clusters
          if (lastObsBin < 0 || (i - lastObsBin) > 6) obs++;
          lastObsBin = i;
        }
      }

      // Robot footprint at origin
      lctx.strokeStyle = 'rgba(255,255,255,0.55)';
      lctx.lineWidth = 2 * ldpr;
      lctx.beginPath(); lctx.arc(cx, cy, 8 * ldpr, 0, Math.PI * 2); lctx.stroke();
      lctx.fillStyle = 'rgba(250,183,183,0.9)';
      lctx.beginPath(); lctx.arc(cx, cy, 4 * ldpr, 0, Math.PI * 2); lctx.fill();
      // forward chevron
      lctx.strokeStyle = 'rgba(255,255,255,0.85)';
      lctx.lineWidth = 2.5 * ldpr;
      lctx.beginPath(); lctx.moveTo(cx, cy - 4 * ldpr); lctx.lineTo(cx, cy - 16 * ldpr); lctx.stroke();

      // HUD update
      if (scanFrames % 6 === 0) {
        const sp = document.getElementById('scan-pts');
        const so = document.getElementById('scan-obs');
        const smin = document.getElementById('scan-min');
        const smax = document.getElementById('scan-max');
        if (sp) sp.textContent = ptCount;
        if (so) so.textContent = obs;
        if (smin && isFinite(minR)) smin.textContent = minR.toFixed(2);
        if (smax) smax.textContent = maxR.toFixed(2);
      }
    }
    requestAnimationFrame(ldraw);

    // Public API for WebSocket-driven /scan messages
    window.__vnLidar = {
      // ranges: Float32Array length 360, meters; 0 = no return
      push(rs) {
        if (!rs || !rs.length) return;
        _hasRealData = true;
        lastScanTime = performance.now();
        const n = Math.min(rs.length, NRAYS);
        for (let i = 0; i < n; i++) {
          const v = rs[i];
          ranges[i] = (isFinite(v) && v > 0) ? v : 0;
          intensity[i] = ranges[i] > 0 ? 0.85 : 0;
        }
      }
    };
  }

  // ── Safety button wiring (purely visual until backend implements) ─
  const estop  = document.getElementById('estop-btn');
  const resume = document.getElementById('resume-btn');
  const stopT  = document.getElementById('stop-btn-top');
  function flashState(label) {
    const t = document.getElementById('toast');
    if (!t) return;
    t.textContent = label;
    t.style.borderColor = 'var(--vn-border-bright)';
    t.classList.add('active');
    setTimeout(() => t.classList.remove('active'), 1800);
  }
  estop?.addEventListener('click', () => {
    flashState('E-STOP ENGAGED');
    resume.disabled = false;
    fetch('/api/estop', { method: 'POST' }).catch(()=>{});
  });
  resume?.addEventListener('click', () => {
    flashState('MISSION RESUMED');
    resume.disabled = true;
    fetch('/api/resume', { method: 'POST' }).catch(()=>{});
  });
  stopT?.addEventListener('click', () => {
    flashState('STOP — motion halted');
    fetch('/api/navigate/cancel', { method: 'POST' }).catch(()=>{});
  });
})();
