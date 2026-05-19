/**
 * Vector Nav — Visual effects layer.
 * - Animated battery (level + ETA, smooth interp)
 * - Beautiful spectrum: 64-bar mirrored bar visualizer with peak hold,
 *   driven by mic AnalyserNode when available, otherwise demo synth.
 * - Mic ring activation (.live class on .mic-wrap and .spectrum-stage)
 */
(() => {
  // ── Battery (supports N instances — topbar + HUD) ─────────
  // Each instance is a `.batt-cell` with `.batt-pct` (text), optional `.batt-volt`,
  // and SVG sub-parts marked by data-batt-glow / data-batt-base-fill / data-batt-path.
  const cells = Array.from(document.querySelectorAll('.batt-cell'));
  // Back-compat: the original topbar also has id="battery-cell" without the new class.
  const legacy = document.getElementById('battery-cell');
  if (legacy && !cells.includes(legacy)) cells.push(legacy);

  let battTarget = 87;
  if (legacy?.dataset.pct) battTarget = parseFloat(legacy.dataset.pct) || 87;
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
    const state = pct <= 20 ? 'low' : pct < 50 ? 'warn' : 'ok';
    cells.forEach(c => {
      c.dataset.state = state;
      c.classList.toggle('animate-pulse', pct <= 20);
    });
  }

  function tickBattery() {
    battShown += (battTarget - battShown) * 0.08;
    document.querySelectorAll('.batt-pct').forEach(el => { el.textContent = battShown.toFixed(0); });
    document.querySelectorAll('.batt-volt').forEach(el => { el.textContent = `${voltShown.toFixed(1)}V`; });

    const pct = battShown;
    const fillY = innerH * (1 - Math.max(0, Math.min(100, pct)) / 100);
    const fillColor = pct > 50 ? "#22c55e" : pct > 20 ? "#eab308" : "#ef4444";
    const d1 = makePath(fillY, AMP * 0.35);
    const d2 = makePath(fillY, AMP * 0.60);
    const d3 = makePath(fillY, AMP * 0.82);
    const d4 = makePath(fillY, AMP);

    cells.forEach(cell => {
      cell.querySelectorAll('[data-batt-glow]').forEach(el => el.setAttribute('fill', fillColor));
      cell.querySelectorAll('[data-batt-base-fill]').forEach(el => {
        el.setAttribute('fill', fillColor);
        el.setAttribute('y', 9 + fillY);
        el.setAttribute('height', innerH - fillY + 4);
      });
      const setPath = (idx, d) => cell.querySelectorAll(`[data-batt-path="${idx}"]`).forEach(el => {
        el.setAttribute('d', d); el.setAttribute('fill', fillColor);
      });
      setPath(1, d1); setPath(2, d2); setPath(3, d3); setPath(4, d4);
    });

    requestAnimationFrame(tickBattery);
  }
  if (cells.length) {
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
    if (w <= 0 || h <= 0) return;
    r = Math.max(0, Math.min(r, Math.min(w, h) / 2));
    ctx.beginPath();
    if (r <= 0) {
      ctx.rect(x, y, w, h);
      ctx.closePath();
      return;
    }
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

  // ── Map popover (Saved Maps / SLAM save controls) ────────
  const popTrigger = document.getElementById('map-pop-trigger');
  const popover    = document.getElementById('map-popover');
  const popLabel   = document.getElementById('current-map-name');

  function setPopOpen(open) {
    if (!popover || !popTrigger) return;
    popover.hidden = !open;
    popTrigger.setAttribute('aria-expanded', open ? 'true' : 'false');
  }
  popTrigger?.addEventListener('click', (e) => {
    e.stopPropagation();
    setPopOpen(popover.hidden);
  });
  // close on outside click + Esc
  document.addEventListener('click', (e) => {
    if (popover?.hidden) return;
    if (!popover.contains(e.target) && e.target !== popTrigger && !popTrigger?.contains(e.target)) {
      setPopOpen(false);
    }
  });
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && popover && !popover.hidden) setPopOpen(false);
  });

  // Keep the chip's label in sync with the active map / mode.
  // App.js doesn't expose state directly, so we watch the DOM it already updates.
  const navBtn  = document.getElementById('mode-btn-nav');
  const slamBtn = document.getElementById('mode-btn-slam');
  const slamCtrls = document.getElementById('slam-controls');
  const carousel  = document.getElementById('map-carousel');
  function refreshMapChipLabel() {
    if (!popLabel) return;
    if (slamBtn?.classList.contains('active')) {
      popLabel.textContent = 'SLAM · live';
    } else {
      const active = carousel?.querySelector('.map-card.active .map-card-name');
      popLabel.textContent = active?.textContent?.trim() || 'no map';
    }
  }
  // Initial + observe changes app.js makes to mode buttons / carousel.
  refreshMapChipLabel();
  const mo = new MutationObserver(refreshMapChipLabel);
  navBtn  && mo.observe(navBtn,  { attributes: true, attributeFilter: ['class'] });
  slamBtn && mo.observe(slamBtn, { attributes: true, attributeFilter: ['class'] });
  carousel && mo.observe(carousel, { childList: true, subtree: true, attributes: true, attributeFilter: ['class'] });
  // When SLAM controls become visible/hidden, refresh.
  slamCtrls && mo.observe(slamCtrls, { attributes: true, attributeFilter: ['hidden'] });

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

      scanFrames++;

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

      // Smooth rendered ranges — snap on appearance so points don't animate from origin
      for (let i = 0; i < NRAYS; i++) {
        if (ranges[i] > 0 && rangesDraw[i] <= 0) {
          rangesDraw[i] = ranges[i]; // instant appear
        } else if (ranges[i] <= 0 && rangesDraw[i] > 0) {
          rangesDraw[i] = 0;         // instant disappear
        } else {
          rangesDraw[i] += (ranges[i] - rangesDraw[i]) * 0.35;
        }
      }

      // Points
      let ptCount = 0, minR = Infinity, maxR = 0, obs = 0;
      let lastObsBin = -1;
      const sz = 1 * ldpr;
      for (let i = 0; i < NRAYS; i++) {
        const r = rangesDraw[i];
        if (r <= 0) continue;
        const a = (i / NRAYS) * Math.PI * 2 - Math.PI / 2;
        const x = cx + Math.cos(a) * r * pxPerM;
        const y = cy + Math.sin(a) * r * pxPerM;

        let col;
        if (r < 1)      col = `rgba(255,91,80,${0.70 + intensity[i] * 0.30})`;
        else if (r < 4) col = `rgba(250,183,183,${0.60 + intensity[i] * 0.35})`;
        else            col = `rgba(255,255,255,${0.50 + intensity[i] * 0.35})`;

        lctx.fillStyle = col;
        lctx.beginPath(); lctx.arc(x, y, sz, 0, Math.PI * 2); lctx.fill();

        ptCount++;
        if (r < minR) minR = r;
        if (r > maxR) maxR = r;
        if (r < 1.5) {
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

})();
