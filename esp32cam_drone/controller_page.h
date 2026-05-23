// ================================================================
//  DRONE CONTROLLER WEB PAGE — Arduino C string literal
//
//  Usage in your ESP32 WebServer sketch:
//
//    #include "controller_page.h"
//
//    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
//      req->send_P(200, "text/html", CONTROLLER_PAGE);
//    });
//
//  OR with ESPAsyncWebServer:
//    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
//      req->send(200, "text/html", CONTROLLER_PAGE);
//    });
// ================================================================

#pragma once

const char CONTROLLER_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>DroneFC Controller</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;400;600;700&display=swap');

  :root {
    --bg:       #080c10;
    --bg2:      #0d1318;
    --bg3:      #111820;
    --panel:    #141d26;
    --border:   #1e2d3d;
    --border2:  #2a3f55;
    --accent:   #00d4ff;
    --accent2:  #0099cc;
    --armed:    #00ff88;
    --danger:   #ff3355;
    --warn:     #ffaa00;
    --dim:      #3a5068;
    --text:     #c8dce8;
    --text2:    #7a9ab0;
    --mono:     'Share Tech Mono', monospace;
    --sans:     'Exo 2', sans-serif;
    --glow:     0 0 12px rgba(0,212,255,0.25);
    --glow-arm: 0 0 16px rgba(0,255,136,0.3);
    --glow-red: 0 0 16px rgba(255,51,85,0.35);
  }

  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }

  html, body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--sans);
    font-size: 14px;
    height: 100%;
    overflow-x: hidden;
    user-select: none;
  }

  /* ── scanline overlay for CRT feel ── */
  body::before {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(0deg, transparent, transparent 2px, rgba(0,0,0,0.08) 2px, rgba(0,0,0,0.08) 4px);
    pointer-events: none;
    z-index: 9999;
  }

  /* ── corner brackets decoration ── */
  .bracket {
    position: relative;
  }
  .bracket::before, .bracket::after {
    content: '';
    position: absolute;
    width: 10px; height: 10px;
    border-color: var(--accent2);
    border-style: solid;
    opacity: 0.5;
  }
  .bracket::before { top: 0; left: 0; border-width: 1px 0 0 1px; }
  .bracket::after  { bottom: 0; right: 0; border-width: 0 1px 1px 0; }

  /* ── layout ── */
  #app {
    display: flex;
    flex-direction: column;
    min-height: 100vh;
    max-width: 480px;
    margin: 0 auto;
    padding: 8px;
    gap: 8px;
  }

  /* ── top bar ── */
  #topbar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 12px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
  }
  #topbar .logo {
    font-family: var(--mono);
    font-size: 13px;
    color: var(--accent);
    letter-spacing: 2px;
  }
  #conn-dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: var(--dim);
    transition: background 0.3s, box-shadow 0.3s;
    flex-shrink: 0;
  }
  #conn-dot.online { background: var(--armed); box-shadow: 0 0 8px var(--armed); }
  #conn-dot.error  { background: var(--danger); box-shadow: 0 0 8px var(--danger); }

  #topbar .conn-label {
    font-family: var(--mono);
    font-size: 11px;
    color: var(--text2);
    letter-spacing: 1px;
  }
  .top-right { display: flex; align-items: center; gap: 8px; }

  /* ── telemetry strip ── */
  #telem {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 6px;
  }
  .telem-cell {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 5px;
    padding: 6px 8px;
    text-align: center;
  }
  .telem-cell .t-label {
    font-family: var(--mono);
    font-size: 9px;
    color: var(--text2);
    letter-spacing: 1px;
    text-transform: uppercase;
    margin-bottom: 3px;
  }
  .telem-cell .t-val {
    font-family: var(--mono);
    font-size: 14px;
    font-weight: 600;
    color: var(--accent);
  }
  .telem-cell .t-val.armed  { color: var(--armed); }
  .telem-cell .t-val.disarmed { color: var(--text2); }
  .telem-cell .t-val.danger { color: var(--danger); }

  /* ── command log ── */
  #log-box {
    background: var(--bg2);
    border: 1px solid var(--border);
    border-radius: 5px;
    padding: 6px 10px;
    height: 48px;
    overflow: hidden;
    position: relative;
  }
  #log-text {
    font-family: var(--mono);
    font-size: 11px;
    color: var(--text2);
    line-height: 1.6;
  }
  #log-text span { display: block; }
  #log-text .ok  { color: var(--armed); }
  #log-text .err { color: var(--danger); }
  #log-text .cmd { color: var(--accent); }

  /* ── arm / disarm / stop buttons ── */
  #arm-row {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 8px;
  }
  .ctrl-btn {
    font-family: var(--mono);
    font-size: 12px;
    letter-spacing: 1.5px;
    font-weight: 600;
    padding: 14px 8px;
    border-radius: 6px;
    border: 1px solid;
    cursor: pointer;
    transition: opacity 0.15s, transform 0.1s;
    background: transparent;
    text-transform: uppercase;
  }
  .ctrl-btn:active { transform: scale(0.96); opacity: 0.8; }

  #btn-arm   { border-color: var(--armed); color: var(--armed); }
  #btn-arm:hover { background: rgba(0,255,136,0.08); box-shadow: var(--glow-arm); }
  #btn-arm.active-state { background: rgba(0,255,136,0.15); box-shadow: var(--glow-arm); }

  #btn-disarm { border-color: var(--warn); color: var(--warn); }
  #btn-disarm:hover { background: rgba(255,170,0,0.08); }

  #btn-stop  { border-color: var(--danger); color: var(--danger); }
  #btn-stop:hover { background: rgba(255,51,85,0.1); box-shadow: var(--glow-red); }

  /* ── throttle ── */
  #throttle-row {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 12px 14px;
  }
  .row-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 10px;
  }
  .row-header .label {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text2);
    letter-spacing: 2px;
    text-transform: uppercase;
  }
  .row-header .val {
    font-family: var(--mono);
    font-size: 16px;
    color: var(--accent);
    font-weight: 600;
  }

  #thr-slider {
    -webkit-appearance: none;
    appearance: none;
    width: 100%;
    height: 6px;
    border-radius: 3px;
    background: var(--border2);
    outline: none;
    cursor: pointer;
  }
  #thr-slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 22px; height: 22px;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 0 10px rgba(0,212,255,0.5);
    cursor: grab;
    border: 2px solid var(--bg);
  }
  #thr-slider::-moz-range-thumb {
    width: 22px; height: 22px;
    border-radius: 50%;
    background: var(--accent);
    border: 2px solid var(--bg);
  }

  #thr-track-fill {
    height: 6px;
    border-radius: 3px;
    background: linear-gradient(90deg, var(--accent2), var(--accent));
    margin-top: -6px;
    pointer-events: none;
    transition: width 0.05s;
    position: relative;
    z-index: -1;
  }

  /* ── joystick grid ── */
  #joystick-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
  }
  .js-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 10px;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
  }
  .js-panel .js-title {
    font-family: var(--mono);
    font-size: 9px;
    letter-spacing: 2px;
    color: var(--text2);
    text-transform: uppercase;
    align-self: flex-start;
  }
  .js-val-row {
    width: 100%;
    display: flex;
    justify-content: space-between;
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text2);
  }
  .js-val-row span { color: var(--accent); font-size: 12px; }

  /* ── canvas joystick ── */
  canvas.joystick {
    border-radius: 50%;
    border: 1px solid var(--border2);
    background: var(--bg2);
    touch-action: none;
    cursor: crosshair;
    display: block;
  }

  /* ── yaw strip (single axis horizontal) ── */
  #yaw-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 12px 14px;
  }
  #yaw-canvas {
    width: 100%;
    height: 56px;
    border-radius: 4px;
    border: 1px solid var(--border2);
    background: var(--bg2);
    touch-action: none;
    cursor: ew-resize;
    display: block;
  }

  /* ── status bar bottom ── */
  #statusbar {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 6px 10px;
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 5px;
    font-family: var(--mono);
    font-size: 10px;
    color: var(--text2);
    letter-spacing: 0.5px;
  }
  #statusbar .hz { color: var(--accent); }

  /* ── ping indicator ── */
  .ping-bar {
    display: flex;
    gap: 2px;
    align-items: flex-end;
    height: 14px;
  }
  .ping-bar span {
    width: 3px;
    background: var(--dim);
    border-radius: 1px;
    transition: background 0.3s;
  }
  .ping-bar span:nth-child(1) { height: 4px; }
  .ping-bar span:nth-child(2) { height: 7px; }
  .ping-bar span:nth-child(3) { height: 10px; }
  .ping-bar span:nth-child(4) { height: 14px; }
  .ping-bar.sig1 span:nth-child(1) { background: var(--danger); }
  .ping-bar.sig2 span:nth-child(1),
  .ping-bar.sig2 span:nth-child(2) { background: var(--warn); }
  .ping-bar.sig3 span:nth-child(1),
  .ping-bar.sig3 span:nth-child(2),
  .ping-bar.sig3 span:nth-child(3) { background: var(--accent); }
  .ping-bar.sig4 span { background: var(--armed); }

  /* flash for armed state */
  @keyframes armed-pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.4; }
  }
  .armed-flash { animation: armed-pulse 1.5s ease-in-out infinite; }
</style>
</head>
<body>
<div id="app">

  <!-- top bar -->
  <div id="topbar" class="bracket">
    <div class="logo">&#9670; DRONE-FC</div>
    <div class="top-right">
      <div class="ping-bar" id="ping-bar">
        <span></span><span></span><span></span><span></span>
      </div>
      <div class="conn-label" id="conn-label">OFFLINE</div>
      <div id="conn-dot"></div>
    </div>
  </div>

  <!-- telemetry strip -->
  <div id="telem">
    <div class="telem-cell">
      <div class="t-label">STATUS</div>
      <div class="t-val disarmed" id="t-arm">DISARM</div>
    </div>
    <div class="telem-cell">
      <div class="t-label">THROTTLE</div>
      <div class="t-val" id="t-thr">0</div>
    </div>
    <div class="telem-cell">
      <div class="t-label">ROLL</div>
      <div class="t-val" id="t-roll">0.0°</div>
    </div>
    <div class="telem-cell">
      <div class="t-label">PITCH</div>
      <div class="t-val" id="t-pitch">0.0°</div>
    </div>
  </div>

  <!-- log -->
  <div id="log-box">
    <div id="log-text"><span class="ok">// DRONE CONTROLLER READY</span></div>
  </div>

  <!-- arm row -->
  <div id="arm-row">
    <button class="ctrl-btn" id="btn-arm"    onclick="sendCmd('ARM')">&#9654; ARM</button>
    <button class="ctrl-btn" id="btn-disarm" onclick="sendCmd('DISARM')">&#9632; DISARM</button>
    <button class="ctrl-btn" id="btn-stop"   onclick="emergencyStop()">&#9888; STOP</button>
  </div>

  <!-- throttle -->
  <div id="throttle-row" class="bracket">
    <div class="row-header">
      <span class="label">&#9650; THROTTLE</span>
      <span class="val" id="thr-readout">0</span>
    </div>
    <input type="range" id="thr-slider" min="0" max="200" step="1" value="0">
  </div>

  <!-- joystick grid: roll + pitch -->
  <div id="joystick-grid">
    <div class="js-panel bracket">
      <div class="js-title">&#9664;&#9654; ROLL</div>
      <canvas class="joystick" id="roll-canvas" width="140" height="140"></canvas>
      <div class="js-val-row">ROLL <span id="roll-readout">0.0°</span></div>
    </div>
    <div class="js-panel bracket">
      <div class="js-title">&#9650;&#9660; PITCH</div>
      <canvas class="joystick" id="pitch-canvas" width="140" height="140"></canvas>
      <div class="js-val-row">PITCH <span id="pitch-readout">0.0°</span></div>
    </div>
  </div>

  <!-- yaw strip -->
  <div id="yaw-panel" class="bracket">
    <div class="row-header">
      <span class="label">&#8635; YAW</span>
      <span class="val" id="yaw-readout">0.0°</span>
    </div>
    <canvas id="yaw-canvas"></canvas>
  </div>

  <!-- status bar -->
  <div id="statusbar">
    <span id="sb-ip">IP: ---.---.---.---</span>
    <span><span class="hz" id="sb-hz">-- </span>Hz</span>
    <span id="sb-time">--:--:--</span>
  </div>

</div>

<script>
// ═══════════════════════════════════════════════════════
//  CONFIGURATION
// ═══════════════════════════════════════════════════════
const DRONE_IP   = window.location.hostname || '192.168.4.1';
const CMD_URL    = `http://${DRONE_IP}/cmd`;
const CMD_RATE   = 80;    // ms between continuous sends (~12.5 Hz)
const KEEPALIVE  = 1500;  // ms — send STATUS heartbeat if no other cmd
const MAX_ANGLE  = 30;    // degrees max for roll/pitch/yaw setpoints
const THR_MAX    = 200;

// ═══════════════════════════════════════════════════════
//  STATE
// ═══════════════════════════════════════════════════════
const state = {
  connected: false,
  armed: false,
  throttle: 0,
  roll: 0,
  pitch: 0,
  yaw: 0,
  lastSent: 0,
  lastCmd: 0,
  txHz: 0,
  hzFrames: 0,
  hzTimer: 0
};

// ═══════════════════════════════════════════════════════
//  LOGGING
// ═══════════════════════════════════════════════════════
const logEl = document.getElementById('log-text');
function log(msg, cls = '') {
  const s = document.createElement('span');
  s.className = cls;
  s.textContent = msg;
  logEl.insertBefore(s, logEl.firstChild);
  while (logEl.children.length > 3) logEl.removeChild(logEl.lastChild);
}

// ═══════════════════════════════════════════════════════
//  HTTP COMMAND SENDER
// ═══════════════════════════════════════════════════════
async function sendCmd(cmd, silent = false) {
  if (!silent) log('> ' + cmd, 'cmd');
  state.lastCmd = Date.now();
  try {
    const r = await fetch(CMD_URL, {
      method: 'POST',
      headers: { 'Content-Type': 'text/plain' },
      body: cmd,
      signal: AbortSignal.timeout(1500)
    });
    const txt = await r.text();
    setConnected(true);
    if (!silent) log(txt.substring(0, 60), 'ok');
    if (txt.startsWith('{')) parseTelemetry(txt);
    if (cmd === 'ARM' && txt.includes('SUCCESS'))   setArmed(true);
    if (cmd === 'DISARM' || cmd === 'ARM_DENIED')   setArmed(false);
    if (txt.includes('DISARMED'))                   setArmed(false);
    return txt;
  } catch (e) {
    setConnected(false);
    if (!silent) log('ERR: ' + (e.message || 'timeout'), 'err');
  }
}

function parseTelemetry(json) {
  try {
    const d = JSON.parse(json);
    if (d.state !== undefined) setArmed(d.state === 'ARMED');
    if (d.roll  !== undefined) document.getElementById('t-roll').textContent  = d.roll.toFixed(1) + '°';
    if (d.pitch !== undefined) document.getElementById('t-pitch').textContent = d.pitch.toFixed(1) + '°';
    if (d.throttle !== undefined) {
      state.throttle = d.throttle;
      document.getElementById('t-thr').textContent = d.throttle;
    }
  } catch (_) {}
}

// ═══════════════════════════════════════════════════════
//  CONNECTION / ARM STATE
// ═══════════════════════════════════════════════════════
function setConnected(ok) {
  state.connected = ok;
  const dot   = document.getElementById('conn-dot');
  const label = document.getElementById('conn-label');
  const pb    = document.getElementById('ping-bar');
  dot.className   = ok ? 'online' : 'error';
  label.textContent = ok ? 'ONLINE' : 'OFFLINE';
  pb.className    = ok ? 'ping-bar sig4' : 'ping-bar';
  document.getElementById('sb-ip').textContent = 'IP: ' + DRONE_IP;
}

function setArmed(armed) {
  state.armed = armed;
  const el = document.getElementById('t-arm');
  const armBtn = document.getElementById('btn-arm');
  el.textContent  = armed ? 'ARMED' : 'DISARM';
  el.className    = armed ? 't-val armed armed-flash' : 't-val disarmed';
  armBtn.className = armed ? 'ctrl-btn active-state' : 'ctrl-btn';
}

function emergencyStop() {
  state.roll = 0; state.pitch = 0; state.yaw = 0;
  state.throttle = 0;
  document.getElementById('thr-slider').value = 0;
  updateThrUI(0);
  sendCmd('DISARM');
  sendCmd('THROTTLE 0');
}

// ═══════════════════════════════════════════════════════
//  THROTTLE SLIDER
// ═══════════════════════════════════════════════════════
const thrSlider = document.getElementById('thr-slider');

function updateThrUI(val) {
  document.getElementById('thr-readout').textContent = val;
  document.getElementById('t-thr').textContent = val;
}

thrSlider.addEventListener('input', () => {
  const v = parseInt(thrSlider.value);
  state.throttle = v;
  updateThrUI(v);
});

// ═══════════════════════════════════════════════════════
//  JOYSTICK — generic 1D horizontal or 2D canvas control
// ═══════════════════════════════════════════════════════
function makeJoystick1D(canvasId, axis, label, outputId, onUpdate) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const W = canvas.width, H = canvas.height;
  let dragging = false;
  let pos = 0.5; // 0..1, 0.5=center

  function draw() {
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, W, H);
    const val = (pos - 0.5) * 2; // -1..1

    // track
    ctx.fillStyle = '#1e2d3d';
    ctx.fillRect(12, H/2 - 3, W - 24, 6);
    ctx.fillStyle = '#2a3f55';
    ctx.fillRect(12, H/2 - 3, W - 24, 6);

    // fill
    const cx = 12 + (W-24)*pos;
    const midX = 12 + (W-24)*0.5;
    ctx.fillStyle = '#00d4ff';
    ctx.globalAlpha = 0.5;
    if (cx > midX) ctx.fillRect(midX, H/2-3, cx-midX, 6);
    else            ctx.fillRect(cx, H/2-3, midX-cx, 6);
    ctx.globalAlpha = 1;

    // center tick
    ctx.strokeStyle = '#2a3f55';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(midX, H/2-10); ctx.lineTo(midX, H/2+10); ctx.stroke();

    // thumb
    ctx.beginPath();
    ctx.arc(cx, H/2, 18, 0, Math.PI*2);
    ctx.fillStyle = '#0d1318';
    ctx.fill();
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // crosshair on thumb
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.6;
    ctx.beginPath(); ctx.moveTo(cx-6, H/2); ctx.lineTo(cx+6, H/2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(cx, H/2-6); ctx.lineTo(cx, H/2+6); ctx.stroke();
    ctx.globalAlpha = 1;

    const deg = (val * MAX_ANGLE).toFixed(1);
    document.getElementById(outputId).textContent = deg + '°';
    return val;
  }

  function getPos(e) {
    const rect = canvas.getBoundingClientRect();
    const x = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left;
    return Math.max(0, Math.min(1, (x - 12) / (W - 24)));
  }

  function start(e) { e.preventDefault(); dragging = true; pos = getPos(e); draw(); }
  function move(e)  { if (!dragging) return; e.preventDefault(); pos = getPos(e); draw(); onUpdate((pos-0.5)*2*MAX_ANGLE); }
  function end(e)   {
    dragging = false;
    pos = 0.5; // spring back to center
    draw();
    onUpdate(0);
  }

  canvas.addEventListener('mousedown',  start);
  canvas.addEventListener('mousemove',  move);
  canvas.addEventListener('mouseup',    end);
  canvas.addEventListener('touchstart', start, {passive:false});
  canvas.addEventListener('touchmove',  move,  {passive:false});
  canvas.addEventListener('touchend',   end);
  canvas.addEventListener('touchcancel',end);

  draw();
}

function makeJoystick2D(canvasId, xLabel, yLabel, xId, yId, onUpdate) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const W = canvas.width, H = canvas.height;
  const R = W / 2;
  let dragging = false;
  let jx = 0, jy = 0; // -1..1

  function draw() {
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, W, H);

    // outer ring
    ctx.beginPath();
    ctx.arc(R, R, R-4, 0, Math.PI*2);
    ctx.strokeStyle = '#1e2d3d';
    ctx.lineWidth = 1;
    ctx.stroke();

    // grid lines
    ctx.strokeStyle = '#1e2d3d';
    ctx.lineWidth = 0.5;
    ctx.beginPath(); ctx.moveTo(R, 4); ctx.lineTo(R, W-4); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(4, R); ctx.lineTo(W-4, R); ctx.stroke();

    // inner ring
    ctx.beginPath();
    ctx.arc(R, R, (R-4)*0.5, 0, Math.PI*2);
    ctx.strokeStyle = '#1e2d3d';
    ctx.lineWidth = 0.5;
    ctx.stroke();

    const tx = R + jx*(R-20);
    const ty = R + jy*(R-20);

    // line from center to thumb
    ctx.beginPath(); ctx.moveTo(R, R); ctx.lineTo(tx, ty);
    ctx.strokeStyle = 'rgba(0,212,255,0.25)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // thumb shadow
    ctx.beginPath();
    ctx.arc(tx, ty, 16, 0, Math.PI*2);
    ctx.fillStyle = 'rgba(0,212,255,0.08)';
    ctx.fill();

    // thumb
    ctx.beginPath();
    ctx.arc(tx, ty, 12, 0, Math.PI*2);
    ctx.fillStyle = '#0d1318';
    ctx.fill();
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // crosshair
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 1;
    ctx.globalAlpha = 0.7;
    ctx.beginPath(); ctx.moveTo(tx-5, ty); ctx.lineTo(tx+5, ty); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(tx, ty-5); ctx.lineTo(tx, ty+5); ctx.stroke();
    ctx.globalAlpha = 1;

    // center dot
    ctx.beginPath(); ctx.arc(R, R, 3, 0, Math.PI*2);
    ctx.fillStyle = '#2a3f55'; ctx.fill();

    const xDeg = (jx * MAX_ANGLE).toFixed(1);
    const yDeg = (jy * MAX_ANGLE).toFixed(1);
    if (xId) document.getElementById(xId).textContent = xDeg + '°';
    if (yId) document.getElementById(yId).textContent = yDeg + '°';
  }

  function getJoy(e) {
    const rect = canvas.getBoundingClientRect();
    const cx = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left;
    const cy = (e.touches ? e.touches[0].clientY : e.clientY) - rect.top;
    let nx = (cx - R) / (R-20);
    let ny = (cy - R) / (R-20);
    const len = Math.sqrt(nx*nx + ny*ny);
    if (len > 1) { nx /= len; ny /= len; }
    return [nx, ny];
  }

  function start(e) { e.preventDefault(); dragging = true; [jx,jy] = getJoy(e); draw(); onUpdate(jx*MAX_ANGLE, jy*MAX_ANGLE); }
  function move(e)  { if(!dragging) return; e.preventDefault(); [jx,jy] = getJoy(e); draw(); onUpdate(jx*MAX_ANGLE, jy*MAX_ANGLE); }
  function end(e)   { dragging = false; jx = 0; jy = 0; draw(); onUpdate(0, 0); }

  canvas.addEventListener('mousedown',  start);
  canvas.addEventListener('mousemove',  move);
  canvas.addEventListener('mouseup',    end);
  canvas.addEventListener('touchstart', start, {passive:false});
  canvas.addEventListener('touchmove',  move,  {passive:false});
  canvas.addEventListener('touchend',   end);
  canvas.addEventListener('touchcancel',end);

  draw();
}

// ═══════════════════════════════════════════════════════
//  INIT CONTROLS
// ═══════════════════════════════════════════════════════

// Roll joystick (2D — left/right only really but allow small vertical)
makeJoystick2D('roll-canvas', 'R', null, 'roll-readout', null,
  (x, y) => { state.roll = x; }
);

// Pitch joystick (2D — up/down)
makeJoystick2D('pitch-canvas', null, 'P', null, 'pitch-readout',
  (x, y) => { state.pitch = y; }
);

// Yaw strip (1D horizontal)
(function() {
  const canvas = document.getElementById('yaw-canvas');
  canvas.width  = canvas.offsetWidth  || 300;
  canvas.height = 56;
  makeJoystick1D('yaw-canvas', 'Y', 'YAW', 'yaw-readout',
    (val) => { state.yaw = val; }
  );
  window.addEventListener('resize', () => {
    canvas.width = canvas.offsetWidth;
  });
})();

// ═══════════════════════════════════════════════════════
//  CONTINUOUS COMMAND LOOP
//  Sends throttle, roll, pitch, yaw at CMD_RATE ms
//  Only sends if value changed or interval elapsed
// ═══════════════════════════════════════════════════════
let prevSent = { thr: -1, roll: -999, pitch: -999, yaw: -999 };

async function cmdLoop() {
  const now = Date.now();
  const thr   = parseInt(thrSlider.value);
  const roll  = Math.round(state.roll  * 10) / 10;
  const pitch = Math.round(state.pitch * 10) / 10;
  const yaw   = Math.round(state.yaw   * 10) / 10;

  const thrChanged   = thr   !== prevSent.thr;
  const rollChanged  = Math.abs(roll  - prevSent.roll)  > 0.4;
  const pitchChanged = Math.abs(pitch - prevSent.pitch) > 0.4;
  const yawChanged   = Math.abs(yaw   - prevSent.yaw)   > 0.4;
  const needKeepalive = (now - state.lastCmd) > KEEPALIVE;

  if (thrChanged || rollChanged || pitchChanged || yawChanged || needKeepalive) {
    // batch into one request body using newline-separated commands
    // ESP32 /cmd accepts one command per request — send most critical first
    // throttle is priority 1
    if (thrChanged) {
      await sendCmd('T:' + thr, true);
      prevSent.thr = thr;
      updateThrUI(thr);
    }
    if (rollChanged)  { await sendCmd('R:' + roll.toFixed(1),  true); prevSent.roll  = roll; document.getElementById('t-roll').textContent  = roll.toFixed(1) + '°'; }
    if (pitchChanged) { await sendCmd('P:' + pitch.toFixed(1), true); prevSent.pitch = pitch; document.getElementById('t-pitch').textContent = pitch.toFixed(1) + '°'; }
    if (yawChanged)   { await sendCmd('Y:' + yaw.toFixed(1),   true); prevSent.yaw   = yaw; }
    if (needKeepalive && !thrChanged && !rollChanged && !pitchChanged && !yawChanged) {
      await sendCmd('STATUS', true);
    }

    state.hzFrames++;
  }
}

// Hz counter
setInterval(() => {
  state.txHz = state.hzFrames;
  state.hzFrames = 0;
  document.getElementById('sb-hz').textContent = state.txHz.toString().padStart(2,'0') + ' ';
}, 1000);

// clock
function updateClock() {
  const d = new Date();
  const h = String(d.getHours()).padStart(2,'0');
  const m = String(d.getMinutes()).padStart(2,'0');
  const s = String(d.getSeconds()).padStart(2,'0');
  document.getElementById('sb-time').textContent = h + ':' + m + ':' + s;
}
setInterval(updateClock, 1000);
updateClock();

// main command loop
setInterval(cmdLoop, CMD_RATE);

// initial connection probe
setTimeout(() => sendCmd('STATUS', true), 500);
</script>
</body>
</html>

)rawliteral";