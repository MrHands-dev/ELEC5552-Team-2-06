#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Types.h"
#include "motors.h"

// ---- Auto-hover forward decls ----
void  Auto_start(float setpoint_m);
void  Auto_estop();
bool  Auto_isActive();
float Auto_getSetpointM();
void  Auto_setSetpoint(float setpoint_m);

static WebServer server(80);

// SoftAP creds
#ifndef TEST_AP_SSID
  #define TEST_AP_SSID "DRONE-TEST"
#endif
#ifndef TEST_AP_PASS
  #define TEST_AP_PASS "test1234"
#endif

extern TelemetryState gState;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name=viewport content="width=device-width, initial-scale=1">
<title>ESP32 Drone — Dual Joysticks</title>
<style>
  :root{--bg:#0b0f14;--card:#121821;--ring:#2a3442;--knob:#e5e7eb;--txt:#e6eef8;--muted:#94a3b8;--good:#16a34a;--warn:#d97706;--bad:#dc2626;--btn:#1f2937}
  *{box-sizing:border-box}
  body{margin:0;padding:16px;background:var(--bg);color:var(--txt);font-family:system-ui,Segoe UI,Roboto,Arial}
  h1{font-size:18px;margin:0 0 12px}
  .grid{display:grid;gap:12px}
  .row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
  .card{background:var(--card);border:1px solid #1b2330;border-radius:16px;padding:14px;box-shadow:0 2px 10px rgba(0,0,0,.25)}
  .pill{padding:4px 10px;border-radius:999px;border:1px solid #334155;font-size:12px}
  .ok{background:rgba(22,163,74,.15);border-color:#14532d}
  .warn{background:rgba(217,119,6,.15);border-color:#7c2d12}
  .bad{background:rgba(220,38,38,.15);border-color:#7f1d1d}
  button{background:var(--btn);color:var(--txt);border:1px solid #334155;border-radius:10px;padding:10px 14px;cursor:pointer}
  button.primary{background:#0b3c91}
  button.danger{background:#7a0b0b}
  label{font-size:12px;color:var(--muted)}
  input[type=number]{width:100%;background:#0c121a;color:var(--txt);border:1px solid #334155;border-radius:8px;padding:8px}
  .kv{display:grid;grid-template-columns:150px 1fr;gap:6px 10px}
  /* Joysticks always side-by-side */
  .joy-wrap{display:grid;grid-template-columns:1fr 1fr;gap:12px;min-width:680px}
  .joy-scroll{overflow-x:auto}
  .pad{position:relative;aspect-ratio:1/1;background:radial-gradient(circle at 50% 50%, #15202b 0 60%, #111827 61% 100%);border:2px solid var(--ring);border-radius:18px;touch-action:none;user-select:none}
  .ring{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);width:80%;height:80%;border:2px dashed #304055;border-radius:50%}
  .gridlines{position:absolute;inset:10%;border-radius:12px;border:1px dashed #233243}
  .knob{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);width:22%;height:22%;border-radius:50%;background:var(--knob);box-shadow:0 6px 16px rgba(0,0,0,.4), inset 0 0 0 3px rgba(0,0,0,.2)}
  .legend{position:absolute;left:10px;top:10px;font-size:12px;color:var(--muted)}
  .readout{position:absolute;right:10px;bottom:10px;font-size:12px;color:#cbd5e1;text-align:right}
</style>
</head>
<body>
  <h1>ESP32 Drone — Dual Joysticks</h1>

  <div class="grid">
    <!-- Arming -->
    <div class="card">
      <div class="row">
        <button onclick="arm(1)" class="primary">ARM</button>
        <button onclick="arm(0)" class="danger">DISARM</button>
        <span id="armed" class="pill warn">—</span>
        <button id="manualBtn" onclick="toggleManual()">Enable Manual</button>
        <span id="mode" class="pill warn">Mode —</span>
      </div>
      <p style="margin:8px 0 0;color:var(--muted);font-size:12px">
        Left stick: <b>Throttle (center = 0)</b> / Yaw. Right stick: Roll / Pitch. Throttle changes while held; center to hold.
      </p>
    </div>

    <!-- Status -->
    <div class="card">
      <div class="kv">
        <div>Connection</div><div><span id="conn" class="pill warn">Waiting…</span></div>
        <div>Flight Mode</div><div><span id="mode2" class="pill warn">—</span></div>
        <div>Crash</div><div><span id="crash" class="pill warn">NO</span></div>
        <div>Hovering</div><div><span id="hover" class="pill warn">NO</span></div>
        <div>Auto-hover</div><div id="autoStatus">—</div>
      </div>
    </div>

    <!-- Joysticks -->
    <div class="card joy-scroll">
      <div class="joy-wrap">
        <!-- LEFT: Throttle (rate, centered 0) / Yaw -->
        <div id="padL" class="pad">
          <div class="legend">THR (center=0) / YAW</div>
          <div class="gridlines"></div>
          <div class="ring"></div>
          <div id="knobL" class="knob"></div>
          <div class="readout">
            <div>Thr: <span id="thrV">0%</span></div>
            <div>Yaw: <span id="yawV">0%</span></div>
          </div>
        </div>
        <!-- RIGHT: Roll / Pitch -->
        <div id="padR" class="pad">
          <div class="legend">ROLL / PITCH</div>
          <div class="gridlines"></div>
          <div class="ring"></div>
          <div id="knobR" class="knob"></div>
          <div class="readout">
            <div>Roll: <span id="rollV">0°</span></div>
            <div>Pitch: <span id="pitchV">0°</span></div>
          </div>
        </div>
      </div>
      <div class="row" style="margin-top:12px">
        <button onclick="centerSticks()">Center Sticks</button>
      </div>
    </div>

    <!-- Auto Hover -->
    <div class="card">
      <div class="row" style="gap:8px;align-items:end">
        <div style="flex:1">
          <label>Setpoint (m)</label>
          <input id="spm" type="number" min="0.2" max="3" step="0.1" value="0.6">
        </div>
        <button class="primary" onclick="autoStart()">Start Auto-Hover</button>
        <button onclick="applySp()">Apply SP</button>
        <button class="danger" onclick="autoEstop()">E-STOP</button>
      </div>
    </div>

    <!-- Telemetry -->
    <div class="card">
      <div class="kv">
        <div>IMU</div><div id="imuR">—</div>
        <div>Roll</div><div id="imuRoll">—</div>
        <div>Pitch</div><div id="imuPitch">—</div>
        <div>Yaw rate</div><div id="imuYaw">—</div>
        <div>ToF1</div><div id="tof1">—</div>
        <div>ToF2</div><div id="tof2">—</div>
      </div>
    </div>
  </div>

<script>
/* ========= Helpers ========= */
function setBadge(el, on, clsOn="ok", txtOn="OK", txtOff="NO"){
  el.className = "pill " + (on ? clsOn : "warn");
  el.textContent = on ? txtOn : txtOff;
}
async function arm(v){ await fetch('/api/arm?val='+v); refresh(); }
async function autoStart(){ const sp=Number(document.getElementById('spm').value||0.6); await fetch('/api/auto_start?sp='+sp); refresh(); }
async function applySp(){ const sp=Number(document.getElementById('spm').value||0.6); await fetch('/api/auto_setpoint?sp='+sp); refresh(); }
async function autoEstop(){ await fetch('/api/estop'); refresh(); }

async function toggleManual(){
  const wantOn = !state.manual;
  await fetch('/api/manual?val='+(wantOn?1:0));
  state.manual = wantOn;
  if(!state.manual) centerSticks(true);
  updateModePills();
}

function updateModePills(){
  const p1 = document.getElementById('mode');  // top row
  const p2 = document.getElementById('mode2'); // status card
  if(state.manual){
    p1.className="pill warn"; p1.textContent="Manual"; document.getElementById('manualBtn').textContent="Disable Manual";
    p2.className="pill warn"; p2.textContent="Manual";
  } else {
    p1.className="pill ok"; p1.textContent="Auto"; document.getElementById('manualBtn').textContent="Enable Manual";
    p2.className="pill ok"; p2.textContent="Auto";
  }
}

/* ========= Joysticks =========
   Left pad (centered throttle):
     x -> yaw [-1..1]
     y -> throttle RATE command in [-1..1] (up=+)
     GUI integrates this into absolute throttle 0..1 and sends.
   Right pad:
     x -> roll ±deg, y -> pitch ±deg (up=+)
*/
const limits = { rollDeg: 20, pitchDeg: 20 };
const THR_RATE_PER_SEC = 0.30; // 30%/s throttle change at full deflection (tweak to taste)

const state = {
  manual: false,
  yaw: 0.0,
  thr: 0.0,        // absolute throttle 0..1
  thrAxis: 0.0,    // -1..1 from stick (centered at 0)
  roll: 0.0, pitch: 0.0,
  sending: false
};

const padL = document.getElementById('padL');
const knobL= document.getElementById('knobL');
const padR = document.getElementById('padR');
const knobR= document.getElementById('knobR');
const thrV = document.getElementById('thrV');
const yawV = document.getElementById('yawV');
const rollV= document.getElementById('rollV');
const pitchV=document.getElementById('pitchV');

function centerSticks(silent){
  state.yaw=0; state.thrAxis=0; state.roll=0; state.pitch=0;
  placeKnob(padL, knobL, 0, 0);
  placeKnob(padR, knobR, 0, 0);
  if(!silent) sendJoy(true);
}

function clamp(v,min,max){ return Math.max(min, Math.min(max,v)); }

/* Pointer helpers */
function padHandlers(pad, knob, onMove){
  let active=false, id=null;
  pad.addEventListener('pointerdown', e=>{
    if(!state.manual) return;
    active=true; id=e.pointerId; pad.setPointerCapture(id); handle(e);
  });
  pad.addEventListener('pointermove', e=>{ if(active && e.pointerId===id) handle(e); });
  ['pointerup','pointercancel','lostpointercapture','pointerleave'].forEach(ev=>{
    pad.addEventListener(ev, e=>{
      if(active && e.pointerId===id){
        active=false; id=null;
        // spring to center
        placeKnob(pad, knob, 0, 0);
        onMove(0,0,true);
      }
    });
  });
  function handle(e){
    const r = pad.getBoundingClientRect();
    const cx = r.left + r.width/2, cy = r.top + r.height/2;
    const dx = (e.clientX - cx) / (r.width/2);
    const dy = (e.clientY - cy) / (r.height/2);
    const mag = Math.hypot(dx,dy);
    const k = mag>1 ? 1/mag : 1;
    const x = clamp(dx*k,-1,1);
    const y = clamp(dy*k,-1,1);
    placeKnob(pad, knob, x, y);
    onMove(x, y, false);
  }
}

function placeKnob(pad, knob, nx, ny){
  const r = pad.getBoundingClientRect();
  const x = (nx*0.38 + 0.5) * r.width;
  const y = (ny*0.38 + 0.5) * r.height;
  knob.style.left = x+"px"; knob.style.top = y+"px";
}

// LEFT: x→yaw [-1..1], y→thrAxis [-1..1]  (invert y: up=+)
padHandlers(padL, knobL, (x,y,release)=>{
  state.yaw     = clamp(x,-1,1);
  state.thrAxis = clamp(-y,-1,1); // centered at 0, up=+1
  yawV.textContent = Math.round(state.yaw*100)+"%";
  // thr readout shows absolute throttle
  thrV.textContent = Math.round(state.thr*100)+"%";
  sendJoy(release);
});

// RIGHT: x→roll ±deg, y→pitch ±deg  (invert y: up=+)
padHandlers(padR, knobR, (x,y,release)=>{
  state.roll  = clamp(x * limits.rollDeg,  -limits.rollDeg,  limits.rollDeg);
  state.pitch = clamp(-y * limits.pitchDeg, -limits.pitchDeg, limits.pitchDeg);
  rollV.textContent  = Math.round(state.roll)+"°";
  pitchV.textContent = Math.round(state.pitch)+"°";
  sendJoy(release);
});

/* Integrate throttle rate → absolute throttle (0..1) */
let lastT = performance.now();
function tick(){
  const now = performance.now();
  const dt = Math.max(0, (now - lastT) / 1000);
  lastT = now;

  if(state.manual){
    // Apply rate only when manual mode is on
    const delta = THR_RATE_PER_SEC * state.thrAxis * dt; // ± per second
    state.thr = clamp(state.thr + delta, 0, 1);
    thrV.textContent = Math.round(state.thr*100)+"%";
    // Send at ~20 Hz
    if(now - lastSend > 50) sendJoy(true);
  }
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

let lastSend=0;
async function sendJoy(force){
  if(!state.manual) return;
  const now = performance.now();
  if(!force && now - lastSend < 50) return; // ~20 Hz
  lastSend = now;
  const url = `/api/joy?roll=${state.roll.toFixed(1)}&pitch=${state.pitch.toFixed(1)}&yaw=${state.yaw.toFixed(3)}&thr=${state.thr.toFixed(3)}`;
  try{ await fetch(url,{cache:'no-store'});}catch(e){}
}

/* ========= Telemetry refresh ========= */
async function refresh(){
  try{
    const r = await fetch('/api/state',{cache:'no-store'});
    const s = await r.json();

    // connection & arming
    setBadge(document.getElementById('conn'), !s.conn.lost && s.conn.stable, s.conn.lost?'bad':'ok', s.conn.lost?'Lost':'Stable connection', 'Waiting…');
    const armedEl = document.getElementById('armed');
    armedEl.className = "pill " + (s.armed?'ok':'warn');
    armedEl.textContent = s.armed ? 'ARMED' : 'DISARMED';

    // mode pills
    state.manual = !!s.mode.manual;
    updateModePills();

    // status
    setBadge(document.getElementById('crash'), s.status.crashed, 'bad', 'TRIGGERED', 'NO');
    setBadge(document.getElementById('hover'), s.status.hovering, 'ok', 'YES', 'NO');
    document.getElementById('autoStatus').textContent = s.auto.active ? ('ACTIVE @ '+s.auto.sp_m.toFixed(2)+' m') : 'INACTIVE';
    document.getElementById('spm').value = s.auto.sp_m.toFixed(2);

    // IMU/ToF
    document.getElementById('imuR').textContent    = s.imu.ready ? 'Ready' : 'Not ready';
    document.getElementById('imuRoll').textContent  = s.imu.roll.toFixed(1)+' °';
    document.getElementById('imuPitch').textContent = s.imu.pitch.toFixed(1)+' °';
    document.getElementById('imuYaw').textContent   = s.imu.yawRate.toFixed(1)+' °/s';
    document.getElementById('tof1').textContent     = s.tof.tof1Ready ? (s.tof.tof1mm+' mm') : '—';
    document.getElementById('tof2').textContent     = s.tof.tof2Ready ? (s.tof.tof2mm+' mm') : '—';

    // If manual is off, mirror current throttle/yaw
    if(!state.manual){
      state.thr = clamp(s.throttle || 0, 0, 1);
      thrV.textContent = Math.round(state.thr*100)+'%';
      yawV.textContent = Math.round((s.yawBias||0)*100)+'%';
    }
  }catch(e){}
}
centerSticks(true);
refresh(); setInterval(refresh, 500);
</script>
</body></html>
)HTML";

/* =================== Server plumbing =================== */
void GUI_begin(){
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(TEST_AP_SSID, TEST_AP_PASS);
  if(ok){
    Serial.printf("SoftAP: %s  IP: %s\n", TEST_AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("SoftAP start failed.");
  }
  server.begin();
}

void GUI_attachRoutes(TelemetryState& st){
  server.on("/", HTTP_GET, [](){ server.send_P(200, "text/html", INDEX_HTML); });

  server.on("/api/state", HTTP_GET, [&st](){
    st.lastClientMs = millis();  // heartbeat from GUI poll

    server.sendHeader("Cache-Control", "no-store, max-age=0, must-revalidate");
    float yawBias = st.yawRateCmdDps/180.0f;
    String j = "{";
    j += "\"armed\":" + String(Motors_isArmed() ? "true":"false") + ",";
    j += "\"throttle\":" + String(st.pilotThrottle,3) + ",";
    j += "\"yawBias\":" + String(yawBias,3) + ",";
    j += "\"imu\":{";
      j += "\"ready\":" + String(st.imuReady ? "true":"false") + ",";
      j += "\"roll\":"  + String(st.rollDeg,2) + ",";
      j += "\"pitch\":" + String(st.pitchDeg,2) + ",";
      j += "\"yawRate\":"+ String(st.yawRateDps,2);
    j += "},";
    j += "\"tof\":{";
      j += "\"tof1Ready\":" + String(st.tof1Ready ? "true":"false") + ",";
      j += "\"tof1mm\":"    + String(st.tof1mm) + ",";
      j += "\"tof2Ready\":" + String(st.tof2Ready ? "true":"false") + ",";
      j += "\"tof2mm\":"    + String(st.tof2mm);
    j += "},";
    j += "\"auto\":{";
      j += "\"active\":" + String(Auto_isActive() ? "true":"false") + ",";
      j += "\"sp_m\":"   + String(Auto_getSetpointM(),3);
    j += "},";
    j += "\"status\":{";
      j += "\"crashed\":"  + String(st.crashed  ? "true":"false") + ",";
      j += "\"hovering\":" + String(st.hovering ? "true":"false");
    j += "},";
    j += "\"conn\":{";
      j += "\"lost\":"   + String(st.connectionLost   ? "true":"false") + ",";
      j += "\"stable\":" + String(st.connectionStable ? "true":"false");
    j += "},";
    j += "\"mode\":{";
      j += "\"manual\":" + String(st.manualOverride ? "true":"false");
    j += "}";
    j += "}";
    server.send(200, "application/json", j);
  });

  // ARM/DISARM
  server.on("/api/arm", HTTP_ANY, [&st](){
    String v = server.hasArg("val") ? server.arg("val") : "";
    if(v=="1" || v=="true"){
      st.crashed = false;
      st.connectionLost = false;
      Motors_arm();
      server.send(200, "text/plain", "ARMED");
    } else {
      Motors_disarm();
      server.send(200, "text/plain", "DISARMED");
    }
  });

  // Manual override toggle
  server.on("/api/manual", HTTP_ANY, [&st](){
    bool enable=false;
    if(server.hasArg("val")){
      String v=server.arg("val");
      enable=(v=="1"||v=="true"||v=="on");
    }
    st.manualOverride = enable;
    server.send(200, "text/plain", String("MANUAL=")+(enable?"ON":"OFF"));
  });

  // Joystick inputs (roll/pitch in deg, yaw in -1..1, thr in 0..1)
  server.on("/api/joy", HTTP_ANY, [&st](){
    if (server.hasArg("roll"))  st.joyRollDeg  = constrain(server.arg("roll").toFloat(),  -30.0f,  30.0f);
    if (server.hasArg("pitch")) st.joyPitchDeg = constrain(server.arg("pitch").toFloat(), -30.0f,  30.0f);
    if (server.hasArg("yaw"))   st.yawRateCmdDps = constrain(server.arg("yaw").toFloat(), -1.0f, 1.0f)*180.0f;
    if (server.hasArg("thr"))   st.pilotThrottle = constrain(server.arg("thr").toFloat(), 0.0f, 1.0f);
    server.send(200, "text/plain", "JOY_OK");
  });

  // Legacy throttle/yaw endpoints (still available)
  server.on("/api/throttle", HTTP_ANY, [&st](){
    float t = server.hasArg("val") ? server.arg("val").toFloat() : 0.0f;
    st.pilotThrottle = constrain(t, 0.0f, 1.0f);
    server.send(200, "text/plain", String("THROTTLE=")+String(st.pilotThrottle,3));
  });
  server.on("/api/yaw", HTTP_ANY, [&st](){
    float v = server.hasArg("val") ? server.arg("val").toFloat() : 0.0f;
    st.yawRateCmdDps = constrain(v,-1.0f,1.0f)*180.0f;
    server.send(200, "text/plain", String("YAW_BIAS=")+String(v,3));
  });

  // Debug pin drive
  server.on("/api/testpin", HTTP_ANY, [](){
    int io = server.hasArg("io") ? server.arg("io").toInt() : -1;
    float duty = server.hasArg("duty") ? server.arg("duty").toFloat() : 0.5f;
    uint32_t hz = server.hasArg("freq") ? (uint32_t)server.arg("freq").toInt() : 20000u;
    if(io<0){ server.send(400,"text/plain","Missing io"); return; }
    bool ok = Motors_debugAttachAndDrive(io,duty,hz);
    server.send(ok?200:500,"text/plain", ok?"OK":"Failed");
  });
  server.on("/api/teststop", HTTP_ANY, [](){ Motors_debugStop(); server.send(200,"text/plain","Stopped"); });

  // Auto-hover routes
  server.on("/api/auto_start", HTTP_ANY, [](){
    float sp = server.hasArg("sp") ? server.arg("sp").toFloat() : 0.6f;
    sp = constrain(sp,0.2f,3.0f);
    Auto_start(sp);
    server.send(200,"text/plain","AUTO_START");
  });
  server.on("/api/auto_setpoint", HTTP_ANY, [](){
    float sp = server.hasArg("sp") ? server.arg("sp").toFloat() : 0.6f;
    sp = constrain(sp,0.2f,3.0f);
    Auto_setSetpoint(sp);
    server.send(200,"text/plain","AUTO_SETPOINT");
  });
  server.on("/api/estop", HTTP_ANY, [](){ Auto_estop(); server.send(200,"text/plain","ESTOP"); });

  server.begin();
}

void GUI_loop(){ server.handleClient(); }
