#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Types.h"
#include "motors.h"

// ---- Auto-hover forward decelarations ----
void  Auto_start(float setpoint_m);
void  Auto_estop();
bool  Auto_isActive();
float Auto_getSetpointM();
void  Auto_setSetpoint(float setpoint_m);

// ---- Camera forward declarations ----
bool CAM_isReady();
bool CAM_isRecording();
bool CAM_startRecording(uint32_t frame_interval_ms = 0);
void CAM_stopRecording();

static WebServer server(80);

// SoftAP credentials
#ifndef TEST_AP_SSID
  #define TEST_AP_SSID "DRONE-TEST"
#endif
#ifndef TEST_AP_PASS
  #define TEST_AP_PASS "test1234"
#endif

extern TelemetryState gState;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name=viewport content="width=device-width, initial-scale=1">
<title>ESP32 Drone — Dual Joysticks + Camera</title>
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
  <h1>ESP32 Drone — Dual Joysticks + Camera</h1>

  <div class="grid">
    <!-- Arming + Flight mode -->
    <div class="card">
      <div class="row">
        <button onclick="arm(1)" class="primary">ARM</button>
        <button onclick="arm(0)" class="danger">DISARM</button>
        <span id="armed" class="pill warn">—</span>
        <button id="manualBtn" onclick="toggleManual()">Enable Manual</button>
        <span id="mode" class="pill warn">Mode —</span>

        <!-- Camera control -->
        <button id="recStartBtn" class="primary" onclick="startRec()">Start Recording</button>
        <button id="recStopBtn" class="danger"  onclick="stopRec()">Stop Recording</button>
        <span id="camPill" class="pill warn">Cam —</span>
      </div>
      <p style="margin:8px 0 0;color:var(--muted);font-size:12px">
        Left stick: <b>Throttle (center = 0)</b> / Yaw. Right stick: Roll / Pitch.
      </p>
    </div>

    <!-- Status -->
    <div class="card">
      <div class="kv">
        <div>Connection</div><div><span id="conn" class="pill warn">Waiting…</span></div>
        <div>Flight Mode</div><div><span id="mode2" class="pill warn">—</span></div>
        <div>Crash</div><div><span id="crash" class="pill warn">NO</span></div>
        <div>Hovering</div><div><span id="hover" class="pill warn">NO</span></div>
        <div>Proximity</div><div><span id="prox" class="pill warn">CLEAR</span></div>
        <div>Auto-hover</div><div id="autoStatus">—</div>
      </div>
    </div>

    <!-- Joysticks -->
    <div class="card joy-scroll">
      <div class="joy-wrap">
        <div id="padL" class="pad">
          <div class="legend">THR (center=0) / YAW</div>
          <div class="gridlines"></div><div class="ring"></div><div id="knobL" class="knob"></div>
          <div class="readout"><div>Thr: <span id="thrV">0%</span></div><div>Yaw: <span id="yawV">0%</span></div></div>
        </div>
        <div id="padR" class="pad">
          <div class="legend">ROLL / PITCH</div>
          <div class="gridlines"></div><div class="ring"></div><div id="knobR" class="knob"></div>
          <div class="readout"><div>Roll: <span id="rollV">0°</span></div><div>Pitch: <span id="pitchV">0°</span></div></div>
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
function setBadge(el, on, clsOn="ok", txtOn="OK", txtOff="NO"){
  el.className = "pill " + (on ? clsOn : "warn");
  el.textContent = on ? txtOn : txtOff;
}
async function arm(v){ await fetch('/api/arm?val='+v); refresh(); }
async function autoStart(){ const sp=Number(document.getElementById('spm').value||0.6); await fetch('/api/auto_start?sp='+sp); refresh(); }
async function applySp(){ const sp=Number(document.getElementById('spm').value||0.6); await fetch('/api/auto_setpoint?sp='+sp); refresh(); }
async function autoEstop(){ await fetch('/api/estop'); refresh(); }

async function startRec(){ try{ await fetch('/api/cam_start'); }catch(e){} refresh(); }
async function stopRec(){  try{ await fetch('/api/cam_stop');  }catch(e){} refresh(); }

async function toggleManual(){
  const wantOn = !state.manual;
  await fetch('/api/manual?val='+(wantOn?1:0));
  state.manual = wantOn;
  if(!state.manual) centerSticks(true);
  updateModePills();
}

function updateCamPill(){
  const pill = document.getElementById('camPill');
  if(state.camReady){
    if(state.rec){ pill.className="pill ok";   pill.textContent="Recording"; }
    else         { pill.className="pill warn"; pill.textContent="Cam Ready"; }
  } else {
    pill.className="pill bad"; pill.textContent="Cam Not Ready";
  }
}

function updateModePills(){
  const p1 = document.getElementById('mode');
  const p2 = document.getElementById('mode2');
  if(state.manual){
    p1.className="pill warn"; p1.textContent="Manual"; document.getElementById('manualBtn').textContent="Disable Manual";
    p2.className="pill warn"; p2.textContent="Manual";
  } else {
    p1.className="pill ok"; p1.textContent="Auto"; document.getElementById('manualBtn').textContent="Enable Manual";
    p2.className="pill ok"; p2.textContent="Auto";
  }
}

const limits = { rollDeg: 20, pitchDeg: 20 };
const THR_RATE_PER_SEC = 0.30;

const state = { manual:false, yaw:0, thr:0, thrAxis:0, roll:0, pitch:0, rec:false, camReady:false, prox:false };

const padL = document.getElementById('padL'), knobL= document.getElementById('knobL');
const padR = document.getElementById('padR'), knobR= document.getElementById('knobR');
const thrV = document.getElementById('thrV'), yawV=document.getElementById('yawV');
const rollV= document.getElementById('rollV'), pitchV=document.getElementById('pitchV');

function centerSticks(silent){
  state.yaw=0; state.thrAxis=0; state.roll=0; state.pitch=0; state.thr=0;
  placeKnob(padL, knobL, 0, 0); placeKnob(padR, knobR, 0, 0);
  thrV.textContent="0%"; if(!silent) sendJoy(true);
}
function clamp(v,min,max){ return Math.max(min, Math.min(max,v)); }

function padHandlers(pad, knob, onMove){
  let active=false, id=null;
  pad.addEventListener('pointerdown', e=>{ if(!state.manual) return; active=true; id=e.pointerId; pad.setPointerCapture(id); handle(e); });
  pad.addEventListener('pointermove', e=>{ if(active && e.pointerId===id) handle(e); });
  ['pointerup','pointercancel','lostpointercapture','pointerleave'].forEach(ev=>{
    pad.addEventListener(ev, e=>{
      if(active && e.pointerId===id){ active=false; id=null; placeKnob(pad, knob, 0, 0);
        if(pad.id==='padL'){ state.thrAxis=0; state.thr=0; thrV.textContent="0%"; }
        onMove(0,0,true);
      }
    });
  });
  function handle(e){
    const r = pad.getBoundingClientRect(), cx=r.left+r.width/2, cy=r.top+r.height/2;
    const dx=(e.clientX-cx)/(r.width/2), dy=(e.clientY-cy)/(r.height/2);
    const mag=Math.hypot(dx,dy), k=mag>1?1/mag:1, x=clamp(dx*k,-1,1), y=clamp(dy*k,-1,1);
    placeKnob(pad, knob, x, y); onMove(x, y, false);
  }
}
function placeKnob(pad, knob, nx, ny){
  const r = pad.getBoundingClientRect();
  knob.style.left = ((nx*0.38 + 0.5) * r.width) +"px";
  knob.style.top  = ((ny*0.38 + 0.5) * r.height)+"px";
}

padHandlers(padL, knobL, (x,y,release)=>{
  state.yaw     = clamp(x,-1,1);
  state.thrAxis = clamp(-y,-1,1);
  yawV.textContent = Math.round(state.yaw*100)+"%";
  thrV.textContent = Math.round(state.thr*100)+"%";
  sendJoy(release);
});
padHandlers(padR, knobR, (x,y,release)=>{
  state.roll  = clamp(x * limits.rollDeg,  -limits.rollDeg,  limits.rollDeg);
  state.pitch = clamp(-y * limits.pitchDeg, -limits.pitchDeg, limits.pitchDeg);
  rollV.textContent  = Math.round(state.roll)+"°";
  pitchV.textContent = Math.round(state.pitch)+"°";
  sendJoy(release);
});

let lastT = performance.now(), lastSend=0;
function tick(){
  const now = performance.now(), dt = Math.max(0,(now-lastT)/1000); lastT = now;
  if(state.manual){
    const delta = THR_RATE_PER_SEC * state.thrAxis * dt;
    state.thr = clamp(state.thr + delta, 0, 1);
    thrV.textContent = Math.round(state.thr*100)+"%";
    if(now - lastSend > 50) sendJoy(true);
  }
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);

async function sendJoy(force){
  if(!state.manual) return;
  const now = performance.now(); if(!force && now - lastSend < 50) return; lastSend = now;
  const url = `/api/joy?roll=${state.roll.toFixed(1)}&pitch=${state.pitch.toFixed(1)}&yaw=${state.yaw.toFixed(3)}&thr=${state.thr.toFixed(3)}`;
  try{ await fetch(url,{cache:'no-store'});}catch(e){}
}

function setProxPill(on){
  const el = document.getElementById('prox');
  if(on){ el.className="pill bad"; el.textContent="OBJECT"; }
  else  { el.className="pill ok";  el.textContent="CLEAR"; }
}

async function refresh(){
  try{
    const r = await fetch('/api/state',{cache:'no-store'}); const s = await r.json();
    setBadge(document.getElementById('conn'), !s.conn.lost && s.conn.stable, s.conn.lost?'bad':'ok', s.conn.lost?'Lost':'Stable connection', 'Waiting…');
    const armedEl = document.getElementById('armed'); armedEl.className="pill "+(s.armed?'ok':'warn'); armedEl.textContent = s.armed?'ARMED':'DISARMED';
    state.manual = !!s.mode.manual; updateModePills();
    setBadge(document.getElementById('crash'), s.status.crashed, 'bad', 'TRIGGERED', 'NO');
    setBadge(document.getElementById('hover'), s.status.hovering, 'ok', 'YES', 'NO');
    document.getElementById('autoStatus').textContent = s.auto.active ? ('ACTIVE @ '+s.auto.sp_m.toFixed(2)+' m') : 'INACTIVE';
    document.getElementById('spm').value = s.auto.sp_m.toFixed(2);

    // Camera pills
    state.camReady = s.cam && s.cam.ready; state.rec = s.cam && s.cam.rec; updateCamPill();

    // IMU/ToF
    document.getElementById('imuR').textContent    = s.imu.ready ? 'Ready' : 'Not ready';
    document.getElementById('imuRoll').textContent  = s.imu.roll.toFixed(1)+' °';
    document.getElementById('imuPitch').textContent = s.imu.pitch.toFixed(1)+' °';
    document.getElementById('imuYaw').textContent   = s.imu.yawRate.toFixed(1)+' °/s';
    document.getElementById('tof1').textContent     = s.tof.tof1Ready ? (s.tof.tof1mm+' mm') : '—';
    document.getElementById('tof2').textContent     = s.tof.tof2Ready ? (s.tof.tof2mm+' mm') : '—';

    // NEW: Proximity pill
    state.prox = s.obj && s.obj.detected;
    setProxPill(!!state.prox);

    if(!state.manual){ state.thr = Math.max(0, Math.min(1, s.throttle || 0)); thrV.textContent = Math.round(state.thr*100)+'%'; yawV.textContent = Math.round((s.yawBias||0)*100)+'%'; }
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
    st.lastClientMs = millis();

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
    j += "},";
    j += "\"cam\":{";
      j += "\"ready\":" + String(CAM_isReady() ? "true":"false") + ",";
      j += "\"rec\":"   + String(CAM_isRecording() ? "true":"false");
    j += "},";
    // NEW: object detection block
    j += "\"obj\":{";
      j += "\"detected\":" + String(st.objectDetected ? "true":"false");
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

  // Manual override
  server.on("/api/manual", HTTP_ANY, [&st](){
    bool enable=false;
    if(server.hasArg("val")){
      String v=server.arg("val");
      enable=(v=="1"||v=="true"||v=="on");
    }
    st.manualOverride = enable;
    server.send(200, "text/plain", String("MANUAL=")+(enable?"ON":"OFF"));
  });

  // Joystick inputs
  server.on("/api/joy", HTTP_ANY, [&st](){
    if (server.hasArg("roll"))  st.joyRollDeg  = constrain(server.arg("roll").toFloat(),  -30.0f,  30.0f);
    if (server.hasArg("pitch")) st.joyPitchDeg = constrain(server.arg("pitch").toFloat(), -30.0f,  30.0f);
    if (server.hasArg("yaw"))   st.yawRateCmdDps = constrain(server.arg("yaw").toFloat(), -1.0f, 1.0f)*180.0f;
    if (server.hasArg("thr"))   st.pilotThrottle = constrain(server.arg("thr").toFloat(), 0.0f, 1.0f);
    server.send(200, "text/plain", "JOY_OK");
  });

  // Camera endpoints (separate)
  server.on("/api/cam_start", HTTP_ANY, [](){
    bool ok = CAM_startRecording();
    server.send(ok?200:500, "text/plain", ok?"REC_ON":"REC_FAIL");
  });
  server.on("/api/cam_stop", HTTP_ANY, [](){
    CAM_stopRecording();
    server.send(200, "text/plain", "REC_OFF_REQ");
  });

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
