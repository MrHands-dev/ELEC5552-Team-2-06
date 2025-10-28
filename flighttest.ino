// flighttest.ino
// ESP32-S3 WROOM-1 — Dual ToF + IMU + Motors + Web GUI + Auto Hover + Manual Override
// Includes: tilt crash kill, connection-loss landing, manual override joystick mode

#include <Arduino.h>
#include <math.h>
#include "Types.h"
#include "IMU.h"
#include "ToF.h"
#include "motors.h"
#include "PID.h"
#include "GUI.h"

// ===== Global telemetry state (shared across modules) =====
TelemetryState gState;

// ===== Simple attitude hold gains (deg -> motor correction fraction) =====
static const float KROLL  = 0.015f;   // fraction per degree
static const float KPITCH = 0.015f;   // fraction per degree

// ===== Timing helpers =====
static uint32_t gLastUs = 0;

static inline float dtSeconds() {
  uint32_t now = micros();
  uint32_t du  = (now - gLastUs);
  gLastUs = now;
  if (du == 0) du = 1000;        // ~1 ms fallback
  return du * 1e-6f;
}

// ===== Crash detection parameters =====
static const float CRASH_TILT_DEG   = 60.0f;   // threshold for |roll| or |pitch|
static const uint32_t CRASH_HOLD_MS = 150;     // require tilt for this long
static uint32_t crashTiltStartMs    = 0;

// ===== Communication loss failsafe =====
static const uint32_t COMM_LOSS_MS       = 30000;  // 30 s without GUI poll => lost
static const float    LAND_RATE_MPS      = 0.20f;  // descend setpoint 0.2 m/s
static const float    LAND_MIN_ALT_M     = 0.15f;  // consider landed below this
static const float    LAND_MIN_THR       = 0.06f;  // keep small thrust while descending
static const float    LAND_THR_RAMP_DN   = 0.40f;  // throttle/sec ramp down after touchdown

static inline bool isTiltBeyondCrash(float rollDeg, float pitchDeg) {
  return (fabsf(rollDeg) >= CRASH_TILT_DEG) || (fabsf(pitchDeg) >= CRASH_TILT_DEG);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== flighttest (ESP32-S3) ===");

  // ----- Motors (PWM) -----
  Motors_setup();

  // ----- GUI (SoftAP + HTTP routes) -----
  GUI_begin();
  GUI_attachRoutes(gState);

  // ----- IMU (sets up its own I2C on IO11/IO10) -----
  IMU_setup(gState);

  // ----- ToF sensors (own TwoWire bus on IO40/41, readdresses if needed) -----
  ToF_setup(gState);

  // Initialize command/status defaults
  gState.pilotThrottle   = 0.0f;
  gState.yawRateCmdDps   = 0.0f;
  gState.desiredRollDeg  = 0.0f;
  gState.desiredPitchDeg = 0.0f;

  gState.manualOverride  = false;
  gState.joyRollDeg      = 0.0f;
  gState.joyPitchDeg     = 0.0f;

  gState.crashed         = false;
  gState.hovering        = false;

  gState.lastClientMs    = 0;        // no GUI seen yet
  gState.connectionLost  = false;
  gState.connectionStable= false;

  gLastUs = micros();
  Serial.println("Setup complete.");
}

void loop() {
  const float dt = dtSeconds();

  // --- Service GUI HTTP ---
  GUI_loop();

  // --- Sensors ---
  IMU_update(gState);
  ToF_update(gState);

  // ===== Compute connection status =====
  const uint32_t nowMs = millis();
  if (gState.lastClientMs == 0) {
    gState.connectionStable = false;
    gState.connectionLost   = (nowMs > COMM_LOSS_MS);
  } else {
    const uint32_t idle = nowMs - gState.lastClientMs;
    gState.connectionStable = (idle <= 2000);
    if (!gState.connectionLost && idle >= COMM_LOSS_MS && Motors_isArmed()) {
      gState.connectionLost = true;
      Serial.println("!! CONNECTION LOST — initiating safe landing");
    }
  }

  // ===== Crash detection (tilt-based, latching) =====
  if (Motors_isArmed()) {
    const bool beyond = isTiltBeyondCrash(gState.rollDeg, gState.pitchDeg);
    if (beyond) {
      if (crashTiltStartMs == 0) crashTiltStartMs = nowMs;
      const uint32_t held = nowMs - crashTiltStartMs;
      if (held >= CRASH_HOLD_MS && !gState.crashed) {
        gState.crashed = true;     // latch
        Auto_estop();              // cuts power and disarms safely
        Serial.println("!! CRASH DETECTED -> E-STOP, motors disarmed");
      }
    } else {
      crashTiltStartMs = 0;
    }
  } else {
    crashTiltStartMs = 0;
  }

  // If crashed, keep outputs zeroed and skip control (GUI still runs)
  if (gState.crashed) {
    gState.pilotThrottle = 0.0f;
    Motors_update(dt);
    return;
  }

  // ===== Communication-loss landing =====
  if (gState.connectionLost && Motors_isArmed()) {
    if (gState.tof1Ready) {
      const float altM = Auto_getAltM();
      if (!Auto_isActive()) {
        float startAlt = (altM > 0.2f ? altM : 0.2f);
        Auto_start(startAlt);
      }
      float newSp = Auto_getSetpointM() - LAND_RATE_MPS * dt;
      if (newSp < 0.20f) newSp = 0.20f;
      Auto_setSetpoint(newSp);

      if (gState.pilotThrottle < LAND_MIN_THR) gState.pilotThrottle = LAND_MIN_THR;

      if (altM < LAND_MIN_ALT_M) {
        gState.pilotThrottle = max(0.0f, gState.pilotThrottle - LAND_THR_RAMP_DN * dt);
        if (gState.pilotThrottle <= 0.01f) {
          Auto_estop();
          Serial.println("Safe landing complete — disarmed due to connection loss.");
        }
      }
    } else {
      gState.pilotThrottle = max(0.0f, gState.pilotThrottle - LAND_THR_RAMP_DN * dt);
      if (gState.pilotThrottle <= 0.01f) {
        Auto_estop();
        Serial.println("Landing (no ToF): throttle to zero — disarmed.");
      }
    }
    Motors_update(dt);
    return; // skip normal control while landing
  }

  // ===== Control path selection =====
  if (gState.manualOverride) {
    // Manual mode: joysticks drive the desired attitude (deg) and throttle/yaw
    gState.desiredRollDeg  = gState.joyRollDeg;
    gState.desiredPitchDeg = gState.joyPitchDeg;
    // yawRateCmdDps and pilotThrottle are already updated by /api/joy
  } else {
    // Auto-hover path: desired angles usually 0 (level)
    gState.desiredRollDeg  = 0.0f;
    gState.desiredPitchDeg = 0.0f;
    // Auto altitude loop
    Auto_update(dt);
  }

  // --- Attitude stabilization (simple P around desired angles) ---
  const float rollErrDeg  = (gState.desiredRollDeg  - gState.rollDeg);
  const float pitchErrDeg = (gState.desiredPitchDeg - gState.pitchDeg);

  float rollCorr  = KROLL  * rollErrDeg;
  float pitchCorr = KPITCH * pitchErrDeg;

  Motors_setStabCorrs(rollCorr, pitchCorr);

  // --- Hovering status heuristic (relaxed). Manual mode never claims hovering.
  bool flat   = (fabsf(gState.rollDeg) < 12.0f) && (fabsf(gState.pitchDeg) < 12.0f);
  bool autoOk = (!gState.manualOverride) && Auto_isActive();
  bool tofOk  = gState.tof1Ready;
  float altM  = Auto_getAltM();
  float spM   = Auto_getSetpointM();
  bool altNear= (tofOk && autoOk && fabsf(altM - spM) < 0.08f);
  bool thrOk  = (gState.pilotThrottle > 0.06f && gState.pilotThrottle < 0.70f);

  gState.hovering = Motors_isArmed() && !gState.crashed && gState.imuReady &&
                    flat && autoOk && altNear && thrOk && !gState.connectionLost;

  // --- Drive motors ---
  Motors_update(dt);
}
