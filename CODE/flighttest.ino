// flighttest.ino
// ESP32-S3 WROOM-1 — Motors + IMU + Dual ToF + Web GUI + Auto Hover + Manual Override
// + Camera (OV2640 SPI) AVI recording to SD (separate Start/Stop)

#include <Arduino.h>
#include <math.h>
#include "Types.h"
#include "IMU.h"
#include "ToF.h"
#include "motors.h"
#include "PID.h"
#include "GUI.h"
#include "CAM.h"

// ===== Global telemetry state =====
TelemetryState gState;

// ===== Simple attitude hold gains =====
static const float KROLL  = 0.015f;
static const float KPITCH = 0.015f;

// ===== Timing helpers =====
static uint32_t gLastUs = 0;
static inline float dtSeconds() {
  uint32_t now = micros();
  uint32_t du  = (now - gLastUs);
  gLastUs = now;
  if (du == 0) du = 1000;
  return du * 1e-6f;
}

// ===== Crash detection =====
static const float CRASH_TILT_DEG   = 60.0f;
static const uint32_t CRASH_HOLD_MS = 150;
static uint32_t crashTiltStartMs    = 0;

static inline bool isTiltBeyondCrash(float rollDeg, float pitchDeg) {
  return (fabsf(rollDeg) >= CRASH_TILT_DEG) || (fabsf(pitchDeg) >= CRASH_TILT_DEG);
}

// ===== Communication loss failsafe =====
static const uint32_t COMM_LOSS_MS       = 30000;
static const float    LAND_RATE_MPS      = 0.20f;
static const float    LAND_MIN_ALT_M     = 0.15f;
static const float    LAND_MIN_THR       = 0.06f;
static const float    LAND_THR_RAMP_DN   = 0.40f;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("=== flighttest (ESP32-S3) ===");

  Motors_setup();
  GUI_begin();
  GUI_attachRoutes(gState);
  IMU_setup(gState);
  ToF_setup(gState);

  // Camera setup
  if (!CAM_setup()) {
    Serial.println("[flighttest] Camera not ready (continuing without recording).");
  }

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

  gState.lastClientMs    = 0;
  gState.connectionLost  = false;
  gState.connectionStable= false;

  gState.objectDetected  = false;

  gLastUs = micros();
  Serial.println("Setup complete.");
}

void loop() {
  const float dt = dtSeconds();

  GUI_loop();
  IMU_update(gState);
  ToF_update(gState);

  // Object detection from ToF1 (≤ 300 mm)
  static bool lastDet = false;
  bool det = (gState.tof1Ready && gState.tof1mm > 0 && gState.tof1mm <= 300);
  if (det != lastDet) {
    lastDet = det;
    gState.objectDetected = det;
    if (det) Serial.println("[ToF] Object detected within 30 cm");
    else     Serial.println("[ToF] Object cleared");
  } else {
    gState.objectDetected = det;
  }

  // Connection status
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

  // Crash detection
  if (Motors_isArmed()) {
    const bool beyond = isTiltBeyondCrash(gState.rollDeg, gState.pitchDeg);
    if (beyond) {
      if (crashTiltStartMs == 0) crashTiltStartMs = nowMs;
      if ((nowMs - crashTiltStartMs) >= CRASH_HOLD_MS && !gState.crashed) {
        gState.crashed = true;
        Auto_estop();
        Serial.println("!! CRASH DETECTED -> E-STOP, motors disarmed");
      }
    } else {
      crashTiltStartMs = 0;
    }
  } else {
    crashTiltStartMs = 0;
  }

  if (gState.crashed) {
    gState.pilotThrottle = 0.0f;
    Motors_update(dt);
    CAM_update(dt);
    return;
  }

  // Connection-loss landing
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
    CAM_update(dt);
    return;
  }

  // Control path
  if (gState.manualOverride) {
    gState.desiredRollDeg  = gState.joyRollDeg;
    gState.desiredPitchDeg = gState.joyPitchDeg;
  } else {
    gState.desiredRollDeg  = 0.0f;
    gState.desiredPitchDeg = 0.0f;
    Auto_update(dt);
  }

  // Stabilisation
  const float rollErrDeg  = (gState.desiredRollDeg  - gState.rollDeg);
  const float pitchErrDeg = (gState.desiredPitchDeg - gState.pitchDeg);
  float rollCorr  = KROLL  * rollErrDeg;
  float pitchCorr = KPITCH * pitchErrDeg;
  Motors_setStabCorrs(rollCorr, pitchCorr);

  // Hovering heuristic
  bool flat   = (fabsf(gState.rollDeg) < 12.0f) && (fabsf(gState.pitchDeg) < 12.0f);
  bool autoOk = (!gState.manualOverride) && Auto_isActive();
  bool tofOk  = gState.tof1Ready;
  float altM  = Auto_getAltM();
  float spM   = Auto_getSetpointM();
  bool altNear= (tofOk && autoOk && fabsf(altM - spM) < 0.08f);
  bool thrOk  = (gState.pilotThrottle > 0.06f && gState.pilotThrottle < 0.70f);
  gState.hovering = Motors_isArmed() && !gState.crashed && gState.imuReady &&
                    flat && autoOk && altNear && thrOk && !gState.connectionLost;

  // Drive motors
  Motors_update(dt);

  // Camera update
  CAM_update(dt);
}
