#pragma once
#include <Arduino.h>

struct TelemetryState {
  // IMU telemetry
  bool  imuReady = false;
  float rollDeg = 0.0f, pitchDeg = 0.0f, yawRateDps = 0.0f;

  // Commands / GUI inputs (used in both auto and manual)
  float desiredRollDeg  = 0.0f;
  float desiredPitchDeg = 0.0f;
  float yawRateCmdDps   = 0.0f;   // proxy for yaw bias
  float pilotThrottle   = 0.0f;   // 0..1

  // Manual override (joystick) inputs
  bool  manualOverride  = false;  // when true, auto-hover is bypassed
  float joyRollDeg      = 0.0f;   // joystick command (deg)
  float joyPitchDeg     = 0.0f;   // joystick command (deg)

  // ToF telemetry (two sensors)
  bool tof1Ready = false;
  uint16_t tof1mm = 0;

  bool tof2Ready = false;
  uint16_t tof2mm = 0;

  // Flight status
  bool crashed  = false;   // tilt crash detected (latched until ARM)
  bool hovering = false;   // heuristic "stable hover"

  // GUI/communication status
  uint32_t lastClientMs     = 0;   // last time /api/state was polled
  bool     connectionLost   = false;   // latched after 30s idle while armed
  bool     connectionStable = false;   // polled within last ~2s
};

extern TelemetryState gState;
