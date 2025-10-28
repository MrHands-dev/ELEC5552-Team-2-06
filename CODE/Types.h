#pragma once
#include <Arduino.h>

struct TelemetryState {
  // Commands / pilot inputs
  float pilotThrottle = 0.0f;       // 0..1
  float yawRateCmdDps = 0.0f;       // deg/s
  float desiredRollDeg = 0.0f;      // deg
  float desiredPitchDeg = 0.0f;     // deg

  // Manual override joystick inputs
  bool  manualOverride = false;
  float joyRollDeg = 0.0f;          // deg
  float joyPitchDeg = 0.0f;         // deg

  // IMU
  bool  imuReady = false;
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawRateDps = 0.0f;

  // ToF sensors
  bool  tof1Ready = false;
  int32_t tof1mm = 0;
  bool  tof2Ready = false;
  int32_t tof2mm = 0;

  // New: object detection from ToF1 (≤300 mm)
  bool  objectDetected = false;

  // Status flags
  bool  crashed = false;
  bool  hovering = false;

  // Connection monitor
  uint32_t lastClientMs = 0;
  bool  connectionLost = false;
  bool  connectionStable = false;
};

// Auto-hover control 
void  Auto_start(float setpoint_m);
void  Auto_estop();
bool  Auto_isActive();
void  Auto_update(float dt);
float Auto_getSetpointM();
void  Auto_setSetpoint(float setpoint_m);
float Auto_getAltM();

// Motors 
void  Motors_setup();
void  Motors_update(float dt);
void  Motors_arm();
void  Motors_disarm();
bool  Motors_isArmed();
bool  Motors_debugAttachAndDrive(int io, float duty, uint32_t hz);
void  Motors_debugStop();
void  Motors_setStabCorrs(float rollCorr, float pitchCorr);
