#pragma once
#include <Arduino.h>
#include <algorithm>
#include "Types.h"

/*
   motors.h — balanced power mode + SurgeGuard (brownout-safe corrections)
   -----------------------------------------------------------------------
   Mapping (confirmed + CH1↔CH3 swap preserved):
     GPIO 6 -> Front-Left   (logical M1; CH1 write goes to GPIO6)
     GPIO 3 -> Front-Right  (M2)
     GPIO 4 -> Back-Right   (logical M3; CH3 write goes to GPIO4)
     GPIO 5 -> Back-Left    (M4)

   SurgeGuard:
     - Dynamically limits how much correction (roll/pitch) can be added to the
       base throttle per motor, based on the available headroom at that moment.
     - Prevents large instantaneous surges that were causing brownouts/resets.
*/

static const int M1_IO = 4;  // Front-Left  (logical M1; physically on CH3)
static const int M2_IO = 3;  // Front-Right (M2)
static const int M3_IO = 6;  // Back-Right  (logical M3; physically on CH1)
static const int M4_IO = 5;  // Back-Left   (M4)

static const int CH1 = 0;
static const int CH2 = 1;
static const int CH3 = 2;
static const int CH4 = 3;

static uint32_t gHz = 20000;
static uint8_t  gResBits = 10;
static uint32_t gDutyMax = (1u<<10) - 1;

static inline uint32_t dutyFromFrac(float f){
  if (f <= 0.0f) return 0;
  if (f >= 1.0f) return gDutyMax;
  return (uint32_t)(f * gDutyMax + 0.5f);
}

static void setupAll(uint32_t hz, uint8_t bits){
  gHz = hz; gResBits = bits; gDutyMax = (1u<<bits) - 1;
  ledcSetup(CH1, hz, bits);
  ledcSetup(CH2, hz, bits);
  ledcSetup(CH3, hz, bits);
  ledcSetup(CH4, hz, bits);
}

// Attach channels 
static void attachAll(){
  ledcAttachPin(M3_IO, CH1); // CH1 -> GPIO6
  ledcAttachPin(M2_IO, CH2); // CH2 -> GPIO3
  ledcAttachPin(M1_IO, CH3); // CH3 -> GPIO4
  ledcAttachPin(M4_IO, CH4); // CH4 -> GPIO5
}

// ====== state & safety ======
static bool gArmed=false;
static uint32_t gArmMs=0;

// -------- Balanced limits/rates (compromise) --------
static const float GLOBAL_THR_CAP = 0.68f;        // overall throttle ceiling
static const float THR_LPF_ALPHA  = 0.25f;        // throttle smoothing (0..1)
static const float SOFTSTART_RATE = 0.80f;        // throttle/sec after arming
static const uint32_t SOFTSTART_MS= 1000;         // soft-start duration
static const float MIX_SLEW_RATE  = 0.85f;        // per-motor slew (frac/sec)

static const float PER_MOTOR_CAP  = 0.78f;        // ceiling per motor
static const float SUM_UP_RATE    = 1.80f;        // sum increase per second
static const float SUM_ABS_CAP    = 4.0f * 0.78f; // total cap (all 4 motors)

// ---- SurgeGuard parameters ----
// Max correction fraction allowed per motor, scaled by available headroom.
// These values cap correction delta to avoid brownout spikes.
static const float CORR_HEADROOM_SCALE = 0.85f;   // how much of headroom can correction consume
static const float CORR_ABS_CAP        = 0.32f;   // absolute ceiling for any single motor correction (safety)

static float gThrFilt = 0.0f;
static float m1_applied = 0.0f, m2_applied = 0.0f, m3_applied = 0.0f, m4_applied = 0.0f;
static float sum_applied = 0.0f;

bool Motors_isArmed(){return gArmed;}
void Motors_arm(){
  gArmed=true; gArmMs=millis();
  gThrFilt=0.0f;
  m1_applied=m2_applied=m3_applied=m4_applied=0.0f;
  sum_applied=0.0f;
}
void Motors_disarm(){
  gArmed=false;
  ledcWrite(CH1,0); ledcWrite(CH2,0); ledcWrite(CH3,0); ledcWrite(CH4,0);
  gThrFilt=0.0f;
  m1_applied=m2_applied=m3_applied=m4_applied=0.0f;
  sum_applied=0.0f;
}

void Motors_setup(){
  const struct{uint32_t hz;uint8_t bits;}cands[]={
    {20000,12},{20000,11},{20000,10},
    {10000,12},{10000,11},
    { 5000,13},{ 5000,12}
  };
  bool ok=false;
  for(auto c:cands){
    if(ledcSetup(CH1,c.hz,c.bits)&&ledcSetup(CH2,c.hz,c.bits)&&
       ledcSetup(CH3,c.hz,c.bits)&&ledcSetup(CH4,c.hz,c.bits)){
      setupAll(c.hz,c.bits); ok=true; break;
    }
  }
  if(!ok) setupAll(20000,10);
  attachAll();
  Motors_disarm();
}

// ===== Telemetry source =====
extern TelemetryState gState;

// ===== Stabilization corrections (BOTH axes inverted) =====
static float gRollCorr = 0.0f;
static float gPitchCorr = 0.0f;

void Motors_setStabCorrs(float rollCorr,float pitchCorr){
  if (rollCorr >  0.5f) rollCorr =  0.5f;
  if (rollCorr < -0.5f) rollCorr = -0.5f;
  if (pitchCorr >  0.5f) pitchCorr =  0.5f;
  if (pitchCorr < -0.5f) pitchCorr = -0.5f;
  gRollCorr  = -rollCorr;   // keep your established sign flips
  gPitchCorr = -pitchCorr;
}

static inline float yawFracFromCmd(float yawRateDps){
  const float k=0.2f/180.0f;
  float v=k*yawRateDps;
  if(v>0.25f)v=0.25f; if(v<-0.25f)v=-0.25f;
  return v;
}

void Motors_update(float dt){
  float t_cmd=gState.pilotThrottle;
  if(!gArmed) t_cmd=0.0f;

  // Global cap + throttle smoothing
  if (t_cmd > GLOBAL_THR_CAP) t_cmd = GLOBAL_THR_CAP;
  gThrFilt = THR_LPF_ALPHA * t_cmd + (1.0f - THR_LPF_ALPHA) * gThrFilt;

  // Soft-start after arming
  static float prevThr = 0.0f;
  if (gArmed && (millis() - gArmMs) < SOFTSTART_MS) {
    float maxStep = SOFTSTART_RATE * dt;
    float up = gThrFilt - prevThr;
    if (up > maxStep) gThrFilt = prevThr + maxStep;
  }
  prevThr = gThrFilt;

  const float yawF = yawFracFromCmd(gState.yawRateCmdDps);

  // ---- SurgeGuard: compute safe correction headroom ----
  // Available headroom above base throttle before hitting per-motor cap
  float headroom = PER_MOTOR_CAP - gThrFilt;
  if (headroom < 0.0f) headroom = 0.0f;

  // Max correction allowed (per motor) this cycle
  float corrMax = CORR_HEADROOM_SCALE * headroom;
  if (corrMax > CORR_ABS_CAP) corrMax = CORR_ABS_CAP;  // hard ceiling
  // Provide a small minimum so there is *some* correction even at high throttle
  const float corrMin = 0.10f; // ensures some stabilization at high thrust
  if (corrMax < corrMin) corrMax = corrMin;
  // But never exceed 50% of base throttle to avoid flip on one side
  if (corrMax > 0.50f * (gThrFilt + 1e-6f)) corrMax = 0.50f * gThrFilt;

  // Apply a gentle scale to incoming corrections if headroom is small
  float scale = 1.0f;
  if (headroom < 0.15f) {
    // linearly taper down corrections as headroom disappears
    scale = headroom / 0.15f;           // 0..1
    if (scale < 0.25f) scale = 0.25f;   // never kill corrections entirely
  }
  const float rCorr = gRollCorr  * scale;
  const float pCorr = gPitchCorr * scale;

  // Quad-X mix (logical)
  float m1 = gThrFilt + pCorr + rCorr - yawF; // FL (logical)
  float m2 = gThrFilt + pCorr - rCorr + yawF; // FR
  float m3 = gThrFilt - pCorr - rCorr - yawF; // BR (logical)
  float m4 = gThrFilt - pCorr + rCorr + yawF; // BL

  // ---- Clamp motor deltas relative to base throttle (SurgeGuard core) ----
  auto clampDelta = [&](float target){
    float delta = target - gThrFilt;
    if (delta >  corrMax) delta =  corrMax;
    if (delta < -corrMax) delta = -corrMax;
    float out = gThrFilt + delta;
    if (out < 0.0f) out = 0.0f;
    if (out > 1.0f) out = 1.0f;
    return out;
  };
  float arr[4] = {
    clampDelta(m1),
    clampDelta(m2),
    clampDelta(m3),
    clampDelta(m4)
  };

  // Per-motor cap 
  float maxv=*std::max_element(arr,arr+4);
  if (maxv > PER_MOTOR_CAP){
    float s = PER_MOTOR_CAP / maxv;
    for(float &v:arr) v *= s;
  }

  // Total cap and rate limit of sum
  float sum_target = arr[0]+arr[1]+arr[2]+arr[3];
  if (sum_target > SUM_ABS_CAP){
    float s2 = SUM_ABS_CAP / sum_target;
    for(float &v:arr) v *= s2;
    sum_target = SUM_ABS_CAP;
  }
  float sum_step = SUM_UP_RATE * dt;
  if (sum_target > sum_applied + sum_step){
    float s3 = (sum_applied + sum_step) / sum_target;
    for(float &v:arr) v *= s3;
    sum_target = sum_applied + sum_step;
  }
  sum_applied = sum_target;

  // Per-motor slew 
  const float step = MIX_SLEW_RATE * dt;
  if (arr[0] > m1_applied) m1_applied = min(m1_applied + step, arr[0]); else m1_applied = max(m1_applied - step, arr[0]);
  if (arr[1] > m2_applied) m2_applied = min(m2_applied + step, arr[1]); else m2_applied = max(m2_applied - step, arr[1]);
  if (arr[2] > m3_applied) m3_applied = min(m3_applied + step, arr[2]); else m3_applied = max(m3_applied - step, arr[2]);
  if (arr[3] > m4_applied) m4_applied = min(m4_applied + step, arr[3]); else m4_applied = max(m4_applied - step, arr[3]);

  if(!gArmed){ m1_applied=m2_applied=m3_applied=m4_applied=0.0f; sum_applied=0.0f; }

  // SWAPPED PWM writes (CH1↔CH3) to map logical motors to physical pins
  ledcWrite(CH1, dutyFromFrac(m3_applied)); // CH1 -> GPIO6 (Front-Left phys)
  ledcWrite(CH2, dutyFromFrac(m2_applied)); // CH2 -> GPIO3 (Front-Right)
  ledcWrite(CH3, dutyFromFrac(m1_applied)); // CH3 -> GPIO4 (Back-Right phys)
  ledcWrite(CH4, dutyFromFrac(m4_applied)); // CH4 -> GPIO5 (Back-Left)
}

// ---- GUI debug helpers ----
bool Motors_debugAttachAndDrive(int io,float dutyFrac,uint32_t hz){
  if (io < 0) return false;
  if (dutyFrac < 0) dutyFrac = 0; if (dutyFrac > 1) dutyFrac = 1;
  ledcDetachPin(M1_IO);
  ledcSetup(CH1, hz, gResBits);
  ledcAttachPin(io, CH1);
  ledcWrite(CH1, dutyFromFrac(dutyFrac));
  Serial.printf("DEBUG: IO%d driven @ %u Hz duty=%.2f\n", io, (unsigned)hz, dutyFrac);
  return true;
}
void Motors_debugStop(){
  setupAll(gHz, gResBits);
  attachAll();
  Motors_disarm();
}
