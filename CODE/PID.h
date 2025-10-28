#pragma once
#include <Arduino.h>
#include "Types.h"
#include "motors.h"

// ===== Simple PID class =====
class PIDF {
public:
  PIDF(float kp=0.0f, float ki=0.0f, float kd=0.0f):Kp(kp),Ki(ki),Kd(kd){}
  inline void setTunings(float kp,float ki,float kd){Kp=kp;Ki=ki;Kd=kd;}
  inline void setOutputLimits(float minOut,float maxOut){
    outMin=min(minOut,maxOut); outMax=max(minOut,maxOut);
    integral=constrain(integral,outMin,outMax);
  }
  inline void reset(float bias=0.0f){prevErr=0.0f;integral=bias;first=true;}
  inline float compute(float error,float dt){
    if(dt<=0) return lastOut;
    if(first){prevErr=error;first=false;}
    integral += Ki*error*dt;
    integral = constrain(integral,outMin,outMax);
    float deriv = (error - prevErr)/dt; prevErr=error;
    float out = Kp*error + integral + Kd*deriv;
    lastOut = constrain(out,outMin,outMax);
    return lastOut;
  }
private:
  float Kp=0,Ki=0,Kd=0;
  float outMin=0.0f,outMax=1.0f;
  float prevErr=0.0f,integral=0.0f,lastOut=0.0f;
  bool first=true;
};

extern TelemetryState gState;

struct _AutoState {
  float Kp=0.55f, Ki=0.28f, Kd=0.10f;    // slightly softer
  float hoverGuess=0.50f;

  float sp_m=0.50f, sp_cmd_m=0.50f;

  float alt_m=0.0f; bool altInit=false;
  float cmdThr=0.0f;

  bool active=false;
  uint32_t tStartMs=0;

  PIDF pid{Kp,Ki,Kd};

  const float THR_MIN=0.00f, THR_MAX=1.00f;
  const float MAX_ASCENT_THR=0.55f;     // lower caps reduce surge
  const float MAX_DESCENT_THR=0.52f;
  const float ALT_ALPHA=0.18f;          // heavier smoothing
  const float RAMP_UP_RATE=0.10f;       // slower throttle rise
  const float RAMP_DOWN_RATE=0.35f;
  const float SP_SLEW_RATE=0.18f;       // slower setpoint slew
  const float ALT_DEADBAND_M=0.03f;
};

inline _AutoState& _auto(){ static _AutoState s; return s; }

inline void Auto_start(float setpoint_m){
  auto& a=_auto();
  a.sp_m=constrain(setpoint_m,0.2f,3.0f);

  if(!Motors_isArmed()) Motors_arm();

  if(gState.tof1Ready && gState.tof1mm>0){
    a.alt_m=gState.tof1mm*0.001f;
    a.sp_cmd_m=a.alt_m;
    a.altInit=true;
  }else{
    a.altInit=false;
    a.sp_cmd_m=a.sp_m;
  }

  a.pid.setTunings(a.Kp,a.Ki,a.Kd);
  a.pid.setOutputLimits(a.THR_MIN,a.THR_MAX);
  a.pid.reset(a.hoverGuess);

  a.cmdThr=max(gState.pilotThrottle,0.05f);
  a.tStartMs=millis();
  a.active=true;

  Serial.printf("AUTO: start sp=%.2f m (slew from %.2f m)\n",a.sp_m,a.sp_cmd_m);
}

inline void Auto_estop(){
  auto& a=_auto();
  a.active=false;
  gState.pilotThrottle=0.0f;
  Motors_disarm();
  Serial.println("AUTO: E-STOP");
}

inline bool  Auto_isActive(){return _auto().active;}
inline float Auto_getSetpointM(){return _auto().sp_m;}
inline float Auto_getAltM(){return _auto().alt_m;}
inline float Auto_elapsed(){auto& a=_auto(); return a.active? (0.001f*(millis()-a.tStartMs)) : 0.0f;}
inline void  Auto_setSetpoint(float sp){ auto& a=_auto(); a.sp_m=constrain(sp,0.2f,3.0f); Serial.printf("AUTO: setpoint -> %.2f m\n",a.sp_m); }

inline void Auto_update(float dt){
  auto& a=_auto(); if(!a.active) return;

  if(!gState.tof1Ready || gState.tof1mm==0){
    Serial.println("AUTO: ToF1 invalid — cancel");
    a.active=false; return;
  }

  float z=gState.tof1mm*0.001f;
  if(!a.altInit){ a.alt_m=z; a.altInit=true; }
  else          { a.alt_m=a.ALT_ALPHA*z + (1.0f-a.ALT_ALPHA)*a.alt_m; }

  // slew the commanded setpoint
  float spErr=a.sp_m - a.sp_cmd_m;
  float spStep=a.SP_SLEW_RATE*dt;
  if(spErr> spStep) a.sp_cmd_m+=spStep;
  else if(spErr< -spStep) a.sp_cmd_m-=spStep;
  else a.sp_cmd_m=a.sp_m;

  float err=a.sp_cmd_m - a.alt_m;
  if(fabsf(err)<a.ALT_DEADBAND_M) err=0.0f;

  float pidOut=a.pid.compute(err,dt);

  // tilt compensation
  float rollRad=gState.rollDeg*0.01745329252f;
  float pitchRad=gState.pitchDeg*0.01745329252f;
  float tilt=cosf(rollRad)*cosf(pitchRad);
  tilt=constrain(tilt,0.75f,1.0f);

  float desired=(a.hoverGuess + (pidOut - a.hoverGuess))/tilt;

  if(err> 0.02f) desired=constrain(desired,a.THR_MIN,a.MAX_ASCENT_THR);
  if(err<-0.02f) desired=constrain(desired,a.THR_MIN,a.MAX_DESCENT_THR);
  desired=constrain(desired,a.THR_MIN,a.THR_MAX);

  float rate=(desired>a.cmdThr)? a.RAMP_UP_RATE : a.RAMP_DOWN_RATE;
  float step=rate*dt;
  if(desired>a.cmdThr) a.cmdThr=min(desired,a.cmdThr+step);
  else                 a.cmdThr=max(desired,a.cmdThr-step);

  gState.pilotThrottle=a.cmdThr;

  if(a.sp_m<0.3f && a.alt_m<0.2f){
    gState.pilotThrottle=0.0f;
  }
}
