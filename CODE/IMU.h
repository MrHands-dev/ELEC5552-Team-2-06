#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "Types.h"

/*
  Minimal MPU6050 read (no external libs)
  - Initializes device at 0x68
  - Reads accel/gyro
  - Complementary filter for pitch/roll
*/

#define MPU_ADDR 0x68

// Forward decls
void IMU_setup(TelemetryState &st);
void IMU_update(TelemetryState &st);

namespace {
  bool  imuFound = false;
  float fRoll = 0.0f, fPitch = 0.0f;
  unsigned long lastUs = 0;

  // gyro scale: 131 LSB/(°/s) for ±250dps
  const float GYRO_LSB_PER_DPS = 131.0f;
  // accel scale: 16384 LSB/g for ±2g
  const float ACC_LSB_PER_G = 16384.0f;

  // IMU bus pins (dedicated I2C bus)
  const int SDA_PIN = 11;   // SDA = IO11
  const int SCL_PIN = 10;   // SCL = IO10
}

static inline void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

static inline void mpuRead(uint8_t reg, uint8_t *buf, size_t len){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)len, (bool)true);
  for(size_t i=0;i<len && Wire.available();++i) buf[i] = Wire.read();
}

void IMU_setup(TelemetryState &st){
  st.imuReady = false;

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);

  // Wake from sleep
  mpuWrite(0x6B, 0x00);      // PWR_MGMT_1: clk = internal, sleep=0
  delay(10);

  // WHO_AM_I (0x75) should be 0x68
  uint8_t who = 0;
  mpuRead(0x75, &who, 1);
  if (who != 0x68) {
    Serial.printf("MPU6050 not found (WHO_AM_I=0x%02X). IMU disabled.\n", who);
    imuFound = false;
    st.imuReady = false;
    return;
  }

  // Gyro ±250 dps (0x1B = 0x00), Accel ±2g (0x1C = 0x00), DLPF ~94Hz (0x1A=0x02)
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1C, 0x00);
  mpuWrite(0x1A, 0x02);

  imuFound = true;
  st.imuReady = true;
  lastUs = micros();
  Serial.println("MPU6050 ready.");
}

void IMU_update(TelemetryState &st){
  if(!imuFound){ st.imuReady = false; return; }

  // Read 14 bytes starting at ACCEL_XOUT_H
  uint8_t raw[14] = {0};
  mpuRead(0x3B, raw, 14);

  int16_t ax = (raw[0] << 8) | raw[1];
  int16_t ay = (raw[2] << 8) | raw[3];
  int16_t az = (raw[4] << 8) | raw[5];
  int16_t gz = (raw[12]<<8) | raw[13];

  float axg = (float)ax / ACC_LSB_PER_G;
  float ayg = (float)ay / ACC_LSB_PER_G;
  float azg = (float)az / ACC_LSB_PER_G;

  // Euler from accel (approx)
  float pitchAcc = atan2f(-axg, sqrtf(ayg*ayg + azg*azg)) * 180.0f / PI;
  float rollAcc  = atan2f( ayg, azg == 0 ? 1e-6f : azg ) * 180.0f / PI;

  // Gyro Z only (for yaw rate display)
  float gyroZ_dps = (float)gz / GYRO_LSB_PER_DPS;

  unsigned long now = micros();
  float dt = (now - lastUs) * 1e-6f;
  if (dt <= 0) dt = 0.001f;
  lastUs = now;

  // Complementary filter
  const float alpha = 0.98f;
  fPitch = alpha * fPitch + (1.0f - alpha) * pitchAcc;
  fRoll  = alpha * fRoll  + (1.0f - alpha) * rollAcc;

  st.pitchDeg   = fPitch;
  st.rollDeg    = fRoll;
  st.yawRateDps = gyroZ_dps;
  st.imuReady   = true;
}
