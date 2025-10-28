// ToF.h — ESP32-S3 WROOM-1 + ST ToF (VL53L0X / VL53L1X)
// Strategy:
//  - If sensors are VL53L1X: use Pololu VL53L1X driver to set unique I2C addresses
//    (ToF1 -> 0x2A, ToF2 -> 0x29) and start continuous ranging on both.
//  - If sensors are VL53L0X: use Adafruit VL53L0X, readdress one to 0x2A.
//  - Mixed (L1X + L0X): L1X stays @0x29, L0X moves to 0x2A.
//
// Requires: Library Manager → install "VL53L1X by Pololu" (official).
//
#pragma once
#include <Arduino.h>
#include <Wire.h>

// ---- L0X support (Adafruit) ----
#include <Adafruit_VL53L0X.h>

// ---- L1X support (Pololu) ----
#include <VL53L1X.h>   // Pololu library

#include "Types.h"

// ===== Pins =====
static constexpr int TOF_SDA   = 40;
static constexpr int TOF_SCL   = 41;

static constexpr int TOF1_XSHUT = 13;  // ToF #1 XSHUT
static constexpr int TOF1_INT   = 1;   // optional input

static constexpr int TOF2_XSHUT = 20;  // ToF #2 XSHUT
static constexpr int TOF2_INT   = 19;  // optional input

// ===== I2C Addresses =====
static constexpr uint8_t ADDR_DEF = 0x29; // default power-on
static constexpr uint8_t ADDR_ALT = 0x2A; // alternate for one device

// ===== I2C bus =====
static TwoWire I2C_TOF(1);

// ===== Driver instances =====
enum class ToFType : uint8_t { NONE=0, L0X, L1X };

static ToFType gToF1Type = ToFType::NONE;
static ToFType gToF2Type = ToFType::NONE;

static uint8_t gToF1Addr = 0x00;
static uint8_t gToF2Addr = 0x00;

// Adafruit L0X (one per sensor, only if L0X detected)
static Adafruit_VL53L0X gTof1L0X;
static Adafruit_VL53L0X gTof2L0X;

// Pololu L1X (one per sensor, only if L1X detected)
static VL53L1X gTof1L1X;
static VL53L1X gTof2L1X;

// ===== Utilities =====
static inline void i2cBegin(uint32_t hz){
  I2C_TOF.begin(TOF_SDA, TOF_SCL);
  I2C_TOF.setClock(hz);
}
static inline void ensureOutput(int pin){
  pinMode(pin, OUTPUT);  // ensure valid output before toggling
}
static inline void bothLow(){
  ensureOutput(TOF1_XSHUT); ensureOutput(TOF2_XSHUT);
  digitalWrite(TOF1_XSHUT, LOW);
  digitalWrite(TOF2_XSHUT, LOW);
}
static inline void xshutHigh(int pin){ ensureOutput(pin); digitalWrite(pin, HIGH); }
static inline void xshutLow (int pin){ ensureOutput(pin); digitalWrite(pin, LOW);  }

static inline void setInputs(){
  pinMode(TOF1_INT, INPUT_PULLUP);
  pinMode(TOF2_INT, INPUT_PULLUP);
}

static bool i2cPresent(uint8_t addr){
  I2C_TOF.beginTransmission(addr);
  return (I2C_TOF.endTransmission() == 0);
}

// ===== L0X helpers (Adafruit) =====
static bool l0x_begin_at(Adafruit_VL53L0X& dev, uint8_t addr){
  return dev.begin(addr, /*debug=*/false, &I2C_TOF);
}

// ===== L1X helpers (Pololu) =====
static bool l1x_begin_default(VL53L1X& dev){
  dev.setBus(&I2C_TOF);         // Pololu lib supports selecting a TwoWire*
  dev.setTimeout(100);
  if (!dev.init()) return false; // init at default 0x29
  return true;
}
static void l1x_configure(VL53L1X& dev){
  // Reasonable defaults; adjust as desired
  dev.setDistanceMode(VL53L1X::Long);      // Short/Medium/Long
  dev.setMeasurementTimingBudget(50000);   // 50 ms budget
  dev.startContinuous(60);                 // period (ms) >= timing budget
}

// ===== Bring-up sequence =====
inline void ToF_setup(TelemetryState& st){
  st.tof1Ready=false; st.tof1mm=0;
  st.tof2Ready=false; st.tof2mm=0;

  setInputs();

  // Start fast; can fallback to 100kHz if your wiring/noise demands it
  i2cBegin(400000);
  bothLow(); delay(10);

  // --- Step 1: Bring up ToF1 ONLY at default address, detect type ---
  xshutHigh(TOF1_XSHUT); delay(80);

  bool t1_isL1X = false;
  bool t1_isL0X = false;

  // Try Pololu L1X first
  t1_isL1X = l1x_begin_default(gTof1L1X);
  if (t1_isL1X){
    gToF1Type = ToFType::L1X;
    // Move ToF1(L1X) to ALT address (0x2A) to free 0x29
    gTof1L1X.setAddress(ADDR_ALT);
    delay(5);
    if (!i2cPresent(ADDR_ALT)){
      Serial.println("ToF1: L1X setAddress(0x2A) failed (not present)!");
    }
    gToF1Addr = ADDR_ALT;
    l1x_configure(gTof1L1X);
    Serial.println("ToF1: VL53L1X → 0x2A (continuous)");
  } else {
    // Not L1X — try L0X via Adafruit at default 0x29
    t1_isL0X = l0x_begin_at(gTof1L0X, ADDR_DEF);
    if (t1_isL0X){
      gToF1Type = ToFType::L0X;
      // Move ToF1 (L0X) to 0x2A
      gTof1L0X.setAddress(ADDR_ALT);
      delay(10);
      if (!l0x_begin_at(gTof1L0X, ADDR_ALT)){
        Serial.println("ToF1: L0X reopen at 0x2A failed!");
      }
      gToF1Addr = ADDR_ALT;
      Serial.println("ToF1: VL53L0X → 0x2A");
    } else {
      gToF1Type = ToFType::NONE;
      gToF1Addr = 0x00;
      Serial.println("ToF1: init FAIL");
    }
  }

  // --- Step 2: Bring up ToF2 at default 0x29 (ToF1 now at 0x2A or off) ---
  xshutHigh(TOF2_XSHUT); delay(80);

  bool t2_isL1X = false;
  bool t2_isL0X = false;

  // L1X at default?
  gTof2L1X.setBus(&I2C_TOF);
  gTof2L1X.setTimeout(100);
  if (gTof2L1X.init()){
    t2_isL1X = true;
  }

  if (t2_isL1X){
    gToF2Type = ToFType::L1X;
    gToF2Addr = ADDR_DEF;  // stays at 0x29
    l1x_configure(gTof2L1X);
    Serial.println("ToF2: VL53L1X @0x29 (continuous)");
  } else {
    // Try L0X at 0x29
    t2_isL0X = l0x_begin_at(gTof2L0X, ADDR_DEF);
    if (t2_isL0X){
      gToF2Type = ToFType::L0X;
      gToF2Addr = ADDR_DEF;
      Serial.println("ToF2: VL53L0X @0x29");
    } else {
      gToF2Type = ToFType::NONE;
      gToF2Addr = 0x00;
      Serial.println("ToF2: init FAIL");
    }
  }

  // Final address report
  Serial.printf("ToF summary: T1=%s@0x%02X  T2=%s@0x%02X\n",
    (gToF1Type==ToFType::L1X?"L1X":(gToF1Type==ToFType::L0X?"L0X":"NONE")), gToF1Addr,
    (gToF2Type==ToFType::L1X?"L1X":(gToF2Type==ToFType::L0X?"L0X":"NONE")), gToF2Addr);
}

// ===== Per-loop update =====
inline void ToF_update(TelemetryState& st){
  // --- ToF1 ---
  switch (gToF1Type){
    case ToFType::L1X: {
      if (gTof1L1X.dataReady()){
        gTof1L1X.read();  // populate ranging_data
        uint16_t mm = gTof1L1X.ranging_data.range_mm;
        uint8_t  rs = gTof1L1X.ranging_data.range_status; // 0 = valid
        bool ok = (rs == 0) && (mm > 0) && (mm != 0xFFFF);
        st.tof1Ready = ok;
        st.tof1mm    = ok ? mm : 0;
      }
    } break;

    case ToFType::L0X: {
      VL53L0X_RangingMeasurementData_t m; gTof1L0X.rangingTest(&m, false);
      if (m.RangeStatus != 4 && m.RangeMilliMeter > 0){
        st.tof1Ready = true; st.tof1mm = (uint16_t)m.RangeMilliMeter;
      } else {
        st.tof1Ready = false; st.tof1mm = 0;
      }
    } break;

    default: st.tof1Ready = false; st.tof1mm = 0; break;
  }

  // --- ToF2 ---
  switch (gToF2Type){
    case ToFType::L1X: {
      if (gTof2L1X.dataReady()){
        gTof2L1X.read();
        uint16_t mm = gTof2L1X.ranging_data.range_mm;
        uint8_t  rs = gTof2L1X.ranging_data.range_status;
        bool ok = (rs == 0) && (mm > 0) && (mm != 0xFFFF);
        st.tof2Ready = ok;
        st.tof2mm    = ok ? mm : 0;
      }
    } break;

    case ToFType::L0X: {
      VL53L0X_RangingMeasurementData_t m; gTof2L0X.rangingTest(&m, false);
      if (m.RangeStatus != 4 && m.RangeMilliMeter > 0){
        st.tof2Ready = true; st.tof2mm = (uint16_t)m.RangeMilliMeter;
      } else {
        st.tof2Ready = false; st.tof2mm = 0;  
      }
    } break;

    default: st.tof2Ready = false; st.tof2mm = 0; break;
  }
}
