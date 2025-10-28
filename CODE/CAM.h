#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <ArduCAM.h>
#include "memorysaver.h"
// Make sure an OV2640 define is enabled in memorysaver.h, e.g. OV2640_MINI_2MP

// ===== Pins =====
static const int PIN_SPI_MOSI = 35;
static const int PIN_SPI_MISO = 37;
static const int PIN_SPI_SCK  = 36;
static const int SD_CS        = 38;
static const int CAM_CS       = 18;
static const int I2C_SDA      = 17;
static const int I2C_SCL      = 16;

// ===== Camera + SPI =====
static ArduCAM   cam(OV2640, CAM_CS);
static SPISettings camBurst(8000000, MSBFIRST, SPI_MODE0); // ~8 MHz burst
static bool gCamReady = false, gSDReady = false;

// ===== Recording state =====
static volatile bool gRecording = false;   // stays true until AVI is finalised
static volatile bool gStopReq   = false;   // set by /api/cam_stop
static uint32_t gFrameInterval  = 0;       // ms, 0 = as fast as possible
static uint32_t gLastFrameMs    = 0;

// ===== Video parameters=====
static const uint16_t VID_W = 640;
static const uint16_t VID_H = 480;
static const uint8_t  FPS_PLACEHOLDER = 30;   // patched to actual on stop
static const uint8_t  JPEG_QUALITY    = 20;   // (OV2640 quality register 10..63, lower=better quality)

// ===== Minimal AVI structures / helpers =====
struct IdxEntry { // 16 bytes each
  uint32_t tag;       // '00dc'
  uint32_t flags;     // 0x10 = keyframe
  uint32_t offset;    // from start of 'movi' LIST data (right after "movi")
  uint32_t size;      // chunk size (un-padded)
};

static const size_t MAX_FRAMES_IDX = 2048; // ~32 KB of RAM for index
static IdxEntry gIdx[MAX_FRAMES_IDX];
static uint32_t gIdxCount = 0;

struct AviCtx {
  File     f;
  String   path;
  uint32_t riff_sz_pos=0, hdrl_sz_pos=0, strl_sz_pos=0;
  uint32_t avih_pos=0, strh_pos=0, movi_sz_pos=0, movi_data_start=0;
  uint32_t frames=0, maxBytesPerSec=0, maxFrameBytes=0;
  uint32_t start_ms=0, bytesThisSec=0, secT0=0;
} avi;

static inline void w32(File& f, uint32_t v){ f.write((uint8_t*)&v,4); }
static inline void w16(File& f, uint16_t v){ f.write((uint8_t*)&v,2); }
static inline void wTag(File& f, const char t[4]){ f.write((const uint8_t*)t,4); }

static uint32_t rdPos(File& f){ return (uint32_t)f.position(); }

// ===== OV2640 quality helper =====
static void setOV2640Quality(uint8_t q){
  if (q < 10) q = 10; if (q > 63) q = 63;
  cam.wrSensorReg8_8(0xFF, 0x00);
  cam.wrSensorReg8_8(0x44, q);
}

// ===== File naming =====
static String nextAVIName(){
  for (int i=1; i<=9999; ++i){
    char n[20]; snprintf(n, sizeof(n), "/VID%04d.AVI", i);
    if (!SD.exists(n)) return String(n);
  }
  return "/VID9999.AVI";
}

// ===== AVI begin/end =====
static void aviBegin(const char* path, uint16_t w, uint16_t h, uint8_t fps){
  avi.path = path;
  avi.f = SD.open(path, FILE_WRITE);
  if (!avi.f){ Serial.println("[CAM] AVI open failed"); return; }
  gIdxCount = 0;
  avi.frames = 0; avi.maxBytesPerSec = 0; avi.maxFrameBytes = 0;
  avi.bytesThisSec = 0; avi.secT0 = millis();

  // RIFF header
  wTag(avi.f, "RIFF"); avi.riff_sz_pos = rdPos(avi.f); w32(avi.f, 0); wTag(avi.f, "AVI ");

  // LIST hdrl
  wTag(avi.f, "LIST"); avi.hdrl_sz_pos = rdPos(avi.f); w32(avi.f, 0); wTag(avi.f, "hdrl");

  // avih (56 bytes)
  wTag(avi.f, "avih"); w32(avi.f, 56); avi.avih_pos = rdPos(avi.f);
  w32(avi.f, 1000000UL / (fps ? fps : 30));    // dwMicroSecPerFrame (patched later)
  w32(avi.f, 0);                                // dwMaxBytesPerSec (patched later)
  w32(avi.f, 0);                                // dwPaddingGranularity
  w32(avi.f, 0x10);                             // dwFlags (0x10 = AVIF_HASINDEX)
  w32(avi.f, 0);                                // dwTotalFrames (patched later)
  w32(avi.f, 0);                                // dwInitialFrames
  w32(avi.f, 1);                                // dwStreams
  w32(avi.f, 0);                                // dwSuggestedBufferSize
  w32(avi.f, w);                                // dwWidth
  w32(avi.f, h);                                // dwHeight
  for (int i=0;i<4;i++) w32(avi.f,0);          // dwReserved[4]

  // LIST strl
  wTag(avi.f, "LIST"); avi.strl_sz_pos = rdPos(avi.f); w32(avi.f, 0); wTag(avi.f, "strl");

  // strh (56 bytes) — vids MJPG stream header
  wTag(avi.f, "strh"); w32(avi.f, 56); avi.strh_pos = rdPos(avi.f);
  wTag(avi.f, "vids"); wTag(avi.f, "MJPG");
  w32(avi.f, 0);              // dwFlags
  w16(avi.f, 0); w16(avi.f,0);// wPriority, wLanguage
  w32(avi.f, 0);              // dwInitialFrames
  w32(avi.f, 1);              // dwScale   (patched later to 1000)
  w32(avi.f, fps?fps:30);     // dwRate    (patched later to fps*1000)
  w32(avi.f, 0);              // dwStart
  w32(avi.f, 0);              // dwLength  (frames) patched later
  w32(avi.f, 0);              // dwSuggestedBufferSize
  w32(avi.f, 0);              // dwQuality
  w32(avi.f, 0);              // dwSampleSize
  w16(avi.f, 0); w16(avi.f,0);// rcFrame left, top
  w16(avi.f, w); w16(avi.f,h);// rcFrame right, bottom

  // strf (BITMAPINFOHEADER, 40 bytes)
  wTag(avi.f, "strf"); w32(avi.f, 40);
  w32(avi.f, 40);       // biSize
  w32(avi.f, w);        // biWidth
  w32(avi.f, h);        // biHeight (positive = bottom-up)
  w16(avi.f, 1);        // biPlanes
  w16(avi.f, 24);       // biBitCount
  wTag(avi.f, "MJPG");  // biCompression
  w32(avi.f, 0);        // biSizeImage
  w32(avi.f, 0);        // biXPelsPerMeter
  w32(avi.f, 0);        // biYPelsPerMeter
  w32(avi.f, 0);        // biClrUsed
  w32(avi.f, 0);        // biClrImportant

  // close strl + hdrl LIST sizes
  uint32_t cur = rdPos(avi.f);
  avi.f.seek(avi.strl_sz_pos); w32(avi.f, cur - (avi.strl_sz_pos + 4));
  avi.f.seek(avi.hdrl_sz_pos); w32(avi.f, cur - (avi.hdrl_sz_pos + 4));
  avi.f.seek(cur);

  // LIST movi
  wTag(avi.f, "LIST"); avi.movi_sz_pos = rdPos(avi.f); w32(avi.f, 0); wTag(avi.f, "movi");
  avi.movi_data_start = rdPos(avi.f);
}

static void aviEnd(){
  if (!avi.f) return;

  // write idx1
  wTag(avi.f, "idx1");
  w32(avi.f, gIdxCount * 16);
  for (uint32_t i=0;i<gIdxCount;i++){
    w32(avi.f, gIdx[i].tag);
    w32(avi.f, gIdx[i].flags);
    w32(avi.f, gIdx[i].offset);
    w32(avi.f, gIdx[i].size);
  }

  // patch movi size
  uint32_t file_end = rdPos(avi.f);
  avi.f.seek(avi.movi_sz_pos);
  w32(avi.f, file_end - (avi.movi_sz_pos + 4));
  avi.f.seek(file_end);

  // patch RIFF size
  avi.f.seek(avi.riff_sz_pos);
  w32(avi.f, file_end - 8);
  avi.f.seek(file_end);

  // patch timing + counts
  uint32_t elapsed_ms = max<uint32_t>(1, millis() - avi.start_ms);
  double fps = (double)avi.frames * 1000.0 / (double)elapsed_ms;
  if (fps < 0.1) fps = FPS_PLACEHOLDER;

  uint32_t usec_per_frame = (uint32_t)(1000000.0 / fps + 0.5);
  uint32_t scale = 1000, rate = (uint32_t)(fps * 1000.0 + 0.5);

  avi.f.seek(avi.avih_pos +  0); w32(avi.f, usec_per_frame);       // dwMicroSecPerFrame
  avi.f.seek(avi.avih_pos +  4); w32(avi.f, avi.maxBytesPerSec);   // dwMaxBytesPerSec
  avi.f.seek(avi.avih_pos + 16); w32(avi.f, avi.frames);           // dwTotalFrames

  avi.f.seek(avi.strh_pos + 20); w32(avi.f, scale);                // dwScale
  avi.f.seek(avi.strh_pos + 24); w32(avi.f, rate);                 // dwRate
  avi.f.seek(avi.strh_pos + 32); w32(avi.f, avi.frames);           // dwLength

  avi.f.flush();
  avi.f.close();
}

// ===== One JPEG frame -> movi '00dc' chunk + idx1 entry =====
static const size_t BURST = 2048;
static uint8_t rx[BURST];

static bool writeOneFrame(){
  // 1) capture
  cam.flush_fifo(); cam.clear_fifo_flag(); cam.start_capture();
  uint32_t t0 = millis();
  while (!cam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
    if (gStopReq) { cam.clear_fifo_flag(); return false; }
    if (millis() - t0 > 3000){ Serial.println("[CAM] capture timeout"); return false; }
    delay(1);
  }

  // 2) length
  uint32_t len = cam.read_fifo_length();
  if (len == 0 || len >= 0x7FFFF){ cam.clear_fifo_flag(); Serial.println("[CAM] bad len"); return false; }

  // 3) chunk header + remember index offset
  // idx1 offsets are from start of movi LIST's data (right after "movi")
  uint32_t chunk_offset_from_movi = rdPos(avi.f) - (avi.movi_data_start);
  wTag(avi.f, "00dc"); w32(avi.f, len);

  // 4) read FIFO in bursts and write to SD
  uint32_t remain = len, frameStart = millis(), bytesThisFrame = 0;
  while (remain){
    if (gStopReq) { cam.clear_fifo_flag(); return false; }

    uint32_t slice = remain > BURST ? BURST : remain;

    SPI.beginTransaction(camBurst);
    cam.CS_LOW();
    cam.set_fifo_burst();
    for (uint32_t i=0;i<slice;i++) rx[i] = SPI.transfer(0x00);
    cam.CS_HIGH();
    SPI.endTransaction();

    avi.f.write(rx, slice);
    remain -= slice;
    bytesThisFrame += slice;

    if (millis() - frameStart > 4000){ // per-frame guard
      cam.clear_fifo_flag();
      Serial.println("[CAM] frame timeout");
      return false;
    }
    yield();
  }
  cam.clear_fifo_flag();

  // even padding
  if (len & 1) avi.f.write((uint8_t)0);

  // index entry
  if (gIdxCount < MAX_FRAMES_IDX){
    gIdx[gIdxCount].tag    = 0x63643030UL;        // '00dc' little-endian
    gIdx[gIdxCount].flags  = 0x10;                // keyframe
    gIdx[gIdxCount].offset = chunk_offset_from_movi;
    gIdx[gIdxCount].size   = len;
    gIdxCount++;
  }

  // stats
  avi.frames++;
  avi.bytesThisSec += (len + (len & 1) + 8); // include chunk header + pad
  if (bytesThisFrame > avi.maxFrameBytes) avi.maxFrameBytes = bytesThisFrame;

  uint32_t now = millis();
  if (now - avi.secT0 >= 1000){
    if (avi.bytesThisSec > avi.maxBytesPerSec) avi.maxBytesPerSec = avi.bytesThisSec;
    avi.bytesThisSec = 0;
    avi.secT0 = now;
  }
  return true;
}

// ===== Public API =====
bool CAM_isReady(){ return gCamReady && gSDReady; }
bool CAM_isRecording(){ return gRecording; }

bool CAM_setup(){
  // SPI + SD
  pinMode(CAM_CS, OUTPUT); digitalWrite(CAM_CS, HIGH);
  pinMode(SD_CS, OUTPUT);  digitalWrite(SD_CS, HIGH);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, SD_CS);
  delay(10);

  if (!SD.begin(SD_CS, SPI)){
    Serial.println("[CAM] SD init FAILED");
    gSDReady = false;
  } else {
    gSDReady = true;
    Serial.println("[CAM] SD init OK");
  }

  // I2C for sensor config
  Wire.begin(I2C_SDA, I2C_SCL);

  // ArduCAM init
  cam.set_format(JPEG);
  cam.InitCAM();
  cam.OV2640_set_JPEG_size(OV2640_640x480);
  setOV2640Quality(JPEG_QUALITY);
  cam.flush_fifo();

  gCamReady = true;
  gRecording = false; gStopReq = false;
  gLastFrameMs = 0;

  Serial.printf("[CAM] Ready=%d  SD=%d\n", (int)gCamReady, (int)gSDReady);
  return CAM_isReady();
}

// default arg lives in GUI.h, keep it there
bool CAM_startRecording(uint32_t frame_interval_ms){
  if (!CAM_isReady()){ Serial.println("[CAM] Not ready"); return false; }
  if (gRecording) return true;

  String name = nextAVIName();
  aviBegin(name.c_str(), VID_W, VID_H, FPS_PLACEHOLDER);
  if (!avi.f){ return false; }

  avi.start_ms = millis();
  avi.secT0    = avi.start_ms;
  avi.bytesThisSec = 0;
  avi.maxBytesPerSec = 0;
  avi.maxFrameBytes  = 0;

  gStopReq = false;
  gRecording = true;
  gFrameInterval = frame_interval_ms;
  gLastFrameMs = 0;

  Serial.printf("[CAM] Recording START -> %s\n", name.c_str());
  return true;
}

void CAM_stopRecording(){
  if (!gRecording) return;
  gStopReq = true;
  Serial.println("[CAM] Recording STOP requested");
}

void CAM_update(float /*dt_sec*/){
  if (!gRecording && !gStopReq) return;
  if (!gCamReady || !gSDReady){ gRecording=false; gStopReq=false; return; }

  const uint32_t now = millis();
  if (gRecording && !gStopReq){
    if (!gFrameInterval || (now - gLastFrameMs) >= gFrameInterval){
      (void)writeOneFrame();
      gLastFrameMs = now;
    }
    return;
  }

  // If stop requested: close out and patch headers immediately
  if (gStopReq){
    // If we were mid-writeOneFrame(), it would have returned false;
    // at this point we just finalise whatever we have.
    aviEnd();
    gRecording = false;
    gStopReq = false;
    Serial.println("[CAM] Recording STOP (AVI finalised)");
  }
}
