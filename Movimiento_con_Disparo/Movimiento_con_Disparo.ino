#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== DISPARO: tipo =====================
enum ShotType : uint8_t {
  SHOT_NONE = 0,
  SHOT_MECH = 1,
  SHOT_ELEC = 2
};

// ===================== IMU =====================
Adafruit_BNO055 bno = Adafruit_BNO055();

// ===================== MIC / DETECTOR =====================
const uint8_t MIC_PIN = A0;
const uint16_t MIC_DELAY_US = 250;      // ~4 kHz
const uint16_t CONFIRM_SAMPLES = 80;    // ~20 ms
const uint16_t COOLDOWN_MS = 250;

// EMA helpers
static inline uint16_t ema_div(uint16_t current, uint16_t target, uint8_t divPow2) {
  int32_t diff = (int32_t)target - (int32_t)current;
  return (uint16_t)((int32_t)current + (diff >> divPow2));
}
static inline uint16_t u16abs(int16_t v) { return (v < 0) ? (uint16_t)(-v) : (uint16_t)v; }

// Envelope
uint16_t env = 0;
uint16_t base = 0;

uint16_t MARGIN_ON  = 90;
uint16_t MARGIN_OFF = 45;

// Clasificación
const uint16_t SAT_LOW  = 5;
const uint16_t SAT_HIGH = 1018;

uint16_t PEAK_ELEC_MIN     = 170;
uint16_t WIDTH_ELEC_MIN_US = 6000;
uint16_t RANGE_ELEC_MIN    = 250;

int16_t micCenter = 512;
uint32_t lastShotMs = 0;

bool candidate = false;
uint16_t candCount = 0;

// stats
uint16_t minRawSeen = 1023;
uint16_t maxRawSeen = 0;
uint16_t peakAbs = 0;

uint16_t widthSamples = 0;
bool aboveOff = false;

void calibrateMicCenter() {
  int32_t sum = 0;
  for (uint16_t i = 0; i < 500; i++) {
    sum += analogRead(MIC_PIN);
    delay(1);
  }
  micCenter = sum / 500;
}

static inline void updateWidth(uint16_t envVal, uint16_t thrOff) {
  if (envVal > thrOff) {
    widthSamples++;
    aboveOff = true;
  } else if (aboveOff) {
    aboveOff = false;
  }
}

void resetStats(uint16_t raw, uint16_t mag, uint16_t thrOff) {
  minRawSeen = raw;
  maxRawSeen = raw;
  peakAbs = mag;
  widthSamples = 0;
  aboveOff = false;
  updateWidth(env, thrOff);
}

// ===================== MIC UPDATE =====================
ShotType micUpdate() {
  uint32_t nowMs = millis();
  if (nowMs - lastShotMs < COOLDOWN_MS) return SHOT_NONE;

  uint16_t raw = analogRead(MIC_PIN);
  int16_t centered = raw - micCenter;
  uint16_t mag = u16abs(centered);

  env  = ema_div(env, mag, 2);  // rápida
  base = ema_div(base, env, 6); // lenta

  uint16_t thrOn  = base + MARGIN_ON;
  uint16_t thrOff = base + MARGIN_OFF;

  if (!candidate) {
    if (env > thrOn) {
      candidate = true;
      candCount = 0;
      resetStats(raw, mag, thrOff);
    }
    return SHOT_NONE;
  }

  candCount++;

  if (raw < minRawSeen) minRawSeen = raw;
  if (raw > maxRawSeen) maxRawSeen = raw;
  if (mag > peakAbs) peakAbs = mag;
  updateWidth(env, thrOff);

  uint16_t range = maxRawSeen - minRawSeen;
  uint32_t width_us = widthSamples * MIC_DELAY_US;

  bool isMech = (minRawSeen <= SAT_LOW) && (maxRawSeen >= SAT_HIGH);

  bool isElec =
    !isMech &&
    width_us >= WIDTH_ELEC_MIN_US &&
    (peakAbs >= PEAK_ELEC_MIN || range >= RANGE_ELEC_MIN);

  isElec = false;

  if (isMech || isElec) {
    candidate = false;
    lastShotMs = nowMs;

    // ---------- DEBUG SOLO PARA ELEC ----------
    if (isElec) {
      Serial.print("ELEC_DBG,peak=");
      Serial.print(peakAbs);
      Serial.print(",width_us=");
      Serial.print(width_us);
      Serial.print(",range=");
      Serial.print(range);
      Serial.print(",minRaw=");
      Serial.print(minRawSeen);
      Serial.print(",maxRaw=");
      Serial.print(maxRawSeen);
      Serial.print(",base=");
      Serial.print(base);
      Serial.print(",env=");
      Serial.println(env);
    }
    // -----------------------------------------

    return isMech ? SHOT_MECH : SHOT_ELEC;
  }

  if (candCount >= CONFIRM_SAMPLES) {
    candidate = false;
  }

  return SHOT_NONE;
}

// ===================== IMU TIMING =====================
const uint32_t IMU_PERIOD_MS = 20;
uint32_t nextImuMs = 0;

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  calibrateMicCenter();

  // inicializar env/base
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs(raw - micCenter);
    env  = ema_div(env, mag, 2);
    base = ema_div(base, env, 8);
    delay(2);
  }

  nextImuMs = millis();
}

void loop() {
  // Detector disparo
  ShotType st = micUpdate();
  if (st == SHOT_MECH) {
    Serial.println("SHOT,MECH");
  } else if (st == SHOT_ELEC) {
    Serial.println("SHOT,ELEC");
  }

  // Stream IMU (igual que antes)
  uint32_t nowMs = millis();
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float pitch = euler.x();
    float yaw   = euler.y() * (-1);

    Serial.print(pitch, 6);
    Serial.print(",");
    Serial.println(yaw, 6);
  }

  delayMicroseconds(MIC_DELAY_US);
}
