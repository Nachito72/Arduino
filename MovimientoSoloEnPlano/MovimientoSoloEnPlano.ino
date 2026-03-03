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

// ===================== ARMADO POR INCLINACIÓN =====================
// Usamos roll (euler.y) y un pitch "nivelado" que vale cerca de 0 tanto en 0° como en 180°
const float LEVEL_ON_DEG  = 15.0;     // se arma si roll y pitchLevel <= esto
const float LEVEL_OFF_DEG = 20.0;    // se desarma si roll o pitchLevel >= esto (histeresis)

const uint8_t LEVEL_ON_COUNT  = 5;  // 15 * 20ms = 300ms estable para armar
const uint8_t LEVEL_OFF_COUNT = 8;   // 8 * 20ms = 160ms para desarmar

bool armed = false;
uint8_t levelOnStreak = 0;
uint8_t levelOffStreak = 0;

static inline float f_abs(float v) { return (v < 0) ? -v : v; }

// Distancia angular a “nivel”: 0° o 180° (lo que esté más cerca)
float pitchLevelDeg(float pitchDeg) {
  float ap = f_abs(pitchDeg);           // |pitch|
  float d180 = f_abs(180.0f - ap);      // distancia a 180
  return (ap < d180) ? ap : d180;       // min(dist a 0, dist a 180)
}

// ===================== MIC / DETECTOR =====================
const uint8_t MIC_PIN = A0;

const uint16_t MIC_DELAY_US = 250;      // ~4 kHz
const uint16_t CONFIRM_SAMPLES = 80;    // ~20 ms
const uint16_t COOLDOWN_MS = 250;

// Envelope/Base (enteros)
static inline uint16_t ema_div(uint16_t current, uint16_t target, uint8_t divPow2) {
  int32_t diff = (int32_t)target - (int32_t)current;
  return (uint16_t)((int32_t)current + (diff >> divPow2));
}
static inline uint16_t u16abs(int16_t v) { return (v < 0) ? (uint16_t)(-v) : (uint16_t)v; }

uint16_t env = 0;
uint16_t base = 0;

uint16_t MARGIN_ON  = 90;
uint16_t MARGIN_OFF = 45;

const uint16_t SAT_LOW  = 5;
const uint16_t SAT_HIGH = 1018;

uint16_t PEAK_ELEC_MIN     = 170;
uint16_t WIDTH_ELEC_MIN_US = 6000;
uint16_t RANGE_ELEC_MIN    = 250;

int16_t micCenter = 512;
uint32_t lastShotMs = 0;

bool candidate = false;
uint16_t candCount = 0;

uint16_t minRawSeen = 1023;
uint16_t maxRawSeen = 0;
uint16_t peakAbs = 0;

uint16_t widthSamples = 0;
bool aboveOff = false;

void calibrateMicCenter() {
  int32_t sum = 0;
  const uint16_t N = 500;
  for (uint16_t i = 0; i < N; i++) { sum += analogRead(MIC_PIN); delay(1); }
  micCenter = (int16_t)(sum / (int32_t)N);
}

static inline void updateWidth(uint16_t envVal, uint16_t thrOff) {
  if (envVal > thrOff) { widthSamples++; aboveOff = true; }
  else if (aboveOff) { aboveOff = false; }
}

void resetStats(uint16_t raw, uint16_t mag, uint16_t thrOff) {
  minRawSeen = raw;
  maxRawSeen = raw;
  peakAbs = mag;
  widthSamples = 0;
  aboveOff = false;
  updateWidth(env, thrOff);
}

void resetMicCandidate() {
  candidate = false;
  candCount = 0;
  widthSamples = 0;
  aboveOff = false;
}

// Devuelve evento cuando detecta; si no, SHOT_NONE
ShotType micUpdate() {
  if (!armed) return SHOT_NONE; // clave: sin armado, no detectamos

  uint32_t nowMs = millis();
  if (nowMs - lastShotMs < COOLDOWN_MS) return SHOT_NONE;

  uint16_t raw = analogRead(MIC_PIN);
  int16_t centered = (int16_t)raw - micCenter;
  uint16_t mag = u16abs(centered);

  env  = ema_div(env, mag, 2);  // /4
  base = ema_div(base, env, 8); // /256

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
  uint32_t width_us = (uint32_t)widthSamples * (uint32_t)MIC_DELAY_US;

  bool isMech = (minRawSeen <= SAT_LOW) && (maxRawSeen >= SAT_HIGH);
  bool isElec =
    !isMech &&
    width_us >= WIDTH_ELEC_MIN_US &&
    (peakAbs >= PEAK_ELEC_MIN || range >= RANGE_ELEC_MIN);

  if (isMech){ // || isElec) {
    candidate = false;
    lastShotMs = nowMs;
    return isMech ? SHOT_MECH : SHOT_ELEC;
  }

  if (candCount >= CONFIRM_SAMPLES) {
    candidate = false;
  }

  return SHOT_NONE;
}

// ===================== Timing IMU =====================
const uint32_t IMU_PERIOD_MS = 20; // 50 Hz
uint32_t nextImuMs = 0;

void updateArmingFromTilt() {
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll  = e.y();   // roll real
  float pitch = e.z();   // pitch real (envuelto)

  float r = f_abs(roll);
  float p = pitchLevelDeg(pitch);

  bool levelOn  = (r <= LEVEL_ON_DEG)  && (p <= LEVEL_ON_DEG);
  bool levelOff = (r >= LEVEL_OFF_DEG) || (p >= LEVEL_OFF_DEG);

  if (!armed) {
    if (levelOn) {
      if (levelOnStreak < 255) levelOnStreak++;
    } else {
      levelOnStreak = 0;
    }

    if (levelOnStreak >= LEVEL_ON_COUNT) {
      armed = true;
      levelOnStreak = 0;
      levelOffStreak = 0;
      resetMicCandidate();
      Serial.println("ARMED");
    }
  } else {
    if (levelOff) {
      if (levelOffStreak < 255) levelOffStreak++;
    } else {
      levelOffStreak = 0;
    }

    if (levelOffStreak >= LEVEL_OFF_COUNT) {
      armed = false;
      levelOffStreak = 0;
      levelOnStreak = 0;
      resetMicCandidate();
      Serial.println("DISARMED");
    }
  }

  // DEBUG opcional: descomenta si quieres ver r/pLevel siempre
  // Serial.print("TILT,armed="); Serial.print(armed ? 1 : 0);
  // Serial.print(",roll="); Serial.print(roll, 2);
  // Serial.print(",pitch="); Serial.print(pitch, 2);
  // Serial.print(",pLevel="); Serial.print(p, 2);
  // Serial.print(",absR="); Serial.print(r, 2);
  // Serial.print(",on="); Serial.print(levelOnStreak);
  // Serial.print(",off="); Serial.println(levelOffStreak);
}

void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  calibrateMicCenter();

  // init env/base
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - micCenter);
    env  = ema_div(env, mag, 2);
    base = ema_div(base, env, 8);
    delay(2);
  }

  nextImuMs = millis();
  Serial.println("Ready");
  Serial.println("DISARMED");
}

void loop() {
  // 1) scheduler IMU (50Hz): actualiza armado y, si armed, envía datos
  uint32_t nowMs = millis();
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    updateArmingFromTilt();

    if (armed) {
      // IMPORTANTE: mantenemos tus ejes de salida para Processing
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      float pitchOut = euler.x();         // tu "pitch"
      float yawOut   = euler.y() * (-1);  // tu "yaw" invertido

      Serial.print(pitchOut, 4);
      Serial.print(",");
      Serial.println(yawOut, 4);
    }
  }

  // 2) Detector disparo (solo actúa si armed)
  ShotType st = micUpdate();
  if (st == SHOT_MECH) {
    Serial.println("SHOT,MECH");
  } else if (st == SHOT_ELEC) {
    Serial.println("SHOT,ELEC");
  }

  // 3) pacing mic
  delayMicroseconds(MIC_DELAY_US);
}
