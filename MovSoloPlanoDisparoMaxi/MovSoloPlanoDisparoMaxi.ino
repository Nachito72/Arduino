#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== LED DISPARO (parpadeo) =====================
const uint8_t LED_SHOT_PIN = LED_BUILTIN;

// Parpadeo visible
const uint16_t LED_BLINK_PERIOD_MS = 200;  // 200ms = 5 Hz (muy visible)
const uint16_t LED_BLINK_TOTAL_MS  = 3000; // 3 segundos parpadeando

bool ledBlinkActive = false;
uint32_t ledBlinkEndMs = 0;
uint32_t ledNextToggleMs = 0;
bool ledState = false;

// ===================== LED DISPARO =====================
const uint16_t LED_SHOT_MS = 120;   // tiempo encendido (ms)

bool ledShotOn = false;
uint32_t ledShotOffMs = 0;

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
const float LEVEL_ON_DEG  = 15.0;
const float LEVEL_OFF_DEG = 20.0;

const uint8_t LEVEL_ON_COUNT  = 5;   // con IMU_PERIOD_MS=10 => 50ms
const uint8_t LEVEL_OFF_COUNT = 8;   // con IMU_PERIOD_MS=10 => 80ms

bool armed = false;
uint8_t levelOnStreak = 0;
uint8_t levelOffStreak = 0;

static inline float f_abs(float v) { return (v < 0) ? -v : v; }

// Distancia angular a “nivel”: 0° o 180° (lo que esté más cerca)
float pitchLevelDeg(float pitchDeg) {
  float ap = f_abs(pitchDeg);
  float d180 = f_abs(180.0f - ap);
  return (ap < d180) ? ap : d180;
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

void startShotBlink(uint32_t nowMs) {
  ledBlinkActive = true;
  ledBlinkEndMs = nowMs + LED_BLINK_TOTAL_MS;
  ledNextToggleMs = nowMs;   // toggle inmediato
  ledState = false;
  digitalWrite(LED_SHOT_PIN, LOW);
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
  if (!armed) return SHOT_NONE;

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

  bool isMech = (minRawSeen <= SAT_LOW) || (maxRawSeen >= SAT_HIGH);
  bool isElec =
    !isMech &&
    width_us >= WIDTH_ELEC_MIN_US &&
    (peakAbs >= PEAK_ELEC_MIN || range >= RANGE_ELEC_MIN);

  // ✅ FIX 1: permitir MECH o ELEC (antes tenías isElec deshabilitado)

  if (isMech) {// || isElec) {
    candidate = false;
    lastShotMs = nowMs;
    
    // 🔴 LED disparo
    startShotBlink(nowMs); 

    //digitalWrite(LED_SHOT_PIN, HIGH);
    //ledShotOn = true;
    //ledShotOffMs = nowMs + LED_SHOT_MS;

    return isMech ? SHOT_MECH : SHOT_ELEC;
  }

  if (candCount >= CONFIRM_SAMPLES) {
    candidate = false;
  }

  return SHOT_NONE;
}

// ===================== Timing IMU =====================
const uint32_t IMU_PERIOD_MS = 10; // 100 Hz
uint32_t nextImuMs = 0;

void updateArmingFromTilt() {
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll  = e.y();
  float pitch = e.z();

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
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_SHOT_PIN, OUTPUT);
  digitalWrite(LED_SHOT_PIN, LOW);

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
  // 1) scheduler IMU: actualiza armado y, si armed, envía datos
  uint32_t nowMs = millis();
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    updateArmingFromTilt();

    if (armed) {
      // Euler (macro movimiento)
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      float pitchOut = euler.x();
      float yawOut   = -euler.y();

      // Gyro → grados/segundo
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      const float RAD2DEG = 57.2957795f;

      float gx = gyro.x() * RAD2DEG;
      float gy = gyro.y() * RAD2DEG;
      float gz = gyro.z() * RAD2DEG;

      float gmag = sqrt(gx*gx + gy*gy + gz*gz);

      uint8_t cs, cg, ca, cm;
      bno.getCalibration(&cs, &cg, &ca, &cm);

      Serial.print(pitchOut, 4); Serial.print(",");
      Serial.print(yawOut, 4);   Serial.print(",");
      Serial.print(gx, 4);       Serial.print(",");
      Serial.print(gy, 4);       Serial.print(",");
      Serial.print(gz, 4);       Serial.print(",");
      Serial.print(gmag, 4);     Serial.print(",");
      Serial.print(cs);          Serial.print(",");
      Serial.print(cg);          Serial.print(",");
      Serial.print(ca);          Serial.print(",");
      Serial.println(cm);
    }
  }

  // 2) Detector disparo: muestrea varias veces para no perder el pulso
  // ✅ FIX 2: "mic burst" para mantener muestreo alto aunque IMU/Serial tarden
  for (uint8_t i = 0; i < 6; i++) { // 6*250us ≈ 1.5ms por loop extra
    ShotType st = micUpdate();
    if (st == SHOT_MECH) {
      Serial.println("SHOT,MECH");
    } else if (st == SHOT_ELEC) {
      Serial.println("SHOT,ELEC");
    }
    delayMicroseconds(MIC_DELAY_US);
  }
  // ⏱ apagar LED disparo sin bloquear
  //if (ledShotOn && millis() >= ledShotOffMs) {
  //  digitalWrite(LED_SHOT_PIN, LOW);
  //  ledShotOn = false;
  //}
  // ===================== LED blink update =====================
  
  if (ledBlinkActive) {
    if ((int32_t)(nowMs - ledBlinkEndMs) >= 0) {
      ledBlinkActive = false;
      ledState = false;
      digitalWrite(LED_SHOT_PIN, LOW);
    } else if ((int32_t)(nowMs - ledNextToggleMs) >= 0) {
      ledState = !ledState;
      digitalWrite(LED_SHOT_PIN, ledState ? HIGH : LOW);
      ledNextToggleMs = nowMs + (LED_BLINK_PERIOD_MS / 2); // on/off = mitad periodo
    }
  }  
}
