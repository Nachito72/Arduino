#include <Arduino.h>

const uint8_t MIC_PIN = A0;

// -------- Sampling --------
const uint16_t SAMPLE_DELAY_US = 250;     // ~4 kHz aprox

// Ventana para confirmar/clasificar (cubre tus duraciones: 5–11 ms)
const uint16_t CONFIRM_SAMPLES = 80;      // 20 ms

// Anti doble disparo
const uint16_t COOLDOWN_MS = 250;

// -------- Envelope/Base (enteros) --------
// env rápida: env += (mag-env)/4  (alpha=0.25)
// base lenta: base += (env-base)/256 (alpha~0.0039)
static inline uint16_t ema_div(uint16_t current, uint16_t target, uint8_t divPow2) {
  int32_t diff = (int32_t)target - (int32_t)current;
  return (uint16_t)((int32_t)current + (diff >> divPow2));
}
static inline uint16_t u16abs(int16_t v) { return (v < 0) ? (uint16_t)(-v) : (uint16_t)v; }

uint16_t env = 0;
uint16_t base = 0;

// Trigger por energía
uint16_t MARGIN_ON  = 90;
uint16_t MARGIN_OFF = 45;

// -------- Clasificación según tus datos --------
// Firma mecánico: doble saturación
const uint16_t SAT_LOW  = 5;
const uint16_t SAT_HIGH = 1018;

// Electrónico: observado mínimo peakAbs=181 y width>=6250us
uint16_t PEAK_ELEC_MIN     = 170;
uint16_t WIDTH_ELEC_MIN_US = 6000;
uint16_t RANGE_ELEC_MIN    = 250;

// -------- Estado --------
int16_t center = 512;
uint32_t lastShotMs = 0;

bool candidate = false;
uint16_t candCount = 0;

// Stats durante ventana
uint16_t minRawSeen = 1023;
uint16_t maxRawSeen = 0;
uint16_t peakAbs = 0;

// width (aprox): contamos muestras con env > thrOff
uint16_t widthSamples = 0;
bool aboveOff = false;

void calibrateCenter() {
  int32_t sum = 0;
  const uint16_t N = 500;
  for (uint16_t i = 0; i < N; i++) { sum += analogRead(MIC_PIN); delay(1); }
  center = (int16_t)(sum / (int32_t)N);
}

void resetStats(uint16_t raw, uint16_t mag) {
  minRawSeen = raw;
  maxRawSeen = raw;
  peakAbs = mag;
  widthSamples = 0;
  aboveOff = false;
}

static inline void updateWidth(uint16_t envVal, uint16_t thrOff) {
  if (envVal > thrOff) {
    widthSamples++;
    aboveOff = true;
  } else if (aboveOff) {
    aboveOff = false;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  calibrateCenter();

  // init env/base
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - center);
    env  = ema_div(env, mag, 2);
    base = ema_div(base, env, 8);
    delay(2);
  }

  Serial.println(F("Ready. Output only: SHOT,MECH or SHOT,ELEC"));
}

void loop() {
  uint32_t nowMs = millis();
  if (nowMs - lastShotMs < COOLDOWN_MS) {
    delayMicroseconds(SAMPLE_DELAY_US);
    return;
  }

  uint16_t raw = analogRead(MIC_PIN);
  int16_t centered = (int16_t)raw - center;
  uint16_t mag = u16abs(centered);

  // env/base
  env  = ema_div(env, mag, 2);
  base = ema_div(base, env, 8);

  uint16_t thrOn  = base + MARGIN_ON;
  uint16_t thrOff = base + MARGIN_OFF;

  if (!candidate) {
    // Disparo candidato: sube energía
    if (env > thrOn) {
      candidate = true;
      candCount = 0;
      resetStats(raw, mag);
      updateWidth(env, thrOff);
    }

    delayMicroseconds(SAMPLE_DELAY_US);
    return;
  }

  // candidate mode: acumular stats en la ventana de confirmación
  candCount++;

  if (raw < minRawSeen) minRawSeen = raw;
  if (raw > maxRawSeen) maxRawSeen = raw;
  if (mag > peakAbs) peakAbs = mag;
  updateWidth(env, thrOff);

  uint16_t range = maxRawSeen - minRawSeen;
  uint32_t width_us = (uint32_t)widthSamples * (uint32_t)SAMPLE_DELAY_US;

  // Clasificación
  bool isMech = (minRawSeen <= SAT_LOW) && (maxRawSeen >= SAT_HIGH);

  bool isElecA = (peakAbs >= PEAK_ELEC_MIN) && (width_us >= WIDTH_ELEC_MIN_US);
  bool isElecB = (range  >= RANGE_ELEC_MIN) && (width_us >= WIDTH_ELEC_MIN_US);
  bool isElec = (!isMech) && (isElecA || isElecB);

  if (isMech) {
    Serial.println(F("SHOT,MECH"));
    lastShotMs = nowMs;
    candidate = false;
  } else if (isElec) {
    Serial.println(F("SHOT,ELEC"));
    lastShotMs = nowMs;
    candidate = false;
  } else if (candCount >= CONFIRM_SAMPLES) {
    // No confirmó -> era ruido/falso
    candidate = false;
  }

  delayMicroseconds(SAMPLE_DELAY_US);
}
