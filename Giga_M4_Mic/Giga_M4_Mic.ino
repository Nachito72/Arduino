// ================================================================
//  Giga_M4_Mic.ino  —  CORE M4 (secundario)
//  Arduino Giga R1 WiFi
//
//  Detector de disparo por micrófono, dedicado 100% al M4.
//  Sin I2C, sin Serial propio: solo ADC + detección + RPC al M7.
//
//  Totalmente independiente del core M7:
//    - No necesita saber si el sistema está armado.
//    - Corre el detector de disparo siempre.
//    - Solo llama al M7 vía RPC cuando detecta un disparo (raro).
//    - M7 nunca llama al M4 durante la operación normal.
//
//  El M4 puede muestrear el ADC a ~5-10 kHz sin interferencia
//  del I2C del BNO055 (que corre en M7).
//
//  INSTRUCCIONES:
//    1. Selecciona Tools → Board → Arduino Giga (M4 core)
//    2. Sube ESTE sketch primero
//    3. Luego sube Giga_MovSoloPlano.ino al M7
// ================================================================

#include <RPC.h>

// ===================== VERSIÓN =====================
#define VERSION_M4 "1.1"
const int VERSION_M4_INT = 110;  // major*100 + minor

// ===================== PIN MIC =====================
const uint8_t MIC_PIN = A1;   // mismo pin que el original

// ===================== PARÁMETROS DETECTOR =====================
// Idénticos a MovSoloPlanoDisparoMaxi para resultados comparables

const uint16_t CONFIRM_SAMPLES = 80;
const uint16_t COOLDOWN_MS     = 5000;
const uint16_t MARGIN_ON       = 90;
const uint16_t MARGIN_OFF      = 45;
const uint16_t RANGE_MIN       = 100;

// Cadencia de muestreo del ADC.
// En M4 dedicado podemos bajar a 200 µs (5 kHz) sin problemas:
// el I2C del IMU ya no interfiere aquí.
const uint32_t MIC_TICK_US = 200;   // 5 kHz — vs 1 kHz del original

// ===================== ESTADO DETECTOR =====================
uint16_t env  = 0;
uint16_t base = 0;

int16_t  micCenter   = 444;
uint32_t lastShotMs  = 0;

bool     candidate   = false;
uint16_t candCount   = 0;
uint16_t minRawSeen  = 1023;
uint16_t maxRawSeen  = 0;
uint16_t peakAbs     = 0;
uint16_t widthSamples = 0;
bool     aboveOff    = false;

uint32_t nextMicUs = 0;

// ================================================================
// Funciones helper (idénticas al original)
// ================================================================
static inline uint16_t ema_div(uint16_t current, uint16_t target, uint8_t divPow2) {
  int32_t diff = (int32_t)target - (int32_t)current;
  return (uint16_t)((int32_t)current + (diff >> divPow2));
}

static inline uint16_t u16abs(int16_t v) {
  return (v < 0) ? (uint16_t)(-v) : (uint16_t)v;
}

static inline void updateWidth(uint16_t envVal, uint16_t thrOff) {
  if (envVal > thrOff) { widthSamples++; aboveOff = true; }
  else if (aboveOff)   { aboveOff = false; }
}

void resetStats(uint16_t raw, uint16_t mag, uint16_t thrOff) {
  minRawSeen  = raw;
  maxRawSeen  = raw;
  peakAbs     = mag;
  widthSamples = 0;
  aboveOff    = false;
  updateWidth(env, thrOff);
}

void resetMicCandidate() {
  candidate    = false;
  candCount    = 0;
  widthSamples = 0;
  aboveOff     = false;
}

// ================================================================
// micUpdate — algoritmo idéntico al original
// ================================================================
bool micUpdate(uint16_t raw, uint32_t nowMs) {
  if (lastShotMs != 0 && (nowMs - lastShotMs) < COOLDOWN_MS) return false;

  int16_t  centered = (int16_t)raw - micCenter;
  uint16_t mag      = u16abs(centered);

  env  = ema_div(env, mag, 2);
  base = ema_div(base, env, 8);

  uint16_t thrOn  = base + MARGIN_ON;
  uint16_t thrOff = base + MARGIN_OFF;

  if (!candidate) {
    if (env > thrOn) {
      candidate = true;
      candCount = 0;
      resetStats(raw, mag, thrOff);
    }
    return false;
  }

  candCount++;
  if (raw < minRawSeen) minRawSeen = raw;
  if (raw > maxRawSeen) maxRawSeen = raw;
  if (mag > peakAbs)   peakAbs    = mag;
  updateWidth(env, thrOff);

  uint16_t range = maxRawSeen - minRawSeen;
  if (range >= RANGE_MIN) {
    resetMicCandidate();
    return true;   // DISPARO
  }

  if (candCount >= CONFIRM_SAMPLES) {
    candidate = false;
  }

  return false;
}

// ================================================================
// RPC: M7 consulta la versión del M4
// ================================================================
int onGetVersionM4() { return VERSION_M4_INT; }

// ================================================================
void calibrateMicCenter() {
  int32_t sum = 0;
  const uint16_t N = 500;
  for (uint16_t i = 0; i < N; i++) {
    sum += analogRead(MIC_PIN);
    delay(1);
  }
  micCenter = (int16_t)(sum / (int32_t)N);
}

// ================================================================
void setup() {
  // M4 no usa Serial propio — la comunicación es via RPC al M7
  RPC.begin();
  RPC.bind("getVersionM4", onGetVersionM4);

  // Calibrar centro del micrófono
  calibrateMicCenter();

  // Calentar EMA con unas muestras iniciales
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - micCenter);
    env  = ema_div(env, mag, 2);
    base = ema_div(base, env, 8);
    delay(2);
  }

  nextMicUs = micros();
}

// ================================================================
void loop() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // Muestreo ADC a 5 kHz (MIC_TICK_US=200µs)
  // El M4 no tiene nada más que gestionar: ADC corre limpio
  if ((int32_t)(nowUs - nextMicUs) >= 0) {
    uint16_t raw = analogRead(MIC_PIN);

    // Detector siempre activo — M7 decide si usa el disparo según su estado de arme.
    // M4 no necesita conocer el estado de arme: los dos cores son independientes.
    bool shot = micUpdate(raw, nowMs);
    if (shot) {
      lastShotMs = nowMs;
      RPC.call("shotDetected");
    }

    nextMicUs += MIC_TICK_US;
  }
}
