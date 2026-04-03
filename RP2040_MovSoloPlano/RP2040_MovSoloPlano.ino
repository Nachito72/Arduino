// ================================================================
//  RP2040_MovSoloPlano.ino  —  Raspberry Pi RP2040-Zero
//
//  CORE 0 (setup / loop):
//    - BNO055 por I2C (Wire: SDA=GP4, SCL=GP5)
//    - Lógica de arme por inclinación
//    - Salida Serial → Processing (500000 baud)
//    - LED parpadeo tras disparo
//
//  CORE 1 (setup1 / loop1):
//    - ADC en A0 (GP26) — micrófono
//    - Detección de disparo (algoritmo EMA idéntico al original)
//    - Cuando detecta disparo → rp2040.fifo.push_nb(1) al Core 0
//
//  COMUNICACIÓN ENTRE CORES:
//    Core 1 → Core 0 : FIFO hardware (disparo detectado)
//    Core 0 → Core 1 : volatile bool armed_c1 (estado de arme)
//
//  SALIDA SERIAL: idéntica a MovSoloPlanoDisparoMaxi
//    "Ready" / "ARMED" / "DISARMED" / "SHOT,MECH" / "qw,qx,qy,qz"
//
//  LIBRERÍA REQUERIDA:
//    Arduino-Pico by Earle F. Philhower III (Boards Manager)
//    https://github.com/earlephilhower/arduino-pico
//    Adafruit BNO055 + Adafruit Unified Sensor
//
//  NOTAS RP2040:
//    - ADC 12 bits → configurado a 10 bits para paridad con el original
//    - I2C en Fast-Mode 400 kHz para reducir latencia BNO055
//    - micros() usa el timer hardware del RP2040 (preciso)
//    - LED_BUILTIN = GP25 (Pico estándar / Pico-Zero)
// ================================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ================================================================
//  CONFIGURACIÓN DE PINES
// ================================================================
const uint8_t MIC_PIN      = A0;   // GP26 — micrófono (Core 1)
// Waveshare RP2040-Zero: LED NeoPixel en GP16 (no es un LED simple).
// Usamos GP25 como indicador simple con un LED externo, o desactivamos el blink.
// Si no tienes LED externo, deja LED_SHOT_PIN = 25 — no hará nada pero no falla.
const uint8_t LED_SHOT_PIN = 25;   // GP25 — LED externo opcional

// ================================================================
//  LED PARPADEO
// ================================================================
const uint16_t LED_BLINK_PERIOD_MS = 200;
const uint16_t LED_BLINK_TOTAL_MS  = 3000;

bool     ledBlinkActive  = false;
uint32_t ledBlinkEndMs   = 0;
uint32_t ledNextToggleMs = 0;
bool     ledState        = false;

void startShotBlink(uint32_t nowMs) {
  ledBlinkActive  = true;
  ledBlinkEndMs   = nowMs + LED_BLINK_TOTAL_MS;
  ledNextToggleMs = nowMs;
  ledState        = false;
  digitalWrite(LED_SHOT_PIN, LOW);
}

// ================================================================
//  IMU — BNO055 (dirección se detecta automáticamente en setup)
// ================================================================
Adafruit_BNO055 bno28_g = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bno29_g = Adafruit_BNO055(55, 0x29, &Wire);
Adafruit_BNO055 *bno_p  = nullptr;  // puntero al objeto activo

// ================================================================
//  ARME POR INCLINACIÓN (mismos rangos que el original)
// ================================================================
const float ROLL_MIN_ON   = -130.0f;
const float ROLL_MAX_ON   =  -60.0f;
const float PITCH_MIN_ON  =  -90.0f;
const float PITCH_MAX_ON  =   10.0f;

const float ROLL_MIN_OFF  = -133.0f;
const float ROLL_MAX_OFF  =  -57.0f;
const float PITCH_MIN_OFF =  -93.0f;
const float PITCH_MAX_OFF =   13.0f;

const uint8_t LEVEL_ON_COUNT  = 7;
const uint8_t LEVEL_OFF_COUNT = 8;

bool    armed          = false;
uint8_t levelOnStreak  = 0;
uint8_t levelOffStreak = 0;

// Variable compartida Core 0 → Core 1 (escribe Core 0, lee Core 1)
// volatile garantiza visibilidad entre cores en ARM M0+
volatile bool armed_c1 = false;

// ================================================================
//  FILTRO COMPLEMENTARIO ELEVACIÓN
//  Idéntico al de MovSoloPlanoDisparoMaxi.ino
// ================================================================
const float ROLL_OFFSET  = -85.0f;  // offset de montaje (la posición "plana" del arma)
const float FILTER_ALPHA = 0.95f;   // peso del giroscopio; τ ≈ 100 ms a 200 Hz
float filteredElev = 0.0f;
bool  filtElevInit = false;

// ================================================================
//  TIMING IMU
// ================================================================
const uint32_t IMU_PERIOD_MS = 5;   // 200 Hz poll (BNO055 fusión ~100 Hz)
uint32_t nextImuMs = 0;

// ================================================================
//  DETECTOR MIC — parámetros (idénticos al original)
// ================================================================
const uint16_t CONFIRM_SAMPLES = 80;
const uint16_t COOLDOWN_MS     = 5000;
const uint16_t MARGIN_ON       = 90;
const uint16_t MARGIN_OFF      = 45;
const uint16_t RANGE_MIN       = 100;

// Core 1 muestrea sin interferencia del I2C:
// 200 µs = 5 kHz (vs 1 kHz en el original, donde el I2C compite)
const uint32_t MIC_TICK_US = 200;

// ================================================================
//  HELPERS comunes (usados en ambos cores — son inline, no tienen estado)
// ================================================================
static inline uint16_t ema_div(uint16_t current, uint16_t target, uint8_t p) {
  int32_t diff = (int32_t)target - (int32_t)current;
  return (uint16_t)((int32_t)current + (diff >> p));
}
static inline uint16_t u16abs(int16_t v) {
  return (v < 0) ? (uint16_t)(-v) : (uint16_t)v;
}

// ================================================================
//  CORE 0 — setup() / loop()
//  BNO055 · arming · Serial · LED
// ================================================================

void updateArmingFromTilt(float roll, float pitch) {
  bool canArm = (roll  >= ROLL_MIN_ON  && roll  <= ROLL_MAX_ON) &&
                (pitch >= PITCH_MIN_ON && pitch <= PITCH_MAX_ON);

  bool shouldDisarm = (roll  < ROLL_MIN_OFF  || roll  > ROLL_MAX_OFF) ||
                      (pitch < PITCH_MIN_OFF || pitch > PITCH_MAX_OFF);

  if (!armed) {
    if (canArm) { if (levelOnStreak  < 255) levelOnStreak++;  }
    else          levelOnStreak = 0;

    if (levelOnStreak >= LEVEL_ON_COUNT) {
      armed      = true;
      armed_c1   = true;   // avisa al Core 1
      levelOnStreak  = 0;
      levelOffStreak = 0;
      Serial.println("ARMED");
    }
  } else {
    if (shouldDisarm) { if (levelOffStreak < 255) levelOffStreak++; }
    else                levelOffStreak = 0;

    if (levelOffStreak >= LEVEL_OFF_COUNT) {
      armed      = false;
      armed_c1   = false;  // avisa al Core 1
      levelOffStreak = 0;
      levelOnStreak  = 0;
      Serial.println("DISARMED");
    }
  }
}

void setup() {
  Serial.begin(115200);   // mismo baud que el original
  while (!Serial && millis() < 3000) {}
  
  Serial.println("Iniciando..3..");
  
  pinMode(LED_SHOT_PIN, OUTPUT);
  digitalWrite(LED_SHOT_PIN, LOW);

  // ADC a 10 bits para paridad con el original (0-1023)
  analogReadResolution(10);

  // BNO055 en modo IMUPLUS (accel + gyro, sin magnetómetro)
  // Probar GP4=SDA/GP5=SCL y también invertido GP4=SCL/GP5=SDA
  bool imuOk = false;
  uint8_t sdaPin = 4, sclPin = 5;

  for (int intento = 0; intento < 2 && !imuOk; intento++) {
    if (intento == 1) { sdaPin = 5; sclPin = 4; }  // invertir en 2º intento

    Wire.end();
    // Activar pull-ups internos del RP2040 antes de Wire.begin()
    // (son débiles ~65kΩ, idealmente usar 4.7kΩ externos)
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
    Wire.setSDA(sdaPin);
    Wire.setSCL(sclPin);
    Wire.begin();
    Wire.setClock(100000);
    delay(300);

    Serial.print("Probando SDA=GP"); Serial.print(sdaPin);
    Serial.print(" SCL=GP"); Serial.println(sclPin);

    // Escanear
    int found = 0;
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("  I2C encontrado en 0x");
        if (addr < 16) Serial.print("0");
        Serial.println(addr, HEX);
        found++;
      }
    }
    if (found == 0) { Serial.println("  Ninguno en este intento"); continue; }

    if (bno28_g.begin(OPERATION_MODE_IMUPLUS)) {
      Serial.println("BNO055 OK en 0x28"); bno_p = &bno28_g; imuOk = true;
    } else if (bno29_g.begin(OPERATION_MODE_IMUPLUS)) {
      Serial.println("BNO055 OK en 0x29"); bno_p = &bno29_g; imuOk = true;
    }
  }

  if (!imuOk) {
    Serial.println("ERROR: BNO055 no encontrado. Comprueba:");
    Serial.println("  1) VIN del BNO055 conectado a VBUS (5V), no a 3V3");
    Serial.println("  2) GND comun entre RP2040 y BNO055");
    Serial.println("  3) SDA/SCL no invertidos");
    while (1) { delay(1000); }
  }

  delay(200);
  bno_p->setExtCrystalUse(true);

  nextImuMs = millis();
  Serial.println("Ready");
  Serial.println("DISARMED");

  // Core 1 arranca automáticamente después de setup() en Arduino-Pico
}

void loop() {
  uint32_t nowMs = millis();

  // --- 1) Leer disparo detectado por Core 1 via FIFO ---
  if (rp2040.fifo.available()) {
    rp2040.fifo.pop();          // consumir el valor (siempre 1 = SHOT)
    startShotBlink(nowMs);
    Serial.println("SHOT,MECH");
  }

  // --- 2) IMU a 200 Hz ---
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    imu::Quaternion q = bno_p->getQuat();
    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    const double R2D = 57.29577951;
    double sinr = 2.0*(qw*qx + qy*qz);
    double cosr = 1.0 - 2.0*(qx*qx + qy*qy);
    double rollDeg  = atan2(sinr, cosr) * R2D;

    double sinp = 2.0*(qw*qy - qz*qx);
    double pitchDeg = (fabs(sinp) >= 1.0) ? copysign(90.0, sinp) : asin(sinp) * R2D;

    updateArmingFromTilt((float)rollDeg, (float)pitchDeg);

    // Filtro complementario: gyro.x() = velocidad angular del eje de roll (°/s)
    // Elimina los "escalones" del acelerómetro manteniendo la referencia absoluta.
    imu::Vector<3> gyroVec = bno_p->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float rollAbs = (float)(rollDeg - ROLL_OFFSET);
    float dtSeg   = IMU_PERIOD_MS / 1000.0f;
    if (!filtElevInit) { filteredElev = rollAbs; filtElevInit = true; }
    else filteredElev = FILTER_ALPHA * (filteredElev + (float)gyroVec.x() * dtSeg)
                      + (1.0f - FILTER_ALPHA) * rollAbs;

    if (armed) {
      // 5 campos: qw,qx,qy,qz,filteredElev
      // Processing usa los 4 cuaterniones para yaw y filteredElev para la elevación.
      Serial.print(qw, 6); Serial.print(",");
      Serial.print(qx, 6); Serial.print(",");
      Serial.print(qy, 6); Serial.print(",");
      Serial.print(qz, 6); Serial.print(",");
      Serial.println(filteredElev, 4);
    }
  }

  // --- 3) LED blink ---
  if (ledBlinkActive) {
    if ((int32_t)(nowMs - ledBlinkEndMs) >= 0) {
      ledBlinkActive = false;
      ledState       = false;
      digitalWrite(LED_SHOT_PIN, LOW);
    } else if ((int32_t)(nowMs - ledNextToggleMs) >= 0) {
      ledState = !ledState;
      digitalWrite(LED_SHOT_PIN, ledState ? HIGH : LOW);
      ledNextToggleMs = nowMs + (LED_BLINK_PERIOD_MS / 2);
    }
  }
}

// ================================================================
//  CORE 1 — setup1() / loop1()
//  ADC A0 · Detección de disparo · FIFO → Core 0
// ================================================================

// Estado del detector (privado al Core 1)
static uint16_t c1_env         = 0;
static uint16_t c1_base        = 0;
static int16_t  c1_micCenter   = 512;   // RP2040 ADC centrado en ~512 (10 bit)
static uint32_t c1_lastShotMs  = 0;
static bool     c1_candidate   = false;
static uint16_t c1_candCount   = 0;
static uint16_t c1_minRaw      = 1023;
static uint16_t c1_maxRaw      = 0;
static uint16_t c1_peakAbs     = 0;
static uint16_t c1_widthSamp   = 0;
static bool     c1_aboveOff    = false;
static uint32_t c1_nextMicUs   = 0;

static void c1_resetStats(uint16_t raw, uint16_t mag, uint16_t thrOff) {
  c1_minRaw    = raw;
  c1_maxRaw    = raw;
  c1_peakAbs   = mag;
  c1_widthSamp = 0;
  c1_aboveOff  = false;
  if (c1_env > thrOff) { c1_widthSamp++; c1_aboveOff = true; }
}

static void c1_resetCandidate() {
  c1_candidate  = false;
  c1_candCount  = 0;
  c1_widthSamp  = 0;
  c1_aboveOff   = false;
}

static bool c1_micUpdate(uint16_t raw, uint32_t nowMs) {
  if (c1_lastShotMs != 0 && (nowMs - c1_lastShotMs) < COOLDOWN_MS) return false;

  int16_t  centered = (int16_t)raw - c1_micCenter;
  uint16_t mag      = u16abs(centered);

  c1_env  = ema_div(c1_env,  mag,    2);
  c1_base = ema_div(c1_base, c1_env, 8);

  uint16_t thrOn  = c1_base + MARGIN_ON;
  uint16_t thrOff = c1_base + MARGIN_OFF;

  if (!c1_candidate) {
    if (c1_env > thrOn) {
      c1_candidate = true;
      c1_candCount = 0;
      c1_resetStats(raw, mag, thrOff);
    }
    return false;
  }

  c1_candCount++;
  if (raw < c1_minRaw) c1_minRaw  = raw;
  if (raw > c1_maxRaw) c1_maxRaw  = raw;
  if (mag > c1_peakAbs) c1_peakAbs = mag;

  if (c1_env > thrOff) { c1_widthSamp++; c1_aboveOff = true; }
  else if (c1_aboveOff) { c1_aboveOff = false; }

  uint16_t range = c1_maxRaw - c1_minRaw;
  if (range >= RANGE_MIN) {
    c1_resetCandidate();
    return true;   // DISPARO DETECTADO
  }

  if (c1_candCount >= CONFIRM_SAMPLES) {
    c1_candidate = false;
  }
  return false;
}

void setup1() {
  // ADC a 10 bits (Core 1 también necesita configurarlo en RP2040)
  analogReadResolution(10);

  // Calibrar centro del micrófono
  int32_t sum = 0;
  const uint16_t N = 500;
  for (uint16_t i = 0; i < N; i++) {
    sum += analogRead(MIC_PIN);
    delay(1);
  }
  c1_micCenter = (int16_t)(sum / (int32_t)N);

  // Calentar EMA
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - c1_micCenter);
    c1_env  = ema_div(c1_env,  mag,    2);
    c1_base = ema_div(c1_base, c1_env, 8);
    delay(2);
  }

  c1_nextMicUs = micros();
}

void loop1() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // Si se desarmó desde Core 0, resetear el candidato activo
  if (!armed_c1) {
    c1_resetCandidate();
    // Esperar a que se arme antes de muestrear
    // (seguimos actualizando EMA para que no pierda la base de ruido)
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - c1_micCenter);
    c1_env  = ema_div(c1_env,  mag,    2);
    c1_base = ema_div(c1_base, c1_env, 8);
    delay(1);
    return;
  }

  // Muestreo ADC a 5 kHz (200 µs por muestra)
  // Core 1 no tiene I2C ni Serial: corre limpio sin interrupciones ajenas
  if ((int32_t)(nowUs - c1_nextMicUs) >= 0) {
    uint16_t raw  = analogRead(MIC_PIN);
    bool     shot = c1_micUpdate(raw, nowMs);

    if (shot) {
      c1_lastShotMs = nowMs;
      // Notificar al Core 0 via FIFO hardware del RP2040
      // push_nb es no bloqueante: si el FIFO está lleno, descarta (no bloquea Core 1)
      rp2040.fifo.push_nb(1);
    }

    c1_nextMicUs += MIC_TICK_US;
  }
}
