#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== LED DISPARO (parpadeo) =====================
const uint8_t LED_SHOT_PIN = LED_BUILTIN;

// Frecuencia y duración del parpadeo del LED al detectar un disparo.
// Subir LED_BLINK_PERIOD_MS → parpadeo más lento (menos visible).
// Subir LED_BLINK_TOTAL_MS  → parpadea durante más tiempo.
const uint16_t LED_BLINK_PERIOD_MS = 200;  // duración de cada ciclo on/off (ms). 200 ms = 5 Hz.
const uint16_t LED_BLINK_TOTAL_MS  = 3000; // tiempo total de parpadeo tras un disparo (ms).

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
// Geometría del arma montada con el BNO055:
//   ROLL  : rotación sobre X. Arma en posición de tiro → aprox. -60° a -130°.
//           Más negativo = más elevada. Más positivo = más baja o volcada.
//   PITCH : rotación sobre Y. Negativo = apunta arriba, positivo = apunta abajo.
//           Rango válido: desde -90° (al cielo) hasta +10° (ligeramente abajo).
//
// Rangos de ARME:
//   ROLL  ∈ [-130°, -60°]   →  ROLL_MIN_ON / ROLL_MAX_ON
//   PITCH ∈ [ -90°, +10°]   →  PITCH_MIN_ON / PITCH_MAX_ON
//
// Rangos de DESARME (histéresis ±3°):
//   ROLL  fuera de [-133°, -57°]  →  ROLL_MIN_OFF / ROLL_MAX_OFF
//   PITCH fuera de [ -93°, +13°]  →  PITCH_MIN_OFF / PITCH_MAX_OFF
const float ROLL_MIN_ON    = -130.0f;
const float ROLL_MAX_ON    =  -60.0f;
const float PITCH_MIN_ON   =  -90.0f;
const float PITCH_MAX_ON   =   10.0f;

const float ROLL_MIN_OFF   = -133.0f;
const float ROLL_MAX_OFF   =  -57.0f;
const float PITCH_MIN_OFF  =  -93.0f;
const float PITCH_MAX_OFF  =   13.0f;

// LEVEL_ON_COUNT  : lecturas consecutivas en rango para ARMAR   (5 × 10ms = 50ms)
// LEVEL_OFF_COUNT : lecturas consecutivas fuera  para DESARMAR  (8 × 10ms = 80ms)
const uint8_t LEVEL_ON_COUNT  = 5;
const uint8_t LEVEL_OFF_COUNT = 8;

bool armed = false;
uint8_t levelOnStreak = 0;
uint8_t levelOffStreak = 0;

static inline float f_abs(float v) { return (v < 0) ? -v : v; }

// ===================== MIC / DETECTOR =====================
const uint8_t MIC_PIN = A1;   // pin analógico donde está conectado el micrófono

// MIC_DELAY_US : tiempo de espera entre muestras del ADC.
//   250 µs → frecuencia de muestreo ~4 kHz (4000 muestras/segundo).
//   Bajar → más muestras por segundo, captura pulsos más cortos, pero deja menos
//           tiempo para el resto del loop (IMU, Serial).
//   Subir → menos carga de CPU pero puede perderse el disparo si es muy breve.
const uint16_t MIC_DELAY_US = 250;

// CONFIRM_SAMPLES : número máximo de muestras que dura una ventana de candidato.
//   Si en ese tiempo no se confirma el disparo, se descarta el candidato.
//   80 muestras × 250 µs = 20 ms de ventana. El disparo real dura ~20 ms en la traza.
//   Subir → ventana más larga, detecta disparos con ataque más lento.
//   Bajar → ventana más corta, descarta antes eventos dudosos.
const uint16_t CONFIRM_SAMPLES = 80;

// COOLDOWN_MS : tiempo mínimo entre dos disparos consecutivos detectados.
//   Evita que un mismo disparo se detecte dos veces.
//   500 ms = 0.5 s. Bajar si los disparos son muy seguidos (tiro rápido).
const uint16_t COOLDOWN_MS = 5000;

// Funciones EMA (media móvil exponencial) para calcular envelope y base.
// ema_div(current, target, n): mueve current hacia target en pasos de 1/2^n.
//   n=2 → converge en ~4 muestras (rápido, para el envelope de la señal).
//   n=8 → converge en ~256 muestras (lento, para la base de ruido ambiente).
static inline uint16_t ema_div(uint16_t current, uint16_t target, uint8_t divPow2) {
  int32_t diff = (int32_t)target - (int32_t)current;
  return (uint16_t)((int32_t)current + (diff >> divPow2));
}
// Valor absoluto de un int16_t devuelto como uint16_t.
static inline uint16_t u16abs(int16_t v) { return (v < 0) ? (uint16_t)(-v) : (uint16_t)v; }

// env  : envelope de la señal (sigue los picos de amplitud rápidamente).
// base : nivel de ruido ambiente (sigue el envelope muy lentamente).
uint16_t env  = 0;
uint16_t base = 0;

// MARGIN_ON  : cuánto debe subir el envelope (env) sobre la base para abrir
//              la ventana de candidato. Es la sensibilidad del disparo.
//   Subir → menos sensible, ignora ruidos pequeños (menos falsos positivos).
//   Bajar → más sensible, abre el candidato con señales más débiles (más falsos positivos).
//   Traza: baseline env ~0, disparo sube env a ~174 en la primera muestra. 90 es seguro.
uint16_t MARGIN_ON  = 90;

// MARGIN_OFF : umbral por debajo del cual el envelope se considera "en silencio".
//   Solo se usa para contar widthSamples (ancho del pulso). No afecta a la detección actual.
//   Mantener ~MARGIN_ON/2.
uint16_t MARGIN_OFF = 45;

// SAT_LOW / SAT_HIGH : límites heredados de versiones anteriores del detector.
//   Ya no se usan en la lógica de detección actual (RANGE_MIN la reemplaza).
//   Se conservan por referencia.
const uint16_t SAT_LOW  = 7;
const uint16_t SAT_HIGH = 700;

// RANGE_MIN : rango ADC pico-a-pico mínimo (maxRaw - minRaw) dentro del candidato
//             para confirmar un disparo mecánico.
//   Traza real del disparo: rango = 713 (vmin=7, vmax=720).
//   Baseline (silencio): rango ~22 (427-449). Hay margen enorme.
//   → Subir si hay falsos positivos (exige una señal más fuerte para disparar).
//   → Bajar si no detecta disparos reales (acepta señales más débiles).
//   Ejemplo: 400 = muy seguro | 300 = más sensible | 500 = muy exigente.
const uint16_t RANGE_MIN = 100;

int16_t micCenter = 444;
uint32_t lastShotMs = 0;

bool candidate = false;
uint16_t candCount = 0;

uint16_t minRawSeen = 1023;
uint16_t maxRawSeen = 0;
uint16_t peakAbs = 0;

uint16_t widthSamples = 0;
bool aboveOff = false;

// Cooldown en muestras (ISR no puede usar millis())
// ===================== Muestreo mic por scheduler de micros() =====================
// Una muestra ADC por vuelta de loop(). El loop corre libre entre ciclos IMU,
// dando ~300-500 muestras/segundo —suficiente para capturar el flanco del disparo
// (~5 ms de duración) sin bloquear el scheduler IMU.
// MIC_TICK_US: cadencia mínima entre muestras. 1000 µs = 1 kHz.
// El loop no se bloquea: si no ha pasado el tick, pasa de largo sin leer el ADC.
const uint32_t MIC_TICK_US = 1000;  // 1 kHz — 1 muestra cada 1 ms como máximo
uint32_t nextMicUs = 0;             // micros() del próximo tick habilitado

bool shotFlag = false;              // disparo detectado, pendiente de enviar

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

// Procesa una muestra del micrófono. Se llama desde loop() (nunca desde ISR).
// raw: valor devuelto por analogRead(MIC_PIN), leído en loop() sin interrupciones.
// nowMs: millis() actual para gestionar el cooldown sin depender de ISR.
ShotType micUpdate(uint16_t raw, uint32_t nowMs) {
  // Cooldown: ignorar muestras hasta que haya pasado COOLDOWN_MS desde el último disparo
  if (lastShotMs != 0 && (nowMs - lastShotMs) < COOLDOWN_MS) return SHOT_NONE;

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
  if (range >= RANGE_MIN) {
    resetMicCandidate();
    return SHOT_MECH;
  }

  if (candCount >= CONFIRM_SAMPLES) {
    candidate = false;
  }

  return SHOT_NONE;
}

// ===================== Timing IMU =====================
const uint32_t IMU_PERIOD_MS = 10; // 100 Hz
uint32_t nextImuMs = 0;

void updateArmingFromTilt(float roll, float pitch) {
  // roll y pitch ya leídos fuera: evita segunda llamada I2C por ciclo

  // ARMAR: roll ∈ [-130°, -60°]  Y  pitch ∈ [-90°, +10°]
  bool canArm = (roll  >= ROLL_MIN_ON  && roll  <= ROLL_MAX_ON) &&
                (pitch >= PITCH_MIN_ON && pitch <= PITCH_MAX_ON);

  // DESARMAR (con histéresis ±3°): cualquier eje fuera de su ventana ampliada
  bool shouldDisarm = (roll  < ROLL_MIN_OFF  || roll  > ROLL_MAX_OFF) ||
                      (pitch < PITCH_MIN_OFF || pitch > PITCH_MAX_OFF);

  if (!armed) {
    if (canArm) {
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
    if (shouldDisarm) {
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
  Serial.begin(500000);

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

  nextMicUs = micros();
  nextImuMs = millis();
  Serial.println("Ready");
  Serial.println("DISARMED");
}

void loop() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // 1) Mic: una sola muestra ADC por vuelta de loop.
  //    El loop corre libre ~300-400 veces/segundo entre ciclos IMU de 10 ms,
  //    lo que da una cadencia real de ~300-400 Hz —suficiente para el flanco
  //    del disparo (~5 ms de duración) sin bloquear el IMU.
  //    Si ya pasó el tick programado, toma la muestra y avanza el scheduler.
  //    Si aún no ha llegado, no hace nada y regresa inmediatamente.
  if ((int32_t)(nowUs - nextMicUs) >= 0) {
    uint16_t raw = analogRead(MIC_PIN);
    ShotType st = micUpdate(raw, nowMs);
    if (st != SHOT_NONE) shotFlag = true;
    nextMicUs += MIC_TICK_US;
  }

  // 2) Comprobar flag de disparo
  if (shotFlag) {
    shotFlag = false;
    lastShotMs = nowMs;
    startShotBlink(nowMs);
    Serial.println("SHOT,MECH");
  }

  // 3) scheduler IMU: una sola lectura I2C por ciclo para arming + serial
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    // Cuaternión: 14 bits de precisión por componente (~0.004° de resolución)
    // vs Euler del BNO055: solo 1/16° = 0.0625°
    imu::Quaternion q = bno.getQuat();
    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    // Convertir a ángulos de Euler (rad → grados) para arming
    // Convención: roll = rotación sobre X, pitch = sobre Y, yaw = sobre Z
    const double R2D = 57.29577951;
    double sinr = 2.0*(qw*qx + qy*qz);
    double cosr = 1.0 - 2.0*(qx*qx + qy*qy);
    double rollDeg  = atan2(sinr, cosr) * R2D;

    double sinp = 2.0*(qw*qy - qz*qx);
    double pitchDeg = (fabs(sinp) >= 1.0) ? copysign(90.0, sinp) : asin(sinp) * R2D;

    double siny = 2.0*(qw*qz + qx*qy);
    double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
    double yawDeg   = atan2(siny, cosy) * R2D;
    if (yawDeg < 0) yawDeg += 360.0;

    // Para arming usamos roll y pitchLat (pitchDeg)
    updateArmingFromTilt((float)rollDeg, (float)pitchDeg);

    if (armed) {
      // Enviar cuaternión como 4 floats: w,x,y,z
      // Processing recalcula roll/yaw/pitchLat con precisión double
      Serial.print(qw, 6); Serial.print(",");
      Serial.print(qx, 6); Serial.print(",");
      Serial.print(qy, 6); Serial.print(",");
      Serial.println(qz, 6);
    }
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
