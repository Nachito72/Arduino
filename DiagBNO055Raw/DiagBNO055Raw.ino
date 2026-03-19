/*  DiagBNO055Raw.ino
 *  ─────────────────────────────────────────────────────────────────
 *  Diagnóstico crudo del BNO055: sin armado, sin restricciones,
 *  sin detección de disparo. Envía SIEMPRE todos los datos para
 *  poder evaluar los ejes en Processing.
 *
 *  Formato CSV (1 trama cada 10 ms → 100 Hz):
 *
 *    tMs , qw , qx , qy , qz , roll , pitch , yaw , mic_raw , mic_env , mic_base
 *
 *  tMs       → millis() (ms, entero largo)
 *  qw..qz    → cuaternión crudo del BNO055 (6 decimales)
 *  roll      → rotación sobre X calculada del cuaternión (°, 2 dec)
 *  pitch     → rotación sobre Y calculada del cuaternión (°, 2 dec)
 *  yaw       → rotación sobre Z [0..360°] calculada del cuaternión (°, 2 dec)
 *  mic_raw   → valor ADC crudo del micrófono (0-1023)
 *  mic_env   → envelope EMA de la señal del micrófono
 *  mic_base  → nivel de ruido base EMA
 *
 *  Serie: 500000 baudios
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ──────────────────── BNO055 ────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055();

// ──────────────────── MIC ───────────────────────
const uint8_t  MIC_PIN      = A1;
const uint32_t MIC_TICK_US  = 1000;   // 1 kHz

int16_t  micCenter = 512;
uint16_t mic_env   = 0;
uint16_t mic_base  = 0;
uint16_t mic_raw_last = 512;          // última muestra leída, para enviarla
uint32_t nextMicUs = 0;

static inline uint16_t ema_div(uint16_t cur, uint16_t tgt, uint8_t n) {
  int32_t d = (int32_t)tgt - (int32_t)cur;
  return (uint16_t)((int32_t)cur + (d >> n));
}
static inline uint16_t u16abs(int16_t v) { return v < 0 ? (uint16_t)(-v) : (uint16_t)v; }

void calibrateMicCenter() {
  int32_t sum = 0;
  for (uint16_t i = 0; i < 500; i++) { sum += analogRead(MIC_PIN); delay(1); }
  micCenter = (int16_t)(sum / 500);
}

// ──────────────────── IMU timing ────────────────
const uint32_t IMU_PERIOD_MS = 10;   // 100 Hz
uint32_t nextImuMs = 0;

// ──────────────────── setup / loop ──────────────
void setup() {
  Serial.begin(500000);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 no encontrado");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  calibrateMicCenter();

  // calentar EMA del micrófono
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - micCenter);
    mic_env  = ema_div(mic_env,  mag,     2);
    mic_base = ema_div(mic_base, mic_env, 8);
    delay(2);
  }

  nextMicUs = micros();
  nextImuMs = millis();

  // Cabecera CSV (Processing la detecta y salta)
  Serial.println("tMs,qw,qx,qy,qz,roll,pitch,yaw,mic_raw,mic_env,mic_base");
}

void loop() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // ── 1) Muestra de micrófono a 1 kHz ──────────
  if ((int32_t)(nowUs - nextMicUs) >= 0) {
    uint16_t raw = analogRead(MIC_PIN);
    mic_raw_last = raw;
    uint16_t mag = u16abs((int16_t)raw - micCenter);
    mic_env  = ema_div(mic_env,  mag,     2);
    mic_base = ema_div(mic_base, mic_env, 8);
    nextMicUs += MIC_TICK_US;
  }

  // ── 2) Trama IMU a 100 Hz ─────────────────────
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    // Cuaternión crudo (14 bits por componente)
    imu::Quaternion q = bno.getQuat();
    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    // Euler calculado aquí para referencia (Processing también los calcula)
    const double R2D = 57.29577951;

    double sinr = 2.0*(qw*qx + qy*qz);
    double cosr = 1.0 - 2.0*(qx*qx + qy*qy);
    double roll  = atan2(sinr, cosr) * R2D;

    double sinp  = 2.0*(qw*qy - qz*qx);
    double pitch = (fabs(sinp) >= 1.0) ? copysign(90.0, sinp) : asin(sinp) * R2D;

    double siny  = 2.0*(qw*qz + qx*qy);
    double cosy  = 1.0 - 2.0*(qy*qy + qz*qz);
    double yaw   = atan2(siny, cosy) * R2D;
    if (yaw < 0) yaw += 360.0;

    // tMs
    Serial.print(nowMs);        Serial.print(",");
    // cuaternión
    Serial.print(qw, 6);        Serial.print(",");
    Serial.print(qx, 6);        Serial.print(",");
    Serial.print(qy, 6);        Serial.print(",");
    Serial.print(qz, 6);        Serial.print(",");
    // euler
    Serial.print(roll,  2);     Serial.print(",");
    Serial.print(pitch, 2);     Serial.print(",");
    Serial.print(yaw,   2);     Serial.print(",");
    // micrófono
    Serial.print(mic_raw_last); Serial.print(",");
    Serial.print(mic_env);      Serial.print(",");
    Serial.println(mic_base);
  }
}
