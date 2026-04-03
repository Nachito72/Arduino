// ================================================================
//  Test_todos_los_ejes.ino
//  Sketch Arduino para usar junto con Test_todos_los_ejes.pde
//
//  Envía por Serial (500000 baud) una línea CSV por cada ciclo IMU:
//    yaw,roll,gx,gy,gz,gmag,lax,lay,laz,lamag,ax,ay,az,temp,mx,my,mz,cs,cg,ca,cm
//
//  Campos:
//    yaw            Ángulo Euler yaw    (°)      [0, 360]
//    roll           Ángulo Euler roll   (°)      [-180, 180]
//    gx,gy,gz       Giroscopio         (°/s)
//    gmag           Módulo giroscopio  (°/s)
//    lax,lay,laz    Aceleración lineal (m/s²)    (gravedad eliminada)
//    lamag          Módulo acel. lineal(m/s²)
//    ax,ay,az       Acelerómetro crudo (m/s²)    (con gravedad)
//    temp           Temperatura BNO055 (°C)
//    mx,my,mz       Magnetómetro       (µT)
//    cs,cg,ca,cm    Calibración sistema/gyro/accel/mag  [0..3]
//
//  Además envía eventos de texto:
//    "Ready"     al arrancar
//    "ARMED"     cuando el arma entra en posición de tiro
//    "DISARMED"  cuando sale de posición de tiro
//    "SHOT,MECH" cuando se detecta un disparo por micrófono
//
//  Modo BNO055: NDOF (accel + gyro + magnetómetro) para ver todos los ejes
//  incluido magnetómetro. Cambia a OPERATION_MODE_IMUPLUS si hay interferencia
//  magnética y no necesitas mx,my,mz.
//
//  CONEXIONES:
//    BNO055 SDA → A4 (Uno/Nano) o SDA del Arduino que uses
//    BNO055 SCL → A5 (Uno/Nano) o SCL del Arduino que uses
//    BNO055 VIN → 3.3V
//    Micrófono OUT → A1
// ================================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

// ===================== IMU =====================
Adafruit_BNO055 bno = Adafruit_BNO055();

// ===================== ARME POR INCLINACIÓN =====================
// Mismos rangos que MovSoloPlanoDisparoMaxi para consistencia
const float ROLL_MIN_ON   = -130.0f;
const float ROLL_MAX_ON   =  -60.0f;
const float PITCH_MIN_ON  =  -90.0f;
const float PITCH_MAX_ON  =   10.0f;

const float ROLL_MIN_OFF  = -133.0f;
const float ROLL_MAX_OFF  =  -57.0f;
const float PITCH_MIN_OFF =  -93.0f;
const float PITCH_MAX_OFF =   13.0f;

const uint8_t LEVEL_ON_COUNT  = 5;
const uint8_t LEVEL_OFF_COUNT = 8;

bool    armed          = false;
uint8_t levelOnStreak  = 0;
uint8_t levelOffStreak = 0;

// ===================== MIC / DETECTOR DISPARO =====================
const uint8_t  MIC_PIN        = A1;
const uint16_t CONFIRM_SAMPLES = 80;
const uint16_t COOLDOWN_MS     = 5000;
const uint16_t MARGIN_ON       = 90;
const uint16_t MARGIN_OFF      = 45;
const uint16_t RANGE_MIN       = 100;
const uint32_t MIC_TICK_US     = 1000;  // 1 kHz

uint16_t env  = 0;
uint16_t base = 0;
int16_t  micCenter  = 512;
uint32_t lastShotMs = 0;
bool     candidate  = false;
uint16_t candCount  = 0;
uint16_t minRawSeen = 1023;
uint16_t maxRawSeen = 0;
uint16_t peakAbs    = 0;
uint32_t nextMicUs  = 0;
bool     shotFlag   = false;

static inline uint16_t ema_div(uint16_t cur, uint16_t tgt, uint8_t p) {
  int32_t d = (int32_t)tgt - (int32_t)cur;
  return (uint16_t)((int32_t)cur + (d >> p));
}
static inline uint16_t u16abs(int16_t v) { return v < 0 ? (uint16_t)(-v) : (uint16_t)v; }

void resetMicCandidate() { candidate = false; candCount = 0; }

bool micUpdate(uint16_t raw, uint32_t nowMs) {
  if (lastShotMs && (nowMs - lastShotMs) < COOLDOWN_MS) return false;
  int16_t  c   = (int16_t)raw - micCenter;
  uint16_t mag = u16abs(c);
  env  = ema_div(env,  mag,    2);
  base = ema_div(base, env,    8);
  uint16_t thrOn = base + MARGIN_ON;
  if (!candidate) {
    if (env > thrOn) { candidate = true; candCount = 0; minRawSeen = raw; maxRawSeen = raw; peakAbs = mag; }
    return false;
  }
  candCount++;
  if (raw < minRawSeen) minRawSeen = raw;
  if (raw > maxRawSeen) maxRawSeen = raw;
  if (mag > peakAbs)    peakAbs    = mag;
  if ((maxRawSeen - minRawSeen) >= RANGE_MIN) { resetMicCandidate(); return true; }
  if (candCount >= CONFIRM_SAMPLES) candidate = false;
  return false;
}

void calibrateMicCenter() {
  int32_t sum = 0;
  for (uint16_t i = 0; i < 500; i++) { sum += analogRead(MIC_PIN); delay(1); }
  micCenter = (int16_t)(sum / 500L);
}

// ===================== TIMING IMU =====================
const uint32_t IMU_PERIOD_MS = 20;  // 50 Hz — suficiente para ver todos los vectores
uint32_t nextImuMs = 0;

// ===================== ARME =====================
void updateArming(float roll, float pitch) {
  bool canArm = roll  >= ROLL_MIN_ON  && roll  <= ROLL_MAX_ON  &&
                pitch >= PITCH_MIN_ON && pitch <= PITCH_MAX_ON;
  bool shouldDisarm = roll  < ROLL_MIN_OFF || roll  > ROLL_MAX_OFF ||
                      pitch < PITCH_MIN_OFF|| pitch > PITCH_MAX_OFF;
  if (!armed) {
    levelOnStreak = canArm ? min(levelOnStreak + 1, 255) : 0;
    if (levelOnStreak >= LEVEL_ON_COUNT) {
      armed = true; levelOnStreak = 0; levelOffStreak = 0;
      resetMicCandidate(); Serial.println("ARMED");
    }
  } else {
    levelOffStreak = shouldDisarm ? min(levelOffStreak + 1, 255) : 0;
    if (levelOffStreak >= LEVEL_OFF_COUNT) {
      armed = false; levelOffStreak = 0; levelOnStreak = 0;
      resetMicCandidate(); Serial.println("DISARMED");
    }
  }
}

// ================================================================
void setup() {
  Serial.begin(500000);
  while (!Serial) {}

  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.println("// ERROR: BNO055 no encontrado");
    while (1) {}
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  calibrateMicCenter();
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - micCenter);
    env  = ema_div(env,  mag,    2);
    base = ema_div(base, env,    8);
    delay(2);
  }

  nextMicUs = micros();
  nextImuMs = millis();
  Serial.println("Ready");
  Serial.println("DISARMED");
}

// ================================================================
void loop() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // --- Mic ---
  if ((int32_t)(nowUs - nextMicUs) >= 0) {
    uint16_t raw = analogRead(MIC_PIN);
    if (micUpdate(raw, nowMs)) shotFlag = true;
    nextMicUs += MIC_TICK_US;
  }

  if (shotFlag) {
    shotFlag   = false;
    lastShotMs = nowMs;
    Serial.println("SHOT,MECH");
  }

  // --- IMU a 50 Hz ---
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    // Euler (usando getVector para tener los 3 ángulos directamente)
    // En NDOF: .x=heading(yaw), .y=roll, .z=pitch  (convención BNO055)
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float yawDeg  = euler.x();   // heading 0-360°
    float rollDeg = euler.y();   // roll ±90°
    float pitchDeg= euler.z();   // pitch ±180°

    // Giroscopio (°/s)
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float gx = gyro.x(), gy = gyro.y(), gz = gyro.z();
    float gmag = sqrt(gx*gx + gy*gy + gz*gz);

    // Aceleración lineal (m/s², sin gravedad)
    imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    float lax = linacc.x(), lay = linacc.y(), laz = linacc.z();
    float lamag = sqrt(lax*lax + lay*lay + laz*laz);

    // Acelerómetro crudo (m/s², con gravedad)
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float ax = acc.x(), ay = acc.y(), az = acc.z();

    // Temperatura
    int8_t temp = bno.getTemp();

    // Magnetómetro (µT)
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    float mx = mag.x(), my = mag.y(), mz = mag.z();

    // Calibración
    uint8_t cs, cg, ca, cm;
    bno.getCalibration(&cs, &cg, &ca, &cm);

    // Lógica de arme usando ángulos Euler del BNO055
    // roll=euler.y, pitch=euler.z (convención Adafruit)
    updateArming(rollDeg, pitchDeg);

    // Enviar CSV: 21 campos
    Serial.print(yawDeg,  2); Serial.print(',');
    Serial.print(rollDeg, 2); Serial.print(',');
    Serial.print(gx, 2);     Serial.print(',');
    Serial.print(gy, 2);     Serial.print(',');
    Serial.print(gz, 2);     Serial.print(',');
    Serial.print(gmag, 2);   Serial.print(',');
    Serial.print(lax, 2);    Serial.print(',');
    Serial.print(lay, 2);    Serial.print(',');
    Serial.print(laz, 2);    Serial.print(',');
    Serial.print(lamag, 2);  Serial.print(',');
    Serial.print(ax, 2);     Serial.print(',');
    Serial.print(ay, 2);     Serial.print(',');
    Serial.print(az, 2);     Serial.print(',');
    Serial.print(temp);      Serial.print(',');
    Serial.print(mx, 2);     Serial.print(',');
    Serial.print(my, 2);     Serial.print(',');
    Serial.print(mz, 2);     Serial.print(',');
    Serial.print(cs);        Serial.print(',');
    Serial.print(cg);        Serial.print(',');
    Serial.print(ca);        Serial.print(',');
    Serial.println(cm);
  }
}
