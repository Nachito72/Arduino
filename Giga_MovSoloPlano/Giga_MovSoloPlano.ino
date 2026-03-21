// ================================================================
//  Giga_MovSoloPlano.ino  —  CORE M7 (principal)
//  Arduino Giga R1 WiFi
//
//  Equivalente exacto a MovSoloPlanoDisparoMaxi pero con dual-core:
//    M7 → IMU BNO055, arming, Serial a Processing, LED
//    M4 → Mic ADC + detección de disparo (ver Giga_M4_Mic.ino)
//
//  SALIDA SERIAL: idéntica al sketch original para poder comparar
//    "Ready"
//    "ARMED" / "DISARMED"
//    "SHOT,MECH"
//    "qw,qx,qy,qz"   (cuando armado, 100 Hz)
//
//  INSTRUCCIONES:
//    1. Sube primero Giga_M4_Mic.ino al M4 (Tools → Board → Giga M4)
//    2. Después sube este sketch al M7 (Tools → Board → Giga M7)
//    3. Abre Serial Monitor a 500000 baud
// ================================================================

#include <Wire.h>
#include <EEPROM.h>
#include <RPC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== LED =====================
const uint8_t LED_SHOT_PIN = LED_BUILTIN;

const uint16_t LED_BLINK_PERIOD_MS = 200;
const uint16_t LED_BLINK_TOTAL_MS  = 3000;

bool     ledBlinkActive  = false;
uint32_t ledBlinkEndMs   = 0;
uint32_t ledNextToggleMs = 0;
bool     ledState        = false;

// ===================== IMU =====================
Adafruit_BNO055 bno = Adafruit_BNO055();

// Dirección EEPROM para offsets de calibración del BNO055
const int  EEPROM_ADDR  = 0;
const byte EEPROM_FIRMA = 0xB7;

// ===================== DISPAROS =====================
// El M4 detecta el disparo y llama a shotDetected() vía RPC.
// M7 recibe la llamada y pone este flag.
volatile bool shotPending = false;

// Cooldown compartido: M7 informa al M4 tras confirmar el disparo
// para que reinicie su cooldown interno.
uint32_t lastShotMs = 0;
const uint16_t COOLDOWN_MS = 5000;

// ===================== ARME POR INCLINACIÓN =====================
// Mismos rangos que MovSoloPlanoDisparoMaxi
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

bool    armed         = false;
uint8_t levelOnStreak  = 0;
uint8_t levelOffStreak = 0;

// ===================== TIMING IMU =====================
const uint32_t IMU_PERIOD_MS = 10;   // 100 Hz — igual que el original
uint32_t nextImuMs = 0;

// ================================================================
// RPC: función que el M4 llama cuando detecta un disparo
// ================================================================
int onShotDetected() {
  shotPending = true;
  return 1;
}

// ================================================================
// EEPROM: cargar offsets de calibración del BNO055
// ================================================================
bool cargarOffsetsBNO() {
  byte firma;
  EEPROM.get(EEPROM_ADDR, firma);
  if (firma != EEPROM_FIRMA) return false;

  adafruit_bno055_offsets_t offsets;
  EEPROM.get(EEPROM_ADDR + 1, offsets);
  bno.setSensorOffsets(offsets);
  return true;
}

// ================================================================
void startShotBlink(uint32_t nowMs) {
  ledBlinkActive  = true;
  ledBlinkEndMs   = nowMs + LED_BLINK_TOTAL_MS;
  ledNextToggleMs = nowMs;
  ledState        = false;
  digitalWrite(LED_SHOT_PIN, LOW);
}

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
      armed = true;
      levelOnStreak  = 0;
      levelOffStreak = 0;
      // Avisar al M4 que está armado para que active la detección
      RPC.call("setArmed", (int)1);
      Serial.println("ARMED");
    }
  } else {
    if (shouldDisarm) { if (levelOffStreak < 255) levelOffStreak++; }
    else                levelOffStreak = 0;

    if (levelOffStreak >= LEVEL_OFF_COUNT) {
      armed          = false;
      levelOffStreak = 0;
      levelOnStreak  = 0;
      // Avisar al M4 que está desarmado
      RPC.call("setArmed", (int)0);
      Serial.println("DISARMED");
    }
  }
}

// ================================================================
void setup() {
  Serial.begin(500000);   // mismo baud que el original
  while (!Serial) {}

  pinMode(LED_SHOT_PIN, OUTPUT);
  digitalWrite(LED_SHOT_PIN, LOW);

  // --- RPC: iniciar comunicación con M4 ---
  RPC.begin();
  RPC.bind("shotDetected", onShotDetected);

  // --- BNO055 ---
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 no encontrado");
    while (1) {}
  }
  delay(100);

  // Cargar calibración guardada (si existe)
  if (cargarOffsetsBNO()) {
    Serial.println("BNO055: calibracion EEPROM cargada");
  }

  delay(100);
  bno.setExtCrystalUse(true);

  nextImuMs = millis();

  Serial.println("Ready");
  Serial.println("DISARMED");
}

// ================================================================
void loop() {
  uint32_t nowMs = millis();

  // --- 1) Procesar disparo detectado por M4 ---
  if (shotPending) {
    shotPending = false;
    lastShotMs  = nowMs;
    startShotBlink(nowMs);
    Serial.println("SHOT,MECH");             // idéntico al original
  }

  // --- 2) Scheduler IMU 100 Hz ---
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    imu::Quaternion q = bno.getQuat();
    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    // Convertir a roll/pitch para arming (misma fórmula que el original)
    const double R2D = 57.29577951;
    double sinr  = 2.0*(qw*qx + qy*qz);
    double cosr  = 1.0 - 2.0*(qx*qx + qy*qy);
    double rollDeg  = atan2(sinr, cosr) * R2D;

    double sinp  = 2.0*(qw*qy - qz*qx);
    double pitchDeg = (fabs(sinp) >= 1.0) ? copysign(90.0, sinp) : asin(sinp) * R2D;

    updateArmingFromTilt((float)rollDeg, (float)pitchDeg);

    if (armed) {
      // Formato idéntico al original: qw,qx,qy,qz
      Serial.print(qw, 6); Serial.print(",");
      Serial.print(qx, 6); Serial.print(",");
      Serial.print(qy, 6); Serial.print(",");
      Serial.println(qz, 6);
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
