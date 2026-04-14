// ================================================================
//  Giga_MovSoloPlano.ino  —  CORE M7 (principal)
//  Arduino Giga R1 WiFi
// Nacho
//  Equivalente a MovSoloPlanoDisparoMaxi con dual-core:
//    M7 → IMU BNO055, arming, Serial a Processing, LED
//    M4 → Mic ADC + detección de disparo (ver Giga_M4_Mic.ino)
//
//  Los dos cores son TOTALMENTE INDEPENDIENTES:
//    - M7 nunca llama al M4 durante la operación normal.
//    - M4 notifica al M7 vía RPC solo cuando detecta un disparo.
//    - M4 corre el detector siempre, sin esperar estado de arme.
//
//  SALIDA SERIAL:
//    "Ready"
//    "ARMED" / "DISARMED"
//    ms,qw,qx,qy,qz,filteredElev,filteredYaw,shot   (cuando armado, 100 Hz)
//    filteredYaw = heading 0-360° con filtro complementario gyro+Euler
//    shot = 0 sin disparo, o volumen ADC del disparo
//
//  INSTRUCCIONES:
//    1. Sube primero Giga_M4_Mic.ino al M4 (Tools → Board → Giga M4)
//    2. Después sube este sketch al M7 (Tools → Board → Giga M7)
//    3. Abre Serial Monitor a 500000 baud
// ================================================================

#include <Wire.h>
#include <RPC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ===================== VERSIÓN =====================
#define VERSION_M7 "1.5"

// ===================== LED =====================
const uint8_t LED_SHOT_PIN = LED_BUILTIN;

const uint16_t LED_BLINK_PERIOD_MS = 200;
const uint16_t LED_BLINK_TOTAL_MS  = 3000;

bool     ledBlinkActive  = false;
uint32_t ledBlinkEndMs   = 0;
uint32_t ledNextToggleMs = 0;
bool     ledState        = false;

// ===================== IMU =====================
Adafruit_BNO055 bno = Adafruit_BNO055();  // 0x28 si ADR=GND, 0x29 si ADR=VCC

// Giga R1 (Mbed OS) no tiene EEPROM — calibración automática en cada arranque

// ===================== DISPAROS =====================
// El M4 detecta el disparo y llama a shotDetected(vol) vía RPC.
// M7 recibe la llamada y guarda el volumen (rango ADC) del disparo.
volatile bool shotPending  = false;
volatile int  shotVolume   = 0;

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

// ===================== FILTRO COMPLEMENTARIO ELEVACIÓN =====================
// Combina giroscopio (suave) con roll absoluto (estable a largo plazo).
// ROLL_OFFSET: ángulo raw que corresponde a pistola plana (0° real).
// FILTER_ALPHA: peso del giroscopio. 0.95 → τ ≈ 200 ms a 100 Hz.
const float ROLL_OFFSET  = -85.0f;
const float FILTER_ALPHA = 0.95f;
float filteredElev = 0.0f;
bool  filtElevInit = false;

// ===================== FILTRO COMPLEMENTARIO YAW =====================
// Igual que filteredElev pero para el heading (0-360°).
// eulerVec.x() = heading BNO055 (resolución 1/16° = 0.0625°)
// gyroVec.z()  = velocidad angular en yaw (°/s, alta resolución)
float filteredYaw = 0.0f;
bool  filtYawInit = false;

// ===================== SHOT FLAG =====================
// En lugar de enviar "SHOT,MECH", se añade 0/1 como último campo CSV.
bool shotThisFrame = false;

// ===================== BNO055 WATCHDOG =====================
// Si el sensor devuelve cuaternión nulo (fallo I2C transitorio) durante
// BNO_MAX_ERR frames consecutivos se intenta reinicializar.
// Tras cada reinit se respeta un periodo de calentamiento (bnoWarmupEndMs)
// durante el cual los ceros del arranque de la fusión NO cuentan como error.
uint8_t  bnoErrCount    = 0;
const uint8_t BNO_MAX_ERR = 50;
uint32_t bnoWarmupEndMs = 0;  // no contar errores hasta que millis() >= este valor

// ================================================================
// Reinicio del BNO055 tras fallo I2C
// ================================================================
bool reinitBNO() {
  Serial.println("// BNO055: fallo I2C — reintentando init...");
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(100000);
  delay(150);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("// BNO055: reinit FALLIDO");
    return false;
  }
  bno.setExtCrystalUse(true);
  filtElevInit    = false;
  filtYawInit     = false;
  bnoWarmupEndMs  = millis() + 1000; // esperar 1s antes de contar errores de fusión
  Serial.println("// BNO055: reinit OK");
  return true;
}

// ================================================================
// RPC: función que el M4 llama cuando detecta un disparo
// Recibe el volumen (rango ADC) del disparo
// ================================================================
int onShotDetected(int vol) {
  shotVolume  = vol;
  shotPending = true;
  return 1;
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
      Serial.println("ARMED");
    }
  } else {
    if (shouldDisarm) { if (levelOffStreak < 255) levelOffStreak++; }
    else                levelOffStreak = 0;

    if (levelOffStreak >= LEVEL_OFF_COUNT) {
      armed          = false;
      levelOffStreak = 0;
      levelOnStreak  = 0;
      Serial.println("DISARMED");
    }
  }
}

// ================================================================
void setup() {
  Serial.begin(500000);
  // Esperar Serial máximo 2 s — no bloquear si no hay monitor abierto
  { uint32_t t0 = millis(); while (!Serial && millis() - t0 < 2000) {} }

  pinMode(LED_SHOT_PIN, OUTPUT);
  digitalWrite(LED_SHOT_PIN, LOW);

  // --- RPC: iniciar comunicación con M4 ---
  RPC.begin();
  RPC.bind("shotDetected", onShotDetected);

  // --- BNO055 (IMUPLUS: fusión accel+gyro, sin magnetómetro) ---
  Wire.begin();
  delay(200);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("ERROR: BNO055 no encontrado");
    while (1) {}
  }
  // 100kHz: más estable bajo Mbed OS/STM32H7 que 400kHz
  Wire.setClock(100000);
  delay(1000);  // esperar a que el algoritmo de fusión converja
  bno.setExtCrystalUse(true);
  bnoWarmupEndMs = 0;  // setup ya esperó: loop() empieza con sensor listo

  nextImuMs = millis();

  // Versiones de ambos programas (prefijo "//" → Processing las ignora)
  Serial.println("// Giga_MovSoloPlano v" VERSION_M7);
  // Consultar versión del M4 vía RPC con timeout de 500 ms
{
  bool ok = true;

  // RPC.call() devuelve object_handle (no future)
  auto oh = RPC.call("getVersionM4");

  // Acceso correcto al valor: object_handle::get() -> msgpack::object -> as<int>()
  int m4v = oh.get().as<int>();

  Serial.print("// Giga_M4_Mic v");
  Serial.print(m4v / 100);
  Serial.print(".");
  Serial.println(m4v % 100);
}

Serial.println("Ready");
Serial.println("DISARMED");
}

// ================================================================
void loop() {
  uint32_t nowMs = millis();

  // --- 1) Procesar disparo detectado por M4 ---
  if (shotPending) {
    shotPending     = false;
    shotThisFrame   = true;   // se enviará como campo de volumen en la próxima trama IMU
    startShotBlink(nowMs);
  }

  // --- 2) Scheduler IMU 100 Hz ---
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    imu::Quaternion q = bno.getQuat();
    double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    // ── Validar cuaternión: norma² debe ser ≈ 1.0 ───────────────────
    // El BNO055 devuelve (0,0,0,0) cuando hay fallo I2C transitorio.
    double quatNorm2 = qw*qw + qx*qx + qy*qy + qz*qz;
    bool quatValido  = (quatNorm2 > 0.5);

    if (!quatValido) {
      // Ignorar este frame: no actualizar arming ni filtro.
      // Solo contar como error si el periodo de calentamiento ya pasó.
      if (nowMs >= bnoWarmupEndMs) {
        if (++bnoErrCount >= BNO_MAX_ERR) {
          bnoErrCount = 0;
          reinitBNO();
        }
      }
    } else {
      bnoErrCount = 0;

      // Convertir a roll/pitch para arming (misma fórmula que el original)
      const double R2D = 57.29577951;
      double sinr  = 2.0*(qw*qx + qy*qz);
      double cosr  = 1.0 - 2.0*(qx*qx + qy*qy);
      double rollDeg  = atan2(sinr, cosr) * R2D;

      double sinp  = 2.0*(qw*qy - qz*qx);
      double pitchDeg = (fabs(sinp) >= 1.0) ? copysign(90.0, sinp) : asin(sinp) * R2D;

      updateArmingFromTilt((float)rollDeg, (float)pitchDeg);

      // Filtro complementario:
      //   Parte GYRO    → gyro.x() integrado: movimiento continuo y suave (0.004° res.)
      //   Parte ABSOLUTA→ Euler BNO055 (.y() = roll, 1/16°=0.0625°): ancla al grado real
      //
      // Usar el Euler como referencia absoluta en lugar del roll del cuaternión hace que
      // el filtro converja suavemente hacia los grados enteros reales sin ningún salto.
      // No hay floorf() ni discontinuidades: el gyro suaviza los "escalones" del Euler.
      imu::Vector<3> gyroVec  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> eulerVec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      float eulerRollAbs = (float)(eulerVec.y() - ROLL_OFFSET);
      float dtSeg        = IMU_PERIOD_MS / 1000.0f;
      if (!filtElevInit) { filteredElev = eulerRollAbs; filtElevInit = true; }
      else filteredElev = FILTER_ALPHA * (filteredElev + (float)gyroVec.x() * dtSeg)
                        + (1.0f - FILTER_ALPHA) * eulerRollAbs;

      // Filtro complementario YAW: combina gyro.z() (suave) con euler.x() (ancla absoluta)
      // Se maneja el wrap-around 0/360° por el método de error de camino más corto.
      float eulerYawAbs = (float)eulerVec.x();
      if (!filtYawInit) { filteredYaw = eulerYawAbs; filtYawInit = true; }
      else {
        float yawPred = filteredYaw + (float)gyroVec.z() * dtSeg;
        while (yawPred >= 360.0f) yawPred -= 360.0f;
        while (yawPred <    0.0f) yawPred += 360.0f;
        float yawErr  = eulerYawAbs - yawPred;
        while (yawErr >  180.0f) yawErr -= 360.0f;
        while (yawErr < -180.0f) yawErr += 360.0f;
        filteredYaw = yawPred + (1.0f - FILTER_ALPHA) * yawErr;
        while (filteredYaw >= 360.0f) filteredYaw -= 360.0f;
        while (filteredYaw <    0.0f) filteredYaw += 360.0f;
      }

      if (armed) {
        // Formato: ms,qw,qx,qy,qz,filteredElev,shot
        // shot = 0 sin disparo, o el volumen (rango ADC) del disparo
        int shotVal = shotThisFrame ? shotVolume : 0;
        shotThisFrame = false;
        if (shotVal > 0) shotVolume = 0;  // reset para el siguiente disparo
        Serial.print(nowMs);        Serial.print(",");
        Serial.print(qw, 6);        Serial.print(",");
        Serial.print(qx, 6);        Serial.print(",");
        Serial.print(qy, 6);        Serial.print(",");
        Serial.print(qz, 6);        Serial.print(",");
        Serial.print(filteredElev, 4); Serial.print(",");
        Serial.print(filteredYaw,  4); Serial.print(",");
        Serial.println(shotVal);
      }
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
