#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

// --- Ajustes iniciales (los afinaremos con tus datos) ---
float LEVEL_ON_DEG  = 12.0;
float LEVEL_OFF_DEG = 18.0;

const uint8_t LEVEL_ON_COUNT  = 20; // 20*20ms = 400ms
const uint8_t LEVEL_OFF_COUNT = 10; // 10*20ms = 200ms

bool armed = false;
uint8_t levelOnStreak = 0;
uint8_t levelOffStreak = 0;

static inline float f_abs(float v) { return (v < 0) ? -v : v; }

const uint32_t IMU_PERIOD_MS = 20;
uint32_t nextImuMs = 0;

void updateArming(float roll, float pitch) {
  bool levelOn  = (f_abs(roll) <= LEVEL_ON_DEG)  && (f_abs(pitch) <= LEVEL_ON_DEG);
  bool levelOff = (f_abs(roll) >= LEVEL_OFF_DEG) || (f_abs(pitch) >= LEVEL_OFF_DEG);

  if (!armed) {
    if (levelOn) levelOnStreak++; else levelOnStreak = 0;
    if (levelOnStreak >= LEVEL_ON_COUNT) {
      armed = true;
      levelOnStreak = 0;
      levelOffStreak = 0;
    }
  } else {
    if (levelOff) levelOffStreak++; else levelOffStreak = 0;
    if (levelOffStreak >= LEVEL_OFF_COUNT) {
      armed = false;
      levelOffStreak = 0;
      levelOnStreak = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) while (1);
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("DBG mode: watch roll=euler.y, pitch=euler.z (deg)");
  nextImuMs = millis();
}

void loop() {
  uint32_t nowMs = millis();
  if ((int32_t)(nowMs - nextImuMs) >= 0) {
    nextImuMs += IMU_PERIOD_MS;

    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float roll  = e.y();
    float pitch = e.z();

    updateArming(roll, pitch);

    Serial.print("DBG,armed=");
    Serial.print(armed ? 1 : 0);

    Serial.print(",roll=");
    Serial.print(roll, 2);
    Serial.print(",pitch=");
    Serial.print(pitch, 2);

    Serial.print(",absR=");
    Serial.print(f_abs(roll), 2);
    Serial.print(",absP=");
    Serial.print(f_abs(pitch), 2);

    Serial.print(",onStreak=");
    Serial.print(levelOnStreak);
    Serial.print(",offStreak=");
    Serial.println(levelOffStreak);
  }
}
