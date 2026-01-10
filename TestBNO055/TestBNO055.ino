#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("No se detectó el sensor BNO055");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  imu::Quaternion quat = bno.getQuat();

  // Enviar cuaterniones por puerto serie
  Serial.print(quat.w(), 4);
  Serial.print(",");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.println(quat.z(), 4);

  delay(50); // 50 Hz
}
