#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float pitch = euler.x();  // Rotación en X
  float yaw = euler.y()*(-1);    // Rotación en Z (dirección horizontal)

  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.println(yaw, 2);

  delay(20);  // 50 Hz aprox.
}
