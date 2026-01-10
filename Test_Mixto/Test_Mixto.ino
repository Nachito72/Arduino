#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("No se detectó el BNO055");
    while (1);
  }
  bno.setExtCrystalUse(true);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  if (isnan(euler.x()) || isnan(euler.y()) || isnan(euler.z())) {
    Serial.println("Error de lectura");
    delay(500);
    return;
  }

  Serial.print(euler.x());  // Pitch
  Serial.print(",");
  Serial.print(euler.y());  // Roll
  Serial.print(",");
  Serial.println(euler.z()); // Heading

  delay(20);
}
