#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

float posX = 0;
float posY = 0;
float posZ = 0;

unsigned long lastTime = 0;
float dt;

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  // segundos
  lastTime = currentTime;

  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Integrar aceleración para obtener velocidad (aproximado)
  static float velX = 0;
  static float velY = 0;

  velX += linAccel.x() * dt;
  velY += linAccel.y() * dt;

  // Integrar velocidad para obtener posición
  posX += velX * dt;
  posY += velY * dt;

  // Limitar desplazamientos para evitar saltos locos
  if (posX > 200) posX = 200;
  if (posX < -200) posX = -200;
  if (posY > 200) posY = 200;
  if (posY < -200) posY = -200;

  Serial.print(quat.w(), 4);
  Serial.print(",");
  Serial.print(quat.x(), 4);
  Serial.print(",");
  Serial.print(quat.y(), 4);
  Serial.print(",");
  Serial.print(quat.z(), 4);
  Serial.print(",");
  Serial.print(posX, 2);
  Serial.print(",");
  Serial.println(posY, 2);

  delay(50);
}
