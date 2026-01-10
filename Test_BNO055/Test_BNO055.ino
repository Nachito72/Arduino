#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Espera a que se abra el puerto serie
  }
  Serial.println("Inicializando BNO055...");

  if (!bno.begin()) {
    Serial.println("No se encontró el sensor BNO055. Verifica conexiones.");
    while (1);
  }
  delay(1000);

  bno.setExtCrystalUse(true);
  Serial.println("BNO055 listo.");
}

void loop() {
  // Leer aceleración lineal (aceleración sin gravedad)
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Leer aceleración total (incluye gravedad)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Leer orientación en Euler (yaw, pitch, roll)
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Leer velocidad angular (gyroscopio)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("Accel (g): X=");
  Serial.print(accel.x(), 4);
  Serial.print(" Y=");
  Serial.print(accel.y(), 4);
  Serial.print(" Z=");
  Serial.print(accel.z(), 4);

  Serial.print(" | LinAccel (g): X=");
  Serial.print(linAccel.x(), 4);
  Serial.print(" Y=");
  Serial.print(linAccel.y(), 4);
  Serial.print(" Z=");
  Serial.print(linAccel.z(), 4);

  Serial.print(" | Euler (deg): Yaw=");
  Serial.print(euler.x(), 2);
  Serial.print(" Pitch=");
  Serial.print(euler.y(), 2);
  Serial.print(" Roll=");
  Serial.print(euler.z(), 2);

  Serial.print(" | Gyro (deg/s): X=");
  Serial.print(gyro.x(), 2);
  Serial.print(" Y=");
  Serial.print(gyro.y(), 2);
  Serial.print(" Z=");
  Serial.print(gyro.z(), 2);

  Serial.println();

  delay(20);  // 50 Hz aprox, puedes cambiar para más o menos frecuencia
}
