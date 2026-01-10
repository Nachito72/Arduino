#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float prevLinAccelX = 0, prevLinAccelY = 0, prevLinAccelZ = 0;
float prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;
float prevGyroX = 0, prevGyroY = 0, prevGyroZ = 0;
float prevEulerX = 0, prevEulerY = 0, prevEulerZ = 0;

unsigned long ultimoDisparo = 0;
const unsigned long tiempoRearme = 500; // ms

// Umbrales de disparo
const float UMBRAL_DELTA_LINACCEL_Y = 0.25;
const float VALOR_MINIMO_LINACCEL_Y = 0.7;
const float UMBRAL_DELTA_GYRO = 2.0;
const float UMBRAL_ROTACION_TOTAL = 0.5;

// Umbrales de inactividad del arma
const float UMBRAL_INMOVIL_LINACCEL_X = 0.70;
const float UMBRAL_INMOVIL_LINACCEL_Y = 0.70;
const float UMBRAL_INMOVIL_LINACCEL_Z = 0.70;
const float UMBRAL_INMOVIL_GYRO_X = 4.0;
const float UMBRAL_INMOVIL_GYRO_Y = 4.0;
const float UMBRAL_INMOVIL_GYRO_Z = 4.0;

// Estado anterior
bool anterior = False;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bno.begin()) {
    Serial.println("No se detectó BNO055. Verifica conexiones.");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  // Inicializar valores previos
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  prevLinAccelX = linAccel.x(); prevLinAccelY = linAccel.y(); prevLinAccelZ = linAccel.z();
  prevAccelX = accel.x(); prevAccelY = accel.y(); prevAccelZ = accel.z();
  prevGyroX = gyro.x(); prevGyroY = gyro.y(); prevGyroZ = gyro.z();
  prevEulerX = euler.x(); prevEulerY = euler.y(); prevEulerZ = euler.z();

  Serial.println("Detectando disparo (umbrales configurables aplicados)");
}

bool armaInmovil(float dLinX, float dLinY, float dLinZ, float dGyroX, float dGyroY, float dGyroZ) {
  return dLinX < UMBRAL_INMOVIL_LINACCEL_X &&
         dLinY < UMBRAL_INMOVIL_LINACCEL_Y &&
         dLinZ < UMBRAL_INMOVIL_LINACCEL_Z &&
         dGyroX < UMBRAL_INMOVIL_GYRO_X &&
         dGyroY < UMBRAL_INMOVIL_GYRO_Y &&
         dGyroZ < UMBRAL_INMOVIL_GYRO_Z;
}

void loop() {
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float deltaLinAccelX = abs(linAccel.x() - prevLinAccelX);
  float deltaLinAccelY = abs(linAccel.y() - prevLinAccelY);
  float deltaLinAccelZ = abs(linAccel.z() - prevLinAccelZ);

  float deltaAccelX = abs(accel.x() - prevAccelX);
  float deltaAccelY = abs(accel.y() - prevAccelY);
  float deltaAccelZ = abs(accel.z() - prevAccelZ);

  float deltaGyroX = abs(gyro.x() - prevGyroX);
  float deltaGyroY = abs(gyro.y() - prevGyroY);
  float deltaGyroZ = abs(gyro.z() - prevGyroZ);

  float deltaEulerX = abs(euler.x() - prevEulerX);
  float deltaEulerY = abs(euler.y() - prevEulerY);
  float deltaEulerZ = abs(euler.z() - prevEulerZ);

  float rotacionTotal = deltaEulerX + deltaEulerY + deltaEulerZ;

  prevLinAccelX = linAccel.x(); prevLinAccelY = linAccel.y(); prevLinAccelZ = linAccel.z();
  prevAccelX = accel.x(); prevAccelY = accel.y(); prevAccelZ = accel.z();
  prevGyroX = gyro.x(); prevGyroY = gyro.y(); prevGyroZ = gyro.z();
  prevEulerX = euler.x(); prevEulerY = euler.y(); prevEulerZ = euler.z();

  bool inmovil = armaInmovil(deltaLinAccelX, deltaLinAccelY, deltaLinAccelZ, deltaGyroX, deltaGyroY, deltaGyroZ);

  if (inmovil){
    Serial.println("\n***** Arma inmóvil *****");    
    Serial.print("LinAccel (X,Y,Z): ");
    Serial.print(linAccel.x(), 3); Serial.print(", ");
    Serial.print(linAccel.y(), 3); Serial.print(", ");
    Serial.print(linAccel.z(), 3);
  } else {
    Serial.println("\n***** MOVIMIENTO *****");    
    if (deltaLinAccelX > UMBRAL_INMOVIL_LINACCEL_X || anterior){
      Serial.print("Aceleracion lineal X:");
      Serial.print(deltaLinAccelX, 3); Serial.print(", ");
    } else {
      Serial.print("Aceleracion lineal X:");
      Serial.print("X.XXX"); Serial.print(", ");
    }
    if (deltaLinAccelY > UMBRAL_INMOVIL_LINACCEL_Y){
      Serial.print("Aceleracion lineal Y:");
      Serial.print(deltaLinAccelY, 3); Serial.print(", ");    
    } else {
      Serial.print("Aceleracion lineal Y:");
      Serial.print("X.XXX"); Serial.print(", ");
    }
    if (deltaLinAccelZ > UMBRAL_INMOVIL_LINACCEL_Z){
      Serial.print("Aceleracion lineal Z:");
      Serial.print(deltaLinAccelZ, 3); Serial.print(", ");    
    } else {
      Serial.print("Aceleracion lineal Z:");
      Serial.print("X.XXX"); Serial.print(", ");
    }
    if (deltaGyroX > UMBRAL_INMOVIL_GYRO_X){
      Serial.print("Aceleracion giro X:");
      Serial.print(deltaGyroX, 3); Serial.print(", ");    
    } else {
      Serial.print("Aceleracion giro X:");
      Serial.print("X.XXX"); Serial.print(", ");
    }
    if (deltaGyroY > UMBRAL_INMOVIL_GYRO_Y){
      Serial.print("Aceleracion giro Y:");
      Serial.print(deltaGyroY, 3); Serial.print(", ");    
    } else {
      Serial.print("Aceleracion giro Y:");
      Serial.print("X.XXX"); Serial.print(", ");
    }
    if (deltaGyroZ > UMBRAL_INMOVIL_GYRO_Z){
      Serial.print("Aceleracion girol Z:");
      Serial.print(deltaGyroZ, 3); Serial.print(", ");    
    } else {
      Serial.print("Aceleracion giro Z:");
      Serial.print("X.XXX"); Serial.print(", ");
    }
  }

  if (inmovil && deltaGyroY > 1.9 && deltaGyroX < 2.0 && deltaGyroY < 1.9)
//      deltaLinAccelY > UMBRAL_DELTA_LINACCEL_Y &&
//      linAccel.y() > VALOR_MINIMO_LINACCEL_Y &&
//      deltaGyroX > UMBRAL_DELTA_GYRO &&
//      deltaGyroY > UMBRAL_DELTA_GYRO &&
//      rotacionTotal < UMBRAL_ROTACION_TOTAL &&
//      (millis() - ultimoDisparo > tiempoRearme)) 
    {

    ultimoDisparo = millis();
    Serial.println("\n***** DISPARO DETECTADO ##############################################################################################");
    Serial.print("Cambio LinAccel (X,Y,Z): ");
    Serial.print(deltaLinAccelX, 3); Serial.print(", ");
    Serial.print(deltaLinAccelY, 3); Serial.print(", ");
    Serial.println(deltaLinAccelZ, 3);

    Serial.print("Cambio Acelerómetro (X,Y,Z): ");
    Serial.print(deltaAccelX, 3); Serial.print(", ");
    Serial.print(deltaAccelY, 3); Serial.print(", ");
    Serial.println(deltaAccelZ, 3);

    Serial.print("Cambio Giroscopio (X,Y,Z): ");
    Serial.print(deltaGyroX, 3); Serial.print(", ");
    Serial.print(deltaGyroY, 3); Serial.print(", ");
    Serial.println(deltaGyroZ, 3);

    Serial.print("Cambio Euler (roll,pitch,yaw): ");
    Serial.print(deltaEulerX, 3); Serial.print(", ");
    Serial.print(deltaEulerY, 3); Serial.print(", ");
    Serial.println(deltaEulerZ, 3);

  } else {
    Serial.println();
  }

  delay(30);
}
