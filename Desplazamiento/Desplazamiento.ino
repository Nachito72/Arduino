#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

float posX = 0, posY = 0;
float velX = 0, velY = 0;

unsigned long lastTime = 0;

// Filtro pasabajos
float accelZFiltered = 0;
float accelYFiltered = 0;
float alpha = 0.7;

// Bias (offset) para eliminar aceleración residual
float biasZ = 0;
float biasY = 0;
int calibrationSamples = 500;

// Umbral mínimo para eliminar ruido
float thresholdZ = 0.15;  // más alto para el eje vertical
float thresholdY = 0.1;

// Amortiguación artificial
float damping = 0.95;

void calibrateSensor() {
  float sumZ = 0;
  float sumY = 0;

  Serial.println("Calibrando sensor... Mantén la placa quieta");

  for (int i = 0; i < calibrationSamples; i++) {
    imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    sumZ += lin_accel.y();
    sumY += lin_accel.z();
    delay(5);
  }

  biasZ = sumZ / calibrationSamples;
  biasY = sumY / calibrationSamples;

  Serial.print("Bias Z (lateral): "); Serial.println(biasZ);
  Serial.print("Bias Y (vertical): "); Serial.println(biasY);
}

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("No se detectó el BNO055");
    while (1);
  }
  bno.setExtCrystalUse(true);
  lastTime = millis();

  calibrateSensor();
}

void loop() {
  imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // tiempo en segundos
  lastTime = currentTime;

  // Aplicar bias
  float accelZ = lin_accel.y() - biasZ; // Lateral
  float accelY = lin_accel.z() - biasY; // Vertical

  // Filtro pasabajos
  accelZFiltered = alpha * accelZFiltered + (1 - alpha) * accelZ;
  accelYFiltered = alpha * accelYFiltered + (1 - alpha) * accelY;

  // Aplicar umbral para eliminar ruido
  float accX = (abs(accelZFiltered) > thresholdZ) ? accelZFiltered : 0;
  float accY = (abs(accelYFiltered) > thresholdY) ? accelYFiltered : 0;

  // Calcular velocidad (integrar aceleración)
  velX += accX * deltaTime * 10;  // Ajusta este multiplicador para más o menos sensibilidad
  velY += accY * deltaTime * 10;

  // Amortiguación cuando no hay aceleración
  if (accX == 0) velX *= damping;
  if (accY == 0) velY *= damping;

  // Si la velocidad es muy baja, detenerla
  if (abs(velX) < 0.01) velX = 0;
  if (abs(velY) < 0.01) velY = 0;

  // Calcular desplazamiento (integrar velocidad)
  posX += velX * deltaTime * 100;
  posY += velY * deltaTime * 100;

  // Auto-centrado lento
  if (accX == 0 && accY == 0 && velX == 0 && velY == 0) {
    posX *= 0.99; // Poco a poco vuelve al centro
    posY *= 0.99;
  }

  // Enviar posiciones a Processing
  Serial.print(posX);
  Serial.print(",");
  Serial.println(posY);

  delay(20); // 50 Hz
}
