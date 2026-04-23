// ================================================================
//  DiagEjesArma.ino
//  Diagnóstico de ejes del BNO055 montado en el arma.
//
//  INSTRUCCIONES:
//    1. Monta el sensor en el trípode exactamente como va en el arma.
//    2. Sube este sketch al Arduino Giga (M7).
//    3. Abre Serial Monitor a 115200 baud.
//    4. Pulsa 'R' para capturar la posición actual como referencia (cero).
//    5. Mueve UN eje cada vez y observa qué columna cambia:
//
//         BAJAR  el cañón verticalmente     → ¿cuál columna cambia?
//         GIRAR  horizontalmente (izda/dcha)→ ¿cuál columna cambia?
//         ROTAR  sobre el eje del cañón     → ¿cuál columna cambia?
//
//  SALIDA (cada 100 ms):
//    heading  roll  pitch  | dH   dR   dP   | gyroX  gyroY  gyroZ
//
//    heading = euler.x()  (0-360°, azimut magnético)
//    roll    = euler.y()  (giro alrededor del eje X del chip)
//    pitch   = euler.z()  (giro alrededor del eje Y del chip)
//    dH dR dP = diferencia relativa al punto de referencia capturado con 'R'
//    gyroX/Y/Z = velocidad angular en °/s — muestra qué eje GIRA al mover
// ================================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

float refH = 0, refR = 0, refP = 0;
bool  refSet = false;

const uint32_t PRINT_MS = 100;
uint32_t nextPrint = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== DiagEjesArma ===");
  Serial.println("Iniciando BNO055...");

  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("ERROR: BNO055 no encontrado. Revisa conexiones.");
    while (1) delay(10);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("OK. Esperando...");
  Serial.println("Pulsa 'R' para capturar referencia (posición cero).");
  Serial.println();
  Serial.println("  heading  eu.y()  eu.z() |    dH    dEuY   dEuZ  |  gyroX  gyroY  gyroZ");
  Serial.println("  (euler.x) (alabeo)(elev) | relativo ref         |  (deg/s, eje activo)");
  Serial.println("  [yaw]    [barrel] [ELEV] |                      |  [Y=elevacion correcto]");
  Serial.println("-----------------------------------------------------------------------");
}

void loop() {
  // Leer comando serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'R' || c == 'r') {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      refH = euler.x();
      refR = euler.y();
      refP = euler.z();
      refSet = true;
      Serial.println();
      Serial.print(">>> REFERENCIA capturada: H=");
      Serial.print(refH, 2);
      Serial.print("  R=");
      Serial.print(refR, 2);
      Serial.print("  P=");
      Serial.println(refP, 2);
      Serial.println();
    }
  }

  uint32_t now = millis();
  if (now < nextPrint) return;
  nextPrint = now + PRINT_MS;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float H = euler.x();  // heading
  float R = euler.y();  // roll
  float P = euler.z();  // pitch

  float dH = 0, dR = 0, dP = 0;
  if (refSet) {
    dH = H - refH;
    // wrap heading a ±180
    while (dH >  180) dH -= 360;
    while (dH < -180) dH += 360;
    dR = R - refR;
    dP = P - refP;
  }

  // Formatear salida: columnas fijas para fácil lectura
  char buf[120];
  snprintf(buf, sizeof(buf),
    "%8.2f %8.2f %7.2f  | %7.2f %7.2f %7.2f  | %7.2f %7.2f %7.2f",
    H, R, P,
    dH, dR, dP,
    gyro.x(), gyro.y(), gyro.z()
  );
  Serial.println(buf);
}
