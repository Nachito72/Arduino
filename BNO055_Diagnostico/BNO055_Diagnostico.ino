// ================================================================
//  BNO055_Diagnostico.ino
//  Diagnóstico y calibración persistente del BNO055.
//
//  COMANDOS por Serial Monitor (115200 baud, "Sin salto de línea"):
//    G  → Guarda los offsets de calibración actuales en EEPROM
//    C  → Carga los offsets desde EEPROM al sensor (se hace automáticamente al arrancar)
//    I  → Imprime los offsets actuales del sensor
//    ?  → Muestra esta ayuda
//
//  FLUJO DE CALIBRACIÓN:
//    1. Sube este sketch al Arduino
//    2. Abre Serial Monitor a 115200 baud
//    3. Realiza los movimientos de calibración hasta ver SYS=3 ACC=3
//    4. Envía 'G' para guardar en EEPROM
//    5. A partir de ahora, cada arranque carga los offsets automáticamente
//
//  NO modifica ni interfiere con MovSoloPlanoDisparoMaxi.
// ================================================================

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

// Dirección EEPROM donde se guardan los offsets (22 bytes)
// y un byte de firma para saber si hay datos válidos
const int  EEPROM_ADDR   = 0;
const byte EEPROM_FIRMA  = 0xB7;  // valor arbitrario para validar

// Intervalo de impresión (ms)
const uint16_t PRINT_INTERVAL_MS = 200;

uint32_t lastPrintMs    = 0;
bool     offsetsCargados = false;

// ----------------------------------------------------------------
// Guarda offsets en EEPROM
// ----------------------------------------------------------------
void guardarOffsets() {
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);

  EEPROM.put(EEPROM_ADDR, EEPROM_FIRMA);
  EEPROM.put(EEPROM_ADDR + 1, offsets);

  Serial.println(">>> Offsets guardados en EEPROM.");
  Serial.print("    Accel: X="); Serial.print(offsets.accel_offset_x);
  Serial.print(" Y=");           Serial.print(offsets.accel_offset_y);
  Serial.print(" Z=");           Serial.println(offsets.accel_offset_z);
  Serial.print("    Gyro:  X="); Serial.print(offsets.gyro_offset_x);
  Serial.print(" Y=");           Serial.print(offsets.gyro_offset_y);
  Serial.print(" Z=");           Serial.println(offsets.gyro_offset_z);
  Serial.print("    Mag:   X="); Serial.print(offsets.mag_offset_x);
  Serial.print(" Y=");           Serial.print(offsets.mag_offset_y);
  Serial.print(" Z=");           Serial.println(offsets.mag_offset_z);
  Serial.print("    Radio Accel="); Serial.print(offsets.accel_radius);
  Serial.print("  Radio Mag=");     Serial.println(offsets.mag_radius);
}

// ----------------------------------------------------------------
// Carga offsets desde EEPROM al sensor
// ----------------------------------------------------------------
bool cargarOffsets() {
  byte firma;
  EEPROM.get(EEPROM_ADDR, firma);

  if (firma != EEPROM_FIRMA) {
    Serial.println(">>> EEPROM: no hay offsets guardados todavia.");
    return false;
  }

  adafruit_bno055_offsets_t offsets;
  EEPROM.get(EEPROM_ADDR + 1, offsets);

  // Hay que poner el sensor en modo CONFIG antes de escribir offsets
  bno.setSensorOffsets(offsets);

  Serial.println(">>> Offsets cargados desde EEPROM al sensor. OK.");
  return true;
}

// ----------------------------------------------------------------
// Imprime los offsets actuales del sensor
// ----------------------------------------------------------------
void imprimirOffsets() {
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);
  Serial.println("--- Offsets actuales del sensor ---");
  Serial.print("    Accel: X="); Serial.print(offsets.accel_offset_x);
  Serial.print(" Y=");           Serial.print(offsets.accel_offset_y);
  Serial.print(" Z=");           Serial.println(offsets.accel_offset_z);
  Serial.print("    Gyro:  X="); Serial.print(offsets.gyro_offset_x);
  Serial.print(" Y=");           Serial.print(offsets.gyro_offset_y);
  Serial.print(" Z=");           Serial.println(offsets.gyro_offset_z);
  Serial.print("    Mag:   X="); Serial.print(offsets.mag_offset_x);
  Serial.print(" Y=");           Serial.print(offsets.mag_offset_y);
  Serial.print(" Z=");           Serial.println(offsets.mag_offset_z);
}

// ----------------------------------------------------------------
void mostrarAyuda() {
  Serial.println("=== COMANDOS ===");
  Serial.println("  G → Guardar offsets actuales en EEPROM (hazlo cuando SYS=3)");
  Serial.println("  C → Cargar offsets desde EEPROM al sensor");
  Serial.println("  I → Imprimir offsets actuales del sensor");
  Serial.println("  ? → Esta ayuda");
  Serial.println("================");
}

// ================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println();
  Serial.println("=== BNO055 Diagnostico + Calibracion Persistente ===");
  Serial.println("Iniciando sensor...");

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 no encontrado. Revisa cableado/direccion I2C.");
    while (1) {}
  }

  delay(100);

  // Intentar cargar offsets guardados antes de activar cristal externo
  offsetsCargados = cargarOffsets();

  delay(100);
  bno.setExtCrystalUse(true);

  Serial.println("Sensor OK.");
  mostrarAyuda();
  Serial.println();
  Serial.println("INSTRUCCIONES CALIBRACION:");
  Serial.println("  GYR: deja el sensor quieto 3 seg");
  Serial.println("  ACC: coloca el sensor en 6 posiciones distintas (cara arriba, abajo, lados)");
  Serial.println("  MAG: mueve el sensor en figura-8 en el aire");
  Serial.println("  Cuando SYS=3 y ACC=3, envia 'G' para guardar.");
  Serial.println("------------------------------------------------------------");
}

// ================================================================
void loop() {
  // --- Leer comandos Serial ---
  if (Serial.available()) {
    char cmd = Serial.read();
    if      (cmd == 'G' || cmd == 'g') guardarOffsets();
    else if (cmd == 'C' || cmd == 'c') cargarOffsets();
    else if (cmd == 'I' || cmd == 'i') imprimirOffsets();
    else if (cmd == '?')               mostrarAyuda();
  }

  // --- Imprimir periódicamente ---
  uint32_t ahora = millis();
  if (ahora - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = ahora;

  // Calibración
  uint8_t sys_cal, gyr_cal, acc_cal, mag_cal;
  bno.getCalibration(&sys_cal, &gyr_cal, &acc_cal, &mag_cal);

  // Cuaterniones → roll/pitch
  imu::Quaternion q = bno.getQuat();
  double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
  float roll_q  = atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy)) * 180.0 / PI;
  float pitch_q = asin (2.0*(qw*qy - qz*qx))                              * 180.0 / PI;

  // Euler directo
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll_e  = euler.y();
  float pitch_e = euler.z();

  // Barra de progreso de calibración visual
  Serial.print("SYS="); Serial.print(sys_cal);
  Serial.print(" GYR="); Serial.print(gyr_cal);
  Serial.print(" ACC="); Serial.print(acc_cal);
  Serial.print(" MAG="); Serial.print(mag_cal);

  // Indicador visual rápido
  Serial.print("  [");
  Serial.print(sys_cal == 3 ? "SYS:OK" : "SYS:--");
  Serial.print("|");
  Serial.print(acc_cal == 3 ? "ACC:OK" : "ACC:--");
  Serial.print("]");

  Serial.print("  Roll_Q=");  Serial.print(roll_q,  1); Serial.print("deg");
  Serial.print("  Pitch_Q="); Serial.print(pitch_q, 1); Serial.print("deg");
  Serial.print("  Roll_E=");  Serial.print(roll_e,  1); Serial.print("deg");
  Serial.print("  Pitch_E="); Serial.print(pitch_e, 1); Serial.print("deg");

  if (offsetsCargados) Serial.print("  [EEPROM:cargada]");
  else                  Serial.print("  [EEPROM:vacia]");

  Serial.println();

  // Avisos importantes
  if (sys_cal == 3 && acc_cal == 3) {
    Serial.println("  *** CALIBRADO! Envia 'G' para guardar en EEPROM ***");
  }
  if (acc_cal < 2) {
    Serial.println("  >> ACC sin calibrar: rota el sensor en 6 posiciones estaticas");
  }
  if (gyr_cal < 3) {
    Serial.println("  >> GYR sin calibrar: deja el sensor completamente quieto");
  }
}
