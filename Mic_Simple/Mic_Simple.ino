// =====================================================================
//  Mic_Simple  –  Visualización directa del micrófono
//
//  Abre el Serial Plotter (115200) para ver la señal en tiempo real.
//  También funciona con el Serial Monitor.
//
//  Comandos:
//    'p' → modo PLOTTER  (un valor por línea → Serial Plotter)
//    'm' → modo MONITOR  (raw + env + base en columnas legibles)
// =====================================================================

const uint8_t MIC_PIN   = A0;
const int     BAUDRATE  = 115200;
const int     DELAY_US  = 500;   // ~2 kHz, suficiente para ver la señal

char modo = 'p';   // empieza en modo plotter

uint16_t env  = 0;
uint16_t base = 0;

// EMA simple: shift=2 → alpha≈0.25 (rápida), shift=8 → alpha≈0.004 (lenta)
static uint16_t ema(uint16_t cur, uint16_t tgt, uint8_t shift) {
  int32_t d = (int32_t)tgt - (int32_t)cur;
  return (uint16_t)((int32_t)cur + (d >> shift));
}

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) {}
  Serial.println(F("# Mic_Simple listo.  p=Plotter  m=Monitor"));
}

void loop() {

  // Comando
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'p') { modo = 'p'; Serial.println(F("# Modo PLOTTER")); }
    if (c == 'm') { modo = 'm'; Serial.println(F("# Modo MONITOR  [raw] [env] [base]")); }
  }

  uint16_t raw = analogRead(MIC_PIN);
  int16_t  mag = abs((int16_t)raw - 512);   // magnitud respecto al centro ideal

  env  = ema(env,  (uint16_t)mag, 2);   // envelope rápido
  base = ema(base, env,           8);   // baseline lenta

  if (modo == 'p') {
    // Serial Plotter: una etiqueta:valor por campo, misma línea
    Serial.print("raw:");  Serial.print(raw);
    Serial.print(",env:"); Serial.print(env);
    Serial.print(",base:"); Serial.println(base);

  } else {
    // Monitor legible
    Serial.print(F("raw="));  Serial.print(raw);
    Serial.print(F("\tenv="));  Serial.print(env);
    Serial.print(F("\tbase=")); Serial.println(base);
  }

  delayMicroseconds(DELAY_US);
}
