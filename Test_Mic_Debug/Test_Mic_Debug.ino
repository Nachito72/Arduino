// =====================================================================
//  Test_Mic_Debug  –  Señal pura del micrófono
//
//  115200 baudios. Sin IMU, sin lógica de armado.
//
//  Comandos por Monitor Serie:
//    'r'  → modo RAW continuo  (raw,env cada muestra ~4kHz)
//    's'  → modo STATS         (solo imprime al detectar pico)
//    'c'  → recalibrar centro
//    '+'  → MARGIN_ON  +10
//    '-'  → MARGIN_ON  -10
//    'h'  → ayuda + parámetros actuales
// =====================================================================

const uint8_t  MIC_PIN         = A0;
const uint16_t MIC_DELAY_US    = 250;    // ~4 kHz
const uint16_t CONFIRM_SAMPLES = 80;     // ventana máxima ~20 ms
const uint16_t COOLDOWN_MS     = 300;    // tiempo mínimo entre disparos

const uint16_t SAT_LOW  = 5;
const uint16_t SAT_HIGH = 1018;

// ──── Parámetros ajustables en runtime ────────────────────────────
uint16_t MARGIN_ON  = 90;   // cuánto por encima de base para abrir candidato
uint16_t MARGIN_OFF = 45;   // umbral inferior para medir anchura de pulso

// ──── EMA ─────────────────────────────────────────────────────────
uint16_t env  = 0;   // envelope rápido  (alpha ~0.25)
uint16_t base = 0;   // baseline lenta   (alpha ~0.004)
int16_t  micCenter = 512;

// ──── Candidato ───────────────────────────────────────────────────
bool     candidate  = false;
uint16_t candCount  = 0;
uint16_t minRaw     = 1023;
uint16_t maxRaw     = 0;
uint16_t peakAbs    = 0;
uint16_t widthSamp  = 0;
bool     aboveOff   = false;
uint32_t lastShotMs = 0;

// ──── Contadores ──────────────────────────────────────────────────
uint32_t totalShots = 0;
uint32_t totalSat   = 0;   // disparos que saturaron el ADC
uint32_t totalFalse = 0;   // candidatos que no llegaron a disparo

char modo = 's';

// =====================================================================
static inline uint16_t ema_div(uint16_t current, uint16_t target, uint8_t divPow2) {
  int32_t diff = (int32_t)target - (int32_t)current;
  return (uint16_t)((int32_t)current + (diff >> divPow2));
}
static inline uint16_t u16abs(int16_t v) { return v < 0 ? (uint16_t)(-v) : (uint16_t)v; }

static inline uint16_t ema(uint16_t cur, uint16_t tgt, uint8_t shift) {
  int32_t d = (int32_t)tgt - (int32_t)cur;
  return (uint16_t)((int32_t)cur + (d >> shift));
}
static inline uint16_t absu(int16_t v) { return v < 0 ? (uint16_t)(-v) : (uint16_t)v; }

// ──────────────────────────────────────────────────────────────────
void calibrar() {
  int32_t sum = 0;
  for (uint16_t i = 0; i < 500; i++) { sum += analogRead(MIC_PIN); delay(1); }
  micCenter = (int16_t)(sum / 500);
  env = 0; base = 0;
  Serial.print(F(">> Centro: ")); Serial.print(micCenter);
  Serial.print(F("  (ideal ~512)"));
  if (micCenter < 400 || micCenter > 624)
    Serial.print(F("  *** FUERA DE RANGO - revisa conexion/alimentacion ***"));
  Serial.println();
}

void printParams() {
  Serial.print(F("  MIC_PIN=A")); Serial.print(MIC_PIN - A0);
  Serial.print(F("  centro="));   Serial.print(micCenter);
  Serial.print(F("  MARGIN_ON=")); Serial.print(MARGIN_ON);
  Serial.print(F("  MARGIN_OFF=")); Serial.println(MARGIN_OFF);
}

void printHelp() {
  Serial.println(F("=============================================="));
  Serial.println(F("  Test_Mic_Debug  -  Solo microfono"));
  Serial.println(F("  r  -> RAW: imprime raw,env por muestra"));
  Serial.println(F("  s  -> STATS: solo eventos (picos/shots)"));
  Serial.println(F("  c  -> recalibrar centro"));
  Serial.println(F("  +  -> MARGIN_ON +10"));
  Serial.println(F("  -  -> MARGIN_ON -10"));
  Serial.println(F("  h  -> esta ayuda"));
  Serial.println(F("=============================================="));
  printParams();
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  calibrar();
  printHelp();
  Serial.println(F(">> Listo. Dispara o aplica palmas cerca del micro."));
}

// =====================================================================
void loop() {

  // ── Comando serie ────────────────────────────────────────────────
  if (Serial.available()) {
    char cmd = (char)Serial.read();
    if      (cmd == 'r') { modo = 'r'; Serial.println(F(">> Modo RAW (raw,env)")); }
    else if (cmd == 's') { modo = 's'; Serial.println(F(">> Modo STATS")); }
    else if (cmd == 'c') { calibrar(); }
    else if (cmd == '+') { MARGIN_ON += 10; Serial.print(F(">> MARGIN_ON=")); Serial.println(MARGIN_ON); }
    else if (cmd == '-') { MARGIN_ON = (MARGIN_ON > 10) ? MARGIN_ON - 10 : 10;
                           Serial.print(F(">> MARGIN_ON=")); Serial.println(MARGIN_ON); }
    else if (cmd == 'h') { printHelp(); }
  }

  // ── Lectura ADC ──────────────────────────────────────────────────
  uint32_t nowMs = millis();
  uint16_t raw   = analogRead(MIC_PIN);
  int16_t  cent  = (int16_t)raw - micCenter;
  uint16_t mag   = absu(cent);

  env  = ema(env,  mag, 2);   // envelope rapido  alpha ~0.25
  base = ema(base, env, 8);   // baseline lenta   alpha ~0.004

  uint16_t thrOn  = base + MARGIN_ON;
  uint16_t thrOff = base + MARGIN_OFF;

  // ── Modo RAW ─────────────────────────────────────────────────────
  if (modo == 'r') {
    Serial.print(raw); Serial.print(','); Serial.println(env);
    delayMicroseconds(MIC_DELAY_US);
    return;
  }

  // ── Ancho de pulso ───────────────────────────────────────────────
  if (candidate) {
    if (env > thrOff) { widthSamp++; aboveOff = true; }
    else if (aboveOff) aboveOff = false;
  }

  bool enCooldown = (nowMs - lastShotMs < COOLDOWN_MS);

  // ── Abrir candidato ──────────────────────────────────────────────
  if (!enCooldown && !candidate && env > thrOn) {
    candidate = true;
    candCount = 0;
    minRaw = raw; maxRaw = raw; peakAbs = mag;
    widthSamp = 0; aboveOff = false;

    Serial.print(F("[PICO]  t=")); Serial.print(nowMs);
    Serial.print(F("ms  env="));  Serial.print(env);
    Serial.print(F("  thrOn=")); Serial.print(thrOn);
    Serial.print(F("  base="));  Serial.println(base);
  }

  // ── Evaluar candidato ────────────────────────────────────────────
  if (candidate) {
    candCount++;
    if (raw < minRaw) minRaw = raw;
    if (raw > maxRaw) maxRaw = raw;
    if (mag > peakAbs) peakAbs = mag;

    uint16_t range    = maxRaw - minRaw;
    uint32_t width_us = (uint32_t)widthSamp * MIC_DELAY_US;
    bool     sat      = (minRaw <= SAT_LOW) || (maxRaw >= SAT_HIGH);

    // Criterio de disparo: saturacion ADC o pico/rango suficiente
    if (sat || peakAbs >= 200 || range >= 200) {
      candidate  = false;
      lastShotMs = nowMs;
      totalShots++;
      if (sat) totalSat++;

      digitalWrite(LED_BUILTIN, HIGH); delay(60); digitalWrite(LED_BUILTIN, LOW);

      Serial.println(F("--------------------------------------------"));
      Serial.print(sat ? F("[SHOT-SAT]  ") : F("[SHOT]      "));
      Serial.print(F("t="));        Serial.print(nowMs);    Serial.print(F("ms"));
      Serial.print(F("  muestras=")); Serial.print(candCount);
      Serial.print(F("  width_us=")); Serial.println(width_us);
      Serial.print(F("  raw_min="));  Serial.print(minRaw);
      Serial.print(F("  raw_max="));  Serial.print(maxRaw);
      Serial.print(F("  range="));    Serial.print(range);
      Serial.print(F("  peakAbs="));  Serial.println(peakAbs);
      Serial.print(F("  base="));     Serial.print(base);
      Serial.print(F("  thrOn="));    Serial.print(thrOn);
      Serial.print(F("  sat="));      Serial.println(sat ? F("SI") : F("NO"));
      Serial.print(F("  Totales -> shots:")); Serial.print(totalShots);
      Serial.print(F("  sat:"));    Serial.print(totalSat);
      Serial.print(F("  falsos:")); Serial.println(totalFalse);
      Serial.println(F("--------------------------------------------"));

    } else if (candCount >= CONFIRM_SAMPLES) {
      // Candidato descartado: sonido demasiado débil
      totalFalse++;
      candidate = false;
      Serial.print(F("[FALSO]  peakAbs=")); Serial.print(peakAbs);
      Serial.print(F("  range="));          Serial.print(maxRaw - minRaw);
      Serial.print(F("  width_us="));       Serial.print(width_us);
      Serial.print(F("  sat="));            Serial.println(sat ? F("SI") : F("NO"));
    }
  }

  delayMicroseconds(MIC_DELAY_US);
}
