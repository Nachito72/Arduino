// MonitorMic – Arduino
// Envía cada muestra: micros,adc,cand,range,env,base
// Al detectar disparo envía también: SHOT,micros

// ── Constantes ────────────────────────────────────────────────────
const uint8_t  MIC_PIN        = A1;
const uint16_t TRIG_MAG       = 60;    // distancia mínima al centro para abrir candidato (señal cruda directa)
const uint16_t RANGE_MIN      = 150;   // rango pico-a-pico mínimo para confirmar disparo
const uint16_t CONFIRM_SAMPLES= 600;   // máx muestras sin confirmar (~6-12ms sin delay)
const uint16_t COOLDOWN_MS    = 2000;  // ms mínimos entre disparos
const uint8_t  PRETRIG        = 8;     // muestras pre-trigger al abrir candidato

// ── EMA helpers ───────────────────────────────────────────────────
static inline uint16_t ema_div(uint16_t cur, uint16_t tgt, uint8_t n) {
  int32_t d = (int32_t)tgt - (int32_t)cur;
  return (uint16_t)((int32_t)cur + (d >> n));
}
static inline uint16_t u16abs(int16_t v) { return v < 0 ? (uint16_t)(-v) : (uint16_t)v; }

// ── Estado global ─────────────────────────────────────────────────
uint16_t env_v     = 0;
uint16_t base_v    = 0;
int16_t  micCenter = 512;

bool     candidate  = false;
uint16_t candCount  = 0;
uint16_t minRawSeen = 1023;
uint16_t maxRawSeen = 0;
uint32_t lastShotMs = 0;

// Buffer circular pre-trigger
uint16_t preBuf[PRETRIG];
uint8_t  preIdx = 0;

// ── Calibración ───────────────────────────────────────────────────
void calibrate() {
  int32_t sum = 0;
  for (uint16_t i = 0; i < 500; i++) { sum += analogRead(MIC_PIN); delay(1); }
  micCenter = (int16_t)(sum / 500);
  for (uint8_t i = 0; i < PRETRIG; i++) preBuf[i] = (uint16_t)micCenter;
  // Warm-up EMA
  for (uint8_t i = 0; i < 60; i++) {
    uint16_t raw = analogRead(MIC_PIN);
    uint16_t mag = u16abs((int16_t)raw - micCenter);
    env_v  = ema_div(env_v,  mag,   2);
    base_v = ema_div(base_v, env_v, 8);
    delay(2);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  calibrate();
  // Línea de configuración para Processing
  Serial.print("CFG,MARGIN_ON=");    Serial.print(TRIG_MAG);
  Serial.print(",RANGE_MIN=");       Serial.print(RANGE_MIN);
  Serial.print(",CONFIRM_SAMPLES="); Serial.print(CONFIRM_SAMPLES);
  Serial.print(",COOLDOWN_MS=");     Serial.print(COOLDOWN_MS);
  Serial.print(",micCenter=");       Serial.println(micCenter);
}

void loop() {
  uint32_t t   = micros();
  uint32_t now = millis();
  uint16_t raw = analogRead(MIC_PIN);
  uint16_t mag = u16abs((int16_t)raw - micCenter);

  // EMA solo para visualización en Processing
  env_v  = ema_div(env_v,  mag,   2);
  base_v = ema_div(base_v, env_v, 8);

  if (!candidate) {
    // Guardar pre-trigger
    preBuf[preIdx] = raw;
    preIdx = (preIdx + 1) % PRETRIG;

    // TRIGGER DIRECTO: la señal cruda se aleja del centro más de TRIG_MAG
    if (mag >= TRIG_MAG && (now - lastShotMs) >= COOLDOWN_MS) {
      candidate  = true;
      candCount  = 0;
      // Inicializar min/max con las muestras pre-trigger
      minRawSeen = raw;
      maxRawSeen = raw;
      for (uint8_t i = 0; i < PRETRIG; i++) {
        uint16_t s = preBuf[i];
        if (s < minRawSeen) minRawSeen = s;
        if (s > maxRawSeen) maxRawSeen = s;
      }
    }
  } else {
    candCount++;
    if (raw < minRawSeen) minRawSeen = raw;
    if (raw > maxRawSeen) maxRawSeen = raw;
    uint16_t range = maxRawSeen - minRawSeen;

    if (range >= RANGE_MIN) {
      // ── DISPARO CONFIRMADO ──
      candidate   = false;
      lastShotMs  = now;
      Serial.print("SHOT,"); Serial.println(t);
      digitalWrite(LED_BUILTIN, HIGH);
      // LED off sin delay — usando timer
    }
    if (candCount >= CONFIRM_SAMPLES) {
      candidate = false;  // timeout sin confirmación
    }
  }

  // Apagar LED sin bloquear (80ms después de encenderlo)
  static uint32_t ledOnMs = 0;
  if (digitalRead(LED_BUILTIN) == HIGH) {
    if (ledOnMs == 0) ledOnMs = now;
    if (now - ledOnMs >= 80) { digitalWrite(LED_BUILTIN, LOW); ledOnMs = 0; }
  }

  uint16_t range_out = candidate ? (maxRawSeen - minRawSeen) : 0;

  // Formato: micros,adc,cand,range,env,base
  Serial.print(t);                    Serial.print(',');
  Serial.print(raw);                  Serial.print(',');
  Serial.print(candidate ? 1 : 0);   Serial.print(',');
  Serial.print(range_out);            Serial.print(',');
  Serial.print(env_v);                Serial.print(',');
  Serial.println(base_v);
}
