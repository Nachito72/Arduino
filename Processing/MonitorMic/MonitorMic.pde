// ╔══════════════════════════════════════════════════════════════════╗
// ║  MonitorMic.pde  –  Visualizador + Detector de disparo en vivo  ║
// ║                                                                  ║
// ║  Recibe de Arduino (115200 baud):                                ║
// ║    CFG,MARGIN_ON=N,...   (una vez al arrancar)                   ║
// ║    micros,adc,cand,range,env,base  (cada muestra)               ║
// ║    SHOT,micros            (cada disparo detectado)               ║
// ║                                                                  ║
// ║  Teclado:                                                        ║
// ║    ESPACIO  → pausa / continuar                                  ║
// ║    1 / Q    → TRIG_MAG   +5 / -5                                 ║
// ║    2 / W    → RANGE_MIN  +5 / -5                                 ║
// ║    R        → resetear contador de disparos                      ║
// ╚══════════════════════════════════════════════════════════════════╝
import processing.serial.*;

// ── Configuración de puerto ───────────────────────────────────────
final String PUERTO = "/dev/cu.usbmodem3101";
final int    BAUDRATE = 115200;
final int    HISTORIA = 800;   // muestras visibles

// ── Constantes de detección (se actualizan con CFG del Arduino) ──
int CFG_TRIG_MAG        = 60;   // distancia directa al centro para abrir candidato
int CFG_RANGE_MIN       = 150;
int CFG_CONFIRM_SAMPLES = 80;
int CFG_COOLDOWN_MS     = 5000;
int CFG_micCenter       = 444;

// ── Buffers de datos ──────────────────────────────────────────────
float[]   buf      = new float[HISTORIA];   // ADC crudo
float[]   envBuf   = new float[HISTORIA];   // env (EMA rápida)
float[]   baseBuf  = new float[HISTORIA];   // base (EMA lenta)
boolean[] candBuf  = new boolean[HISTORIA]; // candidato activo?
int[]     rangeBuf = new int[HISTORIA];     // rango pico-a-pico

float[]   bufF      = new float[HISTORIA];   // frozen (pausa)
float[]   envBufF   = new float[HISTORIA];
float[]   baseBufF  = new float[HISTORIA];
boolean[] candBufF  = new boolean[HISTORIA];
int[]     rangeBufF = new int[HISTORIA];

long[]  tBuf  = new long[HISTORIA];
long[]  tBufF = new long[HISTORIA];

int     head   = 0;
boolean paused = false;

// ── Estado en tiempo real (último valor recibido) ────────────────
float   lastADC   = 0;
float   lastEnv   = 0;
float   lastBase  = 0;
boolean lastCand  = false;
int     lastRange = 0;

// ── Flash de disparo ─────────────────────────────────────────────
int  shotFlash    = 0;   // contador de frames con flash
long lastShotTime = 0;   // millis del último SHOT
int  shotCount    = 0;

// ── Serie ────────────────────────────────────────────────────────
Serial puerto;

// ─────────────────────────────────────────────────────────────────
void setup() {
  size(1500, 650);
  background(15);
  textFont(createFont("Consolas", 12, true));

  try {
    puerto = new Serial(this, PUERTO, BAUDRATE);
    puerto.bufferUntil('\n');
  } catch (Exception e) {
    println("Puerto no disponible. Lista: " + join(Serial.list(), ", "));
  }
}

// ── Parseo de línea serial ────────────────────────────────────────
void serialEvent(Serial p) {
  String s = trim(p.readString());
  if (s == null || s.length() == 0) return;

  // ── Línea CFG: leer constantes del Arduino ──────────────────────
  if (s.startsWith("CFG,")) {
    String[] parts = s.substring(4).split(",");
    for (String part : parts) {
      String[] kv = part.split("=");
      if (kv.length == 2) {
        String k = kv[0].trim();
        int    v = int(kv[1].trim());
        if      (k.equals("MARGIN_ON"))       CFG_TRIG_MAG        = v;  // Arduino envía MARGIN_ON pero es TRIG_MAG
        else if (k.equals("RANGE_MIN"))        CFG_RANGE_MIN       = v;
        else if (k.equals("CONFIRM_SAMPLES"))  CFG_CONFIRM_SAMPLES = v;
        else if (k.equals("COOLDOWN_MS"))      CFG_COOLDOWN_MS     = v;
        else if (k.equals("micCenter"))        CFG_micCenter       = v;
      }
    }
    return;
  }

  // ── Línea SHOT ──────────────────────────────────────────────────
  if (s.startsWith("SHOT,")) {
    shotFlash    = 60;  // ~2s a 30fps
    lastShotTime = millis();
    shotCount++;
    return;
  }

  // ── Línea de datos: micros,adc,cand,range,env,base ─────────────
  try {
    String[] parts = s.split(",");
    if (parts.length >= 6) {
      long    t     = Long.parseLong(parts[0].trim());
      float   adc   = float(parts[1].trim());
      boolean cand  = parts[2].trim().equals("1");
      int     range = int(parts[3].trim());
      float   env   = float(parts[4].trim());
      float   base  = float(parts[5].trim());

      lastADC   = adc;
      lastEnv   = env;
      lastBase  = base;
      lastCand  = cand;
      lastRange = range;

      if (!paused) {
        buf[head]     = adc;
        envBuf[head]  = env;
        baseBuf[head] = base;
        candBuf[head] = cand;
        rangeBuf[head]= range;
        tBuf[head]    = t;
        head = (head + 1) % HISTORIA;
      }
    }
  } catch (Exception e) {}
}

// ─────────────────────────────────────────────────────────────────
void draw() {
  background(15);

  // Layout: panel gráfico arriba, panel de info abajo
  int PANEL_H = 130;  // altura panel inferior
  int ax = 55, ay = 20;
  int aw = width - 65, ah = height - PANEL_H - ay - 10;

  // ── Flash de disparo (fondo rojo) ───────────────────────────────
  if (shotFlash > 0) {
    float alpha = map(shotFlash, 60, 0, 120, 0);
    noStroke();
    fill(200, 40, 40, alpha);
    rect(0, 0, width, height);
    shotFlash--;
  }

  // ── Marco del gráfico ──────────────────────────────────────────
  noFill(); stroke(60); strokeWeight(1);
  rect(ax, ay, aw, ah);

  // Seleccionar fuente de datos (en vivo o congelada)
  float[]   src     = paused ? bufF     : buf;
  float[]   envSrc  = paused ? envBufF  : envBuf;
  boolean[] candSrc = paused ? candBufF : candBuf;
  long[]    tSrc    = paused ? tBufF    : tBuf;

  // ── Ventana temporal ───────────────────────────────────────────
  int  idxOld   = head % HISTORIA;
  int  idxNew   = (head - 1 + HISTORIA) % HISTORIA;
  long tFirst   = tSrc[idxOld];
  long tLast    = tSrc[idxNew];
  long windowUs = (tLast > tFirst) ? (tLast - tFirst) : 0;
  float windowMs = windowUs / 1000.0;

  // ── Grid horizontal (niveles ADC) ─────────────────────────────
  stroke(40); strokeWeight(1);
  textSize(10); fill(90); textAlign(RIGHT, CENTER);
  for (int v = 0; v <= 1023; v += 100) {
    float yg = ay + map(v, 1023, 0, 0, ah);
    line(ax, yg, ax + aw, yg);
    text(v, ax - 4, yg);
  }

  // ── Grid vertical (tiempo) ─────────────────────────────────────
  if (windowUs > 0) {
    stroke(40); strokeWeight(1);
    textSize(9); fill(90); textAlign(CENTER, TOP);
    int nGridX = 10;
    for (int g = 0; g <= nGridX; g++) {
      float fx = ax + aw * g / (float)nGridX;
      line(fx, ay, fx, ay + ah);
      float ms = windowMs * g / nGridX;
      text(nf(ms, 0, 1) + "ms", fx, ay + ah + 2);
    }
  }

  // ── thrOn line (base + MARGIN_ON + micCenter en escala ADC) ───
  // env y base son magnitudes (distancia al centro), así que
  // dibujamos las líneas simétricas respecto a micCenter
  // ── Sombras de candidato activo ─────────────────────────────────
  if (windowUs > 0) {
    boolean inBlock = false;
    float blockStartX = ax;
    for (int i = 0; i < HISTORIA; i++) {
      int  idx = (head + i) % HISTORIA;
      long dt  = tSrc[idx] - tFirst;
      if (dt < 0) dt = 0;
      float x = ax + map(dt, 0, windowUs, 0, aw);
      x = constrain(x, ax, ax + aw);
      if (candSrc[idx] && !inBlock) {
        blockStartX = x;
        inBlock = true;
      } else if (!candSrc[idx] && inBlock) {
        noStroke();
        fill(255, 140, 0, 35);
        rect(blockStartX, ay, x - blockStartX, ah);
        inBlock = false;
      }
    }
    if (inBlock) {
      noStroke();
      fill(255, 140, 0, 35);
      rect(blockStartX, ay, (ax + aw) - blockStartX, ah);
    }
  }

  // ── Línea env EMA (verde oscuro) ± alrededor de micCenter ──────
  float lastEnvVal  = envSrc [(head - 1 + HISTORIA) % HISTORIA];
  float yEnvHi = ay + map(CFG_micCenter + lastEnvVal, 1023, 0, 0, ah);
  float yEnvLo = ay + map(CFG_micCenter - lastEnvVal, 1023, 0, 0, ah);
  stroke(40, 160, 40, 180); strokeWeight(1.5);
  line(ax, yEnvHi, ax + aw, yEnvHi);
  line(ax, yEnvLo, ax + aw, yEnvLo);

  // ── Línea TRIG_MAG (naranja): umbral directo sobre señal cruda ─
  // El trigger se abre cuando |raw - micCenter| >= TRIG_MAG
  float yTrigHi = ay + map(CFG_micCenter + CFG_TRIG_MAG, 1023, 0, 0, ah);
  float yTrigLo = ay + map(CFG_micCenter - CFG_TRIG_MAG, 1023, 0, 0, ah);
  stroke(255, 160, 0, 220); strokeWeight(2);
  line(ax, yTrigHi, ax + aw, yTrigHi);
  line(ax, yTrigLo, ax + aw, yTrigLo);

  // ── Línea RANGE_MIN centrada en micCenter (rojo tenue) ────────
  float yRangeHi = ay + map(CFG_micCenter + CFG_RANGE_MIN / 2.0, 1023, 0, 0, ah);
  float yRangeLo = ay + map(CFG_micCenter - CFG_RANGE_MIN / 2.0, 1023, 0, 0, ah);
  stroke(220, 60, 60, 120); strokeWeight(1);
  line(ax, yRangeHi, ax + aw, yRangeHi);
  line(ax, yRangeLo, ax + aw, yRangeLo);

  // ── Curva env (amarillo) ───────────────────────────────────────
  if (windowUs > 0) {
    stroke(255, 220, 50, 160); strokeWeight(1); noFill();
    beginShape();
    for (int i = 0; i < HISTORIA; i++) {
      int  idx = (head + i) % HISTORIA;
      long dt  = tSrc[idx] - tFirst;
      if (dt < 0) dt = 0;
      float x   = constrain(ax + map(dt, 0, windowUs, 0, aw), ax, ax + aw);
      float ev  = envSrc[idx];
      // env positiva sobre el centro
      float y   = ay + map(CFG_micCenter + ev, 1023, 0, 0, ah);
      vertex(x, y);
    }
    endShape();
    // env negativa (simétrico bajo el centro)
    beginShape();
    for (int i = 0; i < HISTORIA; i++) {
      int  idx = (head + i) % HISTORIA;
      long dt  = tSrc[idx] - tFirst;
      if (dt < 0) dt = 0;
      float x  = constrain(ax + map(dt, 0, windowUs, 0, aw), ax, ax + aw);
      float ev = envSrc[idx];
      float y  = ay + map(CFG_micCenter - ev, 1023, 0, 0, ah);
      vertex(x, y);
    }
    endShape();
  }

  // ── Curva ADC cruda (azul claro) ──────────────────────────────
  stroke(80, 200, 255); strokeWeight(1.2); noFill();
  if (windowUs > 0) {
    beginShape();
    for (int i = 0; i < HISTORIA; i++) {
      int  idx = (head + i) % HISTORIA;
      long dt  = tSrc[idx] - tFirst;
      if (dt < 0) dt = 0;
      float x = constrain(ax + map(dt, 0, windowUs, 0, aw), ax, ax + aw);
      float y = ay + map(src[idx], 1023, 0, 0, ah);
      vertex(x, y);
    }
    endShape();
  } else {
    beginShape();
    for (int i = 0; i < HISTORIA; i++) {
      int   idx = (head + i) % HISTORIA;
      float x   = ax + map(i, 0, HISTORIA - 1, 0, aw);
      float y   = ay + map(src[idx], 1023, 0, 0, ah);
      vertex(x, y);
    }
    endShape();
  }

  // ── "DISPARO!" si flash activo ────────────────────────────────
  if (shotFlash > 0) {
    float alpha2 = map(shotFlash, 60, 0, 255, 80);
    textSize(48); textAlign(CENTER, CENTER);
    fill(255, 60, 60, alpha2);
    text("¡DISPARO! (" + shotCount + ")", width / 2.0, height / 2.0 - PANEL_H / 2.0);
  }

  // ── Leyendas de líneas (esquina superior izq del gráfico) ─────
  int lx = ax + 8, ly = ay + 8;
  textSize(10); textAlign(LEFT, TOP); noStroke();
  fill(80, 200, 255);   text("— Señal ADC",               lx, ly);
  fill(255, 220, 50);   text("— Env (EMA)",                lx, ly + 14);
  fill(40, 160, 40);    text("— Env EMA ± (visual)",       lx, ly + 28);
  fill(255, 160, 0);    text("— TRIG_MAG (umbral apertura)",lx, ly + 42);
  fill(220, 60, 60);    text("— RANGE_MIN / 2",            lx, ly + 56);
  fill(255, 140, 0, 140); text("▌Candidato activo",        lx, ly + 70);

  // ── Info temporal sobre el gráfico ───────────────────────────
  long t_last  = tSrc[(head - 1 + HISTORIA) % HISTORIA];
  long t_prev  = tSrc[(head - 2 + HISTORIA) % HISTORIA];
  long dtUs    = (t_last > t_prev) ? (t_last - t_prev) : 0;
  textSize(11); textAlign(RIGHT, TOP);
  fill(160, 220, 160);
  text("Ventana: " + nf(windowMs, 0, 1) + " ms   dt: " + dtUs + " µs   ~" +
       (dtUs > 0 ? (int)(1000000.0 / dtUs) : 0) + " Hz",
       ax + aw - 4, ay + 4);

  // ── Botón PAUSA ───────────────────────────────────────────────
  int bw = 120, bh = 26, bx2 = width - bw - 8, by2 = 4;
  noStroke();
  fill(paused ? color(200, 70, 70) : color(55, 55, 55));
  rect(bx2, by2, bw, bh, 5);
  fill(255); textSize(12); textAlign(CENTER, CENTER);
  text(paused ? "PAUSADO  ▶" : "PAUSA  ⏸", bx2 + bw / 2.0, by2 + bh / 2.0);

  // ╔══════════════════════════════════════════════════════════════╗
  // ║  PANEL INFERIOR — constantes + estado en tiempo real        ║
  // ╚══════════════════════════════════════════════════════════════╝
  int py = height - PANEL_H + 6;
  int col1 = 10, col2 = 380, col3 = 750, col4 = 1050;

  // Fondo panel
  noStroke();
  fill(22);
  rect(0, height - PANEL_H, width, PANEL_H);
  stroke(50); strokeWeight(1);
  line(0, height - PANEL_H, width, height - PANEL_H);

  textAlign(LEFT, TOP);

  // ── Columna 1: Constantes ─────────────────────────────────────
  fill(200); textSize(12);
  text("CONSTANTES DE DETECCIÓN", col1, py);
  py += 18;

  textSize(11);
  fill(255, 200, 60);
  text("TRIG_MAG   = " + CFG_TRIG_MAG +
       "   [1] +5  [Q] -5", col1, py);
  py += 16;

  fill(220, 80, 80);
  text("RANGE_MIN  = " + CFG_RANGE_MIN +
       "   [2] +5  [W] -5", col1, py);
  py += 16;

  fill(160, 220, 160);
  text("CONFIRM_SAMPLES = " + CFG_CONFIRM_SAMPLES, col1, py);
  py += 16;

  fill(160, 160, 220);
  text("COOLDOWN_MS     = " + CFG_COOLDOWN_MS, col1, py);
  py += 16;

  fill(200, 200, 200);
  text("micCenter       = " + CFG_micCenter, col1, py);

  // ── Columna 2: Estado en tiempo real ─────────────────────────
  int py2 = height - PANEL_H + 6;
  fill(200); textSize(12);
  text("ESTADO EN TIEMPO REAL", col2, py2); py2 += 18;
  textSize(11);

  fill(80, 200, 255);
  text("ADC crudo   = " + (int)lastADC, col2, py2); py2 += 16;

  fill(255, 220, 50);
  text("env         = " + (int)lastEnv, col2, py2); py2 += 16;

  fill(40, 200, 40);
  text("base        = " + (int)lastBase, col2, py2); py2 += 16;

  fill(255, 160, 0);
  float magNow = abs(lastADC - CFG_micCenter);
  text("mag(|ADC-ctr|)= " + (int)magNow +
       "  trig>" + CFG_TRIG_MAG, col2, py2); py2 += 16;

  // Barra de progreso mag vs TRIG_MAG
  float fracMag = (CFG_TRIG_MAG > 0) ? magNow / CFG_TRIG_MAG : 0;
  fracMag = constrain(fracMag, 0, 1.5);
  int barW = 200, barH = 12;
  stroke(60); strokeWeight(1);
  noFill(); rect(col2, py2, barW, barH);
  noStroke();
  fill(fracMag >= 1.0 ? color(255, 80, 80) : color(60, 200, 60));
  rect(col2, py2, min(fracMag * barW, barW), barH);
  fill(200); textSize(9); textAlign(LEFT, CENTER);
  text("mag/TRIG_MAG = " + nf(fracMag * 100, 0, 0) + "%", col2 + barW + 6, py2 + barH / 2.0);
  textAlign(LEFT, TOP); textSize(11);

  // ── Columna 3: Candidato y disparo ────────────────────────────
  int py3 = height - PANEL_H + 6;
  fill(200); textSize(12);
  text("CANDIDATO / DISPARO", col3, py3); py3 += 18;
  textSize(11);

  fill(lastCand ? color(255, 140, 0) : color(100));
  text("Candidato   = " + (lastCand ? "ACTIVO ●" : "no"), col3, py3); py3 += 16;

  fill(lastCand ? color(255, 100, 100) : color(100));
  text("Rango actual= " + lastRange +
       "  / " + CFG_RANGE_MIN, col3, py3); py3 += 16;

  // Barra de progreso rango vs RANGE_MIN
  float fracRng = (CFG_RANGE_MIN > 0) ? (float)lastRange / CFG_RANGE_MIN : 0;
  fracRng = constrain(fracRng, 0, 1.5);
  noFill(); stroke(60); strokeWeight(1);
  rect(col3, py3, barW, barH);
  noStroke();
  fill(fracRng >= 1.0 ? color(255, 50, 50) : color(200, 130, 30));
  rect(col3, py3, min(fracRng * barW, barW), barH);
  fill(200); textSize(9); textAlign(LEFT, CENTER);
  text("range/RANGE_MIN = " + nf(fracRng * 100, 0, 0) + "%", col3 + barW + 6, py3 + barH / 2.0);
  textAlign(LEFT, TOP); textSize(11);
  py3 += barH + 6;

  fill(shotFlash > 0 ? color(255, 80, 80) : color(100));
  text("Disparos    = " + shotCount +
       (lastShotTime > 0 ? "  (último: " + nf((millis() - lastShotTime) / 1000.0, 0, 1) + "s)" : ""),
       col3, py3);

  // ── Columna 4: Ayuda teclas ───────────────────────────────────
  int py4 = height - PANEL_H + 6;
  fill(140); textSize(11);
  text("TECLAS", col4, py4); py4 += 18;
  fill(120);
  text("[ESPACIO] Pausa/continuar", col4, py4); py4 += 16;
  text("[1] MARGIN_ON +5", col4, py4); py4 += 16;
  text("[Q] MARGIN_ON -5", col4, py4); py4 += 16;
  text("[2] RANGE_MIN +5", col4, py4); py4 += 16;
  text("[W] RANGE_MIN -5", col4, py4); py4 += 16;
  text("[R] Reset disparos", col4, py4);
}

// ─────────────────────────────────────────────────────────────────
void mousePressed() {
  int bw = 120, bh = 26, bx2 = width - bw - 8, by2 = 4;
  if (mouseX >= bx2 && mouseX <= bx2 + bw &&
      mouseY >= by2 && mouseY <= by2 + bh) {
    togglePause();
  }
}

void keyPressed() {
  if (key == ' ')  { togglePause(); return; }
  if (key == '1')  { CFG_TRIG_MAG   += 5; return; }
  if (key == 'q' || key == 'Q') { CFG_TRIG_MAG   = max(5, CFG_TRIG_MAG  - 5); return; }
  if (key == '2')  { CFG_RANGE_MIN  += 5; return; }
  if (key == 'w' || key == 'W') { CFG_RANGE_MIN  = max(5, CFG_RANGE_MIN  - 5); return; }
  if (key == 'r' || key == 'R') { shotCount = 0; shotFlash = 0; lastShotTime = 0; }
}

void togglePause() {
  paused = !paused;
  if (paused) {
    arrayCopy(buf,      bufF);
    arrayCopy(envBuf,   envBufF);
    arrayCopy(baseBuf,  baseBufF);
    arrayCopy(candBuf,  candBufF);
    arrayCopy(rangeBuf, rangeBufF);
    arrayCopy(tBuf,     tBufF);
  }
}

