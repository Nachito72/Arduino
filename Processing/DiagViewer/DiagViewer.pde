/*  DiagViewer.pde
 *  ─────────────────────────────────────────────────────────────────
 *  Visualizador en tiempo real para DiagBNO055Raw.ino
 *
 *  Muestra 4 gráficas en tiempo real:
 *   ① Roll  (°)   – rotación sobre X
 *   ② Pitch (°)   – rotación sobre Y
 *   ③ Yaw   (°)   – rotación sobre Z [0..360]
 *   ④ Mic         – raw ADC, envelope y base
 *
 *  Controles:
 *   [R]   – borrar histórico y reiniciar
 *   [P]   – pausar / reanudar
 *   [+/-] – ajustar velocidad de scroll (ancho de ventana)
 *   [1-4] – resaltar la gráfica correspondiente
 *
 *  Formato CSV esperado del puerto serie:
 *   tMs,qw,qx,qy,qz,roll,pitch,yaw,mic_raw,mic_env,mic_base
 *
 *  500000 baudios — ajustar SERIAL_PORT al puerto correcto.
 */

import processing.serial.*;

// ─── CONFIGURACIÓN ──────────────────────────────────────────────────
final String SERIAL_PORT = "COM7";   // ← cambiar si hace falta
final int    BAUD        = 500000;
final int    MAX_SAMPLES = 1000;     // puntos en el histórico por canal
// ────────────────────────────────────────────────────────────────────

Serial port;
boolean paused   = false;
boolean portOpen = false;

// Buffers de datos
float[] rollBuf  = new float[MAX_SAMPLES];
float[] pitchBuf = new float[MAX_SAMPLES];
float[] yawBuf   = new float[MAX_SAMPLES];
float[] micRaw   = new float[MAX_SAMPLES];
float[] micEnv   = new float[MAX_SAMPLES];
float[] micBase  = new float[MAX_SAMPLES];
long[]  tBuf     = new long[MAX_SAMPLES];

int headIdx = 0;   // índice circular: próxima posición de escritura
int count   = 0;   // cuántas muestras hay

// Resaltado de panel (0 = ninguno)
int highlight = 0;

String lastError  = "";
String lastLine   = "";
long   lastTMs    = 0;

// ─── SETUP ──────────────────────────────────────────────────────────
void setup() {
  size(1100, 720);
  textFont(createFont("Monospaced", 11));

  try {
    port = new Serial(this, SERIAL_PORT, BAUD);
    port.bufferUntil('\n');
    portOpen = true;
  } catch (Exception e) {
    lastError = "Puerto " + SERIAL_PORT + " no disponible.";
    println("Puertos disponibles: " + java.util.Arrays.toString(Serial.list()));
  }
}

// ─── SERIAL EVENT ───────────────────────────────────────────────────
void serialEvent(Serial s) {
  if (paused) return;
  String raw = s.readStringUntil('\n');
  if (raw == null) return;
  raw = trim(raw);
  if (raw.length() == 0) return;

  // saltar cabecera y errores
  if (raw.startsWith("tMs") || raw.startsWith("ERROR") || raw.startsWith("Ready")) return;

  String[] t = split(raw, ',');
  if (t.length < 11) return;

  try {
    long   tMs   = Long.parseLong(trim(t[0]));
    // t[1..4] = quat, no los necesitamos aquí (Arduino ya envía Euler)
    float  roll  = float(trim(t[5]));
    float  pitch = -float(trim(t[6]));  // invertido: subir arma → línea baja
    float  yaw   = float(trim(t[7]));
    float  mRaw  = float(trim(t[8]));
    float  mEnv  = float(trim(t[9]));
    float  mBase = float(trim(t[10]));

    rollBuf [headIdx] = roll;
    pitchBuf[headIdx] = pitch;
    yawBuf  [headIdx] = yaw;
    micRaw  [headIdx] = mRaw;
    micEnv  [headIdx] = mEnv;
    micBase [headIdx] = mBase;
    tBuf    [headIdx] = tMs;

    headIdx = (headIdx + 1) % MAX_SAMPLES;
    if (count < MAX_SAMPLES) count++;

    lastLine = "t=" + tMs + "ms  roll=" + nf(roll,1,2) +
               "  pitch=" + nf(pitch,1,2) + "  yaw=" + nf(yaw,1,2) +
               "  mic=" + (int)mRaw;
    lastTMs = tMs;
  } catch (Exception e) {
    lastError = "Parse error: " + raw;
  }
}

// ─── DRAW ────────────────────────────────────────────────────────────
void draw() {
  background(20);

  int panelW = width;
  int panelH = height / 4;

  // Paneles: roll | pitch | yaw | mic
  drawPanel(0, 0,          panelW, panelH, rollBuf,  -180, 180,  "ROLL  (°)",  color(80,200,255),   1, null, null);
  drawPanel(1, panelH,     panelW, panelH, pitchBuf, -90,  90,   "PITCH (°) [inv: sube arma→baja línea]",  color(120,255,120),  2, null, null);
  drawPanel(2, panelH*2,   panelW, panelH, yawBuf,   0,    360,  "YAW   (°)",  color(255,200,60),   3, null, null);
  drawMicPanel(3, panelH*3, panelW, panelH);

  // Barra de estado inferior
  drawStatus();
}

// ─── PANEL GENÉRICO ─────────────────────────────────────────────────
void drawPanel(int panelIdx, int y, int w, int h,
               float[] buf, float minV, float maxV,
               String label, color col, int hotkey,
               float[] buf2, float[] buf3) {

  boolean hi = (highlight == hotkey);
  // fondo
  fill(hi ? color(30,30,50) : color(22,22,22));
  stroke(hi ? color(100,140,255) : color(50));
  strokeWeight(hi ? 2 : 1);
  rect(0, y, w, h);

  // cero
  float zeroY = map(0, minV, maxV, y+h-4, y+4);
  stroke(80); strokeWeight(1);
  line(0, zeroY, w, zeroY);

  // escala
  fill(70); noStroke();
  textSize(9);
  for (float v = minV; v <= maxV; v += (maxV - minV) / 4.0) {
    float ly = map(v, minV, maxV, y+h-4, y+4);
    text(nf(v,1,0)+"°", 4, ly+4);
    stroke(40); strokeWeight(1);
    line(30, ly, w, ly);
    noStroke();
  }

  // curva
  if (count > 1) {
    stroke(col); strokeWeight(1.5); noFill();
    beginShape();
    for (int i = 0; i < count; i++) {
      int idx = (headIdx - count + i + MAX_SAMPLES) % MAX_SAMPLES;
      float x = map(i, 0, count-1, 2, w-2);
      float vy = map(buf[idx], minV, maxV, y+h-4, y+4);
      vertex(x, vy);
    }
    endShape();
  }

  // valor actual
  float cur = (count > 0) ? buf[(headIdx - 1 + MAX_SAMPLES) % MAX_SAMPLES] : 0;
  fill(col); textSize(11);
  text(label + "  " + nf(cur, 1, 3) + "°", 36, y + 16);

  // número de tecla
  fill(hi ? color(255,220,60) : color(100));
  text("[" + hotkey + "]", w - 36, y + 16);
}

// ─── PANEL MIC ───────────────────────────────────────────────────────
void drawMicPanel(int panelIdx, int y, int w, int h) {
  boolean hi = (highlight == 4);
  fill(hi ? color(30,30,50) : color(22,22,22));
  stroke(hi ? color(100,140,255) : color(50));
  strokeWeight(hi ? 2 : 1);
  rect(0, y, w, h);

  float minV = 0, maxV = 1023;

  // líneas de escala
  for (float v = 0; v <= 1023; v += 256) {
    float ly = map(v, minV, maxV, y+h-4, y+4);
    stroke(40); strokeWeight(1);
    line(30, ly, w, ly);
    fill(70); noStroke(); textSize(9);
    text((int)v, 4, ly+4);
  }

  if (count > 1) {
    // raw (gris)
    stroke(120); strokeWeight(1); noFill();
    beginShape();
    for (int i = 0; i < count; i++) {
      int idx = (headIdx - count + i + MAX_SAMPLES) % MAX_SAMPLES;
      float x = map(i, 0, count-1, 2, w-2);
      float vy = map(micRaw[idx], minV, maxV, y+h-4, y+4);
      vertex(x, vy);
    }
    endShape();

    // envelope (amarillo)
    stroke(color(255,200,40)); strokeWeight(1.5); noFill();
    beginShape();
    for (int i = 0; i < count; i++) {
      int idx = (headIdx - count + i + MAX_SAMPLES) % MAX_SAMPLES;
      float x = map(i, 0, count-1, 2, w-2);
      float vy = map(micEnv[idx], minV, maxV, y+h-4, y+4);
      vertex(x, vy);
    }
    endShape();

    // base (rojo)
    stroke(color(255,80,80)); strokeWeight(1.5); noFill();
    beginShape();
    for (int i = 0; i < count; i++) {
      int idx = (headIdx - count + i + MAX_SAMPLES) % MAX_SAMPLES;
      float x = map(i, 0, count-1, 2, w-2);
      float vy = map(micBase[idx], minV, maxV, y+h-4, y+4);
      vertex(x, vy);
    }
    endShape();
  }

  // leyenda
  int curIdx = (headIdx - 1 + MAX_SAMPLES) % MAX_SAMPLES;
  fill(120); textSize(11);
  text("MIC  raw=" + (count>0?(int)micRaw[curIdx]:0), 36, y+16);
  fill(color(255,200,40));
  text("  env=" + (count>0?(int)micEnv[curIdx]:0), 155, y+16);
  fill(color(255,80,80));
  text("  base=" + (count>0?(int)micBase[curIdx]:0), 235, y+16);
  fill(100); text("[4]", w-36, y+16);
}

// ─── BARRA DE ESTADO ────────────────────────────────────────────────
void drawStatus() {
  fill(30); noStroke();
  // sin barra extra — el texto ya va sobre el último panel

  fill(200); textSize(10);
  text(lastLine, 36, height - 4);

  if (!portOpen) {
    fill(color(255,80,80));
    text("  ← " + lastError, 36 + textWidth(lastLine), height - 4);
  }
  if (paused) {
    fill(color(255,200,40));
    text("  [PAUSED]", 850, height - 4);
  }
}

// ─── TECLADO ────────────────────────────────────────────────────────
void keyPressed() {
  if (key == 'r' || key == 'R') {
    headIdx = 0; count = 0; lastLine = ""; lastError = "";
  } else if (key == 'p' || key == 'P') {
    paused = !paused;
  } else if (key >= '1' && key <= '4') {
    int k = key - '0';
    highlight = (highlight == k) ? 0 : k;
  }
}
