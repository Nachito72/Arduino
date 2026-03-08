import processing.serial.*;

// ===================== CONFIGURACIÓN =====================
final String PUERTO    = "COM3";   // ← Cambia al puerto de tu Arduino
final int    BAUDRATE  = 115200;
final int    MAX_PUNTOS = 300;     // Puntos visibles en el gráfico

// ===================== SERIE =====================
Serial puerto;
String lineaEntrada = "";

// ===================== DATOS =====================
// CSV: yaw,roll,gx,gy,gz,gmag,lax,lay,laz,lamag,ax,ay,az,temp,mx,my,mz,cs,cg,ca,cm
float yaw, roll;
float gx, gy, gz, gmag;
float lax, lay, laz, lamag;
float ax, ay, az;
float temp;
float mx, my, mz;
int cs, cg, ca, cm;

boolean armado = false;
String ultimoEvento = "";
int tiempoEvento = 0;

// Historiales por canal
ArrayList<Float> hYaw    = new ArrayList<Float>();
ArrayList<Float> hRoll   = new ArrayList<Float>();
ArrayList<Float> hGx     = new ArrayList<Float>();
ArrayList<Float> hGy     = new ArrayList<Float>();
ArrayList<Float> hGz     = new ArrayList<Float>();
ArrayList<Float> hGmag   = new ArrayList<Float>();
ArrayList<Float> hLax    = new ArrayList<Float>();
ArrayList<Float> hLay    = new ArrayList<Float>();
ArrayList<Float> hLaz    = new ArrayList<Float>();
ArrayList<Float> hLamag  = new ArrayList<Float>();
ArrayList<Float> hAx     = new ArrayList<Float>();
ArrayList<Float> hAy     = new ArrayList<Float>();
ArrayList<Float> hAz     = new ArrayList<Float>();
ArrayList<Float> hTemp   = new ArrayList<Float>();
ArrayList<Float> hMx     = new ArrayList<Float>();
ArrayList<Float> hMy     = new ArrayList<Float>();
ArrayList<Float> hMz     = new ArrayList<Float>();

// ===================== PANELES =====================
// Cada panel: {título, lista, color, min, max}
String[] titulos = {
  "Yaw (°)", "Roll/Elev (°)",
  "Gyro X (°/s)", "Gyro Y (°/s)", "Gyro Z (°/s)", "Gyro Mag (°/s)",
  "LinAcc X (m/s²)", "LinAcc Y (m/s²)", "LinAcc Z (m/s²)", "LinAcc Mag (m/s²)",
  "Acc X (m/s²)", "Acc Y (m/s²)", "Acc Z (m/s²)",
  "Temp (°C)",
  "Mag X (µT)", "Mag Y (µT)", "Mag Z (µT)"
};

color[] colores = {
  color(255, 200,   0), color(255, 120,   0),   // euler
  color(  0, 200, 255), color(  0, 150, 255), color(  0, 100, 255), color(100, 220, 255), // gyro
  color(  0, 255, 120), color(  0, 220,  80), color(  0, 180,  60), color(100, 255, 180), // linacc
  color(255,  80,  80), color(220,  60,  60), color(180,  40,  40), // acc
  color(255, 180, 255),                          // temp
  color(200, 100, 255), color(180,  80, 230), color(160,  60, 210)  // mag
};

float[] rangosMin = {
    0, -90,
  -500, -500, -500, 0,
  -20, -20, -20, 0,
  -20, -20, -20,
   15,
  -100, -100, -100
};

float[] rangosMax = {
  360,  90,
  500,  500,  500, 500,
   20,   20,   20,  30,
   20,   20,   20,
   45,
  100,  100,  100
};

// Layout
final int COLS = 3;
int ROWS;
int panelW, panelH;
int marginX = 5, marginY = 5;
int headerH = 60;

// ===================== SETUP =====================
void setup() {
  size(1200, 900);
  surface.setTitle("Monitor Arma BNO055");

  ROWS = (int) ceil(titulos.length / (float) COLS);
  panelW = (width  - marginX * (COLS + 1)) / COLS;
  panelH = (height - headerH - marginY * (ROWS + 1)) / ROWS;

  textFont(createFont("Arial", 11, true));

  // Abrir puerto serie
  try {
    puerto = new Serial(this, PUERTO, BAUDRATE);
    puerto.bufferUntil('\n');
  } catch (Exception e) {
    println("⚠ No se pudo abrir " + PUERTO + ": " + e.getMessage());
    println("Puertos disponibles: " + join(Serial.list(), ", "));
  }
}

// ===================== SERIE =====================
void serialEvent(Serial p) {
  lineaEntrada = p.readStringUntil('\n');
  if (lineaEntrada == null) return;
  lineaEntrada = trim(lineaEntrada);

  if (lineaEntrada.equals("ARMED")) {
    armado = true; ultimoEvento = "ARMED"; tiempoEvento = millis(); return;
  }
  if (lineaEntrada.equals("DISARMED")) {
    armado = false; ultimoEvento = "DISARMED"; tiempoEvento = millis(); return;
  }
  if (lineaEntrada.startsWith("SHOT")) {
    ultimoEvento = lineaEntrada; tiempoEvento = millis(); return;
  }
  if (lineaEntrada.equals("Ready") || lineaEntrada.startsWith("//")) return;

  String[] t = split(lineaEntrada, ',');
  if (t.length < 21) return;

  try {
    yaw   = float(t[0]);  roll  = float(t[1]);
    gx    = float(t[2]);  gy    = float(t[3]);  gz   = float(t[4]);  gmag  = float(t[5]);
    lax   = float(t[6]);  lay   = float(t[7]);  laz  = float(t[8]);  lamag = float(t[9]);
    ax    = float(t[10]); ay    = float(t[11]); az   = float(t[12]);
    temp  = float(t[13]);
    mx    = float(t[14]); my    = float(t[15]); mz   = float(t[16]);
    cs    = int(t[17]);   cg    = int(t[18]);   ca   = int(t[19]);   cm = int(t[20]);
  } catch (Exception e) { return; }

  pushVal(hYaw,   yaw);   pushVal(hRoll,  roll);
  pushVal(hGx,    gx);    pushVal(hGy,    gy);    pushVal(hGz,   gz);   pushVal(hGmag,  gmag);
  pushVal(hLax,   lax);   pushVal(hLay,   lay);   pushVal(hLaz,  laz);  pushVal(hLamag, lamag);
  pushVal(hAx,    ax);    pushVal(hAy,    ay);    pushVal(hAz,   az);
  pushVal(hTemp,  temp);
  pushVal(hMx,    mx);    pushVal(hMy,    my);    pushVal(hMz,   mz);
}

void pushVal(ArrayList<Float> lista, float v) {
  lista.add(v);
  if (lista.size() > MAX_PUNTOS) lista.remove(0);
}

// ===================== DRAW =====================
void draw() {
  background(20);

  // --- HEADER ---
  dibujarHeader();

  // --- PANELES ---
  ArrayList[] historiales = {
    hYaw, hRoll,
    hGx, hGy, hGz, hGmag,
    hLax, hLay, hLaz, hLamag,
    hAx, hAy, hAz,
    hTemp,
    hMx, hMy, hMz
  };

  for (int i = 0; i < titulos.length; i++) {
    int col = i % COLS;
    int row = i / COLS;
    int px = marginX + col * (panelW + marginX);
    int py = headerH + marginY + row * (panelH + marginY);
    dibujarPanel(px, py, panelW, panelH, titulos[i], historiales[i], colores[i], rangosMin[i], rangosMax[i]);
  }
}

// ===================== HEADER =====================
void dibujarHeader() {
  // Fondo header
  fill(30);
  noStroke();
  rect(0, 0, width, headerH);

  // Estado armado
  color cEstado = armado ? color(0, 255, 80) : color(255, 60, 60);
  fill(cEstado);
  ellipse(30, headerH / 2, 20, 20);
  fill(cEstado);
  textSize(16);
  textAlign(LEFT, CENTER);
  text(armado ? "ARMED" : "DISARMED", 45, headerH / 2);

  // Calibración
  textSize(12);
  fill(200);
  text("Calibración →  SYS:" + cs + "  GYRO:" + cg + "  ACCEL:" + ca + "  MAG:" + cm,
       160, headerH / 2 - 8);

  // Valores principales
  fill(255, 200, 0);
  text("Yaw: " + nf(yaw, 1, 1) + "°   Roll: " + nf(roll, 1, 1) + "°   Temp: " + nf(temp, 1, 1) + "°C",
       160, headerH / 2 + 10);

  // Último evento
  if (millis() - tiempoEvento < 3000 && ultimoEvento.length() > 0) {
    color cEv = ultimoEvento.startsWith("SHOT") ? color(255, 80, 80) :
                ultimoEvento.equals("ARMED")     ? color(0, 255, 80)  : color(200, 200, 200);
    fill(cEv);
    textSize(18);
    textAlign(RIGHT, CENTER);
    text("⚡ " + ultimoEvento, width - 20, headerH / 2);
  }

  // Línea separadora
  stroke(60);
  line(0, headerH - 1, width, headerH - 1);
}

// ===================== PANEL =====================
void dibujarPanel(int px, int py, int pw, int ph, String titulo,
                  ArrayList<Float> datos, color c, float vMin, float vMax) {
  // Fondo
  fill(28);
  stroke(50);
  strokeWeight(1);
  rect(px, py, pw, ph, 4);

  int padL = 38, padR = 6, padT = 18, padB = 18;
  int gw = pw - padL - padR;
  int gh = ph - padT - padB;
  int gx = px + padL;
  int gy = py + padT;

  // Título + valor actual
  fill(180);
  textSize(10);
  textAlign(LEFT, TOP);
  text(titulo, px + 4, py + 3);

  if (datos.size() > 0) {
    float ult = datos.get(datos.size() - 1);
    fill(c);
    textSize(11);
    textAlign(RIGHT, TOP);
    text(nf(ult, 1, 2), px + pw - 4, py + 3);
  }

  // Área gráfico
  fill(18);
  stroke(40);
  rect(gx, gy, gw, gh);

  // Líneas de cuadrícula y etiquetas
  stroke(45);
  strokeWeight(1);
  int nLineas = 4;
  for (int i = 0; i <= nLineas; i++) {
    float fraccion = i / (float) nLineas;
    float yLinea = gy + gh - fraccion * gh;
    line(gx, yLinea, gx + gw, yLinea);
    float valorEtiqueta = vMin + fraccion * (vMax - vMin);
    fill(100);
    textSize(9);
    textAlign(RIGHT, CENTER);
    text(nf(valorEtiqueta, 1, 0), gx - 2, yLinea);
  }

  // Línea cero
  if (vMin < 0 && vMax > 0) {
    float yZero = gy + gh - map(0, vMin, vMax, 0, gh);
    stroke(80);
    strokeWeight(1);
    line(gx, yZero, gx + gw, yZero);
  }

  // Curva
  if (datos.size() < 2) return;
  stroke(c);
  strokeWeight(1.5);
  noFill();
  beginShape();
  for (int i = 0; i < datos.size(); i++) {
    float xPunto = map(i, 0, MAX_PUNTOS - 1, gx, gx + gw);
    float yPunto = map(constrain(datos.get(i), vMin, vMax), vMin, vMax, gy + gh, gy);
    vertex(xPunto, yPunto);
  }
  endShape();
}

// ===================== TECLADO =====================
void keyPressed() {
  // 'r' resetea todos los historiales
  if (key == 'r' || key == 'R') {
    hYaw.clear(); hRoll.clear();
    hGx.clear(); hGy.clear(); hGz.clear(); hGmag.clear();
    hLax.clear(); hLay.clear(); hLaz.clear(); hLamag.clear();
    hAx.clear(); hAy.clear(); hAz.clear();
    hTemp.clear();
    hMx.clear(); hMy.clear(); hMz.clear();
  }
  // 'l' lista puertos disponibles
  if (key == 'l' || key == 'L') {
    println("Puertos: " + join(Serial.list(), ", "));
  }
}
