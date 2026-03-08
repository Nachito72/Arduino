import processing.serial.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.io.File;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.io.FileReader;
import java.text.SimpleDateFormat;

// ===================== CONFIGURACIÓN =====================
final String PUERTO   = "COM7";   // ← Cambia al puerto de tu Arduino
final int    BAUDRATE = 115200;
final String DIR_BASE = System.getProperty("user.home") + File.separator + "Documents" + File.separator + "RegistroDisparos";  // carpeta raíz de grabaciones

// Umbral de pitch lateral para capturar la referencia de rumbo
// Solo cuando |pitchLat| está dentro de este rango se toma el primer yaw como 0°
float PITCH_REF_MIN = -5.0;  // ← ajustar a mano si el sensor lo requiere
float PITCH_REF_MAX =  5.0;

// Ventana previa al disparo que se resalta en azul en las gráficas
float PRESHOT_SEG = 0.200;  // ← 200 ms por defecto, ajustar a mano

void verificarRuta() {
  println("📁 Guardando datos en: " + DIR_BASE);
}

// ===================== SERIE =====================
Serial puerto;

// ===================== ESTADO GLOBAL =====================
boolean armado      = false;
boolean grabando    = false;   // true entre ARMED y DISARMED
int     shotCount   = 0;       // disparos en la sesión actual
boolean shotPendiente = false; // SHOT recibido en sesión actual

// Sesión actual
CopyOnWriteArrayList<String[]> bufferSesion = new CopyOnWriteArrayList<String[]>(); // todas las tramas
long   tInicioSesion  = 0;
String fechaHoraInicio = "";
String diaActual       = "";
int    numSesionDia    = 1;

// ===================== HISTORIAL LISTA =====================
// Cada entrada: "HH:MM:SS | #N | Xshots | Dur Xs"
CopyOnWriteArrayList<String>    listaEntradas    = new CopyOnWriteArrayList<String>();
CopyOnWriteArrayList<String>    listaRutas       = new CopyOnWriteArrayList<String>();  // fichero asociado
int                  listaSeleccion   = -1;
int                  listaScroll      = 0;
int                  listaDeleteConfirm = -1;  // índice esperando confirmación de borrado

// ===================== REPRODUCCIÓN =====================
CopyOnWriteArrayList<String[]>  repDatos        = new CopyOnWriteArrayList<String[]>();
int                  repIdx          = 0;
boolean              repActiva       = false;
String               repTitulo       = "";

// Gráficas de reproducción / sesión activa
CopyOnWriteArrayList<float[]>   grafRoll        = new CopyOnWriteArrayList<float[]>(); // {tSeg, valor}
CopyOnWriteArrayList<float[]>   grafYaw         = new CopyOnWriteArrayList<float[]>();
CopyOnWriteArrayList<float[]>   grafShots       = new CopyOnWriteArrayList<float[]>(); // {tSeg}
float yawRef = Float.NaN;   // referencia de rumbo: yaw capturado cuando pitch entró en ±5°

// ===================== LAYOUT =====================
// Panel izquierdo: lista de sesiones
// Panel derecho: gráficas + info
// Zoom y pan manual por gráfica (0=Roll, 1=Yaw)
// zoomLocked: true=escala fija manual, false=autoescala
float[] zoomMin   = { -90,   0 };
float[] zoomMax   = {  90, 360 };
boolean[] zoomLocked = { false, false };
float ZOOM_MARGIN = 2.0;  // margen en grados alrededor del rango real

// Zoom eje X (tiempo) compartido por ambas gráficas
float tZoomMin = 0;       // segundo inicio ventana visible
float tZoomMax = -1;      // -1 = mostrar todo
float cursorTSeg = -1;    // segundo bajo el cursor del ratón

int LISTA_W   = 300;
int HEADER_H  = 50;
int GRAFICA_H;
int GRAFICA_Y1, GRAFICA_Y2;
int GRAF_PAD_L = 72, GRAF_PAD_R = 15, GRAF_PAD_T = 22, GRAF_PAD_B = 22;

// Días disponibles
CopyOnWriteArrayList<String> dias = new CopyOnWriteArrayList<String>();
int diaSelIdx = 0;

// ===================== SELECTOR DE DÍA (simple) =====================
boolean comboAbierto = false;
int COMBO_X, COMBO_Y, COMBO_W = 180, COMBO_H = 26;

// ===================== COLORES =====================
color COL_BG      = color(20);
color COL_PANEL   = color(30);
color COL_BORDE   = color(100);
color COL_ROLL    = color(255, 180,  30);
color COL_YAW     = color( 80, 200, 255);
color COL_SHOT    = color(255,  60,  60);
color COL_ARMED   = color(  0, 220,  80);
color COL_DISARM  = color(200,  60,  60);
color COL_SEL     = color( 60, 120, 200);
color COL_TEXT    = color(210);
color COL_TEXTDIM = color(180);
color COL_CERO    = color( 80);

PFont fontMono, fontUI;

// ===================== SETUP =====================
void setup() {
  size(1100, 760);
  pixelDensity(1);
  surface.setTitle("Registro de Disparos – BNO055");

  fontMono = createFont("Courier New", 11, true);
  fontUI   = createFont("Arial",       11, true);

  GRAFICA_H  = (height - HEADER_H - 80) / 2;
  GRAFICA_Y1 = HEADER_H + 10;
  GRAFICA_Y2 = GRAFICA_Y1 + GRAFICA_H + 20;

  COMBO_X = LISTA_W + 10;
  COMBO_Y = HEADER_H + 10;

  // Crear carpeta base
  File f = new File(DIR_BASE);
  if (!f.exists()) f.mkdirs();
  verificarRuta();

  cargarDiasDisponibles();
  cargarSesionesDia(diaSelIdx);

  // Puerto serie
  try {
    puerto = new Serial(this, PUERTO, BAUDRATE);
    puerto.bufferUntil('\n');
  } catch (Exception e) {
    println("⚠ Puerto " + PUERTO + " no disponible. Puertos: " + join(Serial.list(), ", "));
  }
}

// ===================== SERIE =====================
void serialEvent(Serial p) {
  if (p == null) return;
  String linea;
  try { linea = trim(p.readStringUntil('\n')); }
  catch (Exception e) { return; }
  if (linea == null || linea.length() == 0) return;

  if (linea.equals("ARMED")) {
    iniciarSesion();
    return;
  }
  if (linea.equals("DISARMED")) {
    finalizarSesion();
    return;
  }
  if (linea.startsWith("SHOT")) {
    if (grabando) {
      shotCount++;
      shotPendiente = true;
      float tSeg = (millis() - tInicioSesion) / 1000.0;
      grafShots.add(new float[]{ tSeg });
      // Guardar el disparo en el buffer CSV para que quede en el fichero
      bufferSesion.add(new String[]{ String.format(java.util.Locale.US, "%.4f", tSeg), "SHOT" });
    }
    return;
  }
  if (linea.equals("Ready") || linea.startsWith("//")) return;

  // Trama CSV normal
  String[] t = split(linea, ',');
  if (t.length < 21) return;

  if (grabando) {
    float tSeg = (millis() - tInicioSesion) / 1000.0;
    // Guardamos tSeg como campo 0, forzando punto decimal (Locale.US)
    String[] fila = new String[t.length + 1];
    fila[0] = String.format(java.util.Locale.US, "%.4f", tSeg);
    for (int i = 0; i < t.length; i++) fila[i + 1] = t[i];
    bufferSesion.add(fila);

    // Actualizar gráficas en vivo
    try {
      float roll     = Float.parseFloat(t[1].trim());
      float yawRaw   = Float.parseFloat(t[0].trim());
      // Pitch lateral (campo 21, si el Arduino ya lo envía)
      float pitchLat = roll; // (t.length > 21) ? Float.parseFloat(t[21].trim()) : 999;
      // Referencia de rumbo: capturar la PRIMERA vez que pitch entra en rango
      // Una vez capturada, la referencia es permanente durante toda la sesión
      if ((Float.isNaN(yawRef) || yawRef == 0) && pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX) {
        yawRef = yawRaw;
      }
      // Mientras no hay referencia (pitch nunca entró en rango) → mostrar 0
      // Con referencia → mostrar deriva respecto al primer yaw válido
      float yawRel;
      if (Float.isNaN(yawRef) || !(pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX)) {
        yawRel = 0;
        yawRef = 0;
      } else {
        yawRel = yawRaw - yawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }
      grafRoll.add(new float[]{ tSeg, -roll });
      grafYaw.add(new float[]{ tSeg, yawRel });
    } catch (Exception e) {}
  }
}

// ===================== GESTIÓN SESIONES =====================
void iniciarSesion() {
  armado          = true;
  grabando        = true;
  shotCount       = 0;
  shotPendiente   = false;
  tInicioSesion   = millis();
  bufferSesion.clear();
  grafRoll.clear();
  grafYaw.clear();
  grafShots.clear();
  yawRef = Float.NaN;
  tZoomMin = 0; tZoomMax = -1;
  repActiva = false;

  SimpleDateFormat sdfFecha = new SimpleDateFormat("yyyy-MM-dd");
  SimpleDateFormat sdfHora  = new SimpleDateFormat("HH:mm:ss");
  Date ahora = new Date();
  diaActual      = sdfFecha.format(ahora);
  fechaHoraInicio = sdfFecha.format(ahora) + " " + sdfHora.format(ahora);

  // Calcular número de sesión del día
  File dirDia = new File(DIR_BASE + File.separator + diaActual);
  if (!dirDia.exists()) dirDia.mkdirs();
  String[] existentes = dirDia.list();
  numSesionDia = (existentes != null) ? existentes.length + 1 : 1;

  println("▶ Sesión iniciada: " + fechaHoraInicio + " (#" + numSesionDia + ")");
}

void finalizarSesion() {
  if (!grabando) { armado = false; return; }
  armado  = false;
  grabando = false;

  float durSeg = (millis() - tInicioSesion) / 1000.0;

  // Nombre de fichero
  SimpleDateFormat sdfHoraF = new SimpleDateFormat("HHmmss");
  String horaStr = new SimpleDateFormat("HH:mm:ss").format(new Date());
  String horaFile = sdfHoraF.format(new Date());
  String nombreFichero = String.format("%03d_%s.csv", numSesionDia, horaFile);
  String rutaFichero = DIR_BASE + File.separator + diaActual + File.separator + nombreFichero;

  // Escribir CSV
  PrintWriter pw = createWriter(rutaFichero);
  pw.println("# Sesion:" + numSesionDia + " Fecha:" + fechaHoraInicio +
             " Shots:" + shotCount + " Duracion:" + nf(durSeg, 1, 2) + "s");
  pw.println("tSeg,yaw,roll,gx,gy,gz,gmag,lax,lay,laz,lamag,ax,ay,az,temp,mx,my,mz,cs,cg,ca,cm,pitchLat");
  for (String[] fila : bufferSesion) pw.println(join(fila, ","));
  pw.flush(); pw.close();

  // Añadir a lista visible
  String entrada = String.format("%s  #%d  %d disp  %.1fs",
                                 horaStr, numSesionDia, shotCount, durSeg);
  listaEntradas.add(0, entrada);
  listaRutas.add(0, rutaFichero);
  listaSeleccion = 0;
  listaScroll    = 0;

  // Recargar días
  cargarDiasDisponibles();
  // Seleccionar día actual
  for (int i = 0; i < dias.size(); i++) {
    if (dias.get(i).equals(diaActual)) { diaSelIdx = i; break; }
  }

  println("■ Sesión guardada: " + rutaFichero + " | " + shotCount + " disparos | " + nf(durSeg, 1, 2) + "s");
}

// ===================== CARGA DATOS =====================
void cargarDiasDisponibles() {
  dias.clear();
  File base = new File(DIR_BASE);
  if (!base.exists()) return;
  String[] subdirs = base.list();
  if (subdirs == null) return;
  Arrays.sort(subdirs, Collections.reverseOrder());
  for (String d : subdirs) {
    File fd = new File(DIR_BASE + File.separator + d);
    if (fd.isDirectory()) dias.add(d);
  }
}

void cargarSesionesDia(int idx) {
  listaEntradas.clear();
  listaRutas.clear();
  listaSeleccion = -1;
  listaScroll    = 0;
  if (idx < 0 || idx >= dias.size()) return;

  String dia = dias.get(idx);
  File dirDia = new File(DIR_BASE + File.separator + dia);
  String[] ficheros = dirDia.list();
  if (ficheros == null) return;
  Arrays.sort(ficheros, Collections.reverseOrder());

  for (String f : ficheros) {
    if (!f.endsWith(".csv")) continue;
    String ruta = DIR_BASE + File.separator + dia + File.separator + f;
    // Leer cabecera
    String meta = "", shots = "?", dur = "?", hora = "?", num = "?";
    BufferedReader br = createReader(ruta);
    try {
      meta = br.readLine();  // línea #
      br.close();
    } catch (Exception e) {}
    if (meta != null && meta.startsWith("#")) {
      if (meta.contains("Sesion:"))  num   = meta.replaceAll(".*Sesion:(\\S+).*",   "$1");
      if (meta.contains("Shots:"))   shots = meta.replaceAll(".*Shots:(\\S+).*",    "$1");
      if (meta.contains("Duracion:"))dur   = meta.replaceAll(".*Duracion:(\\S+).*", "$1");
      if (meta.contains("Fecha:"))   hora  = meta.replaceAll(".*Fecha:\\S+ (\\S+).*","$1");
    }
    String entrada = String.format("%s  #%s  %s disp  %ss", hora, num, shots, dur);
    listaEntradas.add(entrada);
    listaRutas.add(ruta);
  }
}

void cargarReproduccion(String ruta) {
  repDatos.clear();
  grafRoll.clear();
  grafYaw.clear();
  grafShots.clear();
  repIdx    = 0;
  repActiva = false;
  yawRef = Float.NaN;  // resetear referencia al cargar

  String titulo = "";
  BufferedReader br = createReader(ruta);
  try {
    String linea;
    boolean primeraLinea = true;
    while ((linea = br.readLine()) != null) {
      if (primeraLinea) { titulo = linea; primeraLinea = false; continue; }
      if (linea.startsWith("tSeg")) continue; // cabecera columnas
      String[] t = split(linea, ',');
      if (t.length < 22) {
        // Puede ser una línea de disparo: tSeg,SHOT
        if (t.length == 2 && t[1].trim().equals("SHOT")) {
          float tShot = Float.parseFloat(t[0].trim().replace(',', '.'));
          grafShots.add(new float[]{ tShot });
        }
        continue;
      }
      repDatos.add(t);
      float tSeg   = Float.parseFloat(t[0].trim().replace(',', '.'));
      float roll   = Float.parseFloat(t[2].trim().replace(',', '.'));
      float yawRaw = Float.parseFloat(t[1].trim().replace(',', '.'));
      // Pitch lateral: campo 22 si existe, si no usar roll como proxy (igual que en vivo)
      float pitchLat = (t.length > 22) ? Float.parseFloat(t[22].trim().replace(',', '.')) : roll;
      // Capturar referencia la primera vez que pitch entra en rango (misma lógica que en vivo)
      if ((Float.isNaN(yawRef) || yawRef == 0) && pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX) {
        yawRef = yawRaw;
      }
      float yawRel;
      if (Float.isNaN(yawRef) || !(pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX)) {
        yawRel = 0;
        yawRef = 0;
      } else {
        yawRel = yawRaw - yawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }
      grafRoll.add(new float[]{ tSeg, -roll });
      grafYaw.add(new float[]{ tSeg, yawRel });
    }
    br.close();
  } catch (Exception e) { println("Error leyendo: " + e.getMessage()); }

  repTitulo = titulo;
  tZoomMin = 0; tZoomMax = -1;
  println("Cargado: " + ruta + " | " + repDatos.size() + " tramas");
}

void borrarSesion(int idx) {
  if (idx < 0 || idx >= listaRutas.size()) return;
  String ruta = listaRutas.get(idx);
  File f = new File(ruta);
  if (f.exists() && f.delete()) {
    println("🗑 Borrado: " + ruta);
    listaEntradas.remove(idx);
    listaRutas.remove(idx);
    // Si era el seleccionado, limpiar gráficas
    if (listaSeleccion == idx) {
      listaSeleccion = -1;
      grafRoll.clear(); grafYaw.clear(); grafShots.clear();
      repTitulo = "";
    } else if (listaSeleccion > idx) {
      listaSeleccion--;
    }
    listaScroll = constrain(listaScroll, 0, max(0, listaEntradas.size() - 1));
  } else {
    println("⚠ No se pudo borrar: " + ruta);
  }
}

// ===================== DRAW =====================
void draw() {
  background(COL_BG);
  dibujarHeader();
  dibujarCombo();
  dibujarLista();
  dibujarGraficas();
  dibujarInfoSesion();
}

// ===================== HEADER =====================
void dibujarHeader() {
  fill(28); noStroke();
  rect(0, 0, width, HEADER_H);
  stroke(COL_BORDE); line(0, HEADER_H, width, HEADER_H);

  // Estado
  color cE = armado ? COL_ARMED : COL_DISARM;
  fill(cE); noStroke();
  ellipse(25, HEADER_H / 2, 16, 16);
  fill(cE); textFont(fontUI); textSize(14); textAlign(LEFT, CENTER);
  text(armado ? "ARMED  ● GRABANDO" : "DISARMED", 38, HEADER_H / 2);

  // Puerto
  fill(COL_TEXTDIM); textSize(11); textAlign(RIGHT, CENTER);
  text("Puerto: " + PUERTO + "  |  Sesiones: " + listaEntradas.size(), width - 10, HEADER_H / 2);
}

// ===================== COMBO DÍA =====================
void dibujarCombo() {
  String etiqueta = dias.size() > 0 ? dias.get(diaSelIdx) : "Sin datos";

  // Botón principal
  fill(40); stroke(COL_BORDE); strokeWeight(1);
  rect(COMBO_X, COMBO_Y, COMBO_W, COMBO_H, 4);
  fill(COL_TEXT); textFont(fontUI); textSize(12); textAlign(LEFT, CENTER);
  text("📅 " + etiqueta, COMBO_X + 8, COMBO_Y + COMBO_H / 2);
  // Flecha
  fill(COL_TEXTDIM); textAlign(RIGHT, CENTER);
  text(comboAbierto ? "▲" : "▼", COMBO_X + COMBO_W - 8, COMBO_Y + COMBO_H / 2);

  // Lista desplegable
  if (comboAbierto) {
    for (int i = 0; i < dias.size(); i++) {
      int dy = COMBO_Y + COMBO_H + i * COMBO_H;
      fill(i == diaSelIdx ? COL_SEL : color(50));
      stroke(COL_BORDE);
      rect(COMBO_X, dy, COMBO_W, COMBO_H, 2);
      fill(COL_TEXT); textAlign(LEFT, CENTER);
      text(dias.get(i), COMBO_X + 8, dy + COMBO_H / 2);
    }
  }

  // Etiqueta
  fill(COL_TEXTDIM); textAlign(LEFT, TOP); textSize(10);
  text("DÍA:", COMBO_X, COMBO_Y - 14);
}

// ===================== LISTA SESIONES =====================
void dibujarLista() {
  int lx = 0, ly = HEADER_H;
  int lh = height - HEADER_H;
  int itemH = 38;

  fill(25); stroke(COL_BORDE);
  rect(lx, ly, LISTA_W, lh);

  fill(COL_TEXTDIM); textFont(fontUI); textSize(10); textAlign(LEFT, TOP);
  text("SESIONES DEL DÍA", lx + 8, ly + 6);

  int startY = ly + 22;
  int visibles = (lh - 22) / itemH;

  for (int i = listaScroll; i < listaScroll + visibles && i < listaEntradas.size(); i++) {
    int iy = startY + (i - listaScroll) * itemH;
    boolean sel = (i == listaSeleccion);

    fill(sel ? COL_SEL : (i % 2 == 0 ? color(32) : color(28)));
    noStroke();
    rect(lx, iy, LISTA_W, itemH);

    // Número de ítem
    fill(sel ? color(255) : COL_TEXTDIM);
    textFont(fontMono); textSize(10); textAlign(LEFT, TOP);
    text(String.format("%02d", i + 1), lx + 6, iy + 4);

    // Texto principal
    fill(sel ? color(255) : COL_TEXT);
    textFont(fontMono); textSize(11);
    String entrada = listaEntradas.get(i);
    // Partir en dos líneas si es largo
    String[] partes = splitTokens(entrada, "  ");
    if (partes.length >= 2) {
      text(partes[0], lx + 26, iy + 4);
      String resto = "";
      for (int k = 1; k < partes.length; k++) resto += "  " + partes[k];
      fill(sel ? color(220) : COL_TEXTDIM);
      textSize(10);
      text(trim(resto), lx + 26, iy + 19);
    } else {
      text(entrada, lx + 26, iy + 4);
    }

    // Botón borrar
    int btnW = 26, btnH = 20;
    int btnX = lx + LISTA_W - btnW - 4;
    int btnY = iy + (itemH - btnH) / 2;
    if (listaDeleteConfirm == i) {
      // Modo confirmación: mostrar OK / NO
      fill(200, 40, 40); noStroke(); rect(btnX - 28, btnY, 26, btnH, 3);
      fill(255); textFont(fontUI); textSize(10); textAlign(CENTER, CENTER);
      text("OK", btnX - 28 + 13, btnY + btnH / 2);
      fill(60); noStroke(); rect(btnX, btnY, btnW, btnH, 3);
      fill(200); text("NO", btnX + btnW / 2, btnY + btnH / 2);
    } else {
      fill(sel ? color(180, 50, 50) : color(70, 35, 35)); noStroke();
      rect(btnX, btnY, btnW, btnH, 3);
      fill(sel ? color(255) : color(160)); textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
      text("✕", btnX + btnW / 2, btnY + btnH / 2);
    }

    // Línea separadora
    stroke(COL_BORDE); strokeWeight(1);
    line(lx, iy + itemH - 1, lx + LISTA_W, iy + itemH - 1);
  }

  // Scrollbar
  if (listaEntradas.size() > visibles) {
    float frac = (float) listaScroll / listaEntradas.size();
    float barH = (float) visibles / listaEntradas.size() * lh;
    fill(COL_SEL); noStroke();
    rect(LISTA_W - 4, ly + frac * lh, 3, barH, 2);
  }
}

// ===================== GRÁFICAS =====================
void dibujarGraficas() {
  int gx0 = LISTA_W + 10;
  int gw0 = width - gx0 - 10;

  // Eje X compartido: duración de la sesión
  float tMax = 1.0;
  if (grafRoll.size() > 0) tMax = max(tMax, grafRoll.get(grafRoll.size()-1)[0]);

  // Ventana X visible
  float tVisMin = tZoomMin;
  float tVisMax = (tZoomMax < 0) ? tMax : tZoomMax;
  tVisMax = max(tVisMax, tVisMin + 0.1);  // mínimo 0.1s de ventana

  // Calcular segundo bajo el cursor
  int gx0c = LISTA_W + 10;
  int gw0c = width - gx0c - 10;
  int axc  = gx0c + GRAF_PAD_L;
  int awc  = gw0c - GRAF_PAD_L - GRAF_PAD_R;
  if (mouseX >= axc && mouseX <= axc + awc) {
    cursorTSeg = map(mouseX, axc, axc + awc, tVisMin, tVisMax);
  } else {
    cursorTSeg = -1;
  }

  // Autoescala Roll
  float rMin = -90, rMax = 90;
  if (!zoomLocked[0] && grafRoll.size() > 1) {
    rMin = Float.MAX_VALUE; rMax = -Float.MAX_VALUE;
    for (float[] p : grafRoll) {
      rMin = min(rMin, p[1]); rMax = max(rMax, p[1]);
    }
    float rng = max(rMax - rMin, 0.5);  // mínimo 0.5° para no colapsar
    rMin -= ZOOM_MARGIN; rMax += ZOOM_MARGIN;
    zoomMin[0] = rMin; zoomMax[0] = rMax;
  } else if (!zoomLocked[0]) {
    zoomMin[0] = -90; zoomMax[0] = 90;
  }

  // Autoescala Yaw
  float yMin = 0, yMax = 360;
  if (!zoomLocked[1] && grafYaw.size() > 1) {
    yMin = Float.MAX_VALUE; yMax = -Float.MAX_VALUE;
    for (float[] p : grafYaw) {
      yMin = min(yMin, p[1]); yMax = max(yMax, p[1]);
    }
    yMin -= ZOOM_MARGIN; yMax += ZOOM_MARGIN;
    zoomMin[1] = yMin; zoomMax[1] = yMax;
  } else if (!zoomLocked[1]) {
    zoomMin[1] = 0; zoomMax[1] = 360;
  }

  dibujarGrafica(gx0, GRAFICA_Y1 + 30, gw0, GRAFICA_H - 30,
                 "ROLL / Elevación (°)", grafRoll, COL_ROLL,
                 zoomMin[0], zoomMax[0], tVisMin, tVisMax, true, 0);

  dibujarGrafica(gx0, GRAFICA_Y2 + 30, gw0, GRAFICA_H - 30,
                 "YAW / Rumbo (°)", grafYaw, COL_YAW,
                 zoomMin[1], zoomMax[1], tVisMin, tVisMax, true, 1);
}

void dibujarGrafica(int gx, int gy, int gw, int gh,
                    String titulo, List<float[]> datos,
                    color c, float vMin, float vMax,
                    float tVisMin, float tVisMax,
                    boolean showShots, int grafIdx) {

  // Marco
  fill(22); stroke(COL_BORDE); strokeWeight(1);
  rect(gx, gy - GRAF_PAD_T, gw, gh + GRAF_PAD_T + GRAF_PAD_B, 4);

  // Título
  fill(COL_TEXTDIM); textFont(fontUI); textSize(11); textAlign(LEFT, TOP);
  text(titulo, gx + GRAF_PAD_L, gy - GRAF_PAD_T + 4);

  // Indicador autoescala / fija
  fill(zoomLocked[grafIdx] ? color(255,180,0) : color(0,200,120));
  textSize(10); textAlign(LEFT, TOP);
  text(zoomLocked[grafIdx] ? "[FIJO]" : "[AUTO]",
       gx + GRAF_PAD_L + 160, gy - GRAF_PAD_T + 4);

  // Valor actual + rango visible
  if (datos.size() > 0) {
    float ult = datos.get(datos.size()-1)[1];
    fill(c); textSize(12); textAlign(RIGHT, TOP);
    text(nf(ult, 1, 4) + "°   rango: " + nf(vMax-vMin, 1, 2) + "°",
         gx + gw - GRAF_PAD_R, gy - GRAF_PAD_T + 4);
  }

  int ax = gx + GRAF_PAD_L;
  int ay = gy;
  int aw = gw - GRAF_PAD_L - GRAF_PAD_R;
  int ah = gh;

  // Área interior
  fill(15); noStroke();
  rect(ax, ay, aw, ah);

  // Grid horizontal: número de líneas y decimales según rango
  float rango = vMax - vMin;
  int nGrid = rango < 0.01 ? 8 : rango < 0.1 ? 8 : rango < 1.0 ? 6 : 5;
  // Decimales: suficientes para distinguir ticks consecutivos
  float pasoY   = rango / nGrid;
  int dec;
  if      (pasoY < 0.001)  dec = 4;
  else if (pasoY < 0.01)   dec = 4;
  else if (pasoY < 0.1)    dec = 4;
  else if (pasoY < 1.0)    dec = 3;
  else if (pasoY < 10.0)   dec = 2;
  else                     dec = 1;

  for (int i = 0; i <= nGrid; i++) {
    float frac = i / (float) nGrid;
    float yG = ay + ah - frac * ah;
    float vG = vMin + frac * rango;
    stroke(40); strokeWeight(1);
    line(ax, yG, ax + aw, yG);
    fill(175); textFont(fontMono); textSize(9); textAlign(RIGHT, CENTER);
    text(nf(vG, 1, dec), ax - 3, yG);
  }

  // Línea cero
  if (vMin < 0 && vMax > 0) {
    float yZ = ay + ah - map(0, vMin, vMax, 0, ah);
    stroke(COL_CERO); strokeWeight(1);
    line(ax, yZ, ax + aw, yZ);
  }

  // Grid vertical (eje X = tiempo en segundos)
  float tSpan = tVisMax - tVisMin;
  float pasoT = 1.0;  // siempre cada segundo
  // Primer tick alineado al paso
  float tStart = ceil(tVisMin / pasoT) * pasoT;
  for (float t = tStart; t <= tVisMax + 0.001; t += pasoT) {
    float xG = ax + map(t, tVisMin, tVisMax, 0, aw);
    if (xG < ax || xG > ax + aw) continue;
    stroke(80); strokeWeight(1);
    line(xG, ay, xG, ay + ah);
    fill(175); textFont(fontMono); textSize(9); textAlign(CENTER, TOP);
    text(nf(t, 1, 2) + "s", xG, ay + ah + 2);
  }

  // Marcas de disparo: zona azul previa + línea vertical + triángulo + punto sobre la curva
  if (showShots) {
    // 1) Zona azul semitransparente PRESHOT_SEG segundos antes del disparo
    for (float[] sh : grafShots) {
      float xSh  = ax + map(sh[0],                tVisMin, tVisMax, 0, aw);
      float xPre = ax + map(sh[0] - PRESHOT_SEG,  tVisMin, tVisMax, 0, aw);
      xPre = max(xPre, ax);
      xSh  = min(xSh,  ax + aw);
      if (xSh < ax || xPre > ax + aw) continue;
      noStroke(); fill(40, 120, 255, 55);
      rect(xPre, ay, xSh - xPre, ah);
    }
    // 2) Línea vertical roja + triángulo
    for (float[] sh : grafShots) {
      float xSh = ax + map(sh[0], tVisMin, tVisMax, 0, aw);
      if (xSh < ax || xSh > ax + aw) continue;
      stroke(COL_SHOT); strokeWeight(2);
      line(xSh, ay, xSh, ay + ah);
      fill(COL_SHOT); noStroke();
      triangle(xSh - 5, ay, xSh + 5, ay, xSh, ay + 10);
    }
  }

  // Curva (solo puntos dentro de la ventana visible)
  if (datos.size() < 2) return;
  stroke(c); strokeWeight(1.5); noFill();
  beginShape();
  for (float[] punto : datos) {
    if (punto[0] < tVisMin - pasoT || punto[0] > tVisMax + pasoT) continue;
    float xP = ax + map(punto[0], tVisMin, tVisMax, 0, aw);
    float yP = ay + ah - map(constrain(punto[1], vMin, vMax), vMin, vMax, 0, ah);
    vertex(xP, yP);
  }
  endShape();

  // Círculo sobre la curva en el instante de cada disparo (dibujado encima de la curva)
  if (showShots) {
    for (float[] sh : grafShots) {
      float xSh = ax + map(sh[0], tVisMin, tVisMax, 0, aw);
      if (xSh < ax || xSh > ax + aw) continue;
      // Buscar el valor de la curva más cercano al tiempo del disparo
      float valSh = Float.NaN;
      float distMin = Float.MAX_VALUE;
      for (float[] punto : datos) {
        float d = abs(punto[0] - sh[0]);
        if (d < distMin) { distMin = d; valSh = punto[1]; }
      }
      if (!Float.isNaN(valSh)) {
        float ySh = ay + ah - map(constrain(valSh, vMin, vMax), vMin, vMax, 0, ah);
        fill(COL_SHOT); stroke(255); strokeWeight(1.5);
        ellipse(xSh, ySh, 10, 10);
      }
    }
  }

  // Línea de cursor vertical + valor interpolado
  if (cursorTSeg >= tVisMin && cursorTSeg <= tVisMax) {
    float xCur = ax + map(cursorTSeg, tVisMin, tVisMax, 0, aw);
    stroke(200, 200, 200, 160); strokeWeight(1);
    line(xCur, ay, xCur, ay + ah);

    // Buscar valor más cercano al cursor
    float valCur = Float.NaN;
    float distMin = Float.MAX_VALUE;
    for (float[] punto : datos) {
      float d = abs(punto[0] - cursorTSeg);
      if (d < distMin) { distMin = d; valCur = punto[1]; }
    }
    if (!Float.isNaN(valCur)) {
      float yCur = ay + ah - map(constrain(valCur, vMin, vMax), vMin, vMax, 0, ah);
      fill(255); noStroke();
      ellipse(xCur, yCur, 6, 6);
      fill(30); stroke(c); strokeWeight(1);
      rect(xCur + 5, yCur - 10, 70, 14, 3);
      fill(c); noStroke(); textFont(fontMono); textSize(9); textAlign(LEFT, CENTER);
      text(nf(cursorTSeg, 1, 2) + "s  " + nf(valCur, 1, 4) + "°", xCur + 8, yCur - 3);
    }
  }
}

// ===================== INFO SESIÓN =====================
void dibujarInfoSesion() {
  int ix = LISTA_W + 10;
  int iy = GRAFICA_Y1;
  int iw = width - ix - 10;
  int ih = 26;

  fill(35); stroke(COL_BORDE); strokeWeight(1);
  rect(ix, iy, iw, ih, 4);

  String info;
  if (grabando) {
    float dur = (millis() - tInicioSesion) / 1000.0;
    info = "● REC  Inicio: " + fechaHoraInicio +
           "  |  Disparos: " + shotCount +
           "  |  Tramas: " + bufferSesion.size() +
           "  |  Dur: " + nf(dur, 1, 1) + "s";
    fill(COL_ARMED);
  } else if (repTitulo.length() > 0) {
    info = "▶ " + repTitulo;
    fill(COL_YAW);
  } else {
    info = "Esperando conexión...";
    fill(COL_TEXTDIM);
  }
  textFont(fontMono); textSize(10); textAlign(LEFT, CENTER);
  text(info, ix + 8, iy + ih / 2);
}

// ===================== INTERACCIÓN =====================
void mousePressed() {
  // Combo
  if (mouseX >= COMBO_X && mouseX <= COMBO_X + COMBO_W &&
      mouseY >= COMBO_Y && mouseY <= COMBO_Y + COMBO_H) {
    comboAbierto = !comboAbierto;
    return;
  }

  // Ítem del combo desplegado
  if (comboAbierto) {
    for (int i = 0; i < dias.size(); i++) {
      int dy = COMBO_Y + COMBO_H + i * COMBO_H;
      if (mouseX >= COMBO_X && mouseX <= COMBO_X + COMBO_W &&
          mouseY >= dy && mouseY <= dy + COMBO_H) {
        diaSelIdx = i;
        cargarSesionesDia(i);
        comboAbierto = false;
        return;
      }
    }
    comboAbierto = false;
    return;
  }

  // Lista de sesiones
  if (mouseX < LISTA_W) {
    int ly = HEADER_H + 22;
    int itemH = 38;
    int idx = listaScroll + (mouseY - ly) / itemH;
    if (idx >= 0 && idx < listaEntradas.size()) {
      int btnW = 26, btnH = 20;
      int btnX = LISTA_W - btnW - 4;
      int btnY_base = ly + (idx - listaScroll) * itemH + (itemH - btnH) / 2;

      if (listaDeleteConfirm == idx) {
        // Zona OK (borrar)
        if (mouseX >= btnX - 28 && mouseX <= btnX - 2 &&
            mouseY >= btnY_base && mouseY <= btnY_base + btnH) {
          borrarSesion(idx);
          listaDeleteConfirm = -1;
          return;
        }
        // Zona NO (cancelar)
        listaDeleteConfirm = -1;
        return;
      }

      // Click en botón ✕
      if (mouseX >= btnX && mouseX <= btnX + btnW &&
          mouseY >= btnY_base && mouseY <= btnY_base + btnH) {
        listaDeleteConfirm = idx;
        return;
      }

      // Click normal → cargar
      listaDeleteConfirm = -1;
      listaSeleccion = idx;
      cargarReproduccion(listaRutas.get(idx));
    } else {
      listaDeleteConfirm = -1;
    }
  }
}

void mouseWheel(MouseEvent e) {
  if (mouseX < LISTA_W) {
    listaScroll = constrain(listaScroll + (int) e.getCount(),
                            0, max(0, listaEntradas.size() - 10));
    return;
  }

  boolean enGrafica = mouseY > GRAFICA_Y1 + 30 ;
  if (!enGrafica) return;

  int axc = LISTA_W + 10 + GRAF_PAD_L;
  int awc = width - LISTA_W - 10 - GRAF_PAD_L - GRAF_PAD_R;

  if (mouseX >= axc && mouseX <= axc + awc) {
    // Zoom en X centrado en el segundo bajo el cursor
    float tMax = 1.0;
    if (grafRoll.size() > 0) tMax = grafRoll.get(grafRoll.size()-1)[0];
    float tVisMin = tZoomMin;
    float tVisMax = (tZoomMax < 0) ? tMax : tZoomMax;

    float tCursor = map(mouseX, axc, axc + awc, tVisMin, tVisMax);
    float span    = tVisMax - tVisMin;
    float factor  = e.getCount() > 0 ? 1.20 : 0.80;  // rueda abajo=zoom in, arriba=zoom out
    float newSpan = max(span * factor, 0.5);  // mínimo 0.5s visible

    // Mantener el segundo bajo el cursor fijo
    float ratioC = (tCursor - tVisMin) / span;
    tZoomMin = tCursor - ratioC * newSpan;
    tZoomMax = tZoomMin + newSpan;

    // Clamp para no salir del rango total
    if (tZoomMin < 0) { tZoomMax -= tZoomMin; tZoomMin = 0; }
    if (tZoomMax > tMax + 1) { tZoomMin -= (tZoomMax - tMax - 1); tZoomMax = tMax + 1; }
    tZoomMin = max(tZoomMin, 0);
  } else {
    // Zoom en Y si el cursor está fuera del área de gráfica en X
    int grafIdx = (mouseY < GRAFICA_Y2) ? 0 : 1;
    float centro = (zoomMin[grafIdx] + zoomMax[grafIdx]) / 2.0;
    float mitad  = (zoomMax[grafIdx] - zoomMin[grafIdx]) / 2.0;
    float factor = e.getCount() > 0 ? 1.15 : 0.87;
    mitad = max(mitad * factor, 0.1);
    zoomMin[grafIdx] = centro - mitad;
    zoomMax[grafIdx] = centro + mitad;
    zoomLocked[grafIdx] = true;
  }
}

void keyPressed() {
  if (key == 'r' || key == 'R') {
    cargarDiasDisponibles();
    cargarSesionesDia(diaSelIdx);
  }
  if (key == 'l' || key == 'L') {
    println("Puertos disponibles: " + join(Serial.list(), ", "));
  }
  // A = alternar autoescala Y en la gráfica bajo el cursor
  if (key == 'a' || key == 'A') {
    int grafIdx = (mouseY < GRAFICA_Y2) ? 0 : 1;
    zoomLocked[grafIdx] = !zoomLocked[grafIdx];
  }
  // Z = resetear zoom X (ver todo)
  if (key == 'z' || key == 'Z') {
    tZoomMin = 0; tZoomMax = -1;
  }
  // Flechas ↑↓ = pan vertical en la gráfica bajo el cursor (solo si el ratón está sobre gráficas)
  if (mouseX > LISTA_W && mouseY > HEADER_H) {
    if (keyCode == UP || keyCode == DOWN) {
      int grafIdx = (mouseY < GRAFICA_Y2) ? 0 : 1;
      // Paso de desplazamiento: 10% del rango visible
      float rango = zoomMax[grafIdx] - zoomMin[grafIdx];
      float paso  = rango * 0.10;
      if (paso < 0.001) paso = 0.001;
      float delta = (keyCode == UP) ? paso : -paso;
      zoomMin[grafIdx] += delta;
      zoomMax[grafIdx] += delta;
      zoomLocked[grafIdx] = true;  // fijar escala al hacer pan manual
    }
  }
}
