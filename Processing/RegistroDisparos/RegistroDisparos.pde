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
// Windows:
//String PUERTO = "COM7";

// Mac:
//String PUERTO = "/dev/cu.usbmodem14101";  // el número varía

// Linux:
final String PUERTO   = "/dev/ttyACM0";  // ← Cambia al puerto de tu Arduino

final int    BAUDRATE = 500000;
final String DIR_BASE = System.getProperty("user.home") + File.separator + "Documents" + File.separator + "RegistroDisparos";  // carpeta raíz de grabaciones

// Umbral de pitch lateral para capturar la referencia de rumbo
// Solo cuando |pitchLat| está dentro de este rango se toma el primer yaw como 0°
float PITCH_REF_MIN = -8.0;  // ← ajustar a mano si el sensor lo requiere
float PITCH_REF_MAX =  8.0;

// Offset de montaje del sensor en la pistola.
// El BNO055 devuelve roll=0° cuando está horizontal; si está montado inclinado,
// ajusta este valor para que la posición "plana de disparo" muestre 0°.
// Ejemplo: si la zona plana de la gráfica sale en -83°, pon ROLL_OFFSET = -83.0
float ROLL_OFFSET = -85.0;  // ← ángulo raw que corresponde a "pistola plana" (0° real)

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
CopyOnWriteArrayList<String>    listaValores     = new CopyOnWriteArrayList<String>();  // valor impacto por sesión ("" si no hay)
int                  listaSeleccion   = -1;
int                  listaScroll      = 0;
int                  listaDeleteConfirm = -1;  // índice esperando confirmación de borrado

// ===================== MODO COMPARACIÓN =====================
boolean modoComparar = false;
ArrayList<Integer> comparaIdx = new ArrayList<Integer>();   // índices seleccionados (orden de adición)

// 10 colores para las trazas de comparación
color[] COMP_COLORS = {
  color(255, 180,  30),   // 0 naranja
  color( 80, 200, 255),   // 1 azul claro
  color( 80, 255, 120),   // 2 verde
  color(255,  80, 200),   // 3 rosa
  color(255, 220,  50),   // 4 amarillo
  color(160,  80, 255),   // 5 violeta
  color(255, 130,  50),   // 6 naranja oscuro
  color( 50, 220, 220),   // 7 cian
  color(255,  60,  60),   // 8 rojo
  color(180, 255,  80)    // 9 lima
};

// Datos por slot de comparación (máx 10)
CopyOnWriteArrayList[] compRoll  = new CopyOnWriteArrayList[10];
CopyOnWriteArrayList[] compYaw   = new CopyOnWriteArrayList[10];
CopyOnWriteArrayList[] compShots = new CopyOnWriteArrayList[10];

// Alineación temporal de trazas en modo comparar
// 0 = sin alinear (t original)  1 = alineado por disparo
int   modoAlineacion  = 0;
float[] compTimeOffset = new float[10];  // offset en segundos por slot (solo en modo 1)
float   mainTimeOffset = 0;              // offset de la sesión principal (modo 1)

// ===================== ENTRADA DE VALOR DE IMPACTO =====================
boolean pedirValor        = false;   // modal de entrada activo
boolean autoValorTrasDisparo = false; // checkbox: pedir valor automáticamente tras disparo
String  inputValor        = "";      // texto en edición
int     inputParaIdx      = -1;      // índice de sesión que se está editando

// ===================== REPRODUCCIÓN =====================
CopyOnWriteArrayList<String[]>  repDatos        = new CopyOnWriteArrayList<String[]>();
int                  repIdx          = 0;
boolean              repActiva       = false;
String               repTitulo       = "";

// Gráficas de reproducción / sesión activa
CopyOnWriteArrayList<float[]>   grafRoll        = new CopyOnWriteArrayList<float[]>(); // {tSeg, valor}
CopyOnWriteArrayList<float[]>   grafYaw         = new CopyOnWriteArrayList<float[]>();
CopyOnWriteArrayList<float[]>   grafShots       = new CopyOnWriteArrayList<float[]>(); // {tSeg}
double yawRef = Double.NaN;   // referencia de rumbo: yaw capturado cuando pitch entró en ±5°

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

// Botón "Zoom disparo" — coordenadas calculadas en dibujarGraficas() y usadas en mousePressed()
int btnZoomX, btnZoomY, btnZoomW = 110, btnZoomH = 20;
boolean zoomDisparoActivo = false;  // toggle: true=zoom sobre disparo, false=vista completa

// Botón "Diana" — ver trayectoria del disparo centrada en el punto de impacto
int btnDianaX, btnDianaY, btnDianaW = 90, btnDianaH = 20;
boolean modoDiana = false;

// Botón "Alinear trazas" — solo visible en modo comparar
int btnAlinearX, btnAlinearY, btnAlinearW = 130, btnAlinearH = 20;

int LISTA_W   = 300;
int HEADER_H  = 80;
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
color COL_TEXT    = color(240);
color COL_TEXTDIM = color(210);
color COL_CERO    = color( 80);

PFont fontMono, fontUI;

// ===================== SETUP =====================
void setup() {
  size(1400, 820);
  pixelDensity(1);
  surface.setTitle("Registro de Disparos");

  fontMono = createFont("Courier New", 12, true);
  fontUI   = createFont("Arial",       12, true);

  GRAFICA_H  = (height - HEADER_H - 80) / 2;
  GRAFICA_Y1 = HEADER_H + 10;
  GRAFICA_Y2 = GRAFICA_Y1 + GRAFICA_H + 20;

  COMBO_X = LISTA_W + 10;
  COMBO_Y = 8;                  // dentro del header, misma fila que los botones zoom/alinear

  // Crear carpeta base
  File f = new File(DIR_BASE);
  if (!f.exists()) f.mkdirs();
  verificarRuta();

  // Inicializar arrays de comparación
  for (int i = 0; i < 10; i++) {
    compRoll[i]  = new CopyOnWriteArrayList<float[]>();
    compYaw[i]   = new CopyOnWriteArrayList<float[]>();
    compShots[i] = new CopyOnWriteArrayList<float[]>();
  }

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

  // Trama CSV: qw,qx,qy,qz  (cuaternión, 4 campos, precisión 14-bit → ~0.004°)
  String[] t = split(linea, ',');
  if (t.length < 4) return;

  if (grabando) {
    float tSeg = (millis() - tInicioSesion) / 1000.0;
    // Guardamos tSeg como campo 0, forzando punto decimal (Locale.US)
    String[] fila = new String[t.length + 1];
    fila[0] = String.format(java.util.Locale.US, "%.4f", tSeg);
    for (int i = 0; i < t.length; i++) fila[i + 1] = t[i];
    bufferSesion.add(fila);

    // Actualizar gráficas en vivo
    try {
      double qw = Double.parseDouble(t[0].trim());
      double qx = Double.parseDouble(t[1].trim());
      double qy = Double.parseDouble(t[2].trim());
      double qz = Double.parseDouble(t[3].trim());

      // Euler desde cuaternión — precisión double (~0.004° vs 0.0625° de Euler BNO055)
      final double R2D = 57.29577951;
      double sinr = 2.0*(qw*qx + qy*qz);
      double cosr = 1.0 - 2.0*(qx*qx + qy*qy);
      double roll     = Math.atan2(sinr, cosr) * R2D;

      double sinp = 2.0*(qw*qy - qz*qx);
      double pitchDeg = (Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D;

      double siny = 2.0*(qw*qz + qx*qy);
      double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
      double yawRaw   = Math.atan2(siny, cosy) * R2D;
      if (yawRaw < 0) yawRaw += 360.0;

      float pitchLat = (float)pitchDeg;
      // Referencia de rumbo: capturar la PRIMERA vez que pitch entra en rango
      // Una vez capturada, la referencia es permanente durante toda la sesión
      if ((Double.isNaN(yawRef) || yawRef == 0) && pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX) {
        yawRef = yawRaw;
      }
      double yawRel;
      if (Double.isNaN(yawRef) || !(pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX)) {
        yawRel = 0;
        yawRef = 0;
      } else {
        yawRel = yawRaw - yawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }
      grafRoll.add(new float[]{ tSeg, (float)(roll - ROLL_OFFSET) });
      grafYaw.add(new float[]{ tSeg, (float)yawRel });
    } catch (Exception e) {}
  }
}
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
  yawRef = Double.NaN;
  tZoomMin = 0;    tZoomMax = -1;
  zoomMin[0] = -90;  zoomMax[0] = 90;   zoomLocked[0] = false;
  zoomMin[1] =   0;  zoomMax[1] = 360;  zoomLocked[1] = false;
  zoomDisparoActivo = false;
  modoDiana = false;
  modoAlineacion = 0;
  mainTimeOffset = 0;
  for (int s = 0; s < 10; s++) compTimeOffset[s] = 0;
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
  listaValores.add(0, "");
  listaSeleccion = 0;
  listaScroll    = 0;

  // Recargar días
  cargarDiasDisponibles();
  // Seleccionar día actual
  for (int i = 0; i < dias.size(); i++) {
    if (dias.get(i).equals(diaActual)) { diaSelIdx = i; break; }
  }

  println("■ Sesión guardada: " + rutaFichero + " | " + shotCount + " disparos | " + nf(durSeg, 1, 2) + "s");

  // Zoom automático sobre la zona del disparo al finalizar
  if (shotCount > 0 && grafShots.size() > 0) applyZoomDisparo();

  // Auto-pedir valor si está activado
  if (autoValorTrasDisparo && shotCount > 0) {
    inputParaIdx = 0;
    inputValor   = "";
    pedirValor   = true;
  }
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
  listaValores.clear();
  listaSeleccion = -1;
  listaScroll    = 0;
  comparaIdx.clear();
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
    String meta = "", shots = "?", dur = "?", hora = "?", num = "?", valor = "";
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
      if (meta.contains("Valor:"))   valor = meta.replaceAll(".*Valor:([^\\s]+).*",  "$1");
    }
    String entrada = String.format("%s  #%s  %s disp  %ss", hora, num, shots, dur);
    listaEntradas.add(entrada);
    listaRutas.add(ruta);
    listaValores.add(valor);
  }
}

void cargarReproduccion(String ruta) {
  repDatos.clear();
  grafRoll.clear();
  grafYaw.clear();
  grafShots.clear();
  repIdx    = 0;
  repActiva = false;
  yawRef = Double.NaN;  // resetear referencia al cargar

  String titulo = "";
  BufferedReader br = createReader(ruta);
  try {
    String linea;
    boolean primeraLinea = true;
    while ((linea = br.readLine()) != null) {
      if (primeraLinea) { titulo = linea; primeraLinea = false; continue; }
      if (linea.startsWith("tSeg")) continue; // cabecera columnas
      String[] t = split(linea, ',');
      // Formato nuevo: tSeg,qw,qx,qy,qz  (5 campos)
      // Formato antiguo: tSeg,pitchOut,roll,...  (22+ campos)
      boolean formatoNuevo = (t.length == 5 && !t[1].trim().equals("SHOT"));
      boolean formatoAntiguo = (t.length >= 22);
      if (!formatoNuevo && !formatoAntiguo) {
        // Puede ser una línea de disparo: tSeg,SHOT
        if (t.length == 2 && t[1].trim().equals("SHOT")) {
          float tShot = Float.parseFloat(t[0].trim().replace(',', '.'));
          grafShots.add(new float[]{ tShot });
        }
        continue;
      }
      repDatos.add(t);
      float tSeg = Float.parseFloat(t[0].trim().replace(',', '.'));
      double roll, yawRaw;
      float pitchLat;
      if (formatoNuevo) {
        double qw = Double.parseDouble(t[1].trim());
        double qx = Double.parseDouble(t[2].trim());
        double qy = Double.parseDouble(t[3].trim());
        double qz = Double.parseDouble(t[4].trim());
        final double R2D = 57.29577951;
        double sinr = 2.0*(qw*qx + qy*qz);
        double cosr = 1.0 - 2.0*(qx*qx + qy*qy);
        roll = Math.atan2(sinr, cosr) * R2D;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        double siny = 2.0*(qw*qz + qx*qy);
        double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
        yawRaw = Math.atan2(siny, cosy) * R2D;
        if (yawRaw < 0) yawRaw += 360.0;
      } else {
        // Formato antiguo
        roll     = Float.parseFloat(t[2].trim().replace(',', '.'));
        yawRaw   = Float.parseFloat(t[1].trim().replace(',', '.'));
        pitchLat = (t.length > 22) ? Float.parseFloat(t[22].trim().replace(',', '.')) : (float)roll;
      }
      // Capturar referencia la primera vez que pitch entra en rango (misma lógica que en vivo)
      if ((Double.isNaN(yawRef) || yawRef == 0) && pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX) {
        yawRef = yawRaw;
      }
      double yawRel;
      if (Double.isNaN(yawRef) || !(pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX)) {
        yawRel = 0;
        yawRef = 0;
      } else {
        yawRel = yawRaw - yawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }
      grafRoll.add(new float[]{ tSeg, (float)(roll - ROLL_OFFSET) });
      grafYaw.add(new float[]{ tSeg, (float)yawRel });
    }
    br.close();
  } catch (Exception e) { println("Error leyendo: " + e.getMessage()); }

  repTitulo = titulo;
  tZoomMin = 0;    tZoomMax = -1;
  zoomMin[0] = -90;  zoomMax[0] = 90;   zoomLocked[0] = false;
  zoomMin[1] =   0;  zoomMax[1] = 360;  zoomLocked[1] = false;
  zoomDisparoActivo = false;
  modoDiana = false;
  modoAlineacion = 0;
  mainTimeOffset = 0;
  for (int s = 0; s < 10; s++) compTimeOffset[s] = 0;
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
    listaValores.remove(idx);
    // Limpiar comparación si estaba incluida
    comparaIdx.remove(Integer.valueOf(idx));
    // Reindexar comparaIdx para índices mayores
    for (int i = 0; i < comparaIdx.size(); i++) {
      if (comparaIdx.get(i) > idx) comparaIdx.set(i, comparaIdx.get(i) - 1);
    }
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

// Carga una sesión en un slot de comparación (0-9)
void cargarEnSlot(int slot, String ruta) {
  compRoll[slot].clear();
  compYaw[slot].clear();
  compShots[slot].clear();
  double localYawRef = Double.NaN;
  BufferedReader br = createReader(ruta);
  try {
    String linea;
    boolean primera = true;
    while ((linea = br.readLine()) != null) {
      if (primera) { primera = false; continue; }
      if (linea.startsWith("tSeg")) continue;
      String[] t = split(linea, ',');
      boolean formatoNuevo  = (t.length == 5 && !t[1].trim().equals("SHOT"));
      boolean formatoAntiguo = (t.length >= 22);
      if (!formatoNuevo && !formatoAntiguo) {
        if (t.length == 2 && t[1].trim().equals("SHOT")) {
          float tShot = Float.parseFloat(t[0].trim().replace(',', '.'));
          compShots[slot].add(new float[]{ tShot });
        }
        continue;
      }
      float tSeg = Float.parseFloat(t[0].trim().replace(',', '.'));
      double roll, yawRaw;
      float pitchLat;
      if (formatoNuevo) {
        double qw = Double.parseDouble(t[1].trim());
        double qx = Double.parseDouble(t[2].trim());
        double qy = Double.parseDouble(t[3].trim());
        double qz = Double.parseDouble(t[4].trim());
        final double R2D = 57.29577951;
        double sinr = 2.0*(qw*qx + qy*qz);
        double cosr = 1.0 - 2.0*(qx*qx + qy*qy);
        roll = Math.atan2(sinr, cosr) * R2D;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        double siny = 2.0*(qw*qz + qx*qy);
        double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
        yawRaw = Math.atan2(siny, cosy) * R2D;
        if (yawRaw < 0) yawRaw += 360.0;
      } else {
        roll     = Float.parseFloat(t[2].trim().replace(',', '.'));
        yawRaw   = Float.parseFloat(t[1].trim().replace(',', '.'));
        pitchLat = (t.length > 22) ? Float.parseFloat(t[22].trim().replace(',', '.')) : (float)roll;
      }
      if ((Double.isNaN(localYawRef) || localYawRef == 0) && pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX) {
        localYawRef = yawRaw;
      }
      double yawRel;
      if (Double.isNaN(localYawRef) || !(pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX)) {
        yawRel = 0; localYawRef = 0;
      } else {
        yawRel = yawRaw - localYawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }
      compRoll[slot].add(new float[]{ tSeg, (float)(roll - ROLL_OFFSET) });
      compYaw[slot].add(new float[]{ tSeg, (float)yawRel });
    }
    br.close();
  } catch (Exception e) { println("Error cargando slot " + slot + ": " + e.getMessage()); }
}

// Guarda un valor de impacto en la cabecera CSV y actualiza listaValores
void guardarValorImpacto(int idx, String valor) {
  if (idx < 0 || idx >= listaRutas.size()) return;
  String ruta = listaRutas.get(idx);
  File f = new File(ruta);
  if (!f.exists()) return;

  // Leer todo el fichero
  ArrayList<String> lineas = new ArrayList<String>();
  BufferedReader br = createReader(ruta);
  try {
    String l;
    while ((l = br.readLine()) != null) lineas.add(l);
    br.close();
  } catch (Exception e) { return; }

  // Modificar primera línea añadiendo/reemplazando Valor:
  if (lineas.size() > 0) {
    String cabecera = lineas.get(0);
    // Quitar Valor: anterior si existe
    cabecera = cabecera.replaceAll("\\s*Valor:\\S+", "");
    if (valor.length() > 0) cabecera += " Valor:" + valor;
    lineas.set(0, cabecera);
  }

  // Reescribir fichero
  PrintWriter pw = createWriter(ruta);
  for (String l : lineas) pw.println(l);
  pw.flush(); pw.close();

  // Actualizar lista en memoria
  while (listaValores.size() <= idx) listaValores.add("");
  listaValores.set(idx, valor);

  println("💾 Valor guardado: '" + valor + "' → " + ruta);
}

// ===================== DRAW =====================
void draw() {
  background(COL_BG);
  dibujarHeader();
  dibujarCombo();
  dibujarLista();
  dibujarGraficas();
  dibujarInfoSesion();
  if (pedirValor) dibujarInputValor();
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
  fill(cE); textFont(fontUI); textSize(16); textAlign(LEFT, CENTER);
  text(armado ? "ARMED  ● GRABANDO" : "DISARMED", 38, HEADER_H / 2);

  // Puerto
  fill(COL_TEXTDIM); textSize(13); textAlign(RIGHT, CENTER);
  text("Puerto: " + PUERTO + "  |  Sesiones: " + listaEntradas.size(), width - 10, HEADER_H / 2);
}

// ===================== COMBO DÍA =====================
void dibujarCombo() {
  String etiqueta = dias.size() > 0 ? dias.get(diaSelIdx) : "Sin datos";

  // Botón principal
  fill(40); stroke(COL_BORDE); strokeWeight(1);
  rect(COMBO_X, COMBO_Y, COMBO_W, COMBO_H, 4);
  fill(COL_TEXT); textFont(fontUI); textSize(14); textAlign(LEFT, CENTER);
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

  // (etiqueta DÍA eliminada: el combo está en el header junto a los botones)
}

// ===================== LISTA SESIONES =====================
void dibujarLista() {
  int lx = 0, ly = HEADER_H;
  int lh = height - HEADER_H;
  int itemH = 44;   // un poco más alto para valor en 3ª línea

  fill(25); stroke(COL_BORDE);
  rect(lx, ly, LISTA_W, lh);

  // ── Cabecera con checkboxes ──────────────────────────────
  fill(COL_TEXTDIM); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("SESIONES DEL DÍA", lx + 8, ly + 5);

  // Checkbox "Comparar"
  int cbX = lx + 8, cbY = ly + 18, cbW = 12, cbH = 12;
  noFill(); stroke(COL_BORDE); rect(cbX, cbY, cbW, cbH, 2);
  if (modoComparar) { fill(COL_SEL); noStroke(); rect(cbX+2, cbY+2, cbW-4, cbH-4, 1); }
  fill(COL_TEXT); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("Comparar", cbX + cbW + 4, cbY);

  // Checkbox "Auto-valor"
  int cbX2 = cbX + 80, cbY2 = cbY;
  noFill(); stroke(COL_BORDE); rect(cbX2, cbY2, cbW, cbH, 2);
  if (autoValorTrasDisparo) { fill(color(0,180,80)); noStroke(); rect(cbX2+2, cbY2+2, cbW-4, cbH-4, 1); }
  fill(COL_TEXT); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("Auto-valor", cbX2 + cbW + 4, cbY2);

  int topBarH = 34;  // espacio de cabecera en el panel
  int startY = ly + topBarH;
  int visibles = (lh - topBarH) / itemH;

  for (int i = listaScroll; i < listaScroll + visibles && i < listaEntradas.size(); i++) {
    int iy = startY + (i - listaScroll) * itemH;
    boolean sel = (i == listaSeleccion);

    // En modo comparar: buscar si este índice tiene slot asignado
    int slotColor = -1;
    if (modoComparar) {
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (comparaIdx.get(s) == i) { slotColor = s; break; }
      }
    }

    // Fondo del ítem
    color bgItem;
    if (modoComparar && slotColor >= 0) {
      bgItem = COMP_COLORS[slotColor % 10];
      // Oscurecer un poco para que el texto sea legible
      bgItem = lerpColor(bgItem, color(10), 0.55);
    } else if (!modoComparar && sel) {
      bgItem = COL_SEL;
    } else {
      bgItem = (i % 2 == 0) ? color(32) : color(28);
    }
    fill(bgItem); noStroke();
    rect(lx, iy, LISTA_W, itemH);

    // Indicador de color en modo comparar (barra izquierda)
    if (modoComparar && slotColor >= 0) {
      fill(COMP_COLORS[slotColor % 10]); noStroke();
      rect(lx, iy, 4, itemH);
    }

    // Número de ítem
    fill(sel || (modoComparar && slotColor >= 0) ? color(255) : COL_TEXTDIM);
    textFont(fontMono); textSize(12); textAlign(LEFT, TOP);
    text(String.format("%02d", i + 1), lx + 8, iy + 4);

    // Texto principal (1ª línea)
    fill(COL_TEXT);
    textFont(fontMono); textSize(13);
    String entrada = listaEntradas.get(i);
    String[] partes = splitTokens(entrada, "  ");
    if (partes.length >= 2) {
      text(partes[0], lx + 28, iy + 4);
      // 2ª línea: resto de info
      String resto = "";
      for (int k = 1; k < partes.length; k++) resto += "  " + partes[k];
      fill(COL_TEXTDIM); textSize(12);
      text(trim(resto), lx + 28, iy + 17);
    } else {
      text(entrada, lx + 28, iy + 4);
    }

    // 3ª línea: valor de impacto (si existe)
    String val = (i < listaValores.size()) ? listaValores.get(i) : "";
    if (val.length() > 0) {
      fill(color(80,255,160)); textFont(fontMono); textSize(12); textAlign(LEFT, TOP);
      text("🎯 " + val, lx + 28, iy + 30);
    }

    // ── Botones de la derecha ─────────────────────────────
    int btnW = 22, btnH = 16;
    int btnDelX = lx + LISTA_W - btnW - 4;
    int btnValX = btnDelX - btnW - 4;
    int btnY    = iy + (itemH - btnH) / 2;

    if (listaDeleteConfirm == i) {
      // Confirmación borrado: OK / NO
      fill(200, 40, 40); noStroke(); rect(btnValX - 2, btnY, 26, btnH, 3);
      fill(255); textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
      text("OK", btnValX - 2 + 13, btnY + btnH / 2);
      fill(60); noStroke(); rect(btnDelX, btnY, btnW, btnH, 3);
      fill(200); text("NO", btnDelX + btnW / 2, btnY + btnH / 2);
    } else {
      // Botón ✏ valor
      fill(color(40, 80, 50)); noStroke(); rect(btnValX, btnY, btnW, btnH, 3);
      fill(color(100, 220, 130)); textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
      text("✏", btnValX + btnW / 2, btnY + btnH / 2);
      // Botón ✕ borrar
      fill(sel ? color(180, 50, 50) : color(70, 35, 35)); noStroke();
      rect(btnDelX, btnY, btnW, btnH, 3);
      fill(sel ? color(255) : color(160)); textFont(fontUI); textSize(14); textAlign(CENTER, CENTER);
      text("✕", btnDelX + btnW / 2, btnY + btnH / 2);
    }

    // Línea separadora
    stroke(COL_BORDE); strokeWeight(1);
    line(lx, iy + itemH - 1, lx + LISTA_W, iy + itemH - 1);
  }

  // Scrollbar
  if (listaEntradas.size() > visibles) {
    float frac = (float) listaScroll / listaEntradas.size();
    float barH = (float) visibles / listaEntradas.size() * (lh - topBarH);
    fill(COL_SEL); noStroke();
    rect(LISTA_W - 4, startY + frac * (lh - topBarH), 3, barH, 2);
  }
}

// ===================== GRÁFICAS =====================
void dibujarGraficas() {
  int gx0 = LISTA_W + 10;
  int gw0 = width - gx0 - 10;

  // Eje X compartido: duración de la sesión (considerar todas las trazas activas)
  float tMax = 1.0;
  if (grafRoll.size() > 0) tMax = max(tMax, grafRoll.get(grafRoll.size()-1)[0]);
  if (modoComparar) {
    for (int s = 0; s < comparaIdx.size(); s++) {
      CopyOnWriteArrayList<float[]> cr = (CopyOnWriteArrayList<float[]>) compRoll[s];
      if (cr.size() > 0) tMax = max(tMax, cr.get(cr.size()-1)[0]);
    }
  }

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

  // Autoescala Roll (considerando todas las trazas)
  float rMin = -90, rMax = 90;
  if (!zoomLocked[0] && grafRoll.size() > 1) {
    rMin = Float.MAX_VALUE; rMax = -Float.MAX_VALUE;
    for (float[] p : grafRoll) {
      rMin = min(rMin, p[1]); rMax = max(rMax, p[1]);
    }
    if (modoComparar) {
      for (int s = 0; s < comparaIdx.size(); s++) {
        for (float[] p : (CopyOnWriteArrayList<float[]>) compRoll[s]) { rMin = min(rMin, p[1]); rMax = max(rMax, p[1]); }
      }
    }
    rMin -= ZOOM_MARGIN; rMax += ZOOM_MARGIN;
    zoomMin[0] = rMin; zoomMax[0] = rMax;
  } else if (!zoomLocked[0]) {
    zoomMin[0] = -90; zoomMax[0] = 90;
  }

  // Autoescala Yaw (considerando todas las trazas)
  float yMin = 0, yMax = 360;
  if (!zoomLocked[1] && grafYaw.size() > 1) {
    yMin = Float.MAX_VALUE; yMax = -Float.MAX_VALUE;
    for (float[] p : grafYaw) {
      yMin = min(yMin, p[1]); yMax = max(yMax, p[1]);
    }
    if (modoComparar) {
      for (int s = 0; s < comparaIdx.size(); s++) {
        for (float[] p : (CopyOnWriteArrayList<float[]>) compYaw[s]) { yMin = min(yMin, p[1]); yMax = max(yMax, p[1]); }
      }
    }
    yMin -= ZOOM_MARGIN; yMax += ZOOM_MARGIN;
    zoomMin[1] = yMin; zoomMax[1] = yMax;
  } else if (!zoomLocked[1]) {
    zoomMin[1] = 0; zoomMax[1] = 360;
  }

  // ── Botón zoom disparo ───────────────────────────────────
  boolean hayShot = grafShots.size() > 0;
  btnZoomX = width - btnZoomW - 16;
  btnZoomY = 8;                    // fila 1 del header
  boolean hoverZoom = mouseX >= btnZoomX && mouseX <= btnZoomX + btnZoomW &&
                      mouseY >= btnZoomY && mouseY <= btnZoomY + btnZoomH;
  fill(hayShot ? (zoomDisparoActivo ? color(60,160,255) : (hoverZoom ? color(255,180,0) : color(200,130,0))) : color(55));
  noStroke(); rect(btnZoomX, btnZoomY, btnZoomW, btnZoomH, 4);
  fill(hayShot ? color(0) : color(100));
  textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
  text(zoomDisparoActivo ? "\u2316 Ver todo" : "\u2316 Zoom disparo", btnZoomX + btnZoomW / 2, btnZoomY + btnZoomH / 2);

  // ── Botón alinear trazas (modo comparar) ─────────────────
  if (modoComparar) {
    btnAlinearW = 130; btnAlinearH = 20;
    btnAlinearX = btnZoomX - btnAlinearW - 8;
    btnAlinearY = btnZoomY;           // misma fila que zoom
    boolean hoverAl = mouseX >= btnAlinearX && mouseX <= btnAlinearX + btnAlinearW &&
                      mouseY >= btnAlinearY && mouseY <= btnAlinearY + btnAlinearH;
    color cAl = modoAlineacion == 1 ? color(60,160,255) :
                (hoverAl ? color(100,100,140) : color(55));
    fill(cAl); noStroke(); rect(btnAlinearX, btnAlinearY, btnAlinearW, btnAlinearH, 4);
    fill(modoAlineacion == 1 ? color(0) : color(160));
    textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
    String lblAl = modoAlineacion == 1 ? "\u21c4 Inicio sesi\u00f3n" : "\u21c4 Alinear disparo";
    text(lblAl, btnAlinearX + btnAlinearW / 2, btnAlinearY + btnAlinearH / 2);
  }

  // ── Botón Diana ──────────────────────────────────────────
  boolean hayDatosDiana = grafShots.size() > 0 || grafRoll.size() > 0;
  btnDianaX = modoComparar ? (btnAlinearX - btnDianaW - 8) : (btnZoomX - btnDianaW - 8);
  btnDianaY = btnZoomY;
  boolean hoverDiana = mouseX >= btnDianaX && mouseX <= btnDianaX + btnDianaW &&
                       mouseY >= btnDianaY && mouseY <= btnDianaY + btnDianaH;
  fill(modoDiana ? color(200, 60, 60) : (hayDatosDiana ? (hoverDiana ? color(210, 90, 90) : color(140, 35, 35)) : color(55)));
  noStroke(); rect(btnDianaX, btnDianaY, btnDianaW, btnDianaH, 4);
  fill(hayDatosDiana ? color(255) : color(100));
  textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
  text(modoDiana ? "\u00d7 Cerrar" : "\u25ce Diana", btnDianaX + btnDianaW / 2, btnDianaY + btnDianaH / 2);

  // Si modo diana activo, mostrar diana y salir
  if (modoDiana) { dibujarDiana(); return; }

  // ── Dibujar gráfica Roll ─────────────────────────────────
  dibujarGrafica(gx0, GRAFICA_Y1 + 30, gw0, GRAFICA_H - 30,
                 "ROLL / Elevación (°)", grafRoll, COL_ROLL,
                 zoomMin[0], zoomMax[0],
                 tVisMin + mainTimeOffset, tVisMax + mainTimeOffset, true, 0);
  // Trazas de comparación sobre Roll
  if (modoComparar) {
    for (int s = 0; s < comparaIdx.size(); s++) {
      dibujarTracaExtra(gx0, GRAFICA_Y1 + 30, gw0, GRAFICA_H - 30,
                        (CopyOnWriteArrayList<float[]>) compRoll[s],
                        (CopyOnWriteArrayList<float[]>) compShots[s],
                        COMP_COLORS[s % 10],
                        zoomMin[0], zoomMax[0], tVisMin, tVisMax, compTimeOffset[s]);
    }
  }

  // ── Dibujar gráfica Yaw ──────────────────────────────────
  dibujarGrafica(gx0, GRAFICA_Y2 + 30, gw0, GRAFICA_H - 30,
                 "YAW / Rumbo (°)", grafYaw, COL_YAW,
                 zoomMin[1], zoomMax[1],
                 tVisMin + mainTimeOffset, tVisMax + mainTimeOffset, true, 1);
  // Trazas de comparación sobre Yaw
  if (modoComparar) {
    for (int s = 0; s < comparaIdx.size(); s++) {
      dibujarTracaExtra(gx0, GRAFICA_Y2 + 30, gw0, GRAFICA_H - 30,
                        (CopyOnWriteArrayList<float[]>) compYaw[s],
                        (CopyOnWriteArrayList<float[]>) compShots[s],
                        COMP_COLORS[s % 10],
                        zoomMin[1], zoomMax[1], tVisMin, tVisMax, compTimeOffset[s]);
    }
  }

  // ── Leyenda en modo comparar ──────────────────────────────
  if (modoComparar && comparaIdx.size() > 0) {
    int legX = LISTA_W + 20 + GRAF_PAD_L;
    int legY = GRAFICA_Y1 + 35;
    for (int s = 0; s < comparaIdx.size(); s++) {
      int li = comparaIdx.get(s);
      String etiq = (li < listaEntradas.size()) ? listaEntradas.get(li) : "?";
      String val  = (li < listaValores.size() && listaValores.get(li).length() > 0)
                    ? "  🎯" + listaValores.get(li) : "";
      fill(COMP_COLORS[s % 10]); noStroke();
      rect(legX + s * 120, legY, 10, 10, 2);
      fill(COMP_COLORS[s % 10]); textFont(fontMono); textSize(11); textAlign(LEFT, TOP);
      // Mostrar solo la hora (primer token)
      String[] tok = splitTokens(etiq, "  ");
      text((tok.length > 0 ? tok[0] : etiq) + val, legX + s * 120 + 14, legY);
    }
  }
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
  fill(COL_TEXTDIM); textFont(fontUI); textSize(13); textAlign(LEFT, TOP);
  text(titulo, gx + GRAF_PAD_L, gy - GRAF_PAD_T + 4);

  // Indicador autoescala / fija
  fill(zoomLocked[grafIdx] ? color(255,180,0) : color(0,200,120));
  textSize(12); textAlign(LEFT, TOP);
  text(zoomLocked[grafIdx] ? "[FIJO]" : "[AUTO]",
       gx + GRAF_PAD_L + 160, gy - GRAF_PAD_T + 4);

  // Valor actual + rango visible
  if (datos.size() > 0) {
    float ult = datos.get(datos.size()-1)[1];
    fill(c); textSize(14); textAlign(RIGHT, TOP);
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
    fill(210); textFont(fontMono); textSize(11); textAlign(RIGHT, CENTER);
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
    fill(210); textFont(fontMono); textSize(11); textAlign(CENTER, TOP);
    text(nf(t, 1, 2) + "s", xG, ay + ah + 2);
  }

  // Zona azul eléctrica PRESHOT_SEG segundos antes de cada disparo (fondo, antes de la curva)
  if (showShots) {
    for (float[] sh : grafShots) {
      float xSh  = ax + map(sh[0],               tVisMin, tVisMax, 0, aw);
      float xPre = ax + map(sh[0] - PRESHOT_SEG, tVisMin, tVisMax, 0, aw);
      xPre = max(xPre, ax);
      xSh  = min(xSh,  ax + aw);
      if (xSh < ax || xPre > ax + aw) continue;
      noStroke(); fill(0, 60, 160);        // azul eléctrico oscuro sólido
      rect(xPre, ay, xSh - xPre, ah);
    }
  }

  // Curva (solo puntos dentro de la ventana visible) — se dibuja encima del fondo azul
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

  // Línea vertical roja + triángulo encima de la curva
  if (showShots) {
    for (float[] sh : grafShots) {
      float xSh = ax + map(sh[0], tVisMin, tVisMax, 0, aw);
      if (xSh < ax || xSh > ax + aw) continue;
      stroke(COL_SHOT); strokeWeight(2);
      line(xSh, ay, xSh, ay + ah);
      fill(COL_SHOT); noStroke();
      triangle(xSh - 5, ay, xSh + 5, ay, xSh, ay + 10);
    }
  }

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
      fill(c); noStroke(); textFont(fontMono); textSize(11); textAlign(LEFT, CENTER);
      text(nf(cursorTSeg, 1, 2) + "s  " + nf(valCur, 1, 4) + "°", xCur + 8, yCur - 3);
    }
  }
}

// Dibuja una traza adicional (modo comparar) sobre un área de gráfica ya renderizada
void dibujarTracaExtra(int gx, int gy, int gw, int gh,
                       CopyOnWriteArrayList<float[]> datos,
                       CopyOnWriteArrayList<float[]> shots,
                       color c,
                       float vMin, float vMax,
                       float tVisMin, float tVisMax,
                       float tOffset) {
  if (datos.size() < 2) return;
  int ax = gx + GRAF_PAD_L;
  int ay = gy;
  int aw = gw - GRAF_PAD_L - GRAF_PAD_R;
  int ah = gh;

  // Zona preshot coloreada (semitransparente para no tapar las otras)
  for (float[] sh : shots) {
    float tSh = sh[0] - tOffset;
    float xSh  = ax + map(tSh,               tVisMin, tVisMax, 0, aw);
    float xPre = ax + map(tSh - PRESHOT_SEG, tVisMin, tVisMax, 0, aw);
    xPre = max(xPre, ax); xSh = min(xSh, ax + aw);
    if (xSh < ax || xPre > ax + aw) continue;
    noStroke();
    color cPre = lerpColor(color(0, 60, 160), c, 0.30);
    fill(red(cPre), green(cPre), blue(cPre), 120);
    rect(xPre, ay, xSh - xPre, ah);
  }

  // Curva
  stroke(c); strokeWeight(1.5); noFill();
  beginShape();
  for (float[] p : datos) {
    float t = p[0] - tOffset;
    if (t < tVisMin - 1 || t > tVisMax + 1) continue;
    float xP = ax + map(t, tVisMin, tVisMax, 0, aw);
    float yP = ay + ah - map(constrain(p[1], vMin, vMax), vMin, vMax, 0, ah);
    vertex(xP, yP);
  }
  endShape();

  // Marcadores de disparo
  for (float[] sh : shots) {
    float xSh = ax + map(sh[0] - tOffset, tVisMin, tVisMax, 0, aw);
    if (xSh < ax || xSh > ax + aw) continue;
    stroke(c); strokeWeight(1.5);
    line(xSh, ay, xSh, ay + ah);
    fill(c); noStroke();
    triangle(xSh - 4, ay, xSh + 4, ay, xSh, ay + 8);
    // Círculo en la curva
    float valSh = Float.NaN;
    float distMin = Float.MAX_VALUE;
    for (float[] p : datos) {
      float d = abs(p[0] - sh[0]);
      if (d < distMin) { distMin = d; valSh = p[1]; }
    }
    if (!Float.isNaN(valSh)) {
      float ySh = ay + ah - map(constrain(valSh, vMin, vMax), vMin, vMax, 0, ah);
      fill(c); stroke(255); strokeWeight(1);
      ellipse(xSh, ySh, 8, 8);
    }
  }
}

// ===================== DIANA =====================
void dibujarDiana() {
  int gx = LISTA_W + 10;
  int gy = HEADER_H + 10;
  int gw = width - gx - 10;
  int gh = height - gy - 10;

  fill(10); stroke(COL_BORDE); strokeWeight(1);
  rect(gx, gy, gw, gh, 4);

  int cx = gx + gw / 2;
  int cy = gy + gh / 2;
  int r  = min(gw - 120, gh - 120) / 2;

  // Sin datos
  if (grafShots.size() == 0 || min(grafRoll.size(), grafYaw.size()) == 0) {
    fill(COL_TEXTDIM); textFont(fontUI); textSize(15); textAlign(CENTER, CENTER);
    text("Sin datos de disparo.\nSelecciona una sesión con disparos en la lista.", cx, cy);
    return;
  }

  float tShot = grafShots.get(grafShots.size() - 1)[0];
  float tPre  = tShot - 0.300;
  float tPost = tShot + 0.100;

  // Posición en el momento del disparo (centro de la diana = desviación 0)
  float rollShot = 0, yawShot = 0;
  float bestD = Float.MAX_VALUE;
  for (float[] p : grafRoll) { float d = abs(p[0] - tShot); if (d < bestD) { bestD = d; rollShot = p[1]; } }
  bestD = Float.MAX_VALUE;
  for (float[] p : grafYaw)  { float d = abs(p[0] - tShot); if (d < bestD) { bestD = d; yawShot  = p[1]; } }

  // Ventana de puntos (grafRoll[i] y grafYaw[i] comparten timestamp)
  int nPts = min(grafRoll.size(), grafYaw.size());
  ArrayList<float[]> ventana = new ArrayList<float[]>();
  float maxDev = 0.01;
  for (int i = 0; i < nPts; i++) {
    float t  = grafRoll.get(i)[0];
    if (t < tPre || t > tPost) continue;
    float dr = grafRoll.get(i)[1] - rollShot;
    float dy = grafYaw.get(i)[1]  - yawShot;
    ventana.add(new float[]{ t, dr, dy });
    maxDev = max(maxDev, abs(dr), abs(dy));
  }
  maxDev = max(maxDev * 1.3, 0.02);

  // ── Anillos de la diana ────────────────────────────────────────────
  int nRings = 5;
  color[] ringCols = {
    color(180, 20, 20),
    color(150, 22, 22),
    color(30,  30, 52),
    color(24,  24, 42),
    color(18,  18, 32)
  };
  for (int i = nRings; i >= 1; i--) {
    float ri = r * (float) i / nRings;
    fill(ringCols[i - 1]); stroke(80); strokeWeight(1);
    ellipse(cx, cy, ri * 2, ri * 2);
    float devI = maxDev * i / nRings;
    fill(140); textFont(fontMono); textSize(10); textAlign(LEFT, CENTER);
    text(nf(devI, 1, 3) + "°", cx + ri + 4, cy);
    fill(120); textFont(fontMono); textSize(10); textAlign(CENTER, CENTER);
    text(str(nRings + 5 - i), cx, cy - ri + 8);
  }

  // Bullseye central
  fill(255); noStroke(); ellipse(cx, cy, 5, 5);

  // Cruces
  stroke(80, 80, 80, 160); strokeWeight(1);
  line(gx + 8, cy, gx + gw - 8, cy);
  line(cx, gy + 8, cx, gy + gh - 8);

  // ── Trayectoria ─────────────────────────────────────────────────────
  if (ventana.size() >= 2) {

    // Pre-disparo (azul): −300 ms hasta el disparo
    stroke(80, 150, 255); strokeWeight(2.5); noFill();
    beginShape();
    for (float[] p : ventana) {
      if (p[0] > tShot) break;
      float sx = cx + map(p[2], -maxDev, maxDev, -r, r);
      float sy = cy - map(p[1], -maxDev, maxDev, -r, r);
      vertex(sx, sy);
    }
    vertex(cx, cy);  // conectar con el centro (disparo)
    endShape();

    // Post-disparo (verde): disparo → +100 ms
    stroke(80, 255, 130); strokeWeight(1.5); noFill();
    boolean postStarted = false;
    beginShape();
    for (float[] p : ventana) {
      if (p[0] <= tShot) continue;
      if (!postStarted) { vertex(cx, cy); postStarted = true; }
      float sx = cx + map(p[2], -maxDev, maxDev, -r, r);
      float sy = cy - map(p[1], -maxDev, maxDev, -r, r);
      vertex(sx, sy);
    }
    endShape();

    // Puntos individuales sobre la trayectoria
    for (float[] p : ventana) {
      if (abs(p[0] - tShot) < 0.003) continue;
      float sx = cx + map(p[2], -maxDev, maxDev, -r, r);
      float sy = cy - map(p[1], -maxDev, maxDev, -r, r);
      fill(p[0] < tShot ? color(80, 150, 255) : color(80, 255, 130)); noStroke();
      ellipse(sx, sy, 4, 4);
    }

    // Flecha de aproximación al disparo (muestra la dirección de llegada)
    float arrowX = cx, arrowY = cy;
    for (float[] p : ventana) {
      if (p[0] >= tShot) break;
      arrowX = cx + map(p[2], -maxDev, maxDev, -r, r);
      arrowY = cy - map(p[1], -maxDev, maxDev, -r, r);
    }
    float adx = cx - arrowX, ady = cy - arrowY;
    float alen = sqrt(adx * adx + ady * ady);
    if (alen > 6) {
      float nx = adx / alen, ny = ady / alen;
      float headLen = min(14, alen * 0.5);
      float hx2 = cx - nx * headLen, hy2 = cy - ny * headLen;
      fill(80, 150, 255); noStroke();
      triangle(cx, cy, hx2 + (-ny) * 4, hy2 + nx * 4, hx2 - (-ny) * 4, hy2 - nx * 4);
    }
  }

  // Marca del disparo (centro)
  fill(COL_SHOT); stroke(255, 180, 180); strokeWeight(1.5);
  ellipse(cx, cy, 14, 14);
  fill(255); noStroke(); ellipse(cx, cy, 4, 4);

  // ── Título e información ───────────────────────────────────────────
  fill(COL_TEXT); textFont(fontUI); textSize(14); textAlign(CENTER, TOP);
  text("DIANA  —  Trayectoria del disparo  ( \u2212300 ms \u2192 \u25cf \u2192 +100 ms )", cx, gy + 8);

  // Etiquetas de ejes
  pushMatrix();
  translate(gx + 22, cy);
  rotate(-HALF_PI);
  fill(COL_ROLL); textFont(fontUI); textSize(11); textAlign(CENTER, CENTER);
  text("ROLL / Elevaci\u00f3n", 0, 0);
  popMatrix();

  fill(COL_YAW); textFont(fontUI); textSize(11); textAlign(CENTER, CENTER);
  text("YAW / Rumbo", cx, gy + gh - 14);

  fill(COL_TEXTDIM); textFont(fontMono); textSize(10); textAlign(CENTER, CENTER);
  text("\u25b2 +Roll", cx, cy - r - 16);
  text("\u25bc \u2212Roll", cx, cy + r + 16);
  text("\u25c4 \u2212Yaw", gx + 44, cy);
  text("+Yaw \u25ba", gx + gw - 44, cy);

  // Leyenda
  int legX = gx + 12;
  int legDY = gy + 30;
  noStroke(); fill(color(80, 150, 255)); rect(legX, legDY - 1, 20, 3, 1);
  fill(color(80, 150, 255)); textFont(fontUI); textSize(12); textAlign(LEFT, CENTER);
  text("Pre-disparo (300 ms)", legX + 26, legDY + 1);
  legDY += 18;
  noStroke(); fill(color(80, 255, 130)); rect(legX, legDY - 1, 20, 3, 1);
  fill(color(80, 255, 130)); textAlign(LEFT, CENTER);
  text("Post-disparo (100 ms)", legX + 26, legDY + 1);
  legDY += 18;
  fill(COL_SHOT); noStroke(); ellipse(legX + 10, legDY, 10, 10);
  fill(COL_SHOT); textAlign(LEFT, CENTER);
  text("Disparo  Yaw=" + nf(yawShot, 1, 3) + "\u00b0  Roll=" + nf(rollShot, 1, 3) + "\u00b0", legX + 26, legDY);
  legDY += 18;
  fill(COL_TEXTDIM); textFont(fontMono); textSize(11); textAlign(LEFT, CENTER);
  text("Escala: \u00b1" + nf(maxDev, 1, 3) + "\u00b0  |  " + ventana.size() + " muestras  |  t=" + nf(tShot, 1, 3) + "s", legX, legDY);
}

// ===================== INPUT VALOR DE IMPACTO =====================
void dibujarInputValor() {
  // Overlay semitransparente
  fill(0, 0, 0, 160); noStroke();
  rect(0, 0, width, height);

  // Caja de diálogo
  int bw = 400, bh = 130;
  int bx = (width - bw) / 2, by = (height - bh) / 2;
  fill(35); stroke(COL_BORDE); strokeWeight(1);
  rect(bx, by, bw, bh, 8);

  // Título
  String sesLabel = (inputParaIdx >= 0 && inputParaIdx < listaEntradas.size())
                    ? listaEntradas.get(inputParaIdx) : "";
  fill(COL_TEXT); textFont(fontUI); textSize(14); textAlign(CENTER, TOP);
  text("🎯 Valor de impacto", bx + bw/2, by + 10);
  fill(COL_TEXTDIM); textSize(12);
  text(sesLabel, bx + bw/2, by + 28);

  // Campo de texto
  fill(20); stroke(COL_YAW); strokeWeight(1);
  rect(bx + 20, by + 48, bw - 40, 26, 4);
  fill(COL_TEXT); textFont(fontMono); textSize(15); textAlign(LEFT, CENTER);
  text(inputValor + "|", bx + 26, by + 61);

  // Botones
  int okX = bx + bw/2 + 10, okY = by + 84, okW = 80, okH = 22;
  int cnX = bx + bw/2 - 90, cnY = by + 84, cnW = 80, cnH = 22;
  fill(COL_SEL); noStroke(); rect(okX, okY, okW, okH, 4);
  fill(255); textFont(fontUI); textSize(13); textAlign(CENTER, CENTER);
  text("Guardar", okX + okW/2, okY + okH/2);
  fill(color(70, 35, 35)); noStroke(); rect(cnX, cnY, cnW, cnH, 4);
  fill(200); text("Cancelar", cnX + cnW/2, cnY + cnH/2);

  fill(COL_TEXTDIM); textFont(fontUI); textSize(11); textAlign(CENTER, TOP);
  text("Enter=guardar  Esc=cancelar", bx + bw/2, by + 108);
}

// ===================== INFO SESIÓN =====================
// Aplica zoom X e Y centrado en el último disparo (±0.20s, rango Y ±0.10°)
void applyZoomDisparo() {
  if (grafShots.size() == 0) return;
  float tShot  = grafShots.get(grafShots.size() - 1)[0];
  float margen = 0.20;
  tZoomMin = max(0, tShot - margen);
  tZoomMax = tShot + margen;

  // Encontrar el valor de roll y yaw más cercano a tShot
  float rollAt = 0, yawAt = 0;
  float bestDR = Float.MAX_VALUE, bestDY = Float.MAX_VALUE;
  for (float[] p : grafRoll) {
    float d = abs(p[0] - tShot);
    if (d < bestDR) { bestDR = d; rollAt = p[1]; }
  }
  for (float[] p : grafYaw) {
    float d = abs(p[0] - tShot);
    if (d < bestDY) { bestDY = d; yawAt = p[1]; }
  }

  float halfY = 0.10;  // ±0.10° → rango total 0.20°
  zoomMin[0] = rollAt - halfY;  zoomMax[0] = rollAt + halfY;
  zoomMin[1] = yawAt  - halfY;  zoomMax[1] = yawAt  + halfY;
  zoomLocked[0] = true;
  zoomLocked[1] = true;
  zoomDisparoActivo = true;
}

void dibujarInfoSesion() {
  // Misma fila que el combo y los botones, entre el combo y los botones zoom/alinear
  int ix = COMBO_X + COMBO_W + 10;
  int iy = 8;
  int ih = 20;
  // El cuadro llega hasta el botón más a la izquierda: Diana (siempre visible), Alinear (solo comparar)
  int primerBoton = modoComparar ? min(btnDianaX, btnAlinearX) : btnDianaX;
  int iw = primerBoton - ix - 8;

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
  textFont(fontMono); textSize(12); textAlign(LEFT, CENTER);
  text(info, ix + 8, iy + ih / 2);
}

// ===================== INTERACCIÓN =====================
void mousePressed() {
  // Si modal de valor está abierto, manejar sus botones
  if (pedirValor) {
    int bw = 400, bh = 130;
    int bx = (width - bw) / 2, by = (height - bh) / 2;
    int okX = bx + bw/2 + 10, okY = by + 84, okW = 80, okH = 22;
    int cnX = bx + bw/2 - 90, cnY = by + 84, cnW = 80, cnH = 22;
    if (mouseX >= okX && mouseX <= okX+okW && mouseY >= okY && mouseY <= okY+okH) {
      guardarValorImpacto(inputParaIdx, inputValor.trim());
      pedirValor = false; inputValor = ""; inputParaIdx = -1;
    } else if (mouseX >= cnX && mouseX <= cnX+cnW && mouseY >= cnY && mouseY <= cnY+cnH) {
      pedirValor = false; inputValor = ""; inputParaIdx = -1;
    }
    return;
  }

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

  // Botón diana
  if (mouseX >= btnDianaX && mouseX <= btnDianaX + btnDianaW &&
      mouseY >= btnDianaY && mouseY <= btnDianaY + btnDianaH) {
    modoDiana = !modoDiana;
    return;
  }

  // Botón zoom disparo
  if (mouseX >= btnZoomX && mouseX <= btnZoomX + btnZoomW &&
      mouseY >= btnZoomY && mouseY <= btnZoomY + btnZoomH) {
    if (zoomDisparoActivo) {
      // Segundo click: volver a vista completa
      tZoomMin = 0;  tZoomMax = -1;
      zoomMin[0] = -90;  zoomMax[0] = 90;   zoomLocked[0] = false;
      zoomMin[1] =   0;  zoomMax[1] = 360;  zoomLocked[1] = false;
      zoomDisparoActivo = false;
    } else {
      applyZoomDisparo();
    }
    return;
  }

  // Botón alinear trazas (solo activo en modo comparar)
  if (modoComparar &&
      mouseX >= btnAlinearX && mouseX <= btnAlinearX + btnAlinearW &&
      mouseY >= btnAlinearY && mouseY <= btnAlinearY + btnAlinearH) {
    if (modoAlineacion == 1) {
      // Toggle off: volver a tiempo absoluto
      modoAlineacion = 0;
      mainTimeOffset = 0;
      for (int s = 0; s < 10; s++) compTimeOffset[s] = 0;
    } else {
      // Toggle on: alinear por disparo
      // Referencia = el disparo más tardío entre la sesión principal y todas las comparadas
      float tShotRef = (grafShots.size() > 0) ? grafShots.get(grafShots.size()-1)[0] : 0;
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (compShots[s] != null && compShots[s].size() > 0) {
          float[] sh = (float[]) compShots[s].get(0);
          if (sh[0] > tShotRef) tShotRef = sh[0];
        }
      }
      modoAlineacion = 1;
      // Offset de la sesión principal: mueve su traza para que su disparo caiga en tShotRef
      mainTimeOffset = (grafShots.size() > 0) ? grafShots.get(grafShots.size()-1)[0] - tShotRef : 0;
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (compShots[s] != null && compShots[s].size() > 0) {
          float[] sh = (float[]) compShots[s].get(0);
          // offset = tShotComp - tShotRef  →  p[0]-offset pone el disparo en tShotRef
          compTimeOffset[s] = sh[0] - tShotRef;
        } else {
          compTimeOffset[s] = 0;
        }
      }
    }
    return;
  }

  // Panel izquierdo (lista de sesiones)
  if (mouseX < LISTA_W) {
    int ly   = HEADER_H;
    int topBarH = 34;
    int itemH   = 44;
    int startY  = ly + topBarH;

    // ── Checkbox "Comparar" ────────────────────────────────
    int cbX = 8, cbY = ly + 18, cbW = 12, cbH = 12;
    if (mouseX >= cbX && mouseX <= cbX+cbW && mouseY >= cbY && mouseY <= cbY+cbH) {
      modoComparar = !modoComparar;
      if (!modoComparar) {
        // Al desactivar, limpiar selección múltiple y datos de slots
        comparaIdx.clear();
        for (int s = 0; s < 10; s++) { compRoll[s].clear(); compYaw[s].clear(); compShots[s].clear(); compTimeOffset[s] = 0; }
        modoAlineacion = 0;
        mainTimeOffset = 0;
      }
      return;
    }

    // ── Checkbox "Auto-valor" ──────────────────────────────
    int cbX2 = 8 + 80, cbY2 = ly + 18;
    if (mouseX >= cbX2 && mouseX <= cbX2+cbW && mouseY >= cbY2 && mouseY <= cbY2+cbH) {
      autoValorTrasDisparo = !autoValorTrasDisparo;
      return;
    }

    // ── Ítems de la lista ──────────────────────────────────
    int rawIdx = (mouseY - startY) / itemH;
    int idx    = listaScroll + rawIdx;

    if (rawIdx < 0 || idx < 0 || idx >= listaEntradas.size()) {
      listaDeleteConfirm = -1;
      return;
    }

    int btnW    = 22, btnH = 16;
    int btnDelX = LISTA_W - btnW - 4;
    int btnValX = btnDelX - btnW - 4;
    int btnY    = startY + rawIdx * itemH + (itemH - btnH) / 2;

    if (listaDeleteConfirm == idx) {
      // Zona OK (borrar)
      if (mouseX >= btnValX - 2 && mouseX <= btnValX - 2 + 26 &&
          mouseY >= btnY && mouseY <= btnY + btnH) {
        borrarSesion(idx);
        listaDeleteConfirm = -1;
        return;
      }
      // Zona NO o cualquier otro → cancelar
      listaDeleteConfirm = -1;
      return;
    }

    // Click en botón ✏ (valor de impacto)
    if (mouseX >= btnValX && mouseX <= btnValX + btnW &&
        mouseY >= btnY && mouseY <= btnY + btnH) {
      // Prerellenar con valor previo si existe
      inputValor   = (idx < listaValores.size()) ? listaValores.get(idx) : "";
      inputParaIdx = idx;
      pedirValor   = true;
      return;
    }

    // Click en botón ✕ (borrar)
    if (mouseX >= btnDelX && mouseX <= btnDelX + btnW &&
        mouseY >= btnY && mouseY <= btnY + btnH) {
      listaDeleteConfirm = idx;
      return;
    }

    // Click normal en el ítem
    listaDeleteConfirm = -1;
    if (modoComparar) {
      // Modo comparar: toggle de selección
      int slotActual = -1;
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (comparaIdx.get(s) == idx) { slotActual = s; break; }
      }
      if (slotActual >= 0) {
        // Deseleccionar: quitar del slot, limpiar datos
        comparaIdx.remove(slotActual);  // remove by index (int) — OK because slotActual is the position in comparaIdx
        compRoll[slotActual].clear();
        compYaw[slotActual].clear();
        compShots[slotActual].clear();
        // Recompactar: mover slots posteriores hacia arriba
        for (int s = slotActual; s < comparaIdx.size(); s++) {
          compRoll[s].addAll(compRoll[s+1]);
          compYaw[s].addAll(compYaw[s+1]);
          compShots[s].addAll(compShots[s+1]);
          compRoll[s+1].clear(); compYaw[s+1].clear(); compShots[s+1].clear();
        }
      } else if (comparaIdx.size() < 10) {
        // Seleccionar en siguiente slot libre
        int nuevoSlot = comparaIdx.size();
        comparaIdx.add(idx);
        cargarEnSlot(nuevoSlot, listaRutas.get(idx));
      }
    } else {
      // Modo normal: selección única + carga
      listaSeleccion = idx;
      cargarReproduccion(listaRutas.get(idx));
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
    // Zoom en Y: aplica el mismo factor a ambas gráficas
    float factor = e.getCount() > 0 ? 1.15 : 0.87;

    for (int gi = 0; gi < 2; gi++) {
      // Zoom centrado en el punto medio del rango visible actual.
      // Usar el midrange de los datos causaba deriva: si el disparo coincide con
      // el extremo del rango de datos (p.ej. mínimo de Yaw), el centro calculado
      // quedaba desplazado y cada paso de zoom empujaba ese extremo hacia el margen.
      float centro = (zoomMin[gi] + zoomMax[gi]) / 2.0;
      float mitad  = (zoomMax[gi] - zoomMin[gi]) / 2.0;
      mitad = max(mitad * factor, 0.05);
      zoomMin[gi] = centro - mitad;
      zoomMax[gi] = centro + mitad;
      zoomLocked[gi] = true;
    }
  }
}

void keyPressed() {
  // ── Si el modal de valor está activo, capturar teclado ──
  if (pedirValor) {
    if (key == ENTER || key == RETURN) {
      guardarValorImpacto(inputParaIdx, inputValor.trim());
      pedirValor = false; inputValor = ""; inputParaIdx = -1;
    } else if (key == ESC) {
      // Cancelar sin guardar (consumir ESC para que no cierre la app)
      key = 0;
      pedirValor = false; inputValor = ""; inputParaIdx = -1;
    } else if (key == BACKSPACE) {
      if (inputValor.length() > 0) inputValor = inputValor.substring(0, inputValor.length() - 1);
    } else if (key >= 32 && key < 127) {
      if (inputValor.length() < 40) inputValor += key;
    }
    return;
  }

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