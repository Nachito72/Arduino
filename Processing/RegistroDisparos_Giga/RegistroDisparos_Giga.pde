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
import javax.sound.sampled.*;

// ===================== VERSIÓN =====================
final String VERSION_PDE = "1.2";


// ===================== ESTABILIDAD DE ELEVACION =====================
// Pitido cuando el eje de elevacion (roll corregido) se mantiene en +-0.5
// durante al menos ESTAB_N muestras seguidas (~500 ms a 100 Hz)
final int   ESTAB_N               = 50;   // ventana deslizante (num muestras ~500ms a 100Hz)
final float ESTAB_RANGO_ROLL      = 0.020; // rango maximo en grados (elevacion) para considerar estable
final float ESTAB_RANGO_YAW       = 0.020; // rango maximo en grados (deriva)     para considerar estable
final int   PITIDO_INTERVALO      = 600;  // ms entre pitidos mientras hay estabilidad
float[]     estabBuf         = new float[ESTAB_N];  // buffer roll
float[]     estabYawBuf      = new float[ESTAB_N];  // buffer yaw
int         estabBufIdx      = 0;
int         estabBufCount    = 0;
boolean     estableRoll      = false;
boolean     estableYaw       = false;
long        ultimoPitidoRollMs  = 0;
long        ultimoPitidoYawMs   = 0;
long        ultimoPitidoAmbosMs = 0;

// ===================== CONFIGURACION =====================
// Windows:
//final String PUERTO   = "COM9";

// Mac:
final String PUERTO = "/dev/cu.usbmodem3101";

// Linux:
//final String PUERTO   = "/dev/ttyACM0";

final int    BAUDRATE = 500000;
// Carpeta de grabaciones: relativa al sketch → funciona en Windows, Mac y Linux
// Estructura esperada: <repo>/Processing/RegistroDisparos_Giga/  →  <repo>/RegistroDisparos/
String DIR_BASE;

// Umbral de pitch lateral para capturar la referencia de rumbo
// Solo cuando |pitchLat| está dentro de este rango se toma el primer yaw como 0°
float PITCH_REF_MIN = -8.0;  // ← ajustar a mano si el sensor lo requiere
float PITCH_REF_MAX =  8.0;

// NOTA: ROLL_OFFSET ya se aplica en el Arduino Giga (filteredElev incluye el offset).
// Esta constante queda aquí solo para referencia; NO se usa en el cálculo en vivo ni en la reproducción.
final float ROLL_OFFSET = -85.0;  // offset aplicado en firmware Giga_MovSoloPlano

// Ventana previa al disparo que se resalta en azul en las gráficas
float PRESHOT_SEG = 0.200;  // ← 200 ms por defecto

// Ventana de la diana: tiempo antes y después del disparo que se dibuja como rastro
float DIANA_PRE_SEG    = 0.200;  // ventana azul pre-disparo (y referencia del centro)
float DIANA_POST_SEG   = 0.100;  // ventana verde post-disparo
float DIANA_YELLOW_SEG = 3.000;  // ventana amarilla: antes de la ventana azul
float DIANA_TAMANO_MM  = 170.0;  // lado de la diana cuadrada (mm) — 17×17 cm ISSF
float DIANA_SKIP_SEG   = 2.0;    // segundos iniciales de la sesión a descartar en la diana

// Colores de las trazas de la diana (configurables)
color DIANA_COL_PRE    = color( 80, 150, 255);  // azul:     DIANA_PRE_SEG antes del disparo
color DIANA_COL_POST   = color( 80, 255, 130);  // verde:    DIANA_POST_SEG después del disparo
color DIANA_COL_YELLOW = color(255, 220,  50);  // amarillo: DIANA_YELLOW_SEG previos a la ventana azul

// Ventana de suavizado de curvas en pantalla (muestras; 1 = sin suavizado)
final int SMOOTH_N = 7;
boolean suavizadoActivo = false;

// ===================== CALIBRACIÓN ELEVACIÓN POR DISPARO =====================
// Haz clic en un disparo de la gráfica de elevación con Cal.elev activo:
// introduce la distancia a la diana (m) y la altura medida en ella (mm).
boolean calibElevActivo = false;
float   rollRefElev     = Float.NaN; // ángulo (°) del disparo de referencia
float   calHeightMm     = 0;         // altura medida en diana (mm)
float   distanciaDiana  = 10000;     // distancia al blanco (mm), default 10 m
// Transformación de display (actualizados por updateElevCalib()):
float   elevScale       = 1.0;       // 1.0=grados, D*PI/180=mm/° cuando calibrado
float   elevOffset      = 0;         // offset en unidades de display

void updateElevCalib() {
  if (calibElevActivo && !Float.isNaN(rollRefElev)) {
    elevScale  = (float)(distanciaDiana * Math.PI / 180.0);
    elevOffset = calHeightMm - rollRefElev * elevScale;
  } else {
    elevScale  = 1.0;
    elevOffset = 0.0;
  }
}

String elevUnidad() {
  return (calibElevActivo && !Float.isNaN(rollRefElev)) ? "mm" : "°";
}

void verificarRuta() {
  println("📁 Guardando datos en: " + DIR_BASE);
}

// ===================== SERIE =====================
Serial puerto;

// ===================== ESTADO GLOBAL =====================
boolean armado      = false;
boolean grabando    = false;   // true entre ARMED y DISARMED
int     shotCount   = 0;       // disparos en la sesión actual
boolean shotPendiente = false; // shot recibido en sesión actual

// ===================== TEMPORIZADOR INTER-DISPAROS =====================
long    timerInicioMs = 0;
boolean timerActivo   = false;

// Sesión actual
CopyOnWriteArrayList<String[]> bufferSesion = new CopyOnWriteArrayList<String[]>();
long   tInicioSesion  = 0;
long   ms0Sesion      = -1;   // Arduino millis() de la primera trama de la sesión
String fechaHoraInicio = "";
String diaActual       = "";
int    numSesionDia    = 1;

// ===================== HISTORIAL LISTA =====================
CopyOnWriteArrayList<String>    listaEntradas    = new CopyOnWriteArrayList<String>();
CopyOnWriteArrayList<String>    listaRutas       = new CopyOnWriteArrayList<String>();
CopyOnWriteArrayList<String>    listaValores     = new CopyOnWriteArrayList<String>();
int                  listaSeleccion   = -1;
int                  listaScroll      = 0;
int                  listaDeleteConfirm = -1;

// ===================== FILTRO VOLUMEN DISPARO =====================
// Solo se registran/muestran los disparos cuyo volumen (rango ADC) >= este umbral.
// Con el firmware antiguo (shot=0/1) todos los disparos tienen volumen=1 y siempre pasan.
int SHOT_VOL_MIN = 0;   // 0 = sin filtro; subir para ignorar disparos lejanos

// ===================== MODO COMPARACIÓN =====================
boolean modoComparar = false;
ArrayList<Integer> comparaIdx = new ArrayList<Integer>();

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

// Alineación temporal
int   modoAlineacion  = 0;
float[] compTimeOffset = new float[10];
float   mainTimeOffset = 0;

// ===================== ENTRADA DE VALOR DE IMPACTO =====================
boolean pedirValor        = false;
boolean autoValorTrasDisparo = false;
String  inputValor        = "";
int     inputParaIdx      = -1;

// ===================== REPRODUCCIÓN =====================
CopyOnWriteArrayList<String[]>  repDatos        = new CopyOnWriteArrayList<String[]>();
int                  repIdx          = 0;
boolean              repActiva       = false;
String               repTitulo       = "";

// Gráficas de reproducción / sesión activa
CopyOnWriteArrayList<float[]>   grafRoll        = new CopyOnWriteArrayList<float[]>();
CopyOnWriteArrayList<float[]>   grafYaw         = new CopyOnWriteArrayList<float[]>();
CopyOnWriteArrayList<float[]>   grafShots       = new CopyOnWriteArrayList<float[]>();
double yawRef = Double.NaN;

// ── Readout combinado en vivo ──────────────────────────────────────────
float liveElev = 0.0;
float liveYaw  = 0.0;

// ===================== LAYOUT =====================
float[] zoomMin   = { -90,   0 };
float[] zoomMax   = {  90, 360 };
boolean[] zoomLocked = { false, false };
float ZOOM_MARGIN = 2.0;

float tZoomMin = 0;
float tZoomMax = -1;
float cursorTSeg = -1;
// Coordenadas del gráfico de elevación (para detectar clics de calibración):
int   grafElev_ax = 0, grafElev_ay = 0, grafElev_aw = 0, grafElev_ah = 0;
float grafElev_tMin = 0, grafElev_tMax = 10;

int btnZoomX, btnZoomY, btnZoomW = 110, btnZoomH = 20;
boolean zoomDisparoActivo = false;

int btnDianaX, btnDianaY, btnDianaW = 90, btnDianaH = 20;
boolean modoDiana = false;
boolean dianaModoZoom = false;  // true = solo ventana pre-disparo; false = sesión completa
boolean dianaPlayActive  = false;  // reproducción animada en la diana
int     dianaPlayStartMs = 0;      // millis() al iniciar la reproducción
float   dianaPlaySpeed   = 2.0;    // velocidad: 1x, 2x, 5x, 10x

int btnAlinearX, btnAlinearY, btnAlinearW = 130, btnAlinearH = 20;

int LISTA_W   = 300;
int HEADER_H  = 80;
int GRAFICA_H;
int GRAFICA_Y1, GRAFICA_Y2;
int GRAF_PAD_L = 72, GRAF_PAD_R = 15, GRAF_PAD_T = 22, GRAF_PAD_B = 22;

CopyOnWriteArrayList<String> dias = new CopyOnWriteArrayList<String>();
int diaSelIdx = 0;

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
color COL_VECTOR  = color(255, 100, 220); // magenta: vector de dirección al disparo

PFont fontMono, fontUI;

// ===================== SETUP =====================
void setup() {
  size(1400, 820);
  pixelDensity(1);
  surface.setTitle("Registro de Disparos - Giga v" + VERSION_PDE);

  // Carpeta de datos dentro del propio sketch: funciona igual en Windows, Mac y Linux
  DIR_BASE = sketchPath("datos");

  fontMono = createFont("Courier New", 12, true);
  fontUI   = createFont("Arial",       12, true);

  GRAFICA_H  = (height - HEADER_H - 80) / 2;
  GRAFICA_Y1 = HEADER_H + 10;
  GRAFICA_Y2 = GRAFICA_Y1 + GRAFICA_H + 20;

  COMBO_X = LISTA_W + 10;
  COMBO_Y = 8;

  File f = new File(DIR_BASE);
  if (!f.exists()) f.mkdirs();
  verificarRuta();

  for (int i = 0; i < 10; i++) {
    compRoll[i]  = new CopyOnWriteArrayList<float[]>();
    compYaw[i]   = new CopyOnWriteArrayList<float[]>();
    compShots[i] = new CopyOnWriteArrayList<float[]>();
  }

  cargarDiasDisponibles();
  cargarSesionesDia(diaSelIdx);

  try {
    puerto = new Serial(this, PUERTO, BAUDRATE);
    puerto.bufferUntil('\n');
  } catch (Exception e) {
    println("⚠ Puerto " + PUERTO + " no disponible. Puertos: " + join(Serial.list(), ", "));
  }
}

// ===================== SERIE =====================
// Formato Giga: ms,qw,qx,qy,qz,filteredElev,shot  (7 campos)
//   ms          = millis() en el Arduino
//   qw..qz      = cuaternión (6 decimales)
//   filteredElev= elevación con filtro complementario y ROLL_OFFSET ya aplicados
//   shot        = 1 si se detectó disparo en este frame, 0 en caso contrario
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
  if (linea.equals("Ready") || linea.startsWith("//")) return;

  String[] t = split(linea, ',');
  // Formato nuevo (v1.6+): 8 campos  ms,qw,qx,qy,qz,filteredElev,filteredYaw,shot
  // Formato viejo (v1.5-):  7 campos  ms,qw,qx,qy,qz,filteredElev,shot
  if (t.length != 7 && t.length != 8) return;
  boolean nuevoFormato = (t.length == 8);  // tiene filteredYaw en t[6], shot en t[7]

  if (grabando) {
    // Calcular tSeg a partir del timestamp Arduino
    long arduinoMs;
    try { arduinoMs = Long.parseLong(t[0].trim()); }
    catch (Exception e) { return; }
    if (ms0Sesion < 0) ms0Sesion = arduinoMs;
    float tSeg = (arduinoMs - ms0Sesion) / 1000.0;

    boolean esDisparo = false;
    int     shotVol   = 0;
    int     shotIdx   = nuevoFormato ? 7 : 6;  // índice del campo shot
    try { shotVol = Integer.parseInt(t[shotIdx].trim()); } catch (Exception e) {}
    esDisparo = shotVol > 0;

    // Guardar trama en buffer CSV: tSeg + todos los campos Arduino
    String[] fila = new String[t.length + 1];
    fila[0] = String.format(java.util.Locale.US, "%.4f", tSeg);
    for (int i = 0; i < t.length; i++) fila[i + 1] = t[i];
    bufferSesion.add(fila);

    // Disparo detectado por el firmware — filtrar por volumen mínimo
    if (esDisparo && shotVol >= SHOT_VOL_MIN) {
      shotCount++;
      shotPendiente = true;
      grafShots.add(new float[]{ tSeg, (float)shotVol });
    }

    // Actualizar gráficas en vivo
    try {
      double qw = Double.parseDouble(t[1].trim());
      double qx = Double.parseDouble(t[2].trim());
      double qy = Double.parseDouble(t[3].trim());
      double qz = Double.parseDouble(t[4].trim());
      float filteredElev = Float.parseFloat(t[5].trim());

      // Calcular pitch lateral desde cuaternión (necesario para yawRef y yawFalseado)
      final double R2D = 57.29577951;
      double sinp = 2.0*(qw*qy - qz*qx);
      double pitchDeg = (Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D;
      float pitchLat = (float)pitchDeg;

      // YAW: usar filteredYaw del firmware si disponible (más preciso, sin escalón 1/16°)
      // Si no, calcular desde cuaternión como antes
      double yawRaw;
      if (nuevoFormato) {
        yawRaw = Double.parseDouble(t[6].trim());  // filteredYaw: 0-360°
      } else {
        double siny = 2.0*(qw*qz + qx*qy);
        double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
        yawRaw = Math.atan2(siny, cosy) * R2D;
        if (yawRaw < 0) yawRaw += 360.0;
      }

      // Referencia de rumbo: capturar la primera vez que pitch entra en rango
      if ((Double.isNaN(yawRef) || yawRef == 0) && pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX) {
        yawRef = yawRaw;
      }
      double yawRel;
      if (Double.isNaN(yawRef) || yawRef == 0) {
        // Referencia aún no capturada: esperar a que el arma apunte al frente
        yawRel = 0;
      } else {
        // Una vez capturada, mostrar siempre el ángulo relativo real
        // (no forzar a 0 al elevar el arma — pitchLat cambia por el montaje vertical)
        yawRel = yawRaw - yawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }

      // filteredElev ya tiene ROLL_OFFSET aplicado desde el firmware
      grafRoll.add(new float[]{ tSeg, filteredElev });
      grafYaw.add(new float[]{ tSeg, (float)yawRel });
      liveElev = filteredElev;
      liveYaw  = (float)yawRel;

      // ── Estabilidad: buffer circular de los últimos ~500 ms ──────────
      boolean yawFalseado = !(pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX);
      float yawVal  = (float)yawRel;
      estabBuf[estabBufIdx] = filteredElev;
      if (!yawFalseado) estabYawBuf[estabBufIdx] = yawVal;
      estabBufIdx = (estabBufIdx + 1) % ESTAB_N;
      if (estabBufCount < ESTAB_N) estabBufCount++;

      if (estabBufCount >= ESTAB_N) {
        float rMin = estabBuf[0], rMax = estabBuf[0];
        float yMin = estabYawBuf[0], yMax = estabYawBuf[0];
        for (int j = 1; j < ESTAB_N; j++) {
          if (estabBuf[j]    < rMin) rMin = estabBuf[j];    if (estabBuf[j]    > rMax) rMax = estabBuf[j];
          if (estabYawBuf[j] < yMin) yMin = estabYawBuf[j]; if (estabYawBuf[j] > yMax) yMax = estabYawBuf[j];
        }
        estableRoll = (rMax - rMin) <= ESTAB_RANGO_ROLL;
        estableYaw  = !yawFalseado && (yMax - yMin) <= ESTAB_RANGO_YAW;
      } else {
        estableRoll = false;
        estableYaw  = false;
      }

      long ahoraMs = millis();
      boolean ambos = estableRoll && estableYaw;
      if (ambos) {
        if (ahoraMs - ultimoPitidoAmbosMs >= PITIDO_INTERVALO) {
          ultimoPitidoAmbosMs = ahoraMs;
          _beepType = 3; thread("_runBeep");
        }
      } else if (estableRoll) {
        if (ahoraMs - ultimoPitidoRollMs >= PITIDO_INTERVALO) {
          ultimoPitidoRollMs = ahoraMs;
          _beepType = 1; thread("_runBeep");
        }
      } else if (estableYaw) {
        if (ahoraMs - ultimoPitidoYawMs >= PITIDO_INTERVALO) {
          ultimoPitidoYawMs = ahoraMs;
          _beepType = 2; thread("_runBeep");
        }
      }
    } catch (Exception e) {}
  }
}

void iniciarSesion() {
  armado          = true;
  grabando        = true;
  shotCount       = 0;
  shotPendiente   = false;
  tInicioSesion   = millis();
  ms0Sesion       = -1;   // se captura en la primera trama Giga
  bufferSesion.clear();
  grafRoll.clear();
  grafYaw.clear();
  grafShots.clear();
  yawRef = Double.NaN;
  rollRefElev = Float.NaN;
  calHeightMm = 0;
  updateElevCalib();
  estabBufIdx          = 0;
  estabBufCount        = 0;
  estableRoll          = false;
  estableYaw           = false;
  ultimoPitidoRollMs   = 0;
  ultimoPitidoYawMs    = 0;
  ultimoPitidoAmbosMs  = 0;
  tZoomMin = 0;    tZoomMax = -1;
  zoomMin[0] = -90;  zoomMax[0] = 90;   zoomLocked[0] = false;
  zoomMin[1] =   0;  zoomMax[1] = 360;  zoomLocked[1] = false;
  zoomDisparoActivo = false;
  modoDiana = false;
  dianaPlayActive = false;
  modoAlineacion = 0;
  mainTimeOffset = 0;
  for (int s = 0; s < 10; s++) compTimeOffset[s] = 0;
  repActiva = false;

  SimpleDateFormat sdfFecha    = new SimpleDateFormat("yyyy-MM-dd");
  SimpleDateFormat sdfHora     = new SimpleDateFormat("HH:mm:ss");
  SimpleDateFormat sdfCarpeta  = new SimpleDateFormat("yyyy/MM/dd");
  Date ahora = new Date();
  diaActual       = sdfCarpeta.format(ahora);   // → datos/2026/04/03/
  fechaHoraInicio = sdfFecha.format(ahora) + " " + sdfHora.format(ahora);

  File dirDia = new File(DIR_BASE + File.separator + diaActual);
  if (!dirDia.exists()) dirDia.mkdirs();
  String[] existentes = dirDia.list();
  numSesionDia = (existentes != null) ? existentes.length + 1 : 1;

  // El timer inter-disparos NO se toca aquí: debe seguir corriendo durante
  // el arme y hasta el siguiente disparo real. Solo se resetea en finalizarSesion()
  // cuando hay un disparo confirmado.
  println("▶ Sesión iniciada: " + fechaHoraInicio + " (#" + numSesionDia + ")");
}

void finalizarSesion() {
  if (!grabando) { armado = false; return; }
  armado  = false;
  grabando = false;

  float durSeg = (millis() - tInicioSesion) / 1000.0;

  if (durSeg < 5.0) {
    println("⚠ Sesión descartada (duración " + nf(durSeg, 1, 2) + "s < 5s)");
    bufferSesion.clear();
    grafRoll.clear();
    grafYaw.clear();
    grafShots.clear();
    return;
  }

  SimpleDateFormat sdfHoraF = new SimpleDateFormat("HHmmss");
  String horaStr = new SimpleDateFormat("HH:mm:ss").format(new Date());
  String horaFile = sdfHoraF.format(new Date());
  String nombreFichero = String.format("%03d_%s.csv", numSesionDia, horaFile);
  String rutaFichero = DIR_BASE + File.separator + diaActual + File.separator + nombreFichero;

  PrintWriter pw = createWriter(rutaFichero);
  pw.println("# Sesion:" + numSesionDia + " Fecha:" + fechaHoraInicio +
             " Shots:" + shotCount + " Duracion:" + nf(durSeg, 1, 2) + "s");
  // Cabecera de columnas: tSeg (calculado) + 8 campos Giga (con filteredYaw)
  pw.println("tSeg,ms,qw,qx,qy,qz,filteredElev,filteredYaw,shot");
  for (String[] fila : bufferSesion) pw.println(join(fila, ","));
  pw.flush(); pw.close();

  String entrada = String.format("%s  #%d  %d disp  %.1fs",
                                 horaStr, numSesionDia, shotCount, durSeg);
  listaEntradas.add(0, entrada);
  listaRutas.add(0, rutaFichero);
  listaValores.add(0, "");
  listaSeleccion = 0;
  listaScroll    = 0;

  cargarDiasDisponibles();
  for (int i = 0; i < dias.size(); i++) {
    if (dias.get(i).equals(diaActual)) { diaSelIdx = i; break; }
  }

  println("■ Sesión guardada: " + rutaFichero + " | " + shotCount + " disparos | " + nf(durSeg, 1, 2) + "s");

  // Reiniciar el cronómetro solo si hubo un disparo real en esta sesión
  if (shotCount > 0) {
    timerInicioMs = millis();
    timerActivo   = true;
  }

  if (shotCount > 0 && grafShots.size() > 0) applyZoomDisparo();

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
  // Estructura: datos/año/mes/día/
  String[] anos = base.list();
  if (anos == null) return;
  Arrays.sort(anos, Collections.reverseOrder());
  for (String ano : anos) {
    File fAno = new File(base, ano);
    if (!fAno.isDirectory()) continue;
    String[] meses = fAno.list();
    if (meses == null) continue;
    Arrays.sort(meses, Collections.reverseOrder());
    for (String mes : meses) {
      File fMes = new File(fAno, mes);
      if (!fMes.isDirectory()) continue;
      String[] diaList = fMes.list();
      if (diaList == null) continue;
      Arrays.sort(diaList, Collections.reverseOrder());
      for (String d : diaList) {
        if (new File(fMes, d).isDirectory()) dias.add(ano + "/" + mes + "/" + d);
      }
    }
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
    String meta = "", shots = "?", dur = "?", hora = "?", num = "?", valor = "";
    BufferedReader br = createReader(ruta);
    try {
      meta = br.readLine();
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

// ── Detectar el formato del CSV por número de columnas ───────────────
// formatoGiga     : 8 cols  (tSeg, ms, qw, qx, qy, qz, filteredElev, shot)
// formatoFiltrado : 6 cols  (tSeg, qw, qx, qy, qz, filteredElev)
// formatoNuevo    : 5 cols  (tSeg, qw, qx, qy, qz)
// formatoAntiguo  : 22+ cols
// línea de disparo legacy: 2 cols, t[1]=="SHOT"
void cargarReproduccion(String ruta) {
  repDatos.clear();
  grafRoll.clear();
  grafYaw.clear();
  grafShots.clear();
  repIdx    = 0;
  repActiva = false;
  yawRef = Double.NaN;

  String titulo = "";
  BufferedReader br = createReader(ruta);
  try {
    String linea;
    boolean primeraLinea = true;
    while ((linea = br.readLine()) != null) {
      if (primeraLinea) { titulo = linea; primeraLinea = false; continue; }
      if (linea.startsWith("tSeg")) continue;  // cabecera de columnas
      String[] t = split(linea, ',');

      boolean formatoGigaV2  = (t.length == 9 && !t[1].trim().equals("SHOT"));  // nuevo: con filteredYaw
      boolean formatoGiga     = (t.length == 8 && !t[1].trim().equals("SHOT"));
      boolean formatoFiltrado = (t.length == 6 && !t[1].trim().equals("SHOT"));
      boolean formatoNuevo    = (t.length == 5 && !t[1].trim().equals("SHOT"));
      boolean formatoAntiguo  = (t.length >= 22);

      if (!formatoGigaV2 && !formatoGiga && !formatoFiltrado && !formatoNuevo && !formatoAntiguo) {
        // Línea de disparo legacy: tSeg,SHOT
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

      if (formatoGigaV2) {
        // t: tSeg, ms, qw, qx, qy, qz, filteredElev, filteredYaw, shot
        double qw = Double.parseDouble(t[2].trim());
        double qx = Double.parseDouble(t[3].trim());
        double qy = Double.parseDouble(t[4].trim());
        double qz = Double.parseDouble(t[5].trim());
        roll = Double.parseDouble(t[6].trim());  // filteredElev
        yawRaw = Double.parseDouble(t[7].trim()); // filteredYaw (0-360°, alta resolución)
        final double R2D = 57.29577951;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        int shotV = 0;
        try { shotV = Integer.parseInt(t[8].trim()); } catch (Exception e) {}
        if (shotV > 0 && shotV >= SHOT_VOL_MIN) grafShots.add(new float[]{ tSeg, (float)shotV });
      } else if (formatoGiga) {
        // t: tSeg, ms, qw, qx, qy, qz, filteredElev, shot (formato anterior sin filteredYaw)
        double qw = Double.parseDouble(t[2].trim());
        double qx = Double.parseDouble(t[3].trim());
        double qy = Double.parseDouble(t[4].trim());
        double qz = Double.parseDouble(t[5].trim());
        roll = Double.parseDouble(t[6].trim());  // filteredElev — ya con offset
        final double R2D = 57.29577951;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        double siny = 2.0*(qw*qz + qx*qy);
        double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
        yawRaw = Math.atan2(siny, cosy) * R2D;
        if (yawRaw < 0) yawRaw += 360.0;
        // Disparo incrustado en campo shot — puede ser 0 o volumen ADC
        int shotV = 0;
        try { shotV = Integer.parseInt(t[7].trim()); } catch (Exception e) {}
        if (shotV > 0 && shotV >= SHOT_VOL_MIN) grafShots.add(new float[]{ tSeg, (float)shotV });
      } else if (formatoFiltrado) {
        // t: tSeg, qw, qx, qy, qz, filteredElev
        double qw = Double.parseDouble(t[1].trim());
        double qx = Double.parseDouble(t[2].trim());
        double qy = Double.parseDouble(t[3].trim());
        double qz = Double.parseDouble(t[4].trim());
        roll = Double.parseDouble(t[5].trim());  // filteredElev
        final double R2D = 57.29577951;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        double siny = 2.0*(qw*qz + qx*qy);
        double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
        yawRaw = Math.atan2(siny, cosy) * R2D;
        if (yawRaw < 0) yawRaw += 360.0;
      } else if (formatoNuevo) {
        // t: tSeg, qw, qx, qy, qz
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

      // Capturar referencia de rumbo la primera vez que pitch entra en rango
      if ((Double.isNaN(yawRef) || yawRef == 0) && pitchLat >= PITCH_REF_MIN && pitchLat <= PITCH_REF_MAX) {
        yawRef = yawRaw;
      }
      double yawRel;
      if (Double.isNaN(yawRef) || yawRef == 0) {
        // Referencia aún no capturada
        yawRel = 0;
      } else {
        // Una vez capturada, mantener siempre el ángulo relativo real
        yawRel = yawRaw - yawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }

      // Para GigaV2, Giga y formatoFiltrado: roll ya tiene offset aplicado.
      // Para formatoNuevo: aplicar ROLL_OFFSET (comportamiento heredado).
      float rollStored;
      if (formatoGigaV2 || formatoGiga || formatoFiltrado) {
        rollStored = (float)roll;
      } else if (formatoAntiguo) {
        rollStored = (float)roll;  // formato antiguo: roll ya es valor real
      } else {
        rollStored = (float)(roll - ROLL_OFFSET);
      }
      grafRoll.add(new float[]{ tSeg, rollStored });
      grafYaw.add(new float[]{ tSeg, (float)yawRel });
    }
    br.close();
  } catch (Exception e) { println("Error leyendo: " + e.getMessage()); }

  repTitulo = titulo;
  // La calibración de elevación se mantiene al cambiar de sesión —
  // la referencia es del arma, no de la sesión. Solo se resetea al
  // desactivar Cal.elev o al iniciar una nueva sesión de grabación.
  updateElevCalib();
  tZoomMin = 0;    tZoomMax = -1;
  zoomMin[0] = -90;  zoomMax[0] = 90;   zoomLocked[0] = false;
  zoomMin[1] =   0;  zoomMax[1] = 360;  zoomLocked[1] = false;
  zoomDisparoActivo = false;
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
    comparaIdx.remove(Integer.valueOf(idx));
    for (int i = 0; i < comparaIdx.size(); i++) {
      if (comparaIdx.get(i) > idx) comparaIdx.set(i, comparaIdx.get(i) - 1);
    }
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

      boolean formatoGigaV2  = (t.length == 9 && !t[1].trim().equals("SHOT"));  // nuevo: con filteredYaw
      boolean formatoGiga     = (t.length == 8 && !t[1].trim().equals("SHOT"));
      boolean formatoFiltrado = (t.length == 6 && !t[1].trim().equals("SHOT"));
      boolean formatoNuevo   = (t.length == 5 && !t[1].trim().equals("SHOT"));
      boolean formatoAntiguo = (t.length >= 22);

      if (!formatoGigaV2 && !formatoGiga && !formatoFiltrado && !formatoNuevo && !formatoAntiguo) {
        if (t.length == 2 && t[1].trim().equals("SHOT")) {
          float tShot = Float.parseFloat(t[0].trim().replace(',', '.'));
          compShots[slot].add(new float[]{ tShot });
        }
        continue;
      }

      float tSeg = Float.parseFloat(t[0].trim().replace(',', '.'));
      double roll, yawRaw;
      float pitchLat;

      if (formatoGigaV2) {
        // t: tSeg, ms, qw, qx, qy, qz, filteredElev, filteredYaw, shot
        double qw = Double.parseDouble(t[2].trim());
        double qx = Double.parseDouble(t[3].trim());
        double qy = Double.parseDouble(t[4].trim());
        double qz = Double.parseDouble(t[5].trim());
        roll = Double.parseDouble(t[6].trim());
        yawRaw = Double.parseDouble(t[7].trim()); // filteredYaw (0-360°, alta resolución)
        final double R2D = 57.29577951;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        int shotV = 0;
        try { shotV = Integer.parseInt(t[8].trim()); } catch (Exception e) {}
        if (shotV > 0 && shotV >= SHOT_VOL_MIN) compShots[slot].add(new float[]{ tSeg, (float)shotV });
      } else if (formatoGiga) {
        double qw = Double.parseDouble(t[2].trim());
        double qx = Double.parseDouble(t[3].trim());
        double qy = Double.parseDouble(t[4].trim());
        double qz = Double.parseDouble(t[5].trim());
        roll = Double.parseDouble(t[6].trim());
        final double R2D = 57.29577951;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        double siny = 2.0*(qw*qz + qx*qy);
        double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
        yawRaw = Math.atan2(siny, cosy) * R2D;
        if (yawRaw < 0) yawRaw += 360.0;
        int shotV = 0;
        try { shotV = Integer.parseInt(t[7].trim()); } catch (Exception e) {}
        if (shotV > 0 && shotV >= SHOT_VOL_MIN) compShots[slot].add(new float[]{ tSeg, (float)shotV });
      } else if (formatoFiltrado) {
        double qw = Double.parseDouble(t[1].trim());
        double qx = Double.parseDouble(t[2].trim());
        double qy = Double.parseDouble(t[3].trim());
        double qz = Double.parseDouble(t[4].trim());
        roll = Double.parseDouble(t[5].trim());
        final double R2D = 57.29577951;
        double sinp = 2.0*(qw*qy - qz*qx);
        pitchLat = (float)((Math.abs(sinp) >= 1.0) ? (sinp >= 0 ? 90.0 : -90.0) : Math.asin(sinp) * R2D);
        double siny = 2.0*(qw*qz + qx*qy);
        double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
        yawRaw = Math.atan2(siny, cosy) * R2D;
        if (yawRaw < 0) yawRaw += 360.0;
      } else if (formatoNuevo) {
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
      if (Double.isNaN(localYawRef) || localYawRef == 0) {
        // Referencia aún no capturada
        yawRel = 0;
      } else {
        // Una vez capturada, mantener siempre el ángulo relativo real
        yawRel = yawRaw - localYawRef;
        while (yawRel >  180) yawRel -= 360;
        while (yawRel < -180) yawRel += 360;
      }

      float rollStored;
      if (formatoGigaV2 || formatoGiga || formatoFiltrado) {
        rollStored = (float)roll;
      } else if (formatoAntiguo) {
        rollStored = (float)roll;
      } else {
        rollStored = (float)(roll - ROLL_OFFSET);
      }
      compRoll[slot].add(new float[]{ tSeg, rollStored });
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

  ArrayList<String> lineas = new ArrayList<String>();
  BufferedReader br = createReader(ruta);
  try {
    String l;
    while ((l = br.readLine()) != null) lineas.add(l);
    br.close();
  } catch (Exception e) { return; }

  if (lineas.size() > 0) {
    String cabecera = lineas.get(0);
    cabecera = cabecera.replaceAll("\\s*Valor:\\S+", "");
    if (valor.length() > 0) cabecera += " Valor:" + valor;
    lineas.set(0, cabecera);
  }

  PrintWriter pw = createWriter(ruta);
  for (String l : lineas) pw.println(l);
  pw.flush(); pw.close();

  while (listaValores.size() <= idx) listaValores.add("");
  listaValores.set(idx, valor);

  println("💾 Valor guardado: '" + valor + "' → " + ruta);
}

// ===================== SONIDO =====================
float _beepFreq = 880;
int   _beepDur  = 200;
float _beepVol  = 0.6;
int   _beepType = 0;

void pitarBeep(float freqHz, int durMs, float vol) {
  _beepFreq = freqHz;
  _beepDur  = durMs;
  _beepVol  = vol;
  _beepType = 0;
  thread("_runBeep");
}

void _runBeep() {
  if      (_beepType == 1) BeepHelper.playBeep(660,  150, 0.5);
  else if (_beepType == 2) BeepHelper.playBeep(880,  150, 0.5);
  else if (_beepType == 3) BeepHelper.playSequence(880, 100, 1100, 120, 0.6);
  else                     BeepHelper.playBeep(_beepFreq, _beepDur, _beepVol);
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

  // ── Readout combinado: entero=elevación, decimal=yaw ────────────────
  if (grabando) {
    int   elevInt  = (int)liveElev;
    float yawAbs   = abs(liveYaw) % 1.0;
    int   yawMil   = (int)(yawAbs * 1000);
    String numStr  = String.format(java.util.Locale.US, "%+d.%03d", elevInt, yawMil);
    String dirStr  = (abs(liveYaw) < 0.001) ? " \u00B7" : (liveYaw > 0 ? " \u25BA" : " \u25C4");
    color rdColor  = (estableRoll && estableYaw) ? color(0, 220, 80) :
                      estableRoll ? COL_ROLL :
                      estableYaw  ? COL_YAW  : COL_TEXT;
    int rdCX = (LISTA_W + width) / 2;
    int rdCY = HEADER_H / 2;
    textFont(fontMono); textSize(34); textAlign(CENTER, CENTER);
    fill(rdColor);
    text(numStr + dirStr, rdCX, rdCY);
  }

  // ── Temporizador inter-disparos ─────────────────────────────────────
  String timerStr   = "00:00";
  color  timerColor = COL_TEXTDIM;
  if (timerActivo) {
    long totalMs  = millis() - timerInicioMs;
    int  totalSeg = (int)(totalMs / 1000);
    int  tMin     = totalSeg / 60;
    int  tSeg     = totalSeg % 60;
    timerStr  = String.format("%02d:%02d", tMin, tSeg);
    timerColor = armado ? color(180, 220, 255) : color(255, 220, 50);
  }
  int tmW = 120, tmH = 24, tmX = 10, tmY = HEADER_H - tmH - 6;
  fill(30); stroke(timerActivo ? (armado ? color(180, 220, 255) : color(255, 220, 50)) : color(60));
  strokeWeight(1); rect(tmX, tmY, tmW, tmH, 5);
  fill(timerColor); textFont(fontMono); textSize(17); textAlign(LEFT, CENTER);
  text("\u23F1 " + timerStr, tmX + 8, tmY + tmH / 2);

  // Botón de prueba de sonido
  int btnSndW = 100, btnSndH = 20;
  int btnSndX = width - btnSndW - 10;
  int btnSndY = HEADER_H - btnSndH - 6;
  boolean hoverSnd = mouseX >= btnSndX && mouseX <= btnSndX + btnSndW &&
                     mouseY >= btnSndY && mouseY <= btnSndY + btnSndH;
  fill(hoverSnd ? color(60, 180, 80) : color(35, 100, 50)); noStroke();
  rect(btnSndX, btnSndY, btnSndW, btnSndH, 4);
  fill(200); textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
  text("♪ Test sonido", btnSndX + btnSndW / 2, btnSndY + btnSndH / 2);
}

// ===================== COMBO DÍA =====================
void dibujarCombo() {
  String etiqueta = dias.size() > 0 ? dias.get(diaSelIdx) : "Sin datos";

  fill(40); stroke(COL_BORDE); strokeWeight(1);
  rect(COMBO_X, COMBO_Y, COMBO_W, COMBO_H, 4);
  fill(COL_TEXT); textFont(fontUI); textSize(14); textAlign(LEFT, CENTER);
  text("📅 " + etiqueta, COMBO_X + 8, COMBO_Y + COMBO_H / 2);
  fill(COL_TEXTDIM); textAlign(RIGHT, CENTER);
  text(comboAbierto ? "▲" : "▼", COMBO_X + COMBO_W - 8, COMBO_Y + COMBO_H / 2);

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
}

// ===================== LISTA SESIONES =====================
void dibujarLista() {
  int lx = 0, ly = HEADER_H;
  int lh = height - HEADER_H;
  int itemH = 44;

  fill(25); stroke(COL_BORDE);
  rect(lx, ly, LISTA_W, lh);

  fill(COL_TEXTDIM); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("SESIONES DEL DÍA", lx + 8, ly + 5);

  int cbX = lx + 8, cbY = ly + 18, cbW = 12, cbH = 12;
  noFill(); stroke(COL_BORDE); rect(cbX, cbY, cbW, cbH, 2);
  if (modoComparar) { fill(COL_SEL); noStroke(); rect(cbX+2, cbY+2, cbW-4, cbH-4, 1); }
  fill(COL_TEXT); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("Comparar", cbX + cbW + 4, cbY);

  int cbX2 = cbX + 80, cbY2 = cbY;
  noFill(); stroke(COL_BORDE); rect(cbX2, cbY2, cbW, cbH, 2);
  if (autoValorTrasDisparo) { fill(color(0,180,80)); noStroke(); rect(cbX2+2, cbY2+2, cbW-4, cbH-4, 1); }
  fill(COL_TEXT); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("Auto-valor", cbX2 + cbW + 4, cbY2);

  int cbX3 = cbX2 + 82, cbY3 = cbY;
  noFill(); stroke(COL_BORDE); rect(cbX3, cbY3, cbW, cbH, 2);
  if (suavizadoActivo) { fill(color(200,130,0)); noStroke(); rect(cbX3+2, cbY3+2, cbW-4, cbH-4, 1); }
  fill(COL_TEXT); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("Suaviz.", cbX3 + cbW + 4, cbY3);

  int cbX4 = cbX3 + 60, cbY4 = cbY;
  noFill(); stroke(calibElevActivo ? color(0,200,80) : COL_BORDE); rect(cbX4, cbY4, cbW, cbH, 2);
  if (calibElevActivo) { fill(color(0,200,80)); noStroke(); rect(cbX4+2, cbY4+2, cbW-4, cbH-4, 1); }
  fill(calibElevActivo ? color(0,220,100) : COL_TEXT); textFont(fontUI); textSize(12); textAlign(LEFT, TOP);
  text("Cal.elev", cbX4 + cbW + 4, cbY4);
  if (calibElevActivo) {
    String calInfo = Float.isNaN(rollRefElev) ? "\u2190 clic en un disparo para calibrar" :
      "ref: " + nf(calHeightMm, 1, 0) + " mm  @  " + nf(distanciaDiana/1000.0, 1, 1) + " m";
    fill(!Float.isNaN(rollRefElev) ? color(0,220,100) : color(200,180,0));
    textFont(fontUI); textSize(11); textAlign(LEFT, TOP);
    text(calInfo, lx + 8, cbY4 + cbH + 3);
  }

  int topBarH = calibElevActivo ? 48 : 34;
  int startY = ly + topBarH;
  int visibles = (lh - topBarH) / itemH;

  for (int i = listaScroll; i < listaScroll + visibles && i < listaEntradas.size(); i++) {
    int iy = startY + (i - listaScroll) * itemH;
    boolean sel = (i == listaSeleccion);

    int slotColor = -1;
    if (modoComparar) {
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (comparaIdx.get(s) == i) { slotColor = s; break; }
      }
    }

    color bgItem;
    if (modoComparar && slotColor >= 0) {
      bgItem = COMP_COLORS[slotColor % 10];
      bgItem = lerpColor(bgItem, color(10), 0.55);
    } else if (!modoComparar && sel) {
      bgItem = COL_SEL;
    } else {
      bgItem = (i % 2 == 0) ? color(32) : color(28);
    }
    fill(bgItem); noStroke();
    rect(lx, iy, LISTA_W, itemH);

    if (modoComparar && slotColor >= 0) {
      fill(COMP_COLORS[slotColor % 10]); noStroke();
      rect(lx, iy, 4, itemH);
    }

    fill(sel || (modoComparar && slotColor >= 0) ? color(255) : COL_TEXTDIM);
    textFont(fontMono); textSize(12); textAlign(LEFT, TOP);
    text(String.format("%02d", i + 1), lx + 8, iy + 4);

    fill(COL_TEXT);
    textFont(fontMono); textSize(13);
    String entrada = listaEntradas.get(i);
    String[] partes = splitTokens(entrada, "  ");
    if (partes.length >= 2) {
      text(partes[0], lx + 28, iy + 4);
      String resto = "";
      for (int k = 1; k < partes.length; k++) resto += "  " + partes[k];
      fill(COL_TEXTDIM); textSize(12);
      text(trim(resto), lx + 28, iy + 17);
    } else {
      text(entrada, lx + 28, iy + 4);
    }

    String val = (i < listaValores.size()) ? listaValores.get(i) : "";
    if (val.length() > 0) {
      fill(color(80,255,160)); textFont(fontMono); textSize(12); textAlign(LEFT, TOP);
      text("🎯 " + val, lx + 28, iy + 30);
    }

    int btnW = 22, btnH = 16;
    int btnDelX = lx + LISTA_W - btnW - 4;
    int btnValX = btnDelX - btnW - 4;
    int btnY    = iy + (itemH - btnH) / 2;

    if (listaDeleteConfirm == i) {
      fill(200, 40, 40); noStroke(); rect(btnValX - 2, btnY, 26, btnH, 3);
      fill(255); textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
      text("OK", btnValX - 2 + 13, btnY + btnH / 2);
      fill(60); noStroke(); rect(btnDelX, btnY, btnW, btnH, 3);
      fill(200); text("NO", btnDelX + btnW / 2, btnY + btnH / 2);
    } else {
      fill(color(40, 80, 50)); noStroke(); rect(btnValX, btnY, btnW, btnH, 3);
      fill(color(100, 220, 130)); textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
      text("✏", btnValX + btnW / 2, btnY + btnH / 2);
      fill(sel ? color(180, 50, 50) : color(70, 35, 35)); noStroke();
      rect(btnDelX, btnY, btnW, btnH, 3);
      fill(sel ? color(255) : color(160)); textFont(fontUI); textSize(14); textAlign(CENTER, CENTER);
      text("✕", btnDelX + btnW / 2, btnY + btnH / 2);
    }

    stroke(COL_BORDE); strokeWeight(1);
    line(lx, iy + itemH - 1, lx + LISTA_W, iy + itemH - 1);
  }

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

  float tMax = 1.0;
  if (grafRoll.size() > 0) tMax = max(tMax, grafRoll.get(grafRoll.size()-1)[0]);
  if (modoComparar) {
    for (int s = 0; s < comparaIdx.size(); s++) {
      CopyOnWriteArrayList<float[]> cr = (CopyOnWriteArrayList<float[]>) compRoll[s];
      if (cr.size() > 0) tMax = max(tMax, cr.get(cr.size()-1)[0]);
    }
  }

  float tVisMin = tZoomMin;
  float tVisMax = (tZoomMax < 0) ? tMax : tZoomMax;
  tVisMax = max(tVisMax, tVisMin + 0.1);

  int gx0c = LISTA_W + 10;
  int gw0c = width - gx0c - 10;
  int axc  = gx0c + GRAF_PAD_L;
  int awc  = gw0c - GRAF_PAD_L - GRAF_PAD_R;
  if (mouseX >= axc && mouseX <= axc + awc) {
    cursorTSeg = map(mouseX, axc, axc + awc, tVisMin, tVisMax);
  } else {
    cursorTSeg = -1;
  }

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
  btnZoomY = 8;
  boolean hoverZoom = mouseX >= btnZoomX && mouseX <= btnZoomX + btnZoomW &&
                      mouseY >= btnZoomY && mouseY <= btnZoomY + btnZoomH;
  fill(hayShot ? (zoomDisparoActivo ? color(60,160,255) : (hoverZoom ? color(255,180,0) : color(200,130,0))) : color(55));
  noStroke(); rect(btnZoomX, btnZoomY, btnZoomW, btnZoomH, 4);
  fill(hayShot ? color(0) : color(100));
  textFont(fontUI); textSize(12); textAlign(CENTER, CENTER);
  text(zoomDisparoActivo ? "\u2316 Ver todo" : "\u2316 Zoom disparo", btnZoomX + btnZoomW / 2, btnZoomY + btnZoomH / 2);

  // ── Botón alinear trazas ─────────────────
  if (modoComparar) {
    btnAlinearW = 130; btnAlinearH = 20;
    btnAlinearX = btnZoomX - btnAlinearW - 8;
    btnAlinearY = btnZoomY;
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

  if (modoDiana) { dibujarDiana(); return; }

  // ── Dibujar gráfica Roll ─────────────────────────────────
  dibujarGrafica(gx0, GRAFICA_Y1 + 30, gw0, GRAFICA_H - 30,
                 "ROLL / Elevación (°)", grafRoll, COL_ROLL,
                 zoomMin[0], zoomMax[0],
                 tVisMin + mainTimeOffset, tVisMax + mainTimeOffset, true, 0);
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
      String[] tok = splitTokens(etiq, "  ");
      text((tok.length > 0 ? tok[0] : etiq) + val, legX + s * 120 + 14, legY);
    }
  }
}

// Media móvil sobre Y para suavizar la curva en pantalla
ArrayList<float[]> suavizar(ArrayList<float[]> pts, int ventana) {
  if (ventana <= 1 || pts.size() < 2) return pts;
  ArrayList<float[]> out = new ArrayList<float[]>();
  int half = ventana / 2;
  for (int i = 0; i < pts.size(); i++) {
    float sy = 0; int cnt = 0;
    for (int j = max(0, i - half); j <= min(pts.size()-1, i + half); j++) {
      sy += pts.get(j)[1]; cnt++;
    }
    out.add(new float[]{ pts.get(i)[0], sy / cnt });
  }
  return out;
}

void dibujarGrafica(int gx, int gy, int gw, int gh,
                    String titulo, List<float[]> datos,
                    color c, float vMin, float vMax,
                    float tVisMin, float tVisMax,
                    boolean showShots, int grafIdx) {
  // La gráfica siempre en grados; mm solo como etiqueta informativa en disparos
  float dispA = 1.0;
  float dispB = 0.0;
  String unidad = "°";

  fill(22); stroke(COL_BORDE); strokeWeight(1);
  rect(gx, gy - GRAF_PAD_T, gw, gh + GRAF_PAD_T + GRAF_PAD_B, 4);

  fill(COL_TEXTDIM); textFont(fontUI); textSize(13); textAlign(LEFT, TOP);
  text(titulo, gx + GRAF_PAD_L, gy - GRAF_PAD_T + 4);

  fill(zoomLocked[grafIdx] ? color(255,180,0) : color(0,200,120));
  textSize(12); textAlign(LEFT, TOP);
  text(zoomLocked[grafIdx] ? "[FIJO]" : "[AUTO]",
       gx + GRAF_PAD_L + 160, gy - GRAF_PAD_T + 4);

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
  if (grafIdx == 0) {
    grafElev_ax = ax; grafElev_ay = ay;
    grafElev_aw = aw; grafElev_ah = ah;
    grafElev_tMin = tVisMin; grafElev_tMax = tVisMax;
  }

  fill(15); noStroke();
  rect(ax, ay, aw, ah);

  float rango = vMax - vMin;
  int nGrid = rango < 0.01 ? 8 : rango < 0.1 ? 8 : rango < 1.0 ? 6 : 5;
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

  if (vMin < 0 && vMax > 0) {
    float yZ = ay + ah - map(0, vMin, vMax, 0, ah);
    stroke(COL_CERO); strokeWeight(1);
    line(ax, yZ, ax + aw, yZ);
  }

  float tSpan = tVisMax - tVisMin;
  float pasoT = 1.0;
  float tStart = ceil(tVisMin / pasoT) * pasoT;
  for (float t = tStart; t <= tVisMax + 0.001; t += pasoT) {
    float xG = ax + map(t, tVisMin, tVisMax, 0, aw);
    if (xG < ax || xG > ax + aw) continue;
    stroke(80); strokeWeight(1);
    line(xG, ay, xG, ay + ah);
    fill(210); textFont(fontMono); textSize(11); textAlign(CENTER, TOP);
    text(nf(t, 1, 2) + "s", xG, ay + ah + 2);
  }

  if (showShots) {
    for (float[] sh : grafShots) {
      float xSh  = ax + map(sh[0],               tVisMin, tVisMax, 0, aw);
      float xPre = ax + map(sh[0] - PRESHOT_SEG, tVisMin, tVisMax, 0, aw);
      xPre = max(xPre, ax);
      xSh  = min(xSh,  ax + aw);
      if (xSh < ax || xPre > ax + aw) continue;
      noStroke(); fill(0, 60, 160);
      rect(xPre, ay, xSh - xPre, ah);
    }
  }

  if (datos.size() < 2) return;
  ArrayList<float[]> ptsVis = new ArrayList<float[]>();
  for (float[] punto : datos) {
    if (punto[0] < tVisMin - pasoT || punto[0] > tVisMax + pasoT) continue;
    float xP = ax + map(punto[0], tVisMin, tVisMax, 0, aw);
    float yP = ay + ah - map(constrain(punto[1] * dispA + dispB, vMin, vMax), vMin, vMax, 0, ah);
    ptsVis.add(new float[]{ xP, yP });
  }
  ptsVis = suavizar(ptsVis, suavizadoActivo ? SMOOTH_N : 1);
  if (ptsVis.size() >= 2) {
    stroke(c); strokeWeight(1.5); noFill();
    beginShape();
    if (suavizadoActivo) {
      curveVertex(ptsVis.get(0)[0], ptsVis.get(0)[1]);
      for (float[] p : ptsVis) curveVertex(p[0], p[1]);
      curveVertex(ptsVis.get(ptsVis.size()-1)[0], ptsVis.get(ptsVis.size()-1)[1]);
    } else {
      for (float[] p : ptsVis) vertex(p[0], p[1]);
    }
    endShape();
  }

  if (showShots) {
    for (float[] sh : grafShots) {
      float xSh = ax + map(sh[0], tVisMin, tVisMax, 0, aw);
      if (xSh < ax || xSh > ax + aw) continue;
      stroke(COL_SHOT); strokeWeight(2);
      line(xSh, ay, xSh, ay + ah);
      fill(COL_SHOT); noStroke();
      triangle(xSh - 5, ay, xSh + 5, ay, xSh, ay + 10);
      // Mostrar volumen si disponible (sh[1] > 0)
      if (sh.length >= 2 && sh[1] > 0) {
        fill(COL_SHOT); textFont(fontMono); textSize(10); textAlign(CENTER, BOTTOM);
        text("vol:" + (int)sh[1], xSh, ay - 1);
      }
    }
  }

  if (showShots) {
    for (float[] sh : grafShots) {
      float xSh = ax + map(sh[0], tVisMin, tVisMax, 0, aw);
      if (xSh < ax || xSh > ax + aw) continue;
      float valSh = Float.NaN;
      float distMin = Float.MAX_VALUE;
      for (float[] punto : datos) {
        float d = abs(punto[0] - sh[0]);
        if (d < distMin) { distMin = d; valSh = punto[1]; }
      }
      if (!Float.isNaN(valSh)) {
        float ySh    = ay + ah - map(constrain(valSh, vMin, vMax), vMin, vMax, 0, ah);
        boolean arriba = (ySh - ay > 16);

        // ── Vector de dirección al disparo ─────────────────────────────
        // Origen: muestra más cercana a (tDisparo - DIANA_PRE_SEG)
        float tVecRef   = sh[0] - DIANA_PRE_SEG;
        float valVecRef = Float.NaN;
        float dMinVec   = Float.MAX_VALUE;
        for (float[] p : datos) {
          float d = abs(p[0] - tVecRef);
          if (d < dMinVec) { dMinVec = d; valVecRef = p[1]; }
        }
        if (!Float.isNaN(valVecRef)) {
          float xVecRef = ax + map(tVecRef, tVisMin, tVisMax, 0, aw);
          float yVecRef = ay + ah - map(constrain(valVecRef, vMin, vMax), vMin, vMax, 0, ah);
          float deltaV  = valSh - valVecRef;
          if (xVecRef >= ax - 1 && xVecRef <= ax + aw + 1) {
            // Punto de referencia (inicio del vector)
            stroke(COL_VECTOR); strokeWeight(1.5);
            fill(COL_VECTOR); ellipse(xVecRef, yVecRef, 7, 7);
            // Línea del vector
            stroke(COL_VECTOR); strokeWeight(2.0);
            line(xVecRef, yVecRef, xSh, ySh);
            // Punta de flecha en el disparo
            float dvx = xSh - xVecRef, dvy = ySh - yVecRef;
            float vlen = sqrt(dvx * dvx + dvy * dvy);
            if (vlen > 6) {
              float nx = dvx / vlen, ny = dvy / vlen;
              float hL = min(10, vlen * 0.45);
              float hx2 = xSh - nx * hL, hy2 = ySh - ny * hL;
              fill(COL_VECTOR); noStroke();
              triangle(xSh, ySh, hx2 + (-ny) * 4, hy2 + nx * 4, hx2 - (-ny) * 4, hy2 - nx * 4);
            }
          }
          // Etiqueta Δ (lado opuesto al valor de ángulo para no solapar)
          String deltaLbl = "\u0394" + (deltaV >= 0 ? "+" : "") + nf(deltaV, 1, 4) + "\u00b0";
          fill(COL_VECTOR); noStroke(); textFont(fontMono); textSize(10);
          textAlign(LEFT, arriba ? TOP : BOTTOM);
          text(deltaLbl, xSh + 7, arriba ? ySh + 13 : ySh - 13);
        }

        // Círculo del disparo (encima del vector)
        fill(COL_SHOT); stroke(255); strokeWeight(1.5);
        ellipse(xSh, ySh, 10, 10);
        // Etiqueta de valor: grados siempre; mm adicional si hay calibración activa
        if (grafIdx == 0 && calibElevActivo && !Float.isNaN(rollRefElev)) {
          float valMm = valSh * elevScale + elevOffset;
          String lblMm = (valMm >= 0 ? "+" : "") + nf(valMm, 1, 1) + " mm  (" + nf(valSh, 1, 4) + "\u00b0)";
          fill(color(0, 220, 100)); noStroke(); textFont(fontMono); textSize(12); textAlign(LEFT, arriba ? BOTTOM : TOP);
          text(lblMm, xSh + 7, arriba ? ySh - 2 : ySh + 2);
        } else {
          String lblDeg = nf(valSh, 1, 4) + "\u00b0";
          fill(255); noStroke(); textFont(fontMono); textSize(11); textAlign(LEFT, arriba ? BOTTOM : TOP);
          text(lblDeg, xSh + 7, arriba ? ySh - 2 : ySh + 2);
        }
      }
    }
  }

  if (cursorTSeg >= tVisMin && cursorTSeg <= tVisMax) {
    float xCur = ax + map(cursorTSeg, tVisMin, tVisMax, 0, aw);
    stroke(200, 200, 200, 160); strokeWeight(1);
    line(xCur, ay, xCur, ay + ah);

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
      String cursorLbl;
      if (grafIdx == 0 && calibElevActivo && !Float.isNaN(rollRefElev)) {
        float valMm = valCur * elevScale + elevOffset;
        cursorLbl = nf(cursorTSeg, 1, 2) + "s  " + nf(valCur, 1, 2) + "°  (" + (valMm >= 0 ? "+" : "") + nf(valMm, 1, 0) + "mm)";
      } else {
        cursorLbl = nf(cursorTSeg, 1, 2) + "s  " + nf(valCur, 1, 4) + "°";
      }
      int cursorBoxW = (grafIdx == 0 && calibElevActivo && !Float.isNaN(rollRefElev)) ? 130 : 70;
      fill(30); stroke(c); strokeWeight(1);
      rect(xCur + 5, yCur - 10, cursorBoxW, 14, 3);
      fill(c); noStroke(); textFont(fontMono); textSize(11); textAlign(LEFT, CENTER);
      text(cursorLbl, xCur + 8, yCur - 3);
    }
  }
}

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

  ArrayList<float[]> ptsExtra = new ArrayList<float[]>();
  for (float[] p : datos) {
    float t = p[0] - tOffset;
    if (t < tVisMin - 1 || t > tVisMax + 1) continue;
    float xP = ax + map(t, tVisMin, tVisMax, 0, aw);
    float yP = ay + ah - map(constrain(p[1], vMin, vMax), vMin, vMax, 0, ah);
    ptsExtra.add(new float[]{ xP, yP });
  }
  ptsExtra = suavizar(ptsExtra, suavizadoActivo ? SMOOTH_N : 1);
  if (ptsExtra.size() >= 2) {
    stroke(c); strokeWeight(1.5); noFill();
    beginShape();
    if (suavizadoActivo) {
      curveVertex(ptsExtra.get(0)[0], ptsExtra.get(0)[1]);
      for (float[] p : ptsExtra) curveVertex(p[0], p[1]);
      curveVertex(ptsExtra.get(ptsExtra.size()-1)[0], ptsExtra.get(ptsExtra.size()-1)[1]);
    } else {
      for (float[] p : ptsExtra) vertex(p[0], p[1]);
    }
    endShape();
  }

  for (float[] sh : shots) {
    float xSh = ax + map(sh[0] - tOffset, tVisMin, tVisMax, 0, aw);
    if (xSh < ax || xSh > ax + aw) continue;
    stroke(c); strokeWeight(1.5);
    line(xSh, ay, xSh, ay + ah);
    fill(c); noStroke();
    triangle(xSh - 4, ay, xSh + 4, ay, xSh, ay + 8);
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

  if (grafShots.size() == 0 || min(grafRoll.size(), grafYaw.size()) == 0) {
    fill(COL_TEXTDIM); textFont(fontUI); textSize(15); textAlign(CENTER, CENTER);
    text("Sin datos de disparo.\nSelecciona una sesión con disparos en la lista.", cx, cy);
    return;
  }

  float tShot = grafShots.get(grafShots.size() - 1)[0];
  float tRef  = tShot - DIANA_PRE_SEG;
  float tSesionInicio = grafRoll.get(0)[0];
  float tPre  = dianaModoZoom
                ? max(tSesionInicio, tRef - DIANA_YELLOW_SEG)
                : max(tSesionInicio, tSesionInicio + DIANA_SKIP_SEG);
  float tPost = tShot + DIANA_POST_SEG;
  float sesionDur = tShot - tPre;

  int nPts = min(grafRoll.size(), grafYaw.size());
  float rollRef = 0, yawRef_d = 0;
  for (int i = 0; i < nPts; i++) {
    if (grafRoll.get(i)[0] >= tRef) { rollRef = grafRoll.get(i)[1]; yawRef_d = grafYaw.get(i)[1]; break; }
  }

  float rollShot = rollRef, yawShot = yawRef_d;
  float bestD = Float.MAX_VALUE;
  for (float[] p : grafRoll) { float d = abs(p[0] - tShot); if (d < bestD) { bestD = d; rollShot = p[1]; } }
  bestD = Float.MAX_VALUE;
  for (float[] p : grafYaw)  { float d = abs(p[0] - tShot); if (d < bestD) { bestD = d; yawShot  = p[1]; } }

  // Desplazamiento del disparo respecto al punto de referencia (tRef)
  float drShot = rollShot - rollRef;
  float dyShot = yawShot  - yawRef_d;

  ArrayList<float[]> ventana = new ArrayList<float[]>();
  for (int i = 0; i < nPts; i++) {
    float t  = grafRoll.get(i)[0];
    if (t < tPre || t > tPost) continue;
    float dr = grafRoll.get(i)[1] - rollRef;
    float dy = grafYaw.get(i)[1]  - yawRef_d;
    ventana.add(new float[]{ t, dr, dy });
  }

  // Ángulo físico que subtiende la mitad de la diana a la distancia configurada
  float dianaHalfDeg = (float)(Math.atan(DIANA_TAMANO_MM / 2.0 / distanciaDiana) * 180.0 / Math.PI);

  // Escala ADAPTATIVA: abarca todo el recorrido de la ventana + punto de disparo.
  // La diana física se dibuja como cuadrado de referencia superpuesto.
  float maxAbsDr = max(0.005f, abs(drShot));
  float maxAbsDy = max(0.005f, abs(dyShot));
  for (float[] p : ventana) {
    maxAbsDr = max(maxAbsDr, abs(p[1]));
    maxAbsDy = max(maxAbsDy, abs(p[2]));
  }
  // maxDev = el mayor de los dos ejes + 25 % de margen;
  // mínimo: la mitad de la diana física para que el cuadrado siempre sea visible
  float maxDev = max(max(maxAbsDr, maxAbsDy) * 1.25f, dianaHalfDeg * 0.5f);

  float sxShot = cx + map(dyShot, -maxDev, maxDev, -r, r);
  float syShot = cy - map(drShot, -maxDev, maxDev, -r, r);

  // ── Reproducción animada: cortar la traza en el tiempo actual ──────────
  float tCutoff = tPost;
  if (dianaPlayActive) {
    float elapsed = (millis() - dianaPlayStartMs) / 1000.0 * dianaPlaySpeed;
    tCutoff = tPre + elapsed;
    if (tCutoff >= tPost) { tCutoff = tPost; dianaPlayActive = false; }
  }
  ArrayList<float[]> ventanaDraw = new ArrayList<float[]>();
  for (float[] p : ventana) { if (p[0] <= tCutoff) ventanaDraw.add(p); else break; }
  boolean mostrarDisparo = (tCutoff >= tShot);

  int nRings = 5;
  color[] ringCols = {
    color(180, 20, 20),
    color(150, 22, 22),
    color(30,  30, 52),
    color(24,  24, 42),
    color(18,  18, 32)
  };
  for (int i = nRings; i >= 1; i--) {
    float ri   = r * (float) i / nRings;
    float devI = maxDev * i / nRings;
    float mmI  = (float)(Math.tan(devI * Math.PI / 180.0) * distanciaDiana);
    fill(ringCols[i - 1]); stroke(80); strokeWeight(1);
    ellipse(cx, cy, ri * 2, ri * 2);
    fill(140); textFont(fontMono); textSize(10); textAlign(LEFT, CENTER);
    text(nf(mmI, 1, 0) + " mm", cx + ri + 4, cy);
    fill(120); textFont(fontMono); textSize(10); textAlign(CENTER, CENTER);
    text(str(nRings + 5 - i), cx, cy - ri + 8);
  }
  // Borde real de la diana cuadrada (DIANA_TAMANO_MM × DIANA_TAMANO_MM)
  float dianaSqPx = r * (dianaHalfDeg / maxDev);
  stroke(color(255, 210, 40)); strokeWeight(1.5); noFill();
  rect(cx - dianaSqPx, cy - dianaSqPx, dianaSqPx * 2, dianaSqPx * 2, 2);
  fill(color(255, 210, 40)); textFont(fontMono); textSize(9); textAlign(LEFT, TOP);
  text(nf(DIANA_TAMANO_MM, 1, 0) + "\u00d7" + nf(DIANA_TAMANO_MM, 1, 0) + " mm",
       cx + dianaSqPx + 4, cy - dianaSqPx);

  fill(255); noStroke(); ellipse(cx, cy, 5, 5);

  stroke(80, 80, 80, 160); strokeWeight(1);
  line(gx + 8, cy, gx + gw - 8, cy);
  line(cx, gy + 8, cx, gy + gh - 8);

  if (ventanaDraw.size() >= 2) {
    // ── Tramo amarillo: desde tPre hasta tRef ──────────────────────────
    ArrayList<float[]> ptsYellow = new ArrayList<float[]>();
    for (float[] p : ventanaDraw) {
      if (p[0] >= tRef) break;
      ptsYellow.add(new float[]{ cx + map(p[2], -maxDev, maxDev, -r, r),
                                 cy - map(p[1], -maxDev, maxDev, -r, r) });
    }
    for (float[] p : ventanaDraw) {  // punto de unión con el tramo azul
      if (p[0] >= tRef) {
        ptsYellow.add(new float[]{ cx + map(p[2], -maxDev, maxDev, -r, r),
                                   cy - map(p[1], -maxDev, maxDev, -r, r) });
        break;
      }
    }
    ptsYellow = suavizar(ptsYellow, suavizadoActivo ? SMOOTH_N : 1);
    if (ptsYellow.size() >= 2) {
      stroke(DIANA_COL_YELLOW); strokeWeight(1.5); noFill();
      beginShape();
      if (suavizadoActivo) {
        curveVertex(ptsYellow.get(0)[0], ptsYellow.get(0)[1]);
        for (float[] p : ptsYellow) curveVertex(p[0], p[1]);
        curveVertex(ptsYellow.get(ptsYellow.size()-1)[0], ptsYellow.get(ptsYellow.size()-1)[1]);
      } else {
        for (float[] p : ptsYellow) vertex(p[0], p[1]);
      }
      endShape();
    }

    // ── Tramo azul: desde tRef hasta tShot ─────────────────────────────
    ArrayList<float[]> ptsPre = new ArrayList<float[]>();
    for (float[] p : ventanaDraw) {
      if (p[0] < tRef) continue;
      if (p[0] > tShot) break;
      ptsPre.add(new float[]{ cx + map(p[2], -maxDev, maxDev, -r, r),
                              cy - map(p[1], -maxDev, maxDev, -r, r) });
    }
    if (mostrarDisparo) ptsPre.add(new float[]{ sxShot, syShot });
    ptsPre = suavizar(ptsPre, suavizadoActivo ? SMOOTH_N : 1);
    if (ptsPre.size() >= 2) {
      stroke(DIANA_COL_PRE); strokeWeight(2.5); noFill();
      beginShape();
      if (suavizadoActivo) {
        curveVertex(ptsPre.get(0)[0], ptsPre.get(0)[1]);
        for (float[] p : ptsPre) curveVertex(p[0], p[1]);
        curveVertex(ptsPre.get(ptsPre.size()-1)[0], ptsPre.get(ptsPre.size()-1)[1]);
      } else {
        for (float[] p : ptsPre) vertex(p[0], p[1]);
      }
      endShape();
    }

    // ── Tramo verde: post-disparo ───────────────────────────────────────
    ArrayList<float[]> ptsPost = new ArrayList<float[]>();
    if (mostrarDisparo) ptsPost.add(new float[]{ sxShot, syShot });
    for (float[] p : ventanaDraw) {
      if (p[0] <= tShot) continue;
      ptsPost.add(new float[]{ cx + map(p[2], -maxDev, maxDev, -r, r),
                               cy - map(p[1], -maxDev, maxDev, -r, r) });
    }
    ptsPost = suavizar(ptsPost, suavizadoActivo ? SMOOTH_N : 1);
    if (ptsPost.size() >= 2) {
      stroke(DIANA_COL_POST); strokeWeight(1.5); noFill();
      beginShape();
      if (suavizadoActivo) {
        curveVertex(ptsPost.get(0)[0], ptsPost.get(0)[1]);
        for (float[] p : ptsPost) curveVertex(p[0], p[1]);
        curveVertex(ptsPost.get(ptsPost.size()-1)[0], ptsPost.get(ptsPost.size()-1)[1]);
      } else {
        for (float[] p : ptsPost) vertex(p[0], p[1]);
      }
      endShape();
    }

    for (float[] p : ventanaDraw) {
      if (abs(p[0] - tShot) < 0.003) continue;
      float sx = cx + map(p[2], -maxDev, maxDev, -r, r);
      float sy = cy - map(p[1], -maxDev, maxDev, -r, r);
      fill(p[0] < tRef ? DIANA_COL_YELLOW : (p[0] < tShot ? DIANA_COL_PRE : DIANA_COL_POST)); noStroke();
      ellipse(sx, sy, 4, 4);
    }

    if (mostrarDisparo) {
      float arrowX = cx, arrowY = cy;
      for (float[] p : ventanaDraw) {
        if (p[0] >= tShot) break;
        arrowX = cx + map(p[2], -maxDev, maxDev, -r, r);
        arrowY = cy - map(p[1], -maxDev, maxDev, -r, r);
      }
      float adx = sxShot - arrowX, ady = syShot - arrowY;
      float alen = sqrt(adx * adx + ady * ady);
      if (alen > 6) {
        float nx = adx / alen, ny = ady / alen;
        float headLen = min(14, alen * 0.5);
        float hx2 = sxShot - nx * headLen, hy2 = syShot - ny * headLen;
        fill(DIANA_COL_PRE); noStroke();
        triangle(sxShot, syShot, hx2 + (-ny) * 4, hy2 + nx * 4, hx2 - (-ny) * 4, hy2 - nx * 4);
      }
    }
    // Cursor animado (posición actual en reproducción)
    if (dianaPlayActive && ventanaDraw.size() > 0) {
      float[] curPt = ventanaDraw.get(ventanaDraw.size() - 1);
      float curX = cx + map(curPt[2], -maxDev, maxDev, -r, r);
      float curY = cy - map(curPt[1], -maxDev, maxDev, -r, r);
      stroke(255, 240, 80); strokeWeight(2.5);
      fill(255, 240, 80, 180); ellipse(curX, curY, 13, 13);
      fill(255); noStroke(); ellipse(curX, curY, 5, 5);
    }
  }

  fill(255, 255, 255, 180); stroke(200); strokeWeight(1);
  ellipse(cx, cy, 10, 10);
  fill(255); noStroke(); ellipse(cx, cy, 4, 4);

  if (mostrarDisparo) {
    fill(COL_SHOT); stroke(255, 180, 180); strokeWeight(1.5);
    ellipse(sxShot, syShot, 14, 14);
    fill(255); noStroke(); ellipse(sxShot, syShot, 4, 4);
  }

  fill(COL_TEXT); textFont(fontUI); textSize(14); textAlign(CENTER, TOP);
  text("DIANA  \u2014  Trayectoria completa  ( \u2212" + nf(sesionDur, 1, 1) + " s \u2192 \u2212" + nf(DIANA_PRE_SEG * 1000, 1, 0) + " ms \u2192 \u25cf \u2192 +" + nf(DIANA_POST_SEG * 1000, 1, 0) + " ms )", cx, gy + 8);

  pushMatrix();
  translate(gx + 22, cy);
  rotate(-HALF_PI);
  fill(COL_ROLL); textFont(fontUI); textSize(11); textAlign(CENTER, CENTER);
  text("ROLL / Elevaci\u00f3n", 0, 0);
  popMatrix();

  fill(COL_YAW); textFont(fontUI); textSize(11); textAlign(CENTER, CENTER);
  text("YAW / Rumbo", cx, gy + gh - 14);
  textSize(10); textAlign(CENTER, CENTER);
  text("\u25b2 +Roll", cx, cy - r - 16);
  text("\u25bc \u2212Roll", cx, cy + r + 16);
  text("\u25c4 \u2212Yaw", gx + 44, cy);
  text("+Yaw \u25ba", gx + gw - 44, cy);

  // Bot\u00f3n zoom: alterna sesi\u00f3n completa / ventana pre-disparo
  int btnZoomDianaW = 140, btnZoomDianaH = 18;
  int btnZoomDianaX = gx + gw - btnZoomDianaW - 10;
  int btnZoomDianaY = gy + 10;
  boolean hoverZD = mouseX >= btnZoomDianaX && mouseX <= btnZoomDianaX + btnZoomDianaW &&
                    mouseY >= btnZoomDianaY && mouseY <= btnZoomDianaY + btnZoomDianaH;
  fill(dianaModoZoom ? color(60, 160, 255) : (hoverZD ? color(80, 80, 120) : color(45)));
  noStroke(); rect(btnZoomDianaX, btnZoomDianaY, btnZoomDianaW, btnZoomDianaH, 4);
  fill(dianaModoZoom ? color(0) : color(180));
  textFont(fontUI); textSize(11); textAlign(CENTER, CENTER);
  text(dianaModoZoom ? "Ventana " + nf(DIANA_YELLOW_SEG, 1, 0) + "s" : "Sesi\u00f3n completa",
       btnZoomDianaX + btnZoomDianaW / 2, btnZoomDianaY + btnZoomDianaH / 2);

  // ── Botones ▶ Play y velocidad ──────────────────────────────────────
  int btnPlayW = 30, btnPlayH = 18;
  int btnSpeedW = 38, btnSpeedH = 18;
  int btnSpeedX = btnZoomDianaX - btnSpeedW - 4;
  int btnPlayX  = btnSpeedX - btnPlayW - 4;
  boolean hoverPlay  = mouseX >= btnPlayX  && mouseX <= btnPlayX  + btnPlayW  && mouseY >= btnZoomDianaY && mouseY <= btnZoomDianaY + btnPlayH;
  boolean hoverSpeed = mouseX >= btnSpeedX && mouseX <= btnSpeedX + btnSpeedW && mouseY >= btnZoomDianaY && mouseY <= btnZoomDianaY + btnSpeedH;
  fill(dianaPlayActive ? color(200, 60, 60) : (hoverPlay ? color(60, 200, 80) : color(30, 100, 40))); noStroke();
  rect(btnPlayX, btnZoomDianaY, btnPlayW, btnPlayH, 4);
  fill(255); textFont(fontUI); textSize(14); textAlign(CENTER, CENTER);
  text(dianaPlayActive ? "\u25a0" : "\u25ba", btnPlayX + btnPlayW / 2, btnZoomDianaY + btnPlayH / 2);
  fill(hoverSpeed ? color(80, 80, 140) : color(45)); noStroke();
  rect(btnSpeedX, btnZoomDianaY, btnSpeedW, btnSpeedH, 4);
  fill(color(160, 160, 220)); textFont(fontMono); textSize(11); textAlign(CENTER, CENTER);
  text(str((int)dianaPlaySpeed) + "x", btnSpeedX + btnSpeedW / 2, btnZoomDianaY + btnSpeedH / 2);

  // Controles skip − / skip Xs / + (solo en modo sesi\u00f3n completa)
  if (!dianaModoZoom) {
    int skipBtnW = 22, skipBtnH = 18, skipLblW = 70;
    int skipTotalW = skipBtnW * 2 + skipLblW + 4;
    int skipX  = btnPlayX - skipTotalW - 6;
    int skipY  = btnZoomDianaY;
    boolean hSkipM = mouseX >= skipX && mouseX <= skipX + skipBtnW && mouseY >= skipY && mouseY <= skipY + skipBtnH;
    fill(hSkipM ? color(180, 60, 60) : color(80, 35, 35)); noStroke();
    rect(skipX, skipY, skipBtnW, skipBtnH, 3);
    fill(200); textFont(fontUI); textSize(14); textAlign(CENTER, CENTER);
    text("\u2212", skipX + skipBtnW / 2, skipY + skipBtnH / 2);
    int skipLblX = skipX + skipBtnW + 2;
    fill(30); stroke(COL_BORDE); strokeWeight(1);
    rect(skipLblX, skipY, skipLblW, skipBtnH, 2);
    fill(DIANA_SKIP_SEG > 0 ? color(255, 180, 30) : COL_TEXTDIM);
    textFont(fontMono); textSize(11); textAlign(CENTER, CENTER);
    text("skip " + nf(DIANA_SKIP_SEG, 1, 1) + "s", skipLblX + skipLblW / 2, skipY + skipBtnH / 2);
    int skipPlusX = skipLblX + skipLblW + 2;
    boolean hSkipP = mouseX >= skipPlusX && mouseX <= skipPlusX + skipBtnW && mouseY >= skipY && mouseY <= skipY + skipBtnH;
    fill(hSkipP ? color(60, 180, 60) : color(30, 80, 30)); noStroke();
    rect(skipPlusX, skipY, skipBtnW, skipBtnH, 3);
    fill(200); textFont(fontUI); textSize(14); textAlign(CENTER, CENTER);
    text("+", skipPlusX + skipBtnW / 2, skipY + skipBtnH / 2);
  }

  int legX = gx + 12;
  int legDY = gy + 30;
  noStroke(); fill(DIANA_COL_YELLOW); rect(legX, legDY - 1, 20, 3, 1);
  fill(DIANA_COL_YELLOW); textFont(fontUI); textSize(12); textAlign(LEFT, CENTER);
  text("Sesi\u00f3n completa (" + nf(sesionDur, 1, 1) + " s)", legX + 26, legDY + 1);
  legDY += 18;
  noStroke(); fill(DIANA_COL_PRE); rect(legX, legDY - 1, 20, 3, 1);
  fill(DIANA_COL_PRE); textAlign(LEFT, CENTER);
  text("Pre-disparo (" + nf(DIANA_PRE_SEG * 1000, 1, 0) + " ms)", legX + 26, legDY + 1);
  legDY += 18;
  noStroke(); fill(DIANA_COL_POST); rect(legX, legDY - 1, 20, 3, 1);
  fill(DIANA_COL_POST); textAlign(LEFT, CENTER);
  text("Post-disparo (" + nf(DIANA_POST_SEG * 1000, 1, 0) + " ms)", legX + 26, legDY + 1);
  legDY += 18;
  fill(COL_SHOT); noStroke(); ellipse(legX + 10, legDY, 10, 10);
  fill(COL_SHOT); textAlign(LEFT, CENTER);
  text("Disparo  \u0394Yaw=" + nf(dyShot, 1, 3) + "\u00b0  \u0394Roll=" + nf(drShot, 1, 3) + "\u00b0", legX + 26, legDY);
  legDY += 18;
  fill(COL_TEXTDIM); textFont(fontMono); textSize(11); textAlign(LEFT, CENTER);
  text("Diana " + nf(DIANA_TAMANO_MM, 1, 0) + "\u00d7" + nf(DIANA_TAMANO_MM, 1, 0) +
       " mm @ " + nf(distanciaDiana / 1000.0, 1, 0) + " m  |  borde \u00b1" +
       nf(dianaHalfDeg, 1, 3) + "\u00b0  |  " + ventana.size() + " muestras  |  t=" + nf(tShot, 1, 3) + "s", legX, legDY);
}

// ===================== INPUT VALOR DE IMPACTO =====================
void dibujarInputValor() {
  fill(0, 0, 0, 160); noStroke();
  rect(0, 0, width, height);

  int bw = 400, bh = 130;
  int bx = (width - bw) / 2, by = (height - bh) / 2;
  fill(35); stroke(COL_BORDE); strokeWeight(1);
  rect(bx, by, bw, bh, 8);

  String sesLabel = (inputParaIdx >= 0 && inputParaIdx < listaEntradas.size())
                    ? listaEntradas.get(inputParaIdx) : "";
  fill(COL_TEXT); textFont(fontUI); textSize(14); textAlign(CENTER, TOP);
  text("🎯 Valor de impacto", bx + bw/2, by + 10);
  fill(COL_TEXTDIM); textSize(12);
  text(sesLabel, bx + bw/2, by + 28);

  fill(20); stroke(COL_YAW); strokeWeight(1);
  rect(bx + 20, by + 48, bw - 40, 26, 4);
  fill(COL_TEXT); textFont(fontMono); textSize(15); textAlign(LEFT, CENTER);
  text(inputValor + "|", bx + 26, by + 61);

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
void applyZoomDisparo() {
  if (grafShots.size() == 0) return;
  float tShot  = grafShots.get(grafShots.size() - 1)[0];
  float margen = 0.20;
  tZoomMin = max(0, tShot - margen);
  tZoomMax = tShot + margen;

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

  float halfY = 0.10;
  zoomMin[0] = rollAt - halfY;  zoomMax[0] = rollAt + halfY;
  zoomMin[1] = yawAt  - halfY;  zoomMax[1] = yawAt  + halfY;
  zoomLocked[0] = true;
  zoomLocked[1] = true;
  zoomDisparoActivo = true;
}

// Recarga los disparos de la sesión/reproducción actual aplicando el filtro SHOT_VOL_MIN.
// Para la sesión activa: rebarre el bufferSesion. Para reproducción: recarga el fichero.
void recargarConFiltro() {
  if (grabando) {
    // Refiltra grafShots desde el bufferSesion en memoria
    // El campo shot es siempre el último de cada fila (compatible con formato 8 y 9 cols)
    grafShots.clear();
    shotCount = 0;
    for (String[] fila : bufferSesion) {
      if (fila.length < 8) continue;
      int vol = 0;
      try { vol = Integer.parseInt(fila[fila.length - 1].trim()); } catch (Exception e) {}
      if (vol > 0 && vol >= SHOT_VOL_MIN) {
        float tSeg = Float.parseFloat(fila[0].trim());
        grafShots.add(new float[]{ tSeg, (float)vol });
        shotCount++;
      }
    }
  } else if (listaSeleccion >= 0 && listaSeleccion < listaRutas.size()) {
    // Recargar reproducción con el nuevo filtro (también recalcula rollRefElev)
    cargarReproduccion(listaRutas.get(listaSeleccion));
    return;
  }
  // Calibración se mantiene del clic previo del usuario
  updateElevCalib();
  // Recargar slots de comparación
  if (modoComparar) {
    for (int s = 0; s < comparaIdx.size(); s++) {
      int ci = comparaIdx.get(s);
      if (ci < listaRutas.size()) cargarEnSlot(s, listaRutas.get(ci));
    }
  }
}

void dibujarInfoSesion() {
  int ix = COMBO_X + COMBO_W + 10;
  int iy = 8;
  int ih = 20;
  int primerBoton = modoComparar ? min(btnDianaX, btnAlinearX) : btnDianaX;

  // ── Panel filtro de volumen (pegado a los botones de la derecha) ──────
  int volPanelW = 160;
  int volPanelX = primerBoton - volPanelW - 8;
  int btnVolH   = ih;
  int btnVolW   = 20;

  // Botón −
  int btnMinusX = volPanelX;
  boolean hoverMinus = mouseX >= btnMinusX && mouseX <= btnMinusX + btnVolW &&
                       mouseY >= iy && mouseY <= iy + btnVolH;
  fill(hoverMinus ? color(180, 60, 60) : color(80, 35, 35)); noStroke();
  rect(btnMinusX, iy, btnVolW, btnVolH, 3);
  fill(200); textFont(fontUI); textSize(14); textAlign(CENTER, CENTER);
  text("−", btnMinusX + btnVolW / 2, iy + btnVolH / 2);

  // Texto volumen
  int volLblX = btnMinusX + btnVolW + 2;
  int volLblW = volPanelW - btnVolW * 2 - 4;
  fill(30); stroke(COL_BORDE); strokeWeight(1);
  rect(volLblX, iy, volLblW, btnVolH, 2);
  fill(SHOT_VOL_MIN > 0 ? color(255, 180, 30) : COL_TEXTDIM);
  textFont(fontMono); textSize(12); textAlign(CENTER, CENTER);
  text("vol\u2265" + SHOT_VOL_MIN, volLblX + volLblW / 2, iy + btnVolH / 2);

  // Botón +
  int btnPlusX = volLblX + volLblW + 2;
  boolean hoverPlus = mouseX >= btnPlusX && mouseX <= btnPlusX + btnVolW &&
                      mouseY >= iy && mouseY <= iy + btnVolH;
  fill(hoverPlus ? color(60, 180, 60) : color(30, 80, 30)); noStroke();
  rect(btnPlusX, iy, btnVolW, btnVolH, 3);
  fill(200); textFont(fontUI); textSize(14); textAlign(CENTER, CENTER);
  text("+", btnPlusX + btnVolW / 2, iy + btnVolH / 2);

  // ── Barra de info principal ───────────────────────────────────────────
  int iw = volPanelX - ix - 8;
  fill(35); stroke(COL_BORDE); strokeWeight(1);
  rect(ix, iy, iw, ih, 4);

  String info;
  if (grabando) {
    float dur = (millis() - tInicioSesion) / 1000.0;
    // Solo hora (sin fecha: ya visible en el combo); sin "Tramas" para ahorrar espacio
    String soloHora = fechaHoraInicio.contains(" ") ?
                      fechaHoraInicio.substring(fechaHoraInicio.indexOf(' ') + 1) :
                      fechaHoraInicio;
    info = "● REC  " + soloHora +
           "  |  Disparos: " + shotCount +
           "  |  Dur: " + nf(dur, 1, 1) + "s";
    fill(COL_ARMED);
  } else if (repTitulo.length() > 0) {
    // Quitar "Fecha:YYYY-MM-DD HH:MM:SS " (ya está en el combo) para ganar espacio
    info = "▶ " + repTitulo.replaceAll("Fecha:\\S+ \\S+ ", "");
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

  // ── Calibración elevación: clic en disparo de la gráfica (solo si aún sin ref) ──
  if (calibElevActivo && Float.isNaN(rollRefElev) && grafShots.size() > 0 &&
      mouseX >= grafElev_ax && mouseX <= grafElev_ax + grafElev_aw &&
      mouseY >= grafElev_ay && mouseY <= grafElev_ay + grafElev_ah) {
    float tClick = map(mouseX, grafElev_ax, grafElev_ax + grafElev_aw, grafElev_tMin, grafElev_tMax);
    float bestD = Float.MAX_VALUE;
    float tNearest = -1;
    for (float[] sh : grafShots) {
      float d = abs(sh[0] - tClick);
      if (d < bestD) { bestD = d; tNearest = sh[0]; }
    }
    if (tNearest >= 0) {
      float refAngle = Float.NaN;
      float angleD   = Float.MAX_VALUE;
      for (float[] p : grafRoll) {
        float d = abs(p[0] - tNearest);
        if (d < angleD) { angleD = d; refAngle = p[1]; }
      }
      if (!Float.isNaN(refAngle)) {
        Object distDef = String.valueOf((int)(distanciaDiana / 1000));
        String inputDist = (String) javax.swing.JOptionPane.showInputDialog(
          null, "Distancia a la diana (m):", "Calibrar elevación (1/2)",
          javax.swing.JOptionPane.QUESTION_MESSAGE, null, null, distDef);
        if (inputDist != null && !inputDist.trim().isEmpty()) {
          String inputH = (String) javax.swing.JOptionPane.showInputDialog(
            null, "Altura del disparo en diana (mm)\n+ = arriba del centro,  \u2212 = abajo",
            "Calibrar elevación (2/2)",
            javax.swing.JOptionPane.QUESTION_MESSAGE, null, null, String.valueOf((int)calHeightMm));
          if (inputH != null && !inputH.trim().isEmpty()) {
            try {
              float newDist = Float.parseFloat(inputDist.trim().replace(",", ".")) * 1000;
              if (newDist > 0) distanciaDiana = newDist;
              calHeightMm = Float.parseFloat(inputH.trim().replace(",", "."));
              rollRefElev = refAngle;
              updateElevCalib();
            } catch (Exception ex) { /* entrada inválida */ }
          }
        }
      }
    }
    return;
  }

  if (mouseX >= COMBO_X && mouseX <= COMBO_X + COMBO_W &&
      mouseY >= COMBO_Y && mouseY <= COMBO_Y + COMBO_H) {
    comboAbierto = !comboAbierto;
    return;
  }

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

  if (mouseX >= btnDianaX && mouseX <= btnDianaX + btnDianaW &&
      mouseY >= btnDianaY && mouseY <= btnDianaY + btnDianaH) {
    modoDiana = !modoDiana;
    if (!modoDiana) dianaPlayActive = false;  // parar reproducción al cerrar
    return;
  }

  // Botón zoom + skip dentro de la diana
  if (modoDiana) {
    int gxD = LISTA_W + 10, gyD = HEADER_H + 10, gwD = width - (LISTA_W + 10) - 10;
    int btnZoomDianaW = 140, btnZoomDianaH = 18;
    int btnZoomDianaX = gxD + gwD - btnZoomDianaW - 10;
    int btnZoomDianaY = gyD + 10;
    if (mouseX >= btnZoomDianaX && mouseX <= btnZoomDianaX + btnZoomDianaW &&
        mouseY >= btnZoomDianaY && mouseY <= btnZoomDianaY + btnZoomDianaH) {
      dianaModoZoom = !dianaModoZoom;
      return;
    }
    // Botones Play y Velocidad
    int _btnSpeedW = 38, _btnPlayW = 30;
    int _btnSpeedX = btnZoomDianaX - _btnSpeedW - 4;
    int _btnPlayX  = _btnSpeedX - _btnPlayW - 4;
    if (mouseX >= _btnPlayX && mouseX <= _btnPlayX + _btnPlayW &&
        mouseY >= btnZoomDianaY && mouseY <= btnZoomDianaY + 18) {
      if (dianaPlayActive) {
        dianaPlayActive = false;
      } else {
        dianaPlayActive = true;
        dianaPlayStartMs = millis();
      }
      return;
    }
    if (mouseX >= _btnSpeedX && mouseX <= _btnSpeedX + _btnSpeedW &&
        mouseY >= btnZoomDianaY && mouseY <= btnZoomDianaY + 18) {
      if      (dianaPlaySpeed == 1.0) dianaPlaySpeed = 2.0;
      else if (dianaPlaySpeed == 2.0) dianaPlaySpeed = 5.0;
      else if (dianaPlaySpeed == 5.0) dianaPlaySpeed = 10.0;
      else                            dianaPlaySpeed = 1.0;
      return;
    }
    if (!dianaModoZoom) {
      int skipBtnW = 22, skipBtnH = 18, skipLblW = 70;
      int skipTotalW_ = skipBtnW * 2 + skipLblW + 4;
      int skipX  = _btnPlayX - skipTotalW_ - 6;
      int skipY  = btnZoomDianaY;
      if (mouseX >= skipX && mouseX <= skipX + skipBtnW && mouseY >= skipY && mouseY <= skipY + skipBtnH) {
        DIANA_SKIP_SEG = max(0.0, DIANA_SKIP_SEG - 0.5);
        return;
      }
      int skipPlusX = skipX + skipBtnW + 2 + skipLblW + 2;
      if (mouseX >= skipPlusX && mouseX <= skipPlusX + skipBtnW && mouseY >= skipY && mouseY <= skipY + skipBtnH) {
        DIANA_SKIP_SEG += 0.5;
        return;
      }
    }
  }

  // ── Botones filtro de volumen (vol− / vol+) ───────────────────────────
  {
    int primerBoton = modoComparar ? min(btnDianaX, btnAlinearX) : btnDianaX;
    int volPanelW = 160;
    int volPanelX = primerBoton - volPanelW - 8;
    int btnVolH = 20, btnVolW = 20, btnIy = 8;
    int btnMinusX = volPanelX;
    int volLblX   = btnMinusX + btnVolW + 2;
    int volLblW   = volPanelW - btnVolW * 2 - 4;
    int btnPlusX  = volLblX + volLblW + 2;
    if (mouseY >= btnIy && mouseY <= btnIy + btnVolH) {
      if (mouseX >= btnMinusX && mouseX <= btnMinusX + btnVolW) {
        SHOT_VOL_MIN = max(0, SHOT_VOL_MIN - 50);
        recargarConFiltro();
        return;
      }
      if (mouseX >= btnPlusX && mouseX <= btnPlusX + btnVolW) {
        SHOT_VOL_MIN += 50;
        recargarConFiltro();
        return;
      }
    }
  }

  int btnSndW = 100, btnSndH = 20;
  int btnSndX = width - btnSndW - 10;
  int btnSndY = HEADER_H - btnSndH - 6;
  if (mouseX >= btnSndX && mouseX <= btnSndX + btnSndW &&
      mouseY >= btnSndY && mouseY <= btnSndY + btnSndH) {
    pitarBeep(880, 200, 0.6);
    return;
  }

  if (mouseX >= btnZoomX && mouseX <= btnZoomX + btnZoomW &&
      mouseY >= btnZoomY && mouseY <= btnZoomY + btnZoomH) {
    if (zoomDisparoActivo) {
      tZoomMin = 0;  tZoomMax = -1;
      zoomMin[0] = -90;  zoomMax[0] = 90;   zoomLocked[0] = false;
      zoomMin[1] =   0;  zoomMax[1] = 360;  zoomLocked[1] = false;
      zoomDisparoActivo = false;
    } else {
      applyZoomDisparo();
    }
    return;
  }

  if (modoComparar &&
      mouseX >= btnAlinearX && mouseX <= btnAlinearX + btnAlinearW &&
      mouseY >= btnAlinearY && mouseY <= btnAlinearY + btnAlinearH) {
    if (modoAlineacion == 1) {
      modoAlineacion = 0;
      mainTimeOffset = 0;
      for (int s = 0; s < 10; s++) compTimeOffset[s] = 0;
    } else {
      float tShotRef = (grafShots.size() > 0) ? grafShots.get(grafShots.size()-1)[0] : 0;
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (compShots[s] != null && compShots[s].size() > 0) {
          float[] sh = (float[]) compShots[s].get(0);
          if (sh[0] > tShotRef) tShotRef = sh[0];
        }
      }
      modoAlineacion = 1;
      mainTimeOffset = (grafShots.size() > 0) ? grafShots.get(grafShots.size()-1)[0] - tShotRef : 0;
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (compShots[s] != null && compShots[s].size() > 0) {
          float[] sh = (float[]) compShots[s].get(0);
          compTimeOffset[s] = sh[0] - tShotRef;
        } else {
          compTimeOffset[s] = 0;
        }
      }
    }
    return;
  }

  if (mouseX < LISTA_W) {
    int ly   = HEADER_H;
    int topBarH = calibElevActivo ? 48 : 34;
    int itemH   = 44;
    int startY  = ly + topBarH;

    int cbX = 8, cbY = ly + 18, cbW = 12, cbH = 12;
    if (mouseX >= cbX && mouseX <= cbX+cbW && mouseY >= cbY && mouseY <= cbY+cbH) {
      modoComparar = !modoComparar;
      if (!modoComparar) {
        comparaIdx.clear();
        for (int s = 0; s < 10; s++) { compRoll[s].clear(); compYaw[s].clear(); compShots[s].clear(); compTimeOffset[s] = 0; }
        modoAlineacion = 0;
        mainTimeOffset = 0;
      }
      return;
    }

    int cbX2 = 8 + 80, cbY2 = ly + 18;
    if (mouseX >= cbX2 && mouseX <= cbX2+cbW && mouseY >= cbY2 && mouseY <= cbY2+cbH) {
      autoValorTrasDisparo = !autoValorTrasDisparo;
      return;
    }

    int cbX3 = cbX2 + 82, cbY3 = ly + 18;
    if (mouseX >= cbX3 && mouseX <= cbX3+cbW && mouseY >= cbY3 && mouseY <= cbY3+cbH) {
      suavizadoActivo = !suavizadoActivo;
      return;
    }

    int cbX4 = cbX3 + 60, cbY4 = ly + 18;
    if (mouseX >= cbX4 && mouseX <= cbX4+cbW && mouseY >= cbY4 && mouseY <= cbY4+cbH) {
      calibElevActivo = !calibElevActivo;
      if (!calibElevActivo) { rollRefElev = Float.NaN; calHeightMm = 0; }
      updateElevCalib();
      return;
    }

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
      if (mouseX >= btnValX - 2 && mouseX <= btnValX - 2 + 26 &&
          mouseY >= btnY && mouseY <= btnY + btnH) {
        borrarSesion(idx);
        listaDeleteConfirm = -1;
        return;
      }
      listaDeleteConfirm = -1;
      return;
    }

    if (mouseX >= btnValX && mouseX <= btnValX + btnW &&
        mouseY >= btnY && mouseY <= btnY + btnH) {
      inputValor   = (idx < listaValores.size()) ? listaValores.get(idx) : "";
      inputParaIdx = idx;
      pedirValor   = true;
      return;
    }

    if (mouseX >= btnDelX && mouseX <= btnDelX + btnW &&
        mouseY >= btnY && mouseY <= btnY + btnH) {
      listaDeleteConfirm = idx;
      return;
    }

    listaDeleteConfirm = -1;
    if (modoComparar) {
      int slotActual = -1;
      for (int s = 0; s < comparaIdx.size(); s++) {
        if (comparaIdx.get(s) == idx) { slotActual = s; break; }
      }
      if (slotActual >= 0) {
        comparaIdx.remove(slotActual);
        compRoll[slotActual].clear();
        compYaw[slotActual].clear();
        compShots[slotActual].clear();
        for (int s = slotActual; s < comparaIdx.size(); s++) {
          compRoll[s].addAll(compRoll[s+1]);
          compYaw[s].addAll(compYaw[s+1]);
          compShots[s].addAll(compShots[s+1]);
          compRoll[s+1].clear(); compYaw[s+1].clear(); compShots[s+1].clear();
        }
      } else if (comparaIdx.size() < 10) {
        int nuevoSlot = comparaIdx.size();
        comparaIdx.add(idx);
        cargarEnSlot(nuevoSlot, listaRutas.get(idx));
      }
    } else {
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

  boolean enGrafica = mouseY > GRAFICA_Y1 + 30;
  if (!enGrafica) return;

  int axc = LISTA_W + 10 + GRAF_PAD_L;
  int awc = width - LISTA_W - 10 - GRAF_PAD_L - GRAF_PAD_R;

  if (mouseX >= axc && mouseX <= axc + awc) {
    float tMax = 1.0;
    if (grafRoll.size() > 0) tMax = grafRoll.get(grafRoll.size()-1)[0];
    float tVisMin = tZoomMin;
    float tVisMax = (tZoomMax < 0) ? tMax : tZoomMax;

    float tCursor = map(mouseX, axc, axc + awc, tVisMin, tVisMax);
    float span    = tVisMax - tVisMin;
    float factor  = e.getCount() > 0 ? 1.20 : 0.80;
    float newSpan = max(span * factor, 0.5);

    float ratioC = (tCursor - tVisMin) / span;
    tZoomMin = tCursor - ratioC * newSpan;
    tZoomMax = tZoomMin + newSpan;

    if (tZoomMin < 0) { tZoomMax -= tZoomMin; tZoomMin = 0; }
    if (tZoomMax > tMax + 1) { tZoomMin -= (tZoomMax - tMax - 1); tZoomMax = tMax + 1; }
    tZoomMin = max(tZoomMin, 0);
  } else {
    float factor = e.getCount() > 0 ? 1.15 : 0.87;

    for (int gi = 0; gi < 2; gi++) {
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
  if (pedirValor) {
    if (key == ENTER || key == RETURN) {
      guardarValorImpacto(inputParaIdx, inputValor.trim());
      pedirValor = false; inputValor = ""; inputParaIdx = -1;
    } else if (key == ESC) {
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
  if (key == 'a' || key == 'A') {
    int grafIdx = (mouseY < GRAFICA_Y2) ? 0 : 1;
    zoomLocked[grafIdx] = !zoomLocked[grafIdx];
  }
  if (key == 'z' || key == 'Z') {
    tZoomMin = 0; tZoomMax = -1;
  }
  if (mouseX > LISTA_W && mouseY > HEADER_H) {
    if (keyCode == UP || keyCode == DOWN) {
      int grafIdx = (mouseY < GRAFICA_Y2) ? 0 : 1;
      float rango = zoomMax[grafIdx] - zoomMin[grafIdx];
      float paso  = rango * 0.10;
      if (paso < 0.001) paso = 0.001;
      float delta = (keyCode == UP) ? paso : -paso;
      zoomMin[grafIdx] += delta;
      zoomMax[grafIdx] += delta;
      zoomLocked[grafIdx] = true;
    }
  }
}
