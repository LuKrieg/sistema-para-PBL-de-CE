/*
 * POWERGuard 12 - Medidor True-RMS / Factor de Potencia y Control Automático
 * Basado en "Prueba de componentes v4.1 (circuito TAL COMO ESTA)"
 *
 * - I2C por software en A1 (SDA) y A2 (SCL), sin librerías externas.
 * - ZMPT101B en A0 del Nano, SCT-013-000 en ADS1115 (A0-A1 diferencial continuo).
 * - 3 pasos / LEDs en D12 (D1), D11 (D2), D10 (D3).
 * - Compensación de fase por buffer circular (phaseDelayUs, calibrable con CAL:x y
 *   guardada en EEPROM) y ventana de muestreo múltiplo exacto de 60 Hz (167 ms = 10 ciclos).
 * - Q con signo (+ inductiva / - capacitiva) usando v retrasado T/4.
 * - Arranca en MANUAL con todos los pasos apagados (START_IN_AUTO en la sección CONFIGURACIÓN).
 *
 * Comandos Serial (115200 baud):
 *   1 / 2 / 3   -> alternar paso/LED D1/D2/D3 (modo MANUAL)
 *   0           -> apagar todos los pasos/LEDs
 *   l           -> repetir prueba visual de LEDs
 *   r           -> cambiar entre 5 y 10 lecturas por segundo
 *   t           -> alternar salida JSON (Dashboard) / Tabla texto (Monitor Serie)
 *   d           -> mostrar / ocultar detalle en mV (en modo tabla texto)
 *   + / -       -> subir / bajar compensación de fase en 50 us
 *   PH:x        -> fijar compensación de fase en us (ej. PH:1700)
 *   m           -> imprimir la MEMORIA DE CÁLCULO completa (fórmulas con números) por Serial
 *   CAL:x       -> calibrar fase con una carga de FP real conocido x
 *                  (CAL:1.00 con foco/plancha, CAL:0.20 con el motor en vacío sin capacitores)
 *   MODE:AUTO   -> control automático (FP entre 0.95 y 0.99)
 *   MODE:MANUAL -> control manual
 *   STEP:i:v    -> activar (v=1) o desactivar (v=0) paso i (0..2)
 */

#include <EEPROM.h>
// ============================ CONFIGURACIÓN (antes config.h) ============================
// ============================================================================
// CONFIGURACIÓN DE HARDWARE - POWERGuard (Equipo 12, base v4.1)
// ============================================================================
// Circuito físico TAL COMO ESTÁ (I2C por software, sin librerías externas):
//   ADS1115  : SDA -> A1, SCL -> A2 (modo continuo A0-A1 diferencial, 860 SPS)
//   ZMPT101B : salida -> A0 del Arduino Nano
//   SCT-013  : ADS1115 A0 - A1 (diferencial) con R4 = 35.5 Ohm
//   3 LEDs / Pasos: D1 = D12, D2 = D11, D3 = D10 (activos en HIGH)
// ============================================================================

#define SDA_PIN  A1
#define SCL_PIN  A2
#define ZMPT_PIN A0

const uint8_t NUM_STEPS = 3;
const uint8_t LED_PIN[NUM_STEPS] = {12, 11, 10}; // D1=D12, D2=D11, D3=D10
const bool RELAY_ACTIVE_LOW = false;              // false: activos en HIGH

// Capacitancias equivalentes de cada paso (uF)
const float STEP_CAPACITANCE_UF[NUM_STEPS] = {20.0f, 30.0f, 80.0f};
const float DISCHARGE_RESISTOR_OHMS = 100000.0f;  // 100 kOhm

// Conversión ADC
const float ADS_LSB_MV         = 0.125f;          // ADS1115 rango +-4.096 V
const float NANO_LSB_MV        = 5000.0f / 1023.0f;
const float UMBRAL_AC_MV       = 150.0f;          // ZMPT pp mínimo "hay AC"
const float UMBRAL_AC_DEBIL_MV = 80.0f;           // ZMPT pp mínimo si freq 50-70 Hz
const float UMBRAL_SCT_MV      = 5.0f;            // SCT pp mínimo "hay corriente"

// Calibración empírica (v4.1)
const float ZMPT_FACTOR   = 0.142f;  // 117.7 V / 828 mV ACrms
const float RELACION_SCT  = 2000.0f; // SCT-013-000: 100 A : 50 mA
const float R_CARGA_OHM   = 35.5f;   // Resistencia R4 medida
const float FACTOR_CORR_I = 2.0f;    // Factor empírico vs pinza amperimétrica

// ---------------------------------------------------------------------------
// Compensación de fase entre v(t) e i(t)
// ---------------------------------------------------------------------------
// La corriente del ADS1115 (modo continuo, 860 SPS) llega "vieja": cada lectura es el
// promedio de la conversión ANTERIOR (~1.16 ms) y además la lectura I2C por software
// tarda ~0.8 ms. Cada muestra de i se empareja con v(t_lectura - phaseDelayUs).
// Estimado: 0.58 ms (centro de la conversión) + 0.58 ms (retardo medio hasta la lectura)
//           + ~0.55 ms (I2C)  ->  ~1700 us  (~37 grados a 60 Hz si NO se compensa).
// Se calibra en vivo y se guarda en EEPROM (no se pierde al reiniciar):
//   CAL:1.00  -> con carga resistiva pura (foco incandescente, plancha): FP real = 1
//   CAL:0.20  -> con el motor en vacío SIN capacitores, si su FP real es 0.20
//   PH:1700   -> fija el retardo a mano (us);  '+' / '-' -> +/- 50 us (1.08 grados)
const int16_t DEFAULT_PHASE_DELAY_US = 1700;
const int16_t PHASE_DELAY_MIN_US     = -1950;
const int16_t PHASE_DELAY_MAX_US     = 4000;
const uint16_t QUARTER_PERIOD_US     = 4167;   // T/4 a 60 Hz (v retrasado 90 grados -> signo de Q)
// Las lecturas no están sincronizadas con las conversiones: el retardo varía 0-1.16 ms
// y eso atenúa P en sin(x)/x = 0.992. Se compensa con esta ganancia.
const float PHASE_JITTER_GAIN        = 1.008f;

// Control del Factor de Potencia
const float FP_TARGET_MIN = 0.95f;
const float FP_TARGET_MAX = 0.99f;
const unsigned long MIN_SWITCH_INTERVAL_MS   = 3000UL;
const bool START_IN_AUTO = false;   // false: arranca en MANUAL con todos los pasos apagados

// El banco son LEDs que SIMULAN los capacitores (no cambian la corriente real).
// true : el firmware resta la Q de los capacitores "conectados" a la Q medida, y reporta
//        Q, S, Irms y FP del sistema compensado (el FP sube al encender LEDs).
// false: usar solo con capacitores reales conectados por relé.
const bool SIMULATED_BANK = true;

// Bloqueo de reconexión por descarga RC: se espera solo hasta que el capacitor
// baje de LINE_PEAK_V a DISCHARGE_SAFE_V:  t = R*C*ln(Vpico / Vseguro)
// Con 100 kOhm: 20 uF -> 2.4 s, 30 uF -> 3.7 s, 80 uF -> 9.8 s (antes 5*tau: 10 / 15 / 40 s)
const float LINE_PEAK_V      = 169.7f;  // 120 V * raiz(2)
const float DISCHARGE_SAFE_V = 50.0f;   // voltaje seguro residual
const unsigned long MIN_DISCHARGE_LOCKOUT_MS = 2000UL;

// Comunicación Serial
const unsigned long SERIAL_BAUD_RATE = 115200UL;
// =========================================================================================

uint8_t lecturasPorSeg = 5;   // 5 o 10 (comando 'r')
uint8_t adsAddr = 0;          // 0 = no encontrado
bool ledEstado[NUM_STEPS] = {false, false, false};
unsigned long lastDisconnectMs[NUM_STEPS] = {0, 0, 0};
unsigned long dischargeLockoutMs[NUM_STEPS] = {0, 0, 0};
unsigned long lastSwitchMs = 0;

bool autoMode = START_IN_AUTO;
bool awaitingStepCheck = false;
int8_t lastAddedStep = -1;
float fpBeforeStep = 0.0f;
const char* systemStatus = "INIT";

bool jsonMode = true;         // true = JSON para Dashboard, false = tabla v4.1
bool detalle = false;         // comando 'd'
float mCuentas = 512.0f;      // media del ZMPT (en cuentas) de la ventana anterior
int16_t phaseDelayUs = DEFAULT_PHASE_DELAY_US;
uint32_t numero = 0;

char serialCmdBuf[32];
uint8_t serialCmdLen = 0;

// ---------- Calibración de fase guardada en EEPROM ----------
const uint16_t EE_MAGIC = 0x5047;   // "PG"
const int EE_ADDR_MAGIC = 0;
const int EE_ADDR_PHASE = 2;
const int EE_ADDR_POL   = 4;
int8_t polaridadI = 1;              // +1 / -1 según cómo esté puesto el SCT (lo decide CAL)
float lastThetaRawDeg = 0.0f;       // ángulo v-i medido en la última ventana (con el retardo actual)
bool lastThetaValid = false;
const char* calMsg = "";            // resultado del último CAL (se envía una sola vez)

void cargarFase() {
  uint16_t magic = 0;
  EEPROM.get(EE_ADDR_MAGIC, magic);
  if (magic == EE_MAGIC) {
    int16_t d;
    int8_t pol;
    EEPROM.get(EE_ADDR_PHASE, d);
    EEPROM.get(EE_ADDR_POL, pol);
    if (d >= PHASE_DELAY_MIN_US && d <= PHASE_DELAY_MAX_US) phaseDelayUs = d;
    if (pol == 1 || pol == -1) polaridadI = pol;
  }
}

void fijarFase(long us) {
  if (us < PHASE_DELAY_MIN_US) us = PHASE_DELAY_MIN_US;
  if (us > PHASE_DELAY_MAX_US) us = PHASE_DELAY_MAX_US;
  phaseDelayUs = (int16_t)us;
  EEPROM.put(EE_ADDR_MAGIC, EE_MAGIC);
  EEPROM.put(EE_ADDR_PHASE, phaseDelayUs);
  EEPROM.put(EE_ADDR_POL, polaridadI);
}

// Lleva un ángulo a (-180, 180]
float envolverGrados(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

// Buffer circular de últimas lecturas de ZMPT para alinear fase con ADS1115.
// Debe cubrir phaseDelayUs + T/4 (~6 ms) para obtener también v retrasado 90 grados.
const uint8_t V_RING_SIZE = 64;
uint16_t vRingVal[V_RING_SIZE];
uint32_t vRingTime[V_RING_SIZE];
uint8_t vRingHead = 0;
uint8_t vRingCount = 0;

inline void pushVoltageSample(uint16_t val, uint32_t tUs) {
  vRingVal[vRingHead] = val;
  vRingTime[vRingHead] = tUs;
  vRingHead = (vRingHead + 1) % V_RING_SIZE;
  if (vRingCount < V_RING_SIZE) vRingCount++;
}

// Interpola v(tTarget) dentro del buffer circular
float lookupVoltageAt(uint32_t tTargetUs, uint16_t fallbackVal) {
  if (vRingCount == 0) return (float)fallbackVal;
  if (vRingCount == 1) return (float)vRingVal[(vRingHead + V_RING_SIZE - 1) % V_RING_SIZE];

  uint8_t newestIdx = (vRingHead + V_RING_SIZE - 1) % V_RING_SIZE;
  if ((int32_t)(tTargetUs - vRingTime[newestIdx]) >= 0) {
    return (float)vRingVal[newestIdx];
  }

  for (uint8_t step = 0; step < vRingCount - 1; step++) {
    uint8_t idxB = (vRingHead + V_RING_SIZE - 1 - step) % V_RING_SIZE;
    uint8_t idxA = (vRingHead + V_RING_SIZE - 2 - step) % V_RING_SIZE;
    uint32_t tA = vRingTime[idxA];
    uint32_t tB = vRingTime[idxB];
    if ((int32_t)(tTargetUs - tA) >= 0 && (int32_t)(tB - tTargetUs) >= 0) {
      uint32_t span = tB - tA;
      if (span == 0) return (float)vRingVal[idxB];
      float alpha = (float)(tTargetUs - tA) / (float)span;
      return (float)vRingVal[idxA] + alpha * ((float)vRingVal[idxB] - (float)vRingVal[idxA]);
    }
  }

  uint8_t oldestIdx = (vRingHead + V_RING_SIZE - vRingCount) % V_RING_SIZE;
  return (float)vRingVal[oldestIdx];
}

// =====================================================================================
//  MEMORIA DE CÁLCULO  (fórmulas de clase: Lecture 16 · Modelos de circuitos AC)
// =====================================================================================
//  Convención: θ = θv − θi.  θ > 0 -> corriente atrasada (carga INDUCTIVA),
//                            θ < 0 -> corriente adelantada (carga CAPACITIVA).
//
//  1) Medición          P = (1/N)·Σ v(t)·i(t)              (potencia promedio)
//                       S = Vrms · Irms
//  2) Triángulo de      FP = P / S = cos θ
//     potencias         θ  = ± arccos(FP)                    (signo por el desfase medido)
//                       Q  = S · sen θ
//  3) Triángulo de      |Z| = Vrms / Irms
//     impedancia        R = |Z| · cos θ ,  X = |Z| · sen θ  ,  FP = R / |Z|
//  4) Admitancia        Y = 1/Z = G − jB ,  G = R/|Z|² ,  B = X/|Z|²
//  5) Capacitor en      Zc = −j/(ωC)  ->  suma +jωC a la admitancia
//     paralelo          Bc = ωC ,  Qc = ωC·V²
//  6) Compensado        Y' = G − j(B − ωC) ;  FP' = G/|Y'| = cos θ'
//                       I' = V·|Y'| ,  S' = V·I' ,  Q' = V²·(B − ωC) = Q − Qc ,  P = V²·G
//  7) Corrección        Qc = P·(tan θ1 − tan θ2) ,  C = Qc / (ωV²)
//     requerida         C ideal (FP = 1): ωC = B  ->  C = B/ω
//  8) Verificación      FP = cos(θv − θi) con el ángulo medido directamente entre v(t) e i(t)
// =====================================================================================
const float OMEGA = 2.0f * PI * 60.0f;   // ω = 2πf = 377 rad/s

struct Calculo {
  bool valido;
  // 1) medición
  float V, Imed, P, Smed, FPmed, thMed;
  // 2) carga sola (motor, sin capacitores)
  float S0, FP0, th0, Q0, I0;
  // 3) impedancia de la carga
  float Z, R, X;
  // 4) admitancia de la carga (siemens)
  float G, B;
  // 5) banco de capacitores conectado
  float C, Bc, Qc;
  // 6) sistema compensado
  float Bn, Yn, FP, th, In, Sn, Qn;
  // 7) corrección requerida
  float tan1, tan2, Qreq, Creq, C1;
  // 8) verificación por ángulo medido
  float thVI, FPvi;
};
Calculo calc;

inline float aGrados(float rad) { return rad * 180.0f / PI; }

// V, I, P medidos; inductiva = signo del desfase; cUf = capacitancia conectada
void calcularTriangulos(float V, float I, float P, bool inductiva, float cUf) {
  Calculo &k = calc;
  k.valido = false;
  if (V <= 0.0f || I <= 0.0f) return;

  // 1) Medición:  S = Vrms·Irms ,  FP = P/S
  k.V = V; k.Imed = I; k.P = P;
  k.Smed = V * I;
  k.FPmed = P / k.Smed;
  if (k.FPmed > 1.0f) k.FPmed = 1.0f;
  float thMedRad = acos(k.FPmed) * (inductiva ? 1.0f : -1.0f);   // θ = ± arccos(FP)
  k.thMed = aGrados(thMedRad);
  float Qmed = k.Smed * sin(thMedRad);                            // Q = S·sen θ

  // 2) Carga sola. Con banco SIMULADO la medición ya es la del motor solo; con banco
  //    real la medición incluye los capacitores y se les quita su Qc = ωCV².
  k.C = cUf;
  k.Bc = OMEGA * cUf * 1e-6f;                                    // Bc = ωC
  k.Qc = k.Bc * V * V;                                           // Qc = ωC·V²
  k.Q0 = SIMULATED_BANK ? Qmed : Qmed + k.Qc;
  k.S0 = sqrt(P * P + k.Q0 * k.Q0);                              // S = √(P² + Q²)
  k.FP0 = P / k.S0;                                              // FP = P/S
  float th0Rad = atan2(k.Q0, P);                                 // θ (tan θ = Q/P)
  k.th0 = aGrados(th0Rad);
  k.I0 = k.S0 / V;                                               // I = S/V

  // 3) Triángulo de impedancia:  |Z| = V/I ,  R = |Z|cos θ ,  X = |Z|sen θ
  k.Z = V / k.I0;
  k.R = k.Z * cos(th0Rad);
  k.X = k.Z * sin(th0Rad);

  // 4) Admitancia:  Y = 1/Z = G − jB
  k.G = k.R / (k.Z * k.Z);
  k.B = k.X / (k.Z * k.Z);

  // 6) Capacitor en paralelo suma +jωC:  Y' = G − j(B − ωC)
  k.Bn = k.B - k.Bc;
  k.Yn = sqrt(k.G * k.G + k.Bn * k.Bn);                          // |Y'|
  k.FP = k.G / k.Yn;                                             // FP' = G/|Y'| = cos θ'
  k.th = aGrados(atan2(k.Bn, k.G));
  k.In = V * k.Yn;                                               // I' = V·|Y'|
  k.Sn = V * k.In;                                               // S' = V·I'
  k.Qn = V * V * k.Bn;                                           // Q' = V²(B − ωC)

  // 7) Corrección requerida para FP_TARGET_MIN:  Qc = P(tan θ1 − tan θ2) ,  C = Qc/(ωV²)
  k.tan1 = k.Q0 / P;
  k.tan2 = tan(acos(FP_TARGET_MIN));
  k.Qreq = P * (k.tan1 - k.tan2);
  k.Creq = k.Qreq / (OMEGA * V * V) * 1e6f;
  k.C1 = k.B / OMEGA * 1e6f;                                     // FP = 1:  C = B/ω

  // 8) Verificación: FP = cos(θv − θi) con el ángulo medido entre v(t) e i(t)
  k.thVI = lastThetaValid ? lastThetaRawDeg : k.thMed;
  k.FPvi = cos(k.thVI * PI / 180.0f);
  k.valido = true;
}

void imprimirMemoria() {
  const Calculo &k = calc;
  if (!k.valido) { Serial.println(F("MEMORIA: sin carga (no hay V e I)")); return; }
  Serial.println(F("=========== MEMORIA DE CALCULO ==========="));
  Serial.print(F("1) Vrms = ")); Serial.print(k.V, 2); Serial.print(F(" V   Irms = ")); Serial.print(k.Imed, 3);
  Serial.print(F(" A   P = (1/N)Sum v*i = ")); Serial.print(k.P, 1); Serial.println(F(" W"));
  Serial.print(F("   S = Vrms*Irms = ")); Serial.print(k.V, 1); Serial.print(F(" * ")); Serial.print(k.Imed, 3);
  Serial.print(F(" = ")); Serial.print(k.Smed, 1); Serial.println(F(" VA"));
  Serial.print(F("2) Carga sola: FP = P/S = ")); Serial.print(k.P, 1); Serial.print('/'); Serial.print(k.S0, 1);
  Serial.print(F(" = ")); Serial.print(k.FP0, 3); Serial.print(F("   theta = arccos(FP) = ")); Serial.print(k.th0, 1); Serial.println(F(" deg"));
  Serial.print(F("   Q = S*sen(theta) = ")); Serial.print(k.Q0, 1); Serial.print(F(" var   I = S/V = ")); Serial.print(k.I0, 3); Serial.println(F(" A"));
  Serial.print(F("3) |Z| = V/I = ")); Serial.print(k.Z, 2); Serial.print(F(" ohm   R = |Z|cos = ")); Serial.print(k.R, 2);
  Serial.print(F("   X = |Z|sen = ")); Serial.print(k.X, 2); Serial.print(F("   R/|Z| = ")); Serial.println(k.R / k.Z, 3);
  Serial.print(F("4) Y = 1/Z = G - jB = ")); Serial.print(k.G * 1000.0f, 2); Serial.print(F(" - j")); Serial.print(k.B * 1000.0f, 2); Serial.println(F(" mS"));
  Serial.print(F("5) C = ")); Serial.print(k.C, 1); Serial.print(F(" uF  wC = 377*C = ")); Serial.print(k.Bc * 1000.0f, 2);
  Serial.print(F(" mS   Qc = wC*V^2 = ")); Serial.print(k.Qc, 1); Serial.println(F(" var"));
  Serial.print(F("6) Y' = G - j(B - wC) = ")); Serial.print(k.G * 1000.0f, 2); Serial.print(F(" - j")); Serial.print(k.Bn * 1000.0f, 2);
  Serial.print(F(" mS  FP' = G/|Y'| = ")); Serial.print(k.FP, 3); Serial.print(k.Bn >= 0 ? F(" (atraso)") : F(" (adelanto)"));
  Serial.print(F("  I' = V|Y'| = ")); Serial.print(k.In, 3); Serial.print(F(" A  Q' = Q - Qc = ")); Serial.print(k.Qn, 1); Serial.println(F(" var"));
  Serial.print(F("7) Para FP=")); Serial.print(FP_TARGET_MIN, 2); Serial.print(F(": Qc = P(tan1 - tan2) = ")); Serial.print(k.P, 1);
  Serial.print(F("*(")); Serial.print(k.tan1, 3); Serial.print(F(" - ")); Serial.print(k.tan2, 3); Serial.print(F(") = ")); Serial.print(k.Qreq, 1);
  Serial.print(F(" var  ->  C = Qc/(wV^2) = ")); Serial.print(k.Creq, 1); Serial.print(F(" uF   (FP=1: C = B/w = ")); Serial.print(k.C1, 1); Serial.println(F(" uF)"));
  Serial.print(F("8) Verificacion: FP = cos(theta_v - theta_i) = cos(")); Serial.print(k.thVI, 1); Serial.print(F(" deg) = ")); Serial.println(k.FPvi, 3);
  Serial.println(F("=========================================="));
}

// ---------- Sumas de la ventana de medición ----------
struct Ventana {
  uint16_t nA;                 // muestras de corriente
  double sumA, sqA;            // para Irms
  double sumV, sumVI;          // v alineado con i -> P
  float minA, maxA;
  uint16_t nQ;                 // muestras con v retrasado T/4 disponible
  double sumIq, sumVi, sumViI, sumVq, sumVqI;  // para el ángulo y el signo de Q
};
Ventana w;

void acumularCorriente(float mv, uint32_t tTarget, uint16_t vFallback) {
  float vSync = lookupVoltageAt(tTarget, vFallback);
  w.nA++;
  w.sumA += mv;
  w.sqA += (double)mv * mv;
  w.sumV += vSync;
  w.sumVI += (double)vSync * mv;
  if (mv < w.minA) w.minA = mv;
  if (mv > w.maxA) w.maxA = mv;

  // v retrasado un cuarto de periodo (solo si el buffer ya cubre ese instante)
  uint32_t tQ = tTarget - QUARTER_PERIOD_US;
  if (vRingCount >= 2) {
    uint8_t oldestIdx = (vRingHead + V_RING_SIZE - vRingCount) % V_RING_SIZE;
    if ((int32_t)(tQ - vRingTime[oldestIdx]) >= 0) {
      float vQ = lookupVoltageAt(tQ, vFallback);
      w.nQ++;
      w.sumIq += mv;
      w.sumVi += vSync;
      w.sumViI += (double)vSync * mv;
      w.sumVq += vQ;
      w.sumVqI += (double)vQ * mv;
    }
  }
}

// ---------- I2C por software (solo bajar o soltar la línea) ----------
inline void i2cDelay() { delayMicroseconds(5); }
inline void sdaSoltar() { pinMode(SDA_PIN, INPUT); }
inline void sdaBajar()  { pinMode(SDA_PIN, OUTPUT); }
inline void sclBajar()  { pinMode(SCL_PIN, OUTPUT); i2cDelay(); }
inline void sclSoltar() {
  pinMode(SCL_PIN, INPUT);
  uint16_t n = 0;
  while (!digitalRead(SCL_PIN) && n++ < 500) delayMicroseconds(1);
  i2cDelay();
}

void i2cInicio() { sdaSoltar(); sclSoltar(); sdaBajar(); i2cDelay(); sclBajar(); }
void i2cParada() { sdaBajar(); i2cDelay(); sclSoltar(); sdaSoltar(); i2cDelay(); }

bool i2cEscribir(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x80) sdaSoltar(); else sdaBajar();
    i2cDelay(); sclSoltar(); sclBajar();
    b <<= 1;
  }
  sdaSoltar(); i2cDelay(); sclSoltar();
  bool ack = !digitalRead(SDA_PIN);
  sclBajar();
  return ack;
}

uint8_t i2cLeer(bool ack) {
  uint8_t b = 0;
  sdaSoltar();
  for (uint8_t i = 0; i < 8; i++) {
    sclSoltar();
    b = (b << 1) | (digitalRead(SDA_PIN) ? 1 : 0);
    sclBajar();
  }
  if (ack) sdaBajar(); else sdaSoltar();
  i2cDelay(); sclSoltar(); sclBajar();
  sdaSoltar();
  return b;
}

bool i2cProbar(uint8_t dir) {
  i2cInicio();
  bool ack = i2cEscribir(dir << 1);
  i2cParada();
  return ack;
}

// ---------- ADS1115 en modo continuo (A0-A1 diferencial, +-4.096 V, 860 SPS) ----------
void buscarADS() {
  adsAddr = 0;
  for (uint8_t a = 0x48; a <= 0x4B; a++) {
    if (i2cProbar(a)) { adsAddr = a; return; }
  }
}

bool adsIniciarContinuo() {
  i2cInicio();
  bool ok = i2cEscribir(adsAddr << 1) && i2cEscribir(0x01) &&
            i2cEscribir(0x02) && i2cEscribir(0xE3);
  i2cParada();
  if (!ok) return false;
  i2cInicio();
  ok = i2cEscribir(adsAddr << 1) && i2cEscribir(0x00);
  i2cParada();
  return ok;
}

bool adsLeerContinuo(int16_t &v) {
  i2cInicio();
  if (!i2cEscribir((adsAddr << 1) | 1)) { i2cParada(); return false; }
  uint8_t hi = i2cLeer(true);
  uint8_t lo = i2cLeer(false);
  i2cParada();
  v = (int16_t)(((uint16_t)hi << 8) | lo);
  return true;
}

void iniciarADS() {
  buscarADS();
  if (adsAddr && !adsIniciarContinuo()) adsAddr = 0;
}

// ---------- Control de LEDs / Pasos y bloqueo RC ----------
// Tiempo hasta que el capacitor baja a DISCHARGE_SAFE_V: t = R*C*ln(Vpico / Vseguro)
unsigned long computeDischargeLockoutMs(float capacitanceUf) {
  float tauSeconds = DISCHARGE_RESISTOR_OHMS * (capacitanceUf * 1e-6f);
  float tSeconds = tauSeconds * log(LINE_PEAK_V / DISCHARGE_SAFE_V);
  unsigned long lockout = (unsigned long)(tSeconds * 1000.0f);
  return max(lockout, MIN_DISCHARGE_LOCKOUT_MS);
}

unsigned long getRemainingLockoutMs(uint8_t stepIdx) {
  if (stepIdx >= NUM_STEPS || ledEstado[stepIdx]) return 0;
  if (lastDisconnectMs[stepIdx] == 0) return 0;
  unsigned long elapsed = millis() - lastDisconnectMs[stepIdx];
  if (elapsed >= dischargeLockoutMs[stepIdx]) return 0;
  return dischargeLockoutMs[stepIdx] - elapsed;
}

bool setStepState(uint8_t stepIdx, bool turnOn, bool bypassLockout = false) {
  if (stepIdx >= NUM_STEPS) return false;
  if (ledEstado[stepIdx] == turnOn) return true;
  if (turnOn) {
    if (!bypassLockout && getRemainingLockoutMs(stepIdx) > 0) return false;
    ledEstado[stepIdx] = true;
    digitalWrite(LED_PIN[stepIdx], RELAY_ACTIVE_LOW ? LOW : HIGH);
  } else {
    ledEstado[stepIdx] = false;
    digitalWrite(LED_PIN[stepIdx], RELAY_ACTIVE_LOW ? HIGH : LOW);
    lastDisconnectMs[stepIdx] = millis();
  }
  lastSwitchMs = millis();
  return true;
}

float getTotalConnectedCapacitanceUf() {
  float total = 0.0f;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (ledEstado[i]) total += STEP_CAPACITANCE_UF[i];
  }
  return total;
}

void pruebaLED() {
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    digitalWrite(LED_PIN[i], HIGH);
    delay(400);
    digitalWrite(LED_PIN[i], LOW);
    ledEstado[i] = false;
  }
  for (uint8_t i = 0; i < NUM_STEPS; i++) digitalWrite(LED_PIN[i], HIGH);
  delay(400);
  for (uint8_t i = 0; i < NUM_STEPS; i++) digitalWrite(LED_PIN[i], LOW);
}

// ---------- Lógica de control automático (0.95 - 0.99) ----------
int8_t findNextAvailableStepToAdd() {
  int8_t bestIdx = -1;
  float minCap = 1e9f;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (!ledEstado[i] && getRemainingLockoutMs(i) == 0 && STEP_CAPACITANCE_UF[i] < minCap) {
      minCap = STEP_CAPACITANCE_UF[i];
      bestIdx = (int8_t)i;
    }
  }
  return bestIdx;
}

int8_t findStepToRemove() {
  int8_t bestIdx = -1;
  float minCap = 1e9f;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (ledEstado[i] && STEP_CAPACITANCE_UF[i] < minCap) {
      minCap = STEP_CAPACITANCE_UF[i];
      bestIdx = (int8_t)i;
    }
  }
  return bestIdx;
}

void runAutoControl(bool hayV, bool hayI, float fp) {
  if (!adsAddr) { systemStatus = "ADS1115_ERROR"; return; }
  if (!autoMode) { systemStatus = "MANUAL"; return; }

  if (!hayV || !hayI) {
    bool anyOn = false;
    for (uint8_t i = 0; i < NUM_STEPS; i++) {
      if (ledEstado[i]) { setStepState(i, false); anyOn = true; }
    }
    awaitingStepCheck = false;
    systemStatus = anyOn ? "NO_LOAD_RESET" : "NO_LOAD";
    return;
  }

  if ((millis() - lastSwitchMs) < MIN_SWITCH_INTERVAL_MS) {
    systemStatus = "SETTLING";
    return;
  }

  if (awaitingStepCheck && lastAddedStep >= 0) {
    awaitingStepCheck = false;
    if (fp < fpBeforeStep) {
      setStepState((uint8_t)lastAddedStep, false);
      lastAddedStep = -1;
      systemStatus = "OVERCOMP_REVERT";
      return;
    }
    lastAddedStep = -1;
  }

  if (fp < FP_TARGET_MIN) {
    int8_t nextStep = findNextAvailableStepToAdd();
    if (nextStep >= 0) {
      fpBeforeStep = fp;
      if (setStepState((uint8_t)nextStep, true)) {
        lastAddedStep = nextStep;
        awaitingStepCheck = true;
        systemStatus = "STEP_ADDED";
      }
    } else {
      systemStatus = "MAX_CAP_OR_LOCKED";
    }
  } else if (fp > FP_TARGET_MAX) {
    int8_t remStep = findStepToRemove();
    if (remStep >= 0) {
      setStepState((uint8_t)remStep, false);
      systemStatus = "STEP_REMOVED";
    } else {
      systemStatus = "IN_RANGE";
    }
  } else {
    systemStatus = "IN_RANGE";
  }
}

// ---------- Comandos Serial ----------
void processCommand(const char* cmd) {
  if (strcmp(cmd, "MODE:AUTO") == 0) {
    autoMode = true;
    awaitingStepCheck = false;
  } else if (strcmp(cmd, "MODE:MANUAL") == 0) {
    autoMode = false;
    awaitingStepCheck = false;
  } else if (strcmp(cmd, "0") == 0) {
    autoMode = false;
    for (uint8_t i = 0; i < NUM_STEPS; i++) setStepState(i, false, true);
  } else if (strcmp(cmd, "1") == 0 || strcmp(cmd, "2") == 0 || strcmp(cmd, "3") == 0) {
    uint8_t idx = (uint8_t)(cmd[0] - '1');
    autoMode = false;
    setStepState(idx, !ledEstado[idx], true);
  } else if (strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
    autoMode = false;
    pruebaLED();
  } else if (strcmp(cmd, "r") == 0 || strcmp(cmd, "R") == 0) {
    lecturasPorSeg = (lecturasPorSeg == 5) ? 10 : 5;
  } else if (strcmp(cmd, "d") == 0 || strcmp(cmd, "D") == 0) {
    detalle = !detalle;
  } else if (strcmp(cmd, "t") == 0 || strcmp(cmd, "T") == 0) {
    jsonMode = !jsonMode;
  } else if (strcmp(cmd, "m") == 0 || strcmp(cmd, "M") == 0) {
    imprimirMemoria();
  } else if (strcmp(cmd, "+") == 0) {
    fijarFase((long)phaseDelayUs + 50);
  } else if (strcmp(cmd, "-") == 0) {
    fijarFase((long)phaseDelayUs - 50);
  } else if (strncmp(cmd, "PH:", 3) == 0) {
    fijarFase(atol(cmd + 3));
  } else if (strncmp(cmd, "CAL:", 4) == 0) {
    float fpRef = (float)atof(cmd + 4);
    if (fpRef <= 0.0f || fpRef > 1.0f) {
      calMsg = "CAL_VALOR_INVALIDO";
    } else if (!lastThetaValid) {
      calMsg = "CAL_SIN_CARGA";      // hace falta V e I presentes para calibrar
    } else {
      // Error de ángulo que queda con el retardo actual. Se prueban las dos polaridades
      // del SCT y se elige la de menor error. Luego se pasa a us (1 grado = 46.3 us a 60 Hz).
      float refDeg = acos(fpRef) * 180.0f / PI;
      float errMisma = envolverGrados(lastThetaRawDeg - refDeg);
      float errInvertida = envolverGrados(lastThetaRawDeg + 180.0f - refDeg);
      float errDeg = errMisma;
      if (fabs(errInvertida) < fabs(errMisma)) {
        polaridadI = -polaridadI;
        errDeg = errInvertida;
      }
      fijarFase((long)phaseDelayUs + (long)(errDeg * (1e6f / (360.0f * 60.0f))));
      calMsg = "CAL_OK";
    }
  } else if (strncmp(cmd, "STEP:", 5) == 0) {
    int idx = -1, val = 0;
    if (sscanf(cmd, "STEP:%d:%d", &idx, &val) == 2 && idx >= 0 && idx < NUM_STEPS) {
      autoMode = false;
      setStepState((uint8_t)idx, val == 1);
    }
  }
}

void comandos() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (serialCmdLen == 0 && (c == '0' || c == '1' || c == '2' || c == '3' ||
        c == 'l' || c == 'L' || c == 'r' || c == 'R' || c == 'd' || c == 'D' ||
        c == 't' || c == 'T' || c == '+' || c == '-' || c == 'm' || c == 'M')) {
      char single[2] = {c, '\0'};
      processCommand(single);
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        processCommand(serialCmdBuf);
        serialCmdLen = 0;
      }
    } else if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
      serialCmdBuf[serialCmdLen++] = c;
    }
  }
}

void pf(float v, uint8_t ancho, uint8_t dec) {
  char buf[14];
  dtostrf(v, ancho, dec, buf);
  Serial.print(buf);
}

// ---------- Programa ----------
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    pinMode(LED_PIN[i], OUTPUT);
    digitalWrite(LED_PIN[i], LOW);
    dischargeLockoutMs[i] = computeDischargeLockoutMs(STEP_CAPACITANCE_UF[i]);
  }

  pinMode(SDA_PIN, INPUT); digitalWrite(SDA_PIN, LOW);
  pinMode(SCL_PIN, INPUT); digitalWrite(SCL_PIN, LOW);

  cargarFase();
  iniciarADS();
  // Sin prueba de LEDs al arrancar: el banco inicia SIN capacitores conectados.

  if (!adsAddr) {
    systemStatus = "ADS1115_ERROR";
    Serial.println(F("{\"error\":\"ADS1115 no responde en A1(SDA)/A2(SCL)\"}"));
  } else {
    systemStatus = "READY";
  }
}

void loop() {
  comandos();

  static uint32_t tCiclo = 0;
  uint16_t periodo = 1000 / lecturasPorSeg;
  if (millis() - tCiclo < periodo) return;
  tCiclo = millis();
  numero++;

  if (!adsAddr) iniciarADS();

  // Ventana múltiplo exacto de 60 Hz (167 ms = 10 ciclos, 83 ms = 5 ciclos)
  uint16_t ventana = (lecturasPorSeg == 5) ? 167 : 83;

  vRingHead = 0;
  vRingCount = 0;

  uint32_t nN = 0, sumN = 0, sqN = 0;
  uint16_t minN = 1023, maxN = 0;
  uint8_t cruces = 0;
  uint32_t t1 = 0, t2 = 0;
  bool arriba = (analogRead(ZMPT_PIN) > mCuentas);

  memset(&w, 0, sizeof(w));
  w.minA = 1e9f; w.maxA = -1e9f;
  bool adsFalla = false;
  uint32_t tAds = micros();

  // Soporte para phaseDelayUs negativo (esperar v(t + |delay|))
  bool pendingAds = false;
  float pendingMv = 0.0f;
  uint32_t pendingTargetUs = 0;

  uint32_t t0 = millis();
  while (millis() - t0 < ventana) {
    uint32_t tv = micros();
    uint16_t v = analogRead(ZMPT_PIN);
    pushVoltageSample(v, tv);

    nN++;
    sumN += v;
    sqN += (uint32_t)v * v;
    if (v < minN) minN = v;
    if (v > maxN) maxN = v;
    bool a = arriba;
    if (v > mCuentas + 3) a = true; else if (v < mCuentas - 3) a = false;
    if (a && !arriba) {
      uint32_t t = micros();
      if (cruces == 0) t1 = t;
      t2 = t;
      if (cruces < 250) cruces++;
    }
    arriba = a;

    // Resolver muestra pendiente si phaseDelayUs < 0
    if (pendingAds && (int32_t)(tv - pendingTargetUs) >= 0) {
      acumularCorriente(pendingMv, pendingTargetUs, v);
      pendingAds = false;
    }

    if (adsAddr && !adsFalla && (uint32_t)(micros() - tAds) >= 1200) {
      int16_t raw;
      if (adsLeerContinuo(raw)) {
        uint32_t tRead = micros();
        tAds = tRead;
        float mv = raw * ADS_LSB_MV;

        if (phaseDelayUs >= 0) {
          acumularCorriente(mv, tRead - (uint32_t)phaseDelayUs, v);
        } else {
          pendingMv = mv;
          pendingTargetUs = tRead + (uint32_t)(-phaseDelayUs);
          pendingAds = true;
        }
      } else {
        adsFalla = true;
      }
    }
  }
  if (adsFalla) adsAddr = 0;

  // ---------- ZMPT: resultados ----------
  float mN = (nN > 0) ? ((float)sumN / nN) : 512.0f;
  float varN = (nN > 0) ? ((float)sqN / nN - mN * mN) : 0.0f;
  mCuentas = mN;
  float vMedia = mN * NANO_LSB_MV;
  float vRmsMv = (varN > 0 ? sqrt(varN) : 0.0f) * NANO_LSB_MV;
  float vMin   = minN * NANO_LSB_MV;
  float vMax   = maxN * NANO_LSB_MV;
  float ppV    = vMax - vMin;
  float freq   = (cruces >= 3 && t2 > t1) ? (cruces - 1) * 1e6f / (float)(t2 - t1) : 0.0f;

  bool senalFuerte = (ppV >= UMBRAL_AC_MV);
  bool senalDebil  = (ppV >= UMBRAL_AC_DEBIL_MV && freq >= 50.0f && freq <= 70.0f);
  bool hayV = (senalFuerte || senalDebil) && vMedia > 1500.0f && vMedia < 3500.0f;
  float vrms = hayV ? vRmsMv * ZMPT_FACTOR : 0.0f;

  // ---------- SCT: resultados ----------
  bool adsOk = (w.nA >= 10);
  float iMedia = 0.0f, iRmsMv = 0.0f, ppI = 0.0f;
  if (adsOk) {
    iMedia = (float)(w.sumA / w.nA);
    double varA = w.sqA / w.nA - (double)iMedia * iMedia;
    iRmsMv = (varA > 0) ? sqrt(varA) : 0.0f;
    ppI = w.maxA - w.minA;
  }
  bool hayI = adsOk && ppI >= UMBRAL_SCT_MV;
  float iScale = (1.0f / 1000.0f) / R_CARGA_OHM * RELACION_SCT * FACTOR_CORR_I;
  float irms = hayI ? iRmsMv * iScale : 0.0f;

  // ---------- Potencia activa medida y signo del desfase ----------
  // P = (1/N)·Σ v·i  (covarianza: v e i sin su componente DC), con v alineado por phaseDelayUs.
  // El signo de θ = θv − θi sale de cov(v retrasado T/4, i) ~ sen θ  (+ inductiva, − capacitiva).
  float p = 0.0f, s = vrms * irms, q = 0.0f, fp = 0.0f;
  bool hayCarga = hayV && hayI && adsOk && s >= 1.0f;
  bool inductiva = true;
  lastThetaValid = false;
  calc.valido = false;
  if (hayCarga) {
    float vScale = NANO_LSB_MV * ZMPT_FACTOR;
    double covVI = (w.sumVI / w.nA) - (w.sumV / w.nA) * (double)iMedia;
    float pCalc = fabs((float)(covVI * (double)vScale * (double)iScale)) * PHASE_JITTER_GAIN;
    if (pCalc > s) pCalc = s;
    p = pCalc;

    if (w.nQ >= 10) {
      double mI = w.sumIq / w.nQ;
      double cI = w.sumViI / w.nQ - (w.sumVi / w.nQ) * mI;
      double cQ = w.sumVqI / w.nQ - (w.sumVq / w.nQ) * mI;
      cI *= polaridadI;                       // orientación del SCT (calibrada con CAL)
      cQ *= polaridadI;
      lastThetaRawDeg = (float)(atan2(cQ, cI) * 180.0 / PI);   // θv − θi medido
      lastThetaValid = true;
      inductiva = (cQ >= 0);
    }

    // ---------- Triángulos de potencias, impedancia y admitancia + banco ----------
    calcularTriangulos(vrms, irms, p, inductiva, getTotalConnectedCapacitanceUf());

    // Valores que se reportan = sistema con el banco conectado (paso 6 de la memoria)
    s = calc.Sn;
    irms = calc.In;
    q = calc.Qn;
    fp = calc.FP;
    inductiva = (calc.Bn >= 0.0f);
  }

  runAutoControl(hayV, hayCarga, hayCarga ? fp : 1.0f);

  // ---------- Salida ----------
  if (jsonMode) {
    Serial.print(F("{\"t\":"));       Serial.print(millis());
    Serial.print(F(",\"vrms\":"));   Serial.print(vrms, 2);
    Serial.print(F(",\"irms\":"));   Serial.print(irms, 3);
    Serial.print(F(",\"p\":"));      Serial.print(p, 1);
    Serial.print(F(",\"q\":"));      Serial.print(q, 1);
    Serial.print(F(",\"s\":"));      Serial.print(s, 1);
    Serial.print(F(",\"fp\":"));     Serial.print(fp, 3);
    Serial.print(F(",\"freq\":"));   Serial.print(freq, 1);
    Serial.print(F(",\"phase_us\":")); Serial.print(phaseDelayUs);
    Serial.print(F(",\"carga\":\""));
    Serial.print(!hayCarga ? F("-") : (inductiva ? F("IND") : F("CAP")));
    Serial.print('"');
    if (calMsg[0]) {
      Serial.print(F(",\"cal\":\"")); Serial.print(calMsg); Serial.print('"');
      calMsg = "";
    }
    Serial.print(F(",\"steps\":["));
    for (uint8_t i = 0; i < NUM_STEPS; i++) {
      Serial.print(ledEstado[i] ? 1 : 0);
      if (i < NUM_STEPS - 1) Serial.print(',');
    }
    Serial.print(F("],\"locked\":["));
    for (uint8_t i = 0; i < NUM_STEPS; i++) {
      Serial.print(getRemainingLockoutMs(i));
      if (i < NUM_STEPS - 1) Serial.print(',');
    }
    Serial.print(F("],\"cap_uf\":")); Serial.print(getTotalConnectedCapacitanceUf(), 1);
    if (calc.valido) {
      // Memoria de cálculo (carga sola, impedancia, admitancia en mS, banco y corrección)
      Serial.print(F(",\"mc\":{\"s1\":"));  Serial.print(calc.Smed, 1);
      Serial.print(F(",\"fp1\":"));  Serial.print(calc.FPmed, 3);
      Serial.print(F(",\"p0\":"));   Serial.print(calc.P, 1);
      Serial.print(F(",\"q0\":"));   Serial.print(calc.Q0, 1);
      Serial.print(F(",\"s0\":"));   Serial.print(calc.S0, 1);
      Serial.print(F(",\"i0\":"));   Serial.print(calc.I0, 3);
      Serial.print(F(",\"fp0\":"));  Serial.print(calc.FP0, 3);
      Serial.print(F(",\"th0\":"));  Serial.print(calc.th0, 1);
      Serial.print(F(",\"z\":"));    Serial.print(calc.Z, 2);
      Serial.print(F(",\"r\":"));    Serial.print(calc.R, 2);
      Serial.print(F(",\"x\":"));    Serial.print(calc.X, 2);
      Serial.print(F(",\"g\":"));    Serial.print(calc.G * 1000.0f, 3);
      Serial.print(F(",\"b\":"));    Serial.print(calc.B * 1000.0f, 3);
      Serial.print(F(",\"bc\":"));   Serial.print(calc.Bc * 1000.0f, 3);
      Serial.print(F(",\"qc\":"));   Serial.print(calc.Qc, 1);
      Serial.print(F(",\"bn\":"));   Serial.print(calc.Bn * 1000.0f, 3);
      Serial.print(F(",\"th\":"));   Serial.print(calc.th, 1);
      Serial.print(F(",\"t1\":"));   Serial.print(calc.tan1, 3);
      Serial.print(F(",\"t2\":"));   Serial.print(calc.tan2, 3);
      Serial.print(F(",\"qr\":"));   Serial.print(calc.Qreq, 1);
      Serial.print(F(",\"cr\":"));   Serial.print(calc.Creq, 1);
      Serial.print(F(",\"c1\":"));   Serial.print(calc.C1, 1);
      Serial.print(F(",\"thvi\":")); Serial.print(calc.thVI, 1);
      Serial.print(F(",\"fpvi\":")); Serial.print(calc.FPvi, 3);
      Serial.print('}');
    }
    Serial.print(F(",\"mode\":\""));  Serial.print(autoMode ? F("AUTO") : F("MANUAL"));
    Serial.print(F("\",\"status\":\"")); Serial.print(systemStatus);
    Serial.println(F("\"}"));
  } else {
    Serial.print('#'); pf((float)numero, 5, 0);
    Serial.print(F(" | V ")); pf(vrms, 6, 1); Serial.print(F(" Vrms"));
    Serial.print(F(" | I ")); pf(irms, 5, 2); Serial.print(F(" Arms"));
    Serial.print(F(" | P ")); pf(p, 6, 1); Serial.print(F(" W"));
    Serial.print(F(" | FP ")); pf(fp, 5, 3);
    Serial.print(!hayCarga ? F(" -") : (inductiva ? F(" IND") : F(" CAP")));
    Serial.print(F(" | ph=")); Serial.print(phaseDelayUs); Serial.print(F("us"));
    if (calMsg[0]) { Serial.print(F(" | ")); Serial.print(calMsg); calMsg = ""; }
    Serial.print(F(" | ")); Serial.println(systemStatus);
    if (detalle) {
      imprimirMemoria();
      Serial.print(F("       ZMPT mV: media=")); Serial.print(vMedia, 0);
      Serial.print(F(" pp=")); Serial.print(ppV, 0);
      Serial.print(F(" | SCT dif mV: pp=")); Serial.print(ppI, 0);
      Serial.print(F(" | muestras ZMPT=")); Serial.print(nN);
      Serial.print(F(" SCT=")); Serial.println(w.nA);
    }
  }
}
