/*
 * POWERGuard 12 - Medidor True-RMS / Factor de Potencia y Control Automático
 * Basado en "Prueba de componentes v4.1 (circuito TAL COMO ESTA)"
 *
 * - I2C por software en A1 (SDA) y A2 (SCL), sin librerías externas.
 * - ZMPT101B en A0 del Nano, SCT-013-000 en ADS1115 (A0-A1 diferencial continuo).
 * - 3 pasos / LEDs en D12 (D1), D11 (D2), D10 (D3).
 * - Compensación de fase por buffer circular (phaseDelayUs, calibrable con CAL:1.00 y
 *   guardada en EEPROM) y ventana de muestreo múltiplo exacto de 60 Hz (167 ms = 10 ciclos).
 * - Q estimada por cuadratura (+ inductiva / - capacitiva) con tensión casi sinusoidal.
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
 *   CAL:1.00    -> calibrar fase en manual con carga resistiva y pasos apagados
 *   MODE:AUTO   -> control automático (FP entre 0.95 y 1.00)
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
const float STEP_CAPACITANCE_UF[NUM_STEPS] = {60.0f, 60.0f, 60.0f};
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
//   CAL:1.00  -> con carga resistiva pura en manual, con los pasos apagados
//   PH:1700   -> fija el retardo a mano (us);  '+' / '-' -> +/- 50 us (1.08 grados)
const int16_t DEFAULT_PHASE_DELAY_US = 1700;
const int16_t PHASE_DELAY_MIN_US     = -1950;
const int16_t PHASE_DELAY_MAX_US     = 4000;
const uint16_t QUARTER_PERIOD_US     = 4167;   // T/4 a 60 Hz (v retrasado 90 grados -> signo de Q)

// Control del Factor de Potencia
const float FP_TARGET_MIN = 0.95f;
const float FP_MIN_IMPROVEMENT = 0.005f;
const unsigned long MIN_SWITCH_INTERVAL_MS   = 3000UL;
const bool START_IN_AUTO = false;   // false: arranca en MANUAL con todos los pasos apagados

// El banco son LEDs que SIMULAN los capacitores (no cambian la corriente real).
// true : el firmware resta la Q de los capacitores "conectados" a la Q medida, y reporta
//        Q, S, Irms y FP del sistema compensado (el FP sube al encender LEDs).
// false: usar solo con capacitores reales conectados por relé.
const bool SIMULATED_BANK = true;

// Bloqueo de reconexión por descarga RC: se espera solo hasta que el capacitor
// baje de LINE_PEAK_V a DISCHARGE_SAFE_V:  t = R*C*ln(Vpico / Vseguro)
// Con 100 kOhm y 60 uF: aproximadamente 7.33 s por paso.
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
  lastThetaValid = false;
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
//  FP verdadero = P / (Vrms * Irms). Q se estima por cuadratura.
//  El banco cambia Q, y el modelo conserva el residuo sqrt(S^2 - P^2 - Q^2).
// =====================================================================================
const float OMEGA = 2.0f * PI * 60.0f;   // ω = 2πf = 377 rad/s

struct Calculo {
  bool valido;
  // 1) medición
  float V, Imed, P, Smed, FPmed, Qmed, residual, measuredResidual;
  // 2) carga sola (motor, sin capacitores)
  float S0, FP0, th0, Q0, I0;
  // 3) impedancia de la carga
  float Z, R, X;
  // 4) admitancia de la carga (siemens)
  float G, B;
  // 5) banco de capacitores conectado
  float C, Bc, Qc;
  // 6) sistema compensado
  float Bn, FP, th, In, Sn, Qn;
  // 7) corrección requerida
  float tan2, Qreq, Creq, C1;
  bool correctionReachable;
  // 8) verificación por ángulo medido
  float thVI, FPvi;
};
Calculo calc;

inline float aGrados(float rad) { return rad * 180.0f / PI; }

float apparentPower(float p, float q, float residual) {
  return sqrt(p * p + q * q + residual * residual);
}

// Banco en paralelo con tensión casi sinusoidal. El residuo no activo se conserva.
void calcularTriangulos(float V, float I, float P, float qEstimate, float cUf) {
  Calculo &k = calc;
  k.valido = false;
  if (V <= 0.0f || I <= 0.0f) return;

  // 1) Medición:  S = Vrms·Irms ,  FP = P/S
  k.V = V; k.Imed = I; k.P = P;
  k.Smed = V * I;
  k.FPmed = P / k.Smed;
  float nonactive = sqrt(max(0.0f, (k.Smed - fabsf(P)) * (k.Smed + fabsf(P))));
  k.Qmed = constrain(qEstimate, -nonactive, nonactive);
  k.measuredResidual = sqrt(max(0.0f, (nonactive - fabsf(k.Qmed)) * (nonactive + fabsf(k.Qmed))));
  // En la simulación, el equivalente sinusoidal conserva P y S de entrada.
  // La cuadratura aporta el signo. Su magnitud y el residuo quedan como diagnóstico.
  k.residual = SIMULATED_BANK ? 0.0f : k.measuredResidual;

  // 2) Carga sola. Con banco SIMULADO la medición ya es la del motor solo; con banco
  //    real la medición incluye los capacitores y se les quita su Qc = ωCV².
  k.C = cUf;
  k.Bc = OMEGA * cUf * 1e-6f;                                    // Bc = ωC
  k.Qc = k.Bc * V * V;                                           // Qc = ωC·V²
  k.Q0 = SIMULATED_BANK ? (qEstimate < 0.0f ? -nonactive : nonactive) : k.Qmed + k.Qc;
  k.S0 = apparentPower(P, k.Q0, k.residual);
  k.FP0 = k.S0 > 0.0f ? P / k.S0 : 0.0f;
  float th0Rad = atan2(k.Q0, P);                                 // θ (tan θ = Q/P)
  k.th0 = aGrados(th0Rad);
  k.I0 = k.S0 / V;                                               // I = S/V

  // 3) Impedancia equivalente de terminales, a partir de P + jQ.
  float sPQ = apparentPower(P, k.Q0, 0.0f);
  k.Z = sPQ > 0.0f ? V * V / sPQ : 0.0f;
  k.R = k.Z * cos(th0Rad);
  k.X = k.Z * sin(th0Rad);

  // 4) Admitancia:  Y = 1/Z = G − jB
  k.G = P / (V * V);
  k.B = k.Q0 / (V * V);

  // 6) Capacitor en paralelo suma +jωC:  Y' = G − j(B − ωC)
  k.Bn = k.B - k.Bc;
  k.Qn = k.Q0 - k.Qc;
  k.Sn = apparentPower(P, k.Qn, k.residual);
  k.FP = k.Sn > 0.0f ? P / k.Sn : 0.0f;
  k.th = aGrados(atan2(k.Bn, k.G));
  k.In = k.Sn / V;

  // 7) Ajuste adicional desde el estado actual hasta el borde más cercano de la meta.
  k.tan2 = tan(acos(FP_TARGET_MIN));
  float qLimitSq = P * P * k.tan2 * k.tan2 - k.residual * k.residual;
  k.correctionReachable = P > 0.0f && qLimitSq >= 0.0f;
  float qLimit = sqrt(max(0.0f, qLimitSq));
  k.Qreq = k.correctionReachable ? k.Qn - constrain(k.Qn, -qLimit, qLimit) : 0.0f;
  k.Creq = k.Qreq / (OMEGA * V * V) * 1e6f;
  k.C1 = k.B / OMEGA * 1e6f;                                     // C para anular Q de la carga

  // 8) Indicador de desplazamiento aproximado por cuadratura.
  k.thVI = lastThetaRawDeg;
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
  Serial.print(F(" = ")); Serial.print(k.FP0, 3); Serial.print(F("   atan2(Q,P) = ")); Serial.print(k.th0, 1); Serial.println(F(" deg"));
  Serial.print(F("   Q del modelo = ")); Serial.print(k.Q0, 1); Serial.print(F(" var   Residuo de adquisicion = ")); Serial.print(k.measuredResidual, 1); Serial.println(F(" VA"));
  Serial.print(F("3) |Z equivalente| = V^2 / hypot(P,Q) = ")); Serial.print(k.Z, 2);
  Serial.print(F(" ohm   R = ")); Serial.print(k.R, 2); Serial.print(F("   X = ")); Serial.println(k.X, 2);
  Serial.print(F("4) Y = 1/Z = G - jB = ")); Serial.print(k.G * 1000.0f, 2); Serial.print(F(" - j")); Serial.print(k.B * 1000.0f, 2); Serial.println(F(" mS"));
  Serial.print(F("5) C = ")); Serial.print(k.C, 1); Serial.print(F(" uF  wC = 377*C = ")); Serial.print(k.Bc * 1000.0f, 2);
  Serial.print(F(" mS   Qc = wC*V^2 = ")); Serial.print(k.Qc, 1); Serial.println(F(" var"));
  Serial.print(F("6) Y' = G - j(B - wC) = ")); Serial.print(k.G * 1000.0f, 2); Serial.print(F(" - j")); Serial.print(k.Bn * 1000.0f, 2);
  Serial.print(F(" mS  FP' = P/S' = ")); Serial.print(k.FP, 3); Serial.print(k.Bn >= 0 ? F(" (atraso)") : F(" (adelanto)"));
  Serial.print(F("  I' = S'/V = ")); Serial.print(k.In, 3); Serial.print(F(" A  Q' = Q - Qc = ")); Serial.print(k.Qn, 1); Serial.println(F(" var"));
  Serial.print(F("7) Ajuste para llegar a FP 0.95: "));
  if (k.correctionReachable) { Serial.print(k.Creq, 1); Serial.println(F(" uF")); }
  else Serial.println(F("fuera del alcance del modelo capacitivo"));
  Serial.print(F("8) Factor de desplazamiento estimado: cos(")); Serial.print(k.thVI, 1); Serial.print(F(" deg) = ")); Serial.println(k.FPvi, 3);
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

// ---------- Lógica de control automático (0.95 - 1.00) ----------
bool runAutoControl(bool hayCarga) {
  if (!autoMode) {
    systemStatus = adsAddr ? "MANUAL" : "ADS1115_ERROR";
    return false;
  }
  if (!hayCarga || !adsAddr) {
    bool anyOn = getTotalConnectedCapacitanceUf() > 0.0f;
    for (uint8_t i = 0; i < NUM_STEPS; i++) setStepState(i, false);
    systemStatus = !adsAddr ? "ADS1115_ERROR" : (anyOn ? "NO_LOAD_RESET" : "NO_LOAD");
    return anyOn;
  }
  if (!calc.valido || calc.P <= 0.0f) {
    systemStatus = "INVALID_POWER";
    return false;
  }
  if (millis() - lastSwitchMs < MIN_SWITCH_INTERVAL_MS) {
    systemStatus = "SETTLING";
    return false;
  }
  if (calc.FP >= FP_TARGET_MIN) {
    systemStatus = "IN_RANGE";
    return false;
  }

  const bool add = calc.Qn > 0.0f;
  int8_t best = -1;
  float bestFP = calc.FP;
  bool locked = false;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (ledEstado[i] == add) continue;
    float deltaQ = OMEGA * STEP_CAPACITANCE_UF[i] * 1e-6f * calc.V * calc.V;
    float candidateQ = calc.Qn + (add ? -deltaQ : deltaQ);
    float candidateFP = calc.P / apparentPower(calc.P, candidateQ, calc.residual);
    if (candidateFP <= bestFP ||
        (candidateFP < FP_TARGET_MIN && candidateFP - calc.FP < FP_MIN_IMPROVEMENT)) continue;
    if (add && getRemainingLockoutMs(i) > 0) {
      locked = true;
      continue;
    }
    bestFP = candidateFP;
    best = (int8_t)i;
  }
  if (best < 0) {
    systemStatus = locked ? "STEP_LOCKED" : "TARGET_UNREACHABLE";
    return false;
  }
  setStepState((uint8_t)best, add);
  systemStatus = add ? "STEP_ADDED" : "STEP_REMOVED";
  return true;
}

// ---------- Comandos Serial ----------
void processCommand(const char* cmd) {
  if (strcmp(cmd, "MODE:AUTO") == 0) {
    autoMode = true;
  } else if (strcmp(cmd, "MODE:MANUAL") == 0) {
    autoMode = false;
  } else if (strcmp(cmd, "0") == 0) {
    autoMode = false;
    for (uint8_t i = 0; i < NUM_STEPS; i++) setStepState(i, false, true);
  } else if (strcmp(cmd, "1") == 0 || strcmp(cmd, "2") == 0 || strcmp(cmd, "3") == 0) {
    uint8_t idx = (uint8_t)(cmd[0] - '1');
    autoMode = false;
    setStepState(idx, !ledEstado[idx]);
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
    char* end;
    float fpRef = (float)strtod(cmd + 4, &end);
    if (*end != '\0' || fpRef != 1.0f) {
      calMsg = "CAL_VALOR_INVALIDO";
    } else if (autoMode || getTotalConnectedCapacitanceUf() > 0.0f) {
      calMsg = "CAL_APAGAR_BANCO";
    } else if (!lastThetaValid) {
      calMsg = "CAL_SIN_CARGA";
    } else {
      float errMisma = envolverGrados(lastThetaRawDeg);
      float errInvertida = envolverGrados(lastThetaRawDeg + 180.0f);
      bool invertir = fabs(errInvertida) < fabs(errMisma);
      float errDeg = invertir ? errInvertida : errMisma;
      long ajuste = (long)phaseDelayUs + lroundf(errDeg * (1e6f / (360.0f * 60.0f)));
      if (ajuste < PHASE_DELAY_MIN_US || ajuste > PHASE_DELAY_MAX_US) {
        calMsg = "CAL_FUERA_RANGO";
      } else {
        if (invertir) polaridadI = -polaridadI;
        fijarFase(ajuste);
        calMsg = "CAL_OK";
      }
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

    if (adsAddr && !adsFalla && !pendingAds && (uint32_t)(micros() - tAds) >= 1200) {
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

  // ---------- Potencia activa medida y Q estimada por cuadratura ----------
  float p = 0.0f, sMed = vrms * irms, qEstimate = 0.0f;
  bool hayCarga = hayV && hayI && adsAddr && sMed >= 1.0f;
  lastThetaValid = false;
  calc.valido = false;
  if (hayCarga) {
    float powerScale = NANO_LSB_MV * ZMPT_FACTOR * iScale * polaridadI;
    double covVI = w.sumVI / w.nA - (w.sumV / w.nA) * iMedia;
    p = constrain((float)covVI * powerScale, -sMed, sMed);
    if (w.nQ >= 10) {
      double mI = w.sumIq / w.nQ;
      double cI = w.sumViI / w.nQ - (w.sumVi / w.nQ) * mI;
      double cQ = w.sumVqI / w.nQ - (w.sumVq / w.nQ) * mI;
      lastThetaRawDeg = aGrados(atan2(cQ * polaridadI, cI * polaridadI));
      lastThetaValid = true;
      qEstimate = (float)cQ * powerScale;
      calcularTriangulos(vrms, irms, p, qEstimate, getTotalConnectedCapacitanceUf());
    }
  }

  if (SIMULATED_BANK && runAutoControl(hayCarga) && calc.valido) {
    calcularTriangulos(vrms, irms, p, qEstimate, getTotalConnectedCapacitanceUf());
  }
  // Los campos originales siguen describiendo el sistema con el banco.
  float fpMed = hayCarga ? p / sMed : 0.0f;
  float s = SIMULATED_BANK && calc.valido ? calc.Sn : sMed;
  float iSystem = SIMULATED_BANK && calc.valido ? calc.In : irms;
  float q = calc.valido ? (SIMULATED_BANK ? calc.Qn : calc.Qmed) : 0.0f;
  float fp = SIMULATED_BANK && calc.valido ? calc.FP : fpMed;
  bool inductiva = q >= 0.0f;

  // ---------- Salida ----------
  if (jsonMode) {
    Serial.print(F("{\"t\":"));       Serial.print(millis());
    Serial.print(F(",\"vrms\":"));   Serial.print(vrms, 2);
    Serial.print(F(",\"irms\":"));   Serial.print(iSystem, 3);
    Serial.print(F(",\"p\":"));      Serial.print(p, 1);
    Serial.print(F(",\"q\":"));      Serial.print(q, 1);
    Serial.print(F(",\"s\":"));      Serial.print(s, 1);
    Serial.print(F(",\"fp\":"));     Serial.print(fp, 3);
    Serial.print(F(",\"bank_simulated\":")); Serial.print(SIMULATED_BANK ? F("true") : F("false"));
    Serial.print(F(",\"irms_measured\":")); Serial.print(irms, 3);
    Serial.print(F(",\"s_measured\":")); Serial.print(sMed, 1);
    Serial.print(F(",\"fp_measured\":"));
    if (hayCarga) Serial.print(fpMed, 4); else Serial.print(F("null"));
    Serial.print(F(",\"q_estimated\":"));
    if (calc.valido) Serial.print(calc.Qmed, 1); else Serial.print(F("null"));
    Serial.print(F(",\"irms_compensated\":"));
    if (calc.valido && SIMULATED_BANK) Serial.print(calc.In, 3); else Serial.print(F("null"));
    Serial.print(F(",\"fp_compensated\":"));
    if (calc.valido && SIMULATED_BANK) Serial.print(calc.FP, 4); else Serial.print(F("null"));
    Serial.print(F(",\"freq\":"));   Serial.print(freq, 1);
    Serial.print(F(",\"phase_us\":")); Serial.print(phaseDelayUs);
    Serial.print(F(",\"carga\":\""));
    Serial.print(!calc.valido ? F("-") : (inductiva ? F("IND") : F("CAP")));
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
      Serial.print(F(",\"residual\":")); Serial.print(calc.residual, 1);
      Serial.print(F(",\"residual_measured\":")); Serial.print(calc.measuredResidual, 1);
      Serial.print(F(",\"sinusoidal_model\":")); Serial.print(SIMULATED_BANK ? F("true") : F("false"));
      Serial.print(F(",\"t2\":"));   Serial.print(calc.tan2, 3);
      Serial.print(F(",\"qr\":"));
      if (calc.correctionReachable) Serial.print(calc.Qreq, 1); else Serial.print(F("null"));
      Serial.print(F(",\"cr\":"));
      if (calc.correctionReachable) Serial.print(calc.Creq, 1); else Serial.print(F("null"));
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
    Serial.print(F(" | I medida ")); pf(irms, 5, 2); Serial.print(F(" Arms"));
    Serial.print(F(" | P ")); pf(p, 6, 1); Serial.print(F(" W"));
    Serial.print(F(" | FP medido ")); pf(fpMed, 5, 3);
    if (SIMULATED_BANK && calc.valido) {
      Serial.print(F(" | I / FP estimados ")); pf(iSystem, 5, 2);
      Serial.print(F(" A / ")); pf(fp, 5, 3);
    }
    Serial.print(!calc.valido ? F(" -") : (inductiva ? F(" IND") : F(" CAP")));
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
  if (!SIMULATED_BANK) runAutoControl(hayCarga);
}

