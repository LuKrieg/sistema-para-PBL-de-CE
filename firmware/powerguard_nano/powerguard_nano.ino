/*
 * POWERGuard 12 - Medidor True-RMS / Factor de Potencia y Control Automático
 * Basado en "Prueba de componentes v4.1 (circuito TAL COMO ESTA)"
 *
 * - I2C por software en A1 (SDA) y A2 (SCL), sin librerías externas.
 * - ZMPT101B en A0 del Nano, SCT-013-000 en ADS1115 (A0-A1 diferencial continuo).
 * - 3 pasos / LEDs en D12 (D1), D11 (D2), D10 (D3).
 * - Compensación de fase por buffer circular (DEFAULT_PHASE_DELAY_US = 580 us)
 *   y ventana de muestreo múltiplo exacto de 60 Hz (167 ms = 10 ciclos).
 *
 * Comandos Serial (115200 baud):
 *   1 / 2 / 3   -> alternar paso/LED D1/D2/D3 (modo MANUAL)
 *   0           -> apagar todos los pasos/LEDs
 *   l           -> repetir prueba visual de LEDs
 *   r           -> cambiar entre 5 y 10 lecturas por segundo
 *   t           -> alternar salida JSON (Dashboard) / Tabla texto (Monitor Serie)
 *   d           -> mostrar / ocultar detalle en mV (en modo tabla texto)
 *   + / -       -> subir / bajar compensación de fase en 50 us
 *   MODE:AUTO   -> control automático (FP entre 0.95 y 0.99)
 *   MODE:MANUAL -> control manual
 *   STEP:i:v    -> activar (v=1) o desactivar (v=0) paso i (0..2)
 */

#include "config.h"

uint8_t lecturasPorSeg = 5;   // 5 o 10 (comando 'r')
uint8_t adsAddr = 0;          // 0 = no encontrado
bool ledEstado[NUM_STEPS] = {false, false, false};
unsigned long lastDisconnectMs[NUM_STEPS] = {0, 0, 0};
unsigned long dischargeLockoutMs[NUM_STEPS] = {0, 0, 0};
unsigned long lastSwitchMs = 0;

bool autoMode = true;
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

// Buffer circular de últimas lecturas de ZMPT para alinear fase con ADS1115
const uint8_t V_RING_SIZE = 16;
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
unsigned long computeDischargeLockoutMs(float capacitanceUf) {
  float tauSeconds = DISCHARGE_RESISTOR_OHMS * (capacitanceUf * 1e-6f);
  unsigned long lockout = (unsigned long)(DISCHARGE_TAU_MULTIPLIER * tauSeconds * 1000.0f);
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
  } else if (strcmp(cmd, "+") == 0) {
    if (phaseDelayUs <= 1950) phaseDelayUs += 50;
  } else if (strcmp(cmd, "-") == 0) {
    if (phaseDelayUs >= -1950) phaseDelayUs -= 50;
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
        c == 't' || c == 'T' || c == '+' || c == '-')) {
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

  iniciarADS();
  pruebaLED();

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

  uint16_t nA = 0;
  double sumA = 0, sqA = 0;
  double sumVsync = 0, sumVIsync = 0;
  float minA = 1e9, maxA = -1e9;
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
      float vSync = lookupVoltageAt(pendingTargetUs, v);
      nA++;
      sumA += pendingMv;
      sqA += (double)pendingMv * pendingMv;
      sumVsync += vSync;
      sumVIsync += (double)vSync * pendingMv;
      if (pendingMv < minA) minA = pendingMv;
      if (pendingMv > maxA) maxA = pendingMv;
      pendingAds = false;
    }

    if (adsAddr && !adsFalla && (uint32_t)(micros() - tAds) >= 1200) {
      int16_t raw;
      if (adsLeerContinuo(raw)) {
        uint32_t tRead = micros();
        tAds = tRead;
        float mv = raw * ADS_LSB_MV;

        if (phaseDelayUs >= 0) {
          uint32_t tTarget = tRead - (uint32_t)phaseDelayUs;
          float vSync = lookupVoltageAt(tTarget, v);
          nA++;
          sumA += mv;
          sqA += (double)mv * mv;
          sumVsync += vSync;
          sumVIsync += (double)vSync * mv;
          if (mv < minA) minA = mv;
          if (mv > maxA) maxA = mv;
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
  bool adsOk = (nA >= 10);
  float iMedia = 0.0f, iRmsMv = 0.0f, ppI = 0.0f;
  if (adsOk) {
    iMedia = (float)(sumA / nA);
    double varA = sqA / nA - (double)iMedia * iMedia;
    iRmsMv = (varA > 0) ? sqrt(varA) : 0.0f;
    ppI = maxA - minA;
  }
  bool hayI = adsOk && ppI >= UMBRAL_SCT_MV;
  float iScale = (1.0f / 1000.0f) / R_CARGA_OHM * RELACION_SCT * FACTOR_CORR_I;
  float irms = hayI ? iRmsMv * iScale : 0.0f;

  // ---------- Potencia Activa (P), Aparente (S), Reactiva (Q) y FP verdadero ----------
  float p = 0.0f, s = vrms * irms, q = 0.0f, fp = 1.0f;
  if (hayV && hayI && adsOk && s >= 1.0f) {
    double meanVsync = sumVsync / nA;
    double covVI = (sumVIsync / nA) - (meanVsync * (double)iMedia);
    float vScale = NANO_LSB_MV * ZMPT_FACTOR;
    float pCalc = fabs((float)(covVI * (double)vScale * (double)iScale));
    if (pCalc > s) pCalc = s;
    p = pCalc;
    q = sqrt(max(0.0f, s * s - p * p));
    fp = p / s;
  }

  runAutoControl(hayV, hayI, fp);

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
    Serial.print(F(",\"mode\":\""));  Serial.print(autoMode ? F("AUTO") : F("MANUAL"));
    Serial.print(F("\",\"status\":\"")); Serial.print(systemStatus);
    Serial.println(F("\"}"));
  } else {
    Serial.print('#'); pf((float)numero, 5, 0);
    Serial.print(F(" | V ")); pf(vrms, 6, 1); Serial.print(F(" Vrms"));
    Serial.print(F(" | I ")); pf(irms, 5, 2); Serial.print(F(" Arms"));
    Serial.print(F(" | P ")); pf(p, 6, 1); Serial.print(F(" W"));
    Serial.print(F(" | FP ")); pf(fp, 5, 3);
    Serial.print(F(" | ph=")); Serial.print(phaseDelayUs); Serial.print(F("us"));
    Serial.print(F(" | ")); Serial.println(systemStatus);
    if (detalle) {
      Serial.print(F("       ZMPT mV: media=")); Serial.print(vMedia, 0);
      Serial.print(F(" pp=")); Serial.print(ppV, 0);
      Serial.print(F(" | SCT dif mV: pp=")); Serial.print(ppI, 0);
      Serial.print(F(" | muestras ZMPT=")); Serial.print(nN);
      Serial.print(F(" SCT=")); Serial.println(nA);
    }
  }
}
