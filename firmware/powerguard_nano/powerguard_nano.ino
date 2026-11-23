/*
 * POWERGuard - Medidor True-RMS/FP y Controlador de Banco de Capacitores
 * Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute
 *
 * - Sin librerías externas: habla con el ADS1115 directo por Wire (400 kHz).
 * - ZMPT101B en ADS1115 A2 (single-ended) y SCT-013 en ADS1115 A0-A1 (diferencial).
 * - 3 LEDs/pasos en D10, D11, D12 (activos en HIGH).
 * - Comandos Serial (115200 baud):
 *     1 / 2 / 3   -> alternar paso/LED D10 / D11 / D12 (pasa a modo MANUAL)
 *     0           -> apagar todos los pasos/LEDs
 *     l           -> repetir prueba visual de LEDs
 *     MODE:AUTO   -> control automático (FP entre 0.95 y 0.99)
 *     MODE:MANUAL -> control manual
 *     STEP:i:v    -> activar (v=1) o desactivar (v=0) paso i (0..2)
 *     STATUS      -> enviar lectura JSON inmediata
 */

#include <Wire.h>
#include "config.h"

bool stepState[NUM_STEPS] = {false, false, false};
unsigned long lastDisconnectMs[NUM_STEPS] = {0, 0, 0};
unsigned long dischargeLockoutMs[NUM_STEPS] = {0, 0, 0};
unsigned long lastSwitchMs = 0;

bool adsPresente = false;
bool autoMode = true;
bool awaitingStepCheck = false;
int8_t lastAddedStep = -1;
float fpBeforeStep = 0.0f;
const char* systemStatus = "INIT";

// Buffers en int16_t para cuidar la SRAM del Arduino Nano
int16_t vRaw[MAX_SAMPLES];
int16_t iRaw[MAX_SAMPLES];
unsigned long vTimeUs[MAX_SAMPLES];
unsigned long iTimeUs[MAX_SAMPLES];

struct PowerMetrics {
  float vrms;
  float irms;
  float p;
  float q;
  float s;
  float fp;
  uint8_t sampleCount;
};

PowerMetrics lastMetrics = {0, 0, 0, 0, 0, 0, 0};
unsigned long lastTelemetryMs = 0;
char serialCmdBuf[32];
uint8_t serialCmdLen = 0;

// ============================================================================
// Comunicación directa con ADS1115 por I2C (Wire)
// ============================================================================

bool detectarADS() {
  Wire.beginTransmission(ADS_ADDR);
  return Wire.endTransmission() == 0;
}

bool adsWriteConfig(uint16_t cfg) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write((uint8_t)0x01);
  Wire.write((uint8_t)(cfg >> 8));
  Wire.write((uint8_t)(cfg & 0xFF));
  return Wire.endTransmission() == 0;
}

bool adsReadReg(uint8_t reg, int16_t &valor) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(ADS_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  valor = (int16_t)(((uint16_t)hi << 8) | lo);
  return true;
}

// mux: 0 = A0-A1 dif, 4..7 = A0..A3 contra GND (6 = A2)
bool adsRead(uint8_t mux, int16_t &out) {
  // OS=1 (iniciar), MUX, PGA +-4.096 V, single-shot, 860 SPS, comparador apagado
  uint16_t cfg = 0x8000 | ((uint16_t)mux << 12) | 0x0200 | 0x0100 | 0x00E0 | 0x0003;
  if (!adsWriteConfig(cfg)) return false;
  delayMicroseconds(1200);
  uint32_t t0 = millis();
  int16_t c;
  do {
    if (!adsReadReg(0x01, c)) return false;
    if (millis() - t0 > 20) return false;
  } while (!((uint16_t)c & 0x8000));
  return adsReadReg(0x00, out);
}

// ============================================================================
// Control de LEDs / Pasos y bloqueo por descarga RC
// ============================================================================

unsigned long computeDischargeLockoutMs(float capacitanceUf) {
  float tauSeconds = DISCHARGE_RESISTOR_OHMS * (capacitanceUf * 1e-6f);
  unsigned long lockout = (unsigned long)(DISCHARGE_TAU_MULTIPLIER * tauSeconds * 1000.0f);
  return max(lockout, MIN_DISCHARGE_LOCKOUT_MS);
}

void writeStepPin(uint8_t stepIdx, bool turnOn) {
  if (stepIdx >= NUM_STEPS) return;
  uint8_t level = turnOn ? (RELAY_ACTIVE_LOW ? LOW : HIGH)
                         : (RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(STEP_PINS[stepIdx], level);
}

unsigned long getRemainingLockoutMs(uint8_t stepIdx) {
  if (stepIdx >= NUM_STEPS || stepState[stepIdx]) return 0;
  if (lastDisconnectMs[stepIdx] == 0) return 0;
  unsigned long elapsed = millis() - lastDisconnectMs[stepIdx];
  if (elapsed >= dischargeLockoutMs[stepIdx]) return 0;
  return dischargeLockoutMs[stepIdx] - elapsed;
}

bool setStepState(uint8_t stepIdx, bool turnOn, bool bypassLockout = false) {
  if (stepIdx >= NUM_STEPS) return false;
  if (stepState[stepIdx] == turnOn) return true;
  if (turnOn) {
    if (!bypassLockout && getRemainingLockoutMs(stepIdx) > 0) return false;
    stepState[stepIdx] = true;
    writeStepPin(stepIdx, true);
  } else {
    stepState[stepIdx] = false;
    writeStepPin(stepIdx, false);
    lastDisconnectMs[stepIdx] = millis();
  }
  lastSwitchMs = millis();
  return true;
}

float getTotalConnectedCapacitanceUf() {
  float total = 0.0f;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (stepState[i]) total += STEP_CAPACITANCE_UF[i];
  }
  return total;
}

void pruebaLED() {
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    writeStepPin(i, true);
    delay(400);
    writeStepPin(i, false);
    stepState[i] = false;
  }
  for (uint8_t i = 0; i < NUM_STEPS; i++) writeStepPin(i, true);
  delay(400);
  for (uint8_t i = 0; i < NUM_STEPS; i++) writeStepPin(i, false);
}

// ============================================================================
// Medición True-RMS y Factor de Potencia
// ============================================================================

PowerMetrics measurePower() {
  PowerMetrics m = {0, 0, 0, 0, 0, 0, 0};
  adsPresente = detectarADS();
  if (!adsPresente) {
    systemStatus = "ADS1115_ERROR";
    return m;
  }

  unsigned long startMs = millis();
  uint8_t n = 0;

  while ((millis() - startMs) < MEASUREMENT_WINDOW_MS && n < MAX_SAMPLES) {
    int16_t rawV = 0, rawI = 0;
    vTimeUs[n] = micros();
    if (!adsRead(MUX_ZMPT, rawV)) {
      adsPresente = false;
      systemStatus = "ADS1115_ERROR";
      return m;
    }
    vRaw[n] = rawV;

    iTimeUs[n] = micros();
    if (!adsRead(MUX_SCT_DIFF, rawI)) {
      adsPresente = false;
      systemStatus = "ADS1115_ERROR";
      return m;
    }
    iRaw[n] = rawI;
    n++;
  }

  if (n < 10) return m;

  long sumV = 0, sumI = 0;
  for (uint8_t k = 0; k < n; k++) {
    sumV += vRaw[k];
    sumI += iRaw[k];
  }
  float offsetV = (float)sumV / n;
  float offsetI = (float)sumI / n;

  float vScale = ADS_LSB_VOLTS * VOLTAGE_CAL_FACTOR;
  float iScale = ADS_LSB_VOLTS * CURRENT_CAL_FACTOR;

  double sumV2 = 0.0, sumI2 = 0.0, sumP = 0.0;
  uint8_t validCount = n - 1;

  for (uint8_t k = 0; k < validCount; k++) {
    float v0 = ((float)vRaw[k] - offsetV) * vScale;
    float v1 = ((float)vRaw[k + 1] - offsetV) * vScale;
    float iVal = ((float)iRaw[k] - offsetI) * iScale;

    unsigned long dtSpan = vTimeUs[k + 1] - vTimeUs[k];
    float alpha = 0.5f;
    if (dtSpan > 0) {
      alpha = (float)(iTimeUs[k] - vTimeUs[k]) / (float)dtSpan + PHASE_TRIM_SAMPLES;
      if (alpha < 0.0f) alpha = 0.0f;
      if (alpha > 1.0f) alpha = 1.0f;
    }

    float vAligned = v0 + alpha * (v1 - v0);
    sumV2 += (double)vAligned * (double)vAligned;
    sumI2 += (double)iVal * (double)iVal;
    sumP  += (double)vAligned * (double)iVal;
  }

  m.vrms = sqrt(sumV2 / validCount);
  m.irms = sqrt(sumI2 / validCount);
  m.p    = (float)(sumP / validCount);
  m.s    = m.vrms * m.irms;
  m.sampleCount = validCount;

  if (m.irms < MIN_CURRENT_THRESHOLD_A || m.s < 1.0f) {
    m.p = 0.0f; m.q = 0.0f; m.fp = 1.0f;
  } else {
    float pAbs = fabs(m.p);
    if (pAbs > m.s) pAbs = m.s;
    m.p  = pAbs;
    m.q  = sqrt(max(0.0f, (m.s * m.s) - (m.p * m.p)));
    m.fp = m.p / m.s;
  }
  return m;
}

// ============================================================================
// Lógica de control automático (0.95 - 0.99)
// ============================================================================

int8_t findNextAvailableStepToAdd() {
  int8_t bestIdx = -1;
  float minCap = 1e9f;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (!stepState[i] && getRemainingLockoutMs(i) == 0 && STEP_CAPACITANCE_UF[i] < minCap) {
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
    if (stepState[i] && STEP_CAPACITANCE_UF[i] < minCap) {
      minCap = STEP_CAPACITANCE_UF[i];
      bestIdx = (int8_t)i;
    }
  }
  return bestIdx;
}

void runAutoControl(const PowerMetrics &m) {
  if (!adsPresente) { systemStatus = "ADS1115_ERROR"; return; }
  if (!autoMode)    { systemStatus = "MANUAL"; return; }

  if (m.irms < MIN_CURRENT_THRESHOLD_A) {
    bool anyOn = false;
    for (uint8_t i = 0; i < NUM_STEPS; i++) {
      if (stepState[i]) { setStepState(i, false); anyOn = true; }
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
    if (m.fp < fpBeforeStep) {
      setStepState((uint8_t)lastAddedStep, false);
      lastAddedStep = -1;
      systemStatus = "OVERCOMP_REVERT";
      return;
    }
    lastAddedStep = -1;
  }

  if (m.fp < FP_TARGET_MIN) {
    int8_t nextStep = findNextAvailableStepToAdd();
    if (nextStep >= 0) {
      fpBeforeStep = m.fp;
      if (setStepState((uint8_t)nextStep, true)) {
        lastAddedStep = nextStep;
        awaitingStepCheck = true;
        systemStatus = "STEP_ADDED";
      }
    } else {
      systemStatus = "MAX_CAP_OR_LOCKED";
    }
  } else if (m.fp > FP_TARGET_MAX) {
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

// ============================================================================
// Telemetría JSON y comandos por Serial
// ============================================================================

void sendTelemetry(const PowerMetrics &m) {
  Serial.print(F("{\"t\":"));       Serial.print(millis());
  Serial.print(F(",\"vrms\":"));   Serial.print(m.vrms, 2);
  Serial.print(F(",\"irms\":"));   Serial.print(m.irms, 3);
  Serial.print(F(",\"p\":"));      Serial.print(m.p, 1);
  Serial.print(F(",\"q\":"));      Serial.print(m.q, 1);
  Serial.print(F(",\"s\":"));      Serial.print(m.s, 1);
  Serial.print(F(",\"fp\":"));     Serial.print(m.fp, 3);
  Serial.print(F(",\"steps\":["));
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    Serial.print(stepState[i] ? 1 : 0);
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
}

void processSerialCommand(const char* cmd) {
  if (strcmp(cmd, "MODE:AUTO") == 0) {
    autoMode = true;
    awaitingStepCheck = false;
  } else if (strcmp(cmd, "MODE:MANUAL") == 0) {
    autoMode = false;
    awaitingStepCheck = false;
  } else if (strcmp(cmd, "STATUS") == 0) {
    sendTelemetry(lastMetrics);
  } else if (strcmp(cmd, "0") == 0) {
    autoMode = false;
    for (uint8_t i = 0; i < NUM_STEPS; i++) setStepState(i, false, true);
  } else if (strcmp(cmd, "1") == 0 || strcmp(cmd, "2") == 0 || strcmp(cmd, "3") == 0) {
    uint8_t idx = (uint8_t)(cmd[0] - '1');
    autoMode = false;
    setStepState(idx, !stepState[idx], true);
  } else if (strcmp(cmd, "l") == 0 || strcmp(cmd, "L") == 0) {
    autoMode = false;
    pruebaLED();
  } else if (strncmp(cmd, "STEP:", 5) == 0) {
    int idx = -1, val = 0;
    if (sscanf(cmd, "STEP:%d:%d", &idx, &val) == 2 && idx >= 0 && idx < NUM_STEPS) {
      autoMode = false;
      setStepState((uint8_t)idx, val == 1);
    }
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialCmdLen > 0) {
        serialCmdBuf[serialCmdLen] = '\0';
        processSerialCommand(serialCmdBuf);
        serialCmdLen = 0;
      }
    } else if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
      serialCmdBuf[serialCmdLen++] = c;
    }
  }
}

// ============================================================================
// Setup y Loop principal
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    writeStepPin(i, false);
    dischargeLockoutMs[i] = computeDischargeLockoutMs(STEP_CAPACITANCE_UF[i]);
  }

  Wire.begin();
  Wire.setClock(400000);

  adsPresente = detectarADS();
  pruebaLED();

  if (!adsPresente) {
    systemStatus = "ADS1115_ERROR";
    Serial.println(F("{\"error\":\"ADS1115 no responde en 0x48 (revisar SDA->A4, SCL->A5)\"}"));
  } else {
    systemStatus = "READY";
  }
}

void loop() {
  pollSerialCommands();
  lastMetrics = measurePower();
  runAutoControl(lastMetrics);
  if ((millis() - lastTelemetryMs) >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    sendTelemetry(lastMetrics);
  }
}
