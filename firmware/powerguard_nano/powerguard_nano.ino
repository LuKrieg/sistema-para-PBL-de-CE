/*
 * POWERGuard - Compensador Automático de Factor de Potencia
 * Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute
 *
 * Descripción:
 *   Mide en tiempo real Vrms, Irms, Potencia Activa (P), Reactiva (Q),
 *   Aparente (S) y Factor de Potencia (FP) verdadero mediante muestreo de
 *   v(t) e i(t) con el ADC ADS1115. Corrige el retraso de multiplexado del
 *   ADS1115 mediante interpolación lineal temporal.
 *
 *   Controla un banco de capacitores de 4 pasos (J1-J4) mediante relés
 *   (sin MOC3031 ni triacs) para mantener el FP entre 0.95 y 0.99, con
 *   detección de sobrecompensación y bloqueo por descarga RC.
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "config.h"

Adafruit_ADS1115 ads;

// --- Estado del banco de capacitores ---
bool stepState[NUM_STEPS] = {false, false, false, false};
unsigned long lastDisconnectMs[NUM_STEPS] = {0, 0, 0, 0};
unsigned long dischargeLockoutMs[NUM_STEPS] = {0, 0, 0, 0};
unsigned long lastSwitchMs = 0;

// --- Control automático vs manual ---
bool autoMode = true;
bool awaitingStepCheck = false;
int8_t lastAddedStep = -1;
float fpBeforeStep = 0.0f;
String systemStatus = "INIT";

// --- Buffers de muestreo (tamaño acotado para SRAM del Nano) ---
float vSamples[MAX_SAMPLES];
float iSamples[MAX_SAMPLES];
unsigned long vTimeUs[MAX_SAMPLES];
unsigned long iTimeUs[MAX_SAMPLES];

// --- Últimas mediciones ---
struct PowerMetrics {
  float vrms;
  float irms;
  float p;
  float q;
  float s;
  float fp;
  uint16_t sampleCount;
};

PowerMetrics lastMetrics = {0, 0, 0, 0, 0, 0, 0};
unsigned long lastTelemetryMs = 0;
String serialInputBuffer = "";

// ============================================================================
// Funciones auxiliares de hardware (Relés y bloqueo de descarga)
// ============================================================================

unsigned long computeDischargeLockoutMs(float capacitanceUf) {
  // tau = R * C; C en Faradios = capacitanceUf * 1e-6
  float tauSeconds = DISCHARGE_RESISTOR_OHMS * (capacitanceUf * 1e-6f);
  unsigned long lockout = (unsigned long)(DISCHARGE_TAU_MULTIPLIER * tauSeconds * 1000.0f);
  return max(lockout, MIN_DISCHARGE_LOCKOUT_MS);
}

void writeRelayPin(uint8_t stepIdx, bool turnOn) {
  if (stepIdx >= NUM_STEPS) return;
  uint8_t level = turnOn ? (RELAY_ACTIVE_LOW ? LOW : HIGH)
                         : (RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(STEP_PINS[stepIdx], level);
}

unsigned long getRemainingLockoutMs(uint8_t stepIdx) {
  if (stepIdx >= NUM_STEPS || stepState[stepIdx]) return 0;
  if (lastDisconnectMs[stepIdx] == 0) return 0; // Nunca desconectado aún
  unsigned long elapsed = millis() - lastDisconnectMs[stepIdx];
  if (elapsed >= dischargeLockoutMs[stepIdx]) return 0;
  return dischargeLockoutMs[stepIdx] - elapsed;
}

bool setStepState(uint8_t stepIdx, bool turnOn) {
  if (stepIdx >= NUM_STEPS) return false;
  if (stepState[stepIdx] == turnOn) return true;

  if (turnOn) {
    // Seguridad: no reconectar hasta que el capacitor se haya descargado
    if (getRemainingLockoutMs(stepIdx) > 0) {
      return false;
    }
    stepState[stepIdx] = true;
    writeRelayPin(stepIdx, true);
  } else {
    stepState[stepIdx] = false;
    writeRelayPin(stepIdx, false);
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

// ============================================================================
// Medición True-RMS y Factor de Potencia con compensación de retardo ADS1115
// ============================================================================

PowerMetrics measurePower() {
  PowerMetrics m = {0, 0, 0, 0, 0, 0, 0};

  unsigned long startMs = millis();
  uint16_t n = 0;

  // 1. Adquisición alternada de tensión y corriente con marcas de tiempo
  while ((millis() - startMs) < MEASUREMENT_WINDOW_MS && n < MAX_SAMPLES) {
    vTimeUs[n] = micros();
    int16_t rawV = ads.readADC_SingleEnded(ADS_CH_VOLTAGE);
    vSamples[n] = (float)rawV * ADS_LSB_VOLTS;

    iTimeUs[n] = micros();
    int16_t rawI = ads.readADC_SingleEnded(ADS_CH_CURRENT);
    iSamples[n] = (float)rawI * ADS_LSB_VOLTS;

    n++;
  }

  if (n < 10) {
    return m;
  }

  // 2. Calcular offset DC promedio (Vcc/2) para centrar ambas ondas en cero
  double sumV = 0.0;
  double sumI = 0.0;
  for (uint16_t k = 0; k < n; k++) {
    sumV += vSamples[k];
    sumI += iSamples[k];
  }
  float offsetV = (float)(sumV / n);
  float offsetI = (float)(sumI / n);

  // 3. Remover DC y aplicar calibración física (Voltios y Amperios instantáneos)
  for (uint16_t k = 0; k < n; k++) {
    vSamples[k] = (vSamples[k] - offsetV) * VOLTAGE_CAL_FACTOR;
    iSamples[k] = (iSamples[k] - offsetI) * CURRENT_CAL_FACTOR;
  }

  // 4. Compensación de retraso de multiplexado del ADS1115 mediante
  //    interpolación lineal temporal de v(t) en el instante iTimeUs[k].
  double sumV2 = 0.0;
  double sumI2 = 0.0;
  double sumP = 0.0;
  uint16_t validCount = n - 1;

  for (uint16_t k = 0; k < validCount; k++) {
    unsigned long dtSpan = vTimeUs[k + 1] - vTimeUs[k];
    float alpha = 0.5f;
    if (dtSpan > 0) {
      alpha = (float)(iTimeUs[k] - vTimeUs[k]) / (float)dtSpan;
      alpha += PHASE_TRIM_SAMPLES;
      if (alpha < 0.0f) alpha = 0.0f;
      if (alpha > 1.0f) alpha = 1.0f;
    }

    float vAligned = vSamples[k] + alpha * (vSamples[k + 1] - vSamples[k]);
    float iVal = iSamples[k];

    sumV2 += (double)vAligned * (double)vAligned;
    sumI2 += (double)iVal * (double)iVal;
    sumP += (double)vAligned * (double)iVal;
  }

  m.vrms = sqrt(sumV2 / validCount);
  m.irms = sqrt(sumI2 / validCount);
  m.p = (float)(sumP / validCount);
  m.s = m.vrms * m.irms;
  m.sampleCount = validCount;

  if (m.irms < MIN_CURRENT_THRESHOLD_A || m.s < 1.0f) {
    // Carga apagada o corriente despreciable
    m.p = 0.0f;
    m.q = 0.0f;
    m.fp = 1.0f;
  } else {
    float pAbs = fabs(m.p);
    if (pAbs > m.s) pAbs = m.s;
    m.p = pAbs;
    m.q = sqrt(max(0.0f, (m.s * m.s) - (m.p * m.p)));
    m.fp = m.p / m.s;
  }

  return m;
}

// ============================================================================
// Lógica de control automático del banco de capacitores (0.95 - 0.99)
// ============================================================================

int8_t findNextAvailableStepToAdd() {
  // Busca el paso apagado de menor capacitancia que no esté en bloqueo RC
  int8_t bestIdx = -1;
  float minCap = 1e9f;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (!stepState[i] && getRemainingLockoutMs(i) == 0) {
      if (STEP_CAPACITANCE_UF[i] < minCap) {
        minCap = STEP_CAPACITANCE_UF[i];
        bestIdx = (int8_t)i;
      }
    }
  }
  return bestIdx;
}

int8_t findStepToRemove() {
  // Desconecta primero el paso encendido de menor capacitancia para ajuste fino
  int8_t bestIdx = -1;
  float minCap = 1e9f;
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    if (stepState[i]) {
      if (STEP_CAPACITANCE_UF[i] < minCap) {
        minCap = STEP_CAPACITANCE_UF[i];
        bestIdx = (int8_t)i;
      }
    }
  }
  return bestIdx;
}

void runAutoControl(const PowerMetrics &m) {
  if (!autoMode) {
    systemStatus = "MANUAL";
    return;
  }

  if (m.irms < MIN_CURRENT_THRESHOLD_A) {
    // Si el motor está apagado, desconectar todos los pasos por seguridad
    bool anyOn = false;
    for (uint8_t i = 0; i < NUM_STEPS; i++) {
      if (stepState[i]) {
        setStepState(i, false);
        anyOn = true;
      }
    }
    awaitingStepCheck = false;
    systemStatus = anyOn ? "NO_LOAD_RESET" : "NO_LOAD";
    return;
  }

  unsigned long now = millis();
  if ((now - lastSwitchMs) < MIN_SWITCH_INTERVAL_MS) {
    systemStatus = "SETTLING";
    return;
  }

  // Verificar si el último paso agregado mejoró el FP o causó sobrecompensación
  if (awaitingStepCheck && lastAddedStep >= 0) {
    awaitingStepCheck = false;
    if (m.fp < fpBeforeStep) {
      // El FP bajó -> cruzamos a zona capacitiva (adelantada). Revertir paso.
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
// Telemetría JSON y manejo de comandos por USB Serial
// ============================================================================

void sendTelemetry(const PowerMetrics &m) {
  Serial.print(F("{\"t\":"));
  Serial.print(millis());
  Serial.print(F(",\"vrms\":"));
  Serial.print(m.vrms, 2);
  Serial.print(F(",\"irms\":"));
  Serial.print(m.irms, 3);
  Serial.print(F(",\"p\":"));
  Serial.print(m.p, 1);
  Serial.print(F(",\"q\":"));
  Serial.print(m.q, 1);
  Serial.print(F(",\"s\":"));
  Serial.print(m.s, 1);
  Serial.print(F(",\"fp\":"));
  Serial.print(m.fp, 3);

  Serial.print(F(",\"steps\":["));
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    Serial.print(stepState[i] ? 1 : 0);
    if (i < NUM_STEPS - 1) Serial.print(',');
  }
  Serial.print(']');

  Serial.print(F(",\"locked\":["));
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    Serial.print(getRemainingLockoutMs(i));
    if (i < NUM_STEPS - 1) Serial.print(',');
  }
  Serial.print(']');

  Serial.print(F(",\"cap_uf\":"));
  Serial.print(getTotalConnectedCapacitanceUf(), 1);
  Serial.print(F(",\"mode\":\""));
  Serial.print(autoMode ? F("AUTO") : F("MANUAL"));
  Serial.print(F("\",\"status\":\""));
  Serial.print(systemStatus);
  Serial.println(F("\"}"));
}

void processSerialCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "MODE:AUTO") {
    autoMode = true;
    awaitingStepCheck = false;
  } else if (cmd == "MODE:MANUAL") {
    autoMode = false;
    awaitingStepCheck = false;
  } else if (cmd == "STATUS") {
    sendTelemetry(lastMetrics);
  } else if (cmd.startsWith("STEP:")) {
    // Formato: STEP:<idx>:<0|1>
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);
    if (secondColon > firstColon) {
      int idx = cmd.substring(firstColon + 1, secondColon).toInt();
      int val = cmd.substring(secondColon + 1).toInt();
      if (idx >= 0 && idx < NUM_STEPS) {
        autoMode = false;
        setStepState((uint8_t)idx, val == 1);
      }
    }
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialInputBuffer.length() > 0) {
        processSerialCommand(serialInputBuffer);
        serialInputBuffer = "";
      }
    } else if (serialInputBuffer.length() < 48) {
      serialInputBuffer += c;
    }
  }
}

// ============================================================================
// Setup y Loop principal
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();

  // Inicializar pines de relés apagados de forma segura
  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    pinMode(STEP_PINS[i], OUTPUT);
    writeRelayPin(i, false);
    dischargeLockoutMs[i] = computeDischargeLockoutMs(STEP_CAPACITANCE_UF[i]);
  }

  // Inicializar ADS1115 a máxima velocidad (860 SPS) y ganancia +-4.096 V
  if (!ads.begin()) {
    systemStatus = "ADS1115_ERROR";
    Serial.println(F("{\"error\":\"ADS1115 no detectado en I2C 0x48\"}"));
  } else {
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);
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
