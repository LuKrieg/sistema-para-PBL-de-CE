#ifndef POWERGUARD_CONFIG_H
#define POWERGUARD_CONFIG_H

#include <Arduino.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE - POWERGuard (Equipo 12)
// ============================================================================
// Sin librerías externas: comunicación directa con ADS1115 por Wire (I2C 400 kHz)
//   ADS1115 : SDA -> A4, SCL -> A5, VDD -> 5V, GND -> GND, ADDR -> GND (0x48)
//   SCT-013 : ADS1115 A0 y A1 en modo diferencial (MUX = 0)
//   ZMPT101B: ADS1115 A2 en modo single-ended (MUX = 6)
//   3 LEDs / Pasos: D10, D11, D12 (cátodo a GND, activos en HIGH)
// ============================================================================

const uint8_t NUM_STEPS = 3;
const uint8_t STEP_PINS[NUM_STEPS] = {10, 11, 12}; // Paso 1=D10, Paso 2=D11, Paso 3=D12
const bool RELAY_ACTIVE_LOW = false;               // false: LEDs/salidas activas en HIGH

// Capacitancias equivalentes de cada paso (uF)
const float STEP_CAPACITANCE_UF[NUM_STEPS] = {22.0f, 47.0f, 69.0f};
const float DISCHARGE_RESISTOR_OHMS = 100000.0f;   // 100 kOhm

// Dirección y canales MUX del ADS1115
const uint8_t ADS_ADDR     = 0x48;
const uint8_t MUX_SCT_DIFF = 0; // A0 - A1 diferencial
const uint8_t MUX_ZMPT     = 6; // A2 single-ended (4 + 2)

// ADS1115 con PGA +-4.096 V (16 bits -> 0.125 mV/bit = 0.000125 V/bit)
const float ADS_LSB_VOLTS      = 0.000125f;
const float VOLTAGE_CAL_FACTOR = 245.0f; // Ajustar con multímetro (V_red / Vrms_ZMPT)
const float CURRENT_CAL_FACTOR = 30.0f;  // SCT-013-030 (30 A : 1 V)
const float PHASE_TRIM_SAMPLES = 0.0f;   // Ajuste fino de fase por software

// Muestreo optimizado para 2 KB SRAM del Nano
const uint16_t MEASUREMENT_WINDOW_MS = 200; // ~12 ciclos de 60 Hz
const uint8_t MAX_SAMPLES = 45;
const float MIN_CURRENT_THRESHOLD_A = 0.15f;

// Control del Factor de Potencia
const float FP_TARGET_MIN = 0.95f;
const float FP_TARGET_MAX = 0.99f;
const unsigned long MIN_SWITCH_INTERVAL_MS   = 3000UL;
const float DISCHARGE_TAU_MULTIPLIER         = 5.0f;
const unsigned long MIN_DISCHARGE_LOCKOUT_MS = 6000UL;

// Comunicación Serial
const unsigned long SERIAL_BAUD_RATE      = 115200UL;
const unsigned long TELEMETRY_INTERVAL_MS = 500UL;

#endif // POWERGUARD_CONFIG_H
