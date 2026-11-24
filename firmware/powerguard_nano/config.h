#ifndef POWERGUARD_CONFIG_H
#define POWERGUARD_CONFIG_H

#include <Arduino.h>

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
const float STEP_CAPACITANCE_UF[NUM_STEPS] = {22.0f, 47.0f, 69.0f};
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

// Control del Factor de Potencia
const float FP_TARGET_MIN = 0.95f;
const float FP_TARGET_MAX = 0.99f;
const unsigned long MIN_SWITCH_INTERVAL_MS   = 3000UL;
const float DISCHARGE_TAU_MULTIPLIER         = 5.0f;
const unsigned long MIN_DISCHARGE_LOCKOUT_MS = 6000UL;

// Comunicación Serial
const unsigned long SERIAL_BAUD_RATE = 115200UL;

#endif // POWERGUARD_CONFIG_H
