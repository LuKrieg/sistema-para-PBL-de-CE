#ifndef POWERGUARD_CONFIG_H
#define POWERGUARD_CONFIG_H

#include <Arduino.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE - POWERGuard (Equipo 12)
// ============================================================================
// Microcontrolador: Arduino Nano (ATmega328P, 5V, I2C A4=SDA, A5=SCL)
// ADC corriente: ADS1115 (I2C 0x48, 16 bits, 860 SPS, diferencial A0-A1)
// Sensor tensión: ZMPT101B -> pin analógico A0 del Arduino Nano
// Sensor corriente: SCT-013-030 -> ADS1115 A0 y A1 (referenciado a Vcc/2)
// Conmutación: 3 pasos por relés / salidas digitales D12, D11, D10 (sin MOC/Triac)
// ============================================================================

const uint8_t NUM_STEPS = 3;
const uint8_t STEP_PINS[NUM_STEPS] = {12, 11, 10}; // D12, D11, D10 según esquemático
const bool RELAY_ACTIVE_LOW = false;               // false: se encienden con HIGH

// Capacitancias por paso (uF) -> ajustar tras medir con multímetro
const float STEP_CAPACITANCE_UF[NUM_STEPS] = {22.0f, 47.0f, 69.0f};
const float DISCHARGE_RESISTOR_OHMS = 100000.0f;   // 100 kOhm

// Sensores
const uint8_t PIN_ZMPT = A0;                       // OUT del ZMPT101B
const float NANO_ADC_VOLTS = 5.0f / 1023.0f;

// ADS1115 con GAIN_ONE (+-4.096 V -> 0.125 mV/bit)
const float ADS_LSB_VOLTS = 0.000125f;
const float VOLTAGE_CAL_FACTOR = 245.0f; // Calibrar con multímetro
const float CURRENT_CAL_FACTOR = 30.0f;  // SCT-013-030 (30 A : 1 V)
const float PHASE_TRIM_SAMPLES = 0.0f;

// Muestreo optimizado para 2 KB SRAM del Nano
const uint16_t MEASUREMENT_WINDOW_MS = 200;
const uint8_t MAX_SAMPLES = 45;
const float MIN_CURRENT_THRESHOLD_A = 0.15f;

// Control del Factor de Potencia
const float FP_TARGET_MIN = 0.95f;
const float FP_TARGET_MAX = 0.99f;
const unsigned long MIN_SWITCH_INTERVAL_MS = 3000UL;
const float DISCHARGE_TAU_MULTIPLIER = 5.0f;
const unsigned long MIN_DISCHARGE_LOCKOUT_MS = 6000UL;

// Comunicación Serial
const unsigned long SERIAL_BAUD_RATE = 115200UL;
const unsigned long TELEMETRY_INTERVAL_MS = 500UL;

#endif // POWERGUARD_CONFIG_H
