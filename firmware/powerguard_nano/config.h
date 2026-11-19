#ifndef POWERGUARD_CONFIG_H
#define POWERGUARD_CONFIG_H

#include <Arduino.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE - POWERGuard (Equipo 12)
// ============================================================================
// Microcontrolador: Arduino Nano (ATmega328P, 5V, I2C A4=SDA, A5=SCL)
// ADC: ADS1115 (I2C 0x48, 16 bits, 860 SPS)
// Sensores: ZMPT101B (tensión) + SCT-013-030 (corriente, o SCT-013-000 respaldo)
// Conmutación: Módulo de 4 relés (se retiraron MOC3031 y triacs)
// ============================================================================

// --- Pines digitales para los 4 pasos del banco (J1 a J4) ---
const uint8_t NUM_STEPS = 4;
const uint8_t STEP_PINS[NUM_STEPS] = {4, 5, 6, 7}; // D4, D5, D6, D7

// Muchos módulos de relés para Arduino se activan en LOW (optocoplados).
// Cambiar a false si los relés o drivers se activan en HIGH.
const bool RELAY_ACTIVE_LOW = true;

// --- Capacitancias efectivas por paso (en microfaradios) ---
// Pares opuestos (espalda con espalda) de electrolíticos 22 uF y 47 uF:
//   - Par de 22 uF -> ~11.0 uF
//   - Par de 47 uF -> ~23.5 uF
// Ajustar con los valores reales medidos con multímetro en la Semana 15.
const float STEP_CAPACITANCE_UF[NUM_STEPS] = {11.0f, 23.5f, 23.5f, 47.0f};

// --- Resistencia de descarga en paralelo con cada paso (ohms) ---
const float DISCHARGE_RESISTOR_OHMS = 100000.0f; // 100 kOhm (R9-R12)

// --- Canales del ADS1115 ---
// Lectura single-ended respecto a GND (señales centradas en Vcc/2 ~ 2.5 V)
const uint8_t ADS_CH_VOLTAGE = 0; // A0 -> ZMPT101B
const uint8_t ADS_CH_CURRENT = 1; // A1 -> SCT-013 (con divisor 10k/10k + 10uF)

// --- Calibración de sensores ---
// ADS1115 con GAIN_ONE (+-4.096 V) -> 0.125 mV/bit = 0.000125 V/count
// Si se usa Vcc=5V y GAIN_TWOTHIRDS (+-6.144 V) -> 0.1875 mV/bit
const float ADS_LSB_VOLTS = 0.000125f;

// ZMPT101B: ajustar con multímetro de referencia (Vrms_real / Vrms_adc_ac)
const float VOLTAGE_CAL_FACTOR = 245.0f;

// Selección de sensor de corriente:
// 1 = SCT-013-030 (30 A : 1 V, burden integrado -> 30.0 A/V)
// 2 = SCT-013-000 (100 A : 50 mA, requiere R_burden externa ~33 Ohm)
#define CURRENT_SENSOR_MODEL 1

#if CURRENT_SENSOR_MODEL == 1
const float CURRENT_CAL_FACTOR = 30.0f; // A por Voltio RMS en salida del sensor
#else
const float SCT000_RATIO = 2000.0f;     // 100 A / 0.05 A
const float SCT000_BURDEN_OHM = 33.0f;  // Resistencia externa de carga
const float CURRENT_CAL_FACTOR = SCT000_RATIO / SCT000_BURDEN_OHM;
#endif

// Ajuste fino de fase por software (en radianes o fracción de muestra)
// Compensa el desfase residual de los transformadores (ZMPT101B ~0.33° + SCT-013)
const float PHASE_TRIM_SAMPLES = 0.0f;

// --- Parámetros de muestreo ---
const float LINE_FREQUENCY_HZ = 60.0f;
const uint16_t MEASUREMENT_WINDOW_MS = 200; // ~12 ciclos de 60 Hz por cálculo
const uint16_t MAX_SAMPLES = 90;            // Buffer seguro para ATmega328P (2 KB RAM)
const float MIN_CURRENT_THRESHOLD_A = 0.15f; // Por debajo se considera carga apagada

// --- Parámetros de control del Factor de Potencia ---
const float FP_TARGET_MIN = 0.95f;
const float FP_TARGET_MAX = 0.99f;

// Tiempo mínimo entre conmutaciones de relés (evita rebotes / hunting)
const unsigned long MIN_SWITCH_INTERVAL_MS = 3000UL;

// Factor de seguridad para descarga RC (5 * tau = 5 * R * C asegura < 1% de voltaje inicial)
// Con R = 100 kOhm y C = 47 uF -> 5 * tau = 23.5 s; mínimo 6000 ms para pasos pequeños.
const float DISCHARGE_TAU_MULTIPLIER = 5.0f;
const unsigned long MIN_DISCHARGE_LOCKOUT_MS = 6000UL;

// --- Comunicación Serial ---
const unsigned long SERIAL_BAUD_RATE = 115200UL;
const unsigned long TELEMETRY_INTERVAL_MS = 500UL;

#endif // POWERGUARD_CONFIG_H
