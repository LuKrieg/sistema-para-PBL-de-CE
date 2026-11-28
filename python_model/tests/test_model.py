import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from powerguard_model import (
    REFERENCE_VOLTAGE_V,
    WEG_MOTOR_POINTS,
    correccion_por_admitancia,
    discharge_voltage,
    memoria_de_calculo,
    fp_from_phase_delay,
    measurement_error_pct,
    power_triangle,
    predict_compensated_state,
    required_compensation,
    simulate_all_bank_combinations,
    time_to_safe_voltage,
)
from serial_bridge import PowerGuardSimulator
from validate_measurements import compare_csv_row


class TestPowerGuardModel(unittest.TestCase):
    def test_fp_from_phase_delay(self):
        # 60 Hz: 1 ciclo = 16.6667 ms. 60 grados = 1/6 de ciclo
        dt = (1.0 / 60.0) / 6.0
        phi, fp = fp_from_phase_delay(dt, 60.0)
        self.assertAlmostEqual(phi, 60.0, places=4)
        self.assertAlmostEqual(fp, 0.5, places=4)

    def test_power_triangle(self):
        tri = power_triangle(120.0, 10.0, 0.8)
        self.assertAlmostEqual(tri["S_va"], 1200.0, places=2)
        self.assertAlmostEqual(tri["P_w"], 960.0, places=2)
        self.assertAlmostEqual(tri["Q_var"], 720.0, places=2)

    def test_required_compensation_full_load(self):
        # Con P_in = 626.9 W, FP = 0.59, FP_obj = 0.95 a 120V/60Hz -> ~120.1 uF
        full_load = WEG_MOTOR_POINTS[-1]
        comp = required_compensation(full_load.p_in_w, full_load.fp_base, 0.95, 120.0, 60.0)
        self.assertAlmostEqual(comp["C_uf"], 120.1, delta=0.5)

    def test_discharge_rc_curve(self):
        # Paso de 47 uF con 100 kOhm: tau = 4.7 s; a 50 V desde 170 V toma ~5.75 s
        t_safe = time_to_safe_voltage(47.0, v_safe=50.0, v0_peak=170.0, r_ohms=100_000.0)
        self.assertAlmostEqual(t_safe, 5.75, delta=0.1)
        v_after = discharge_voltage(t_safe, 47.0, v0_peak=170.0, r_ohms=100_000.0)
        self.assertAlmostEqual(v_after, 50.0, delta=0.1)

    def test_correccion_diapositiva_celda(self):
        # Diapositiva "Corrección con un capacitor": Y_tab = 0.245 − j0.135 S, 240 V, 60 Hz
        # -> C = 0.135/377 = 358.1 uF, Qc = 7.78 kVAR
        r = correccion_por_admitancia(0.245, 0.135, 240.0)
        self.assertAlmostEqual(r["C_uf"], 358.1, delta=0.2)
        self.assertAlmostEqual(r["Qc_var"] / 1000.0, 7.78, delta=0.01)

    def test_memoria_de_calculo_consistente(self):
        # Medición real del motor en vacío: 118.5 V, 7.90 A, 186.5 W
        m = memoria_de_calculo(118.5, 7.90, 186.5, inductiva=True, c_uf=166.0, fp_target=0.95)
        # Triángulo de potencias y de impedancia tienen el mismo ángulo: FP = P/S = R/|Z|
        self.assertAlmostEqual(m["FP"], m["FP_R_sobre_Z"], places=9)
        # P = V²·G (la parte real de la admitancia lleva la potencia activa)
        self.assertAlmostEqual(m["P_desde_G_w"], 186.5, places=6)
        # Q' = Q − Qc  (admitancia y triángulo de potencias dan lo mismo)
        self.assertAlmostEqual(m["Q_compensada_var"], m["Q_var"] - m["Qc_var"], places=6)
        # El método de admitancia coincide con predict_compensated_state (Q − Qc)
        ref = predict_compensated_state(186.5, m["FP"], 166.0, 118.5)
        self.assertAlmostEqual(m["FP_compensado"], ref["FP_new"], places=9)
        # Con 166 uF el FP ya está en la meta y no necesita ajuste adicional.
        self.assertTrue(0.95 <= m["FP_compensado"] <= 1.0)
        req = required_compensation(186.5, m["FP"], 0.95, 118.5)
        self.assertAlmostEqual(m["C_requerida_uf"], 0.0, places=6)
        uncompensated = memoria_de_calculo(118.5, 7.90, 186.5)
        self.assertAlmostEqual(uncompensated["C_requerida_uf"], req["C_uf"], places=6)

    def test_distortion_remains_when_reactive_power_is_cancelled(self):
        p, q, residual, voltage = 200.0, 400.0, 100.0, 118.0
        current = math.hypot(p, q, residual) / voltage
        cancellation_uf = q / (2 * math.pi * 60 * voltage ** 2) * 1e6
        m = memoria_de_calculo(voltage, current, p, c_uf=cancellation_uf, q_var=q)
        self.assertAlmostEqual(m["Q_compensada_var"], 0.0)
        self.assertAlmostEqual(m["FP_compensado"], p / math.hypot(p, residual))
        self.assertIsNone(m["C_requerida_uf"])

    def test_signed_adjustment_and_zero_active_power(self):
        capacitive = required_compensation(200, 0.7, inductive=False)
        self.assertLess(capacitive["C_uf"], 0)
        m = memoria_de_calculo(118, 5, 0, q_var=-590)
        self.assertEqual(m["FP"], 0)
        self.assertIsNone(m["C_requerida_uf"])

    def test_sinusoidal_prediction_reproduces_motor_capture(self):
        m = memoria_de_calculo(117.2, 8.14, 191.4, c_uf=180, q_var=891.5, sinusoidal_model=True)
        self.assertAlmostEqual(m["FP"], 0.201, places=3)
        self.assertGreater(m["FP_compensado"], 0.999)
        self.assertGreater(m["Residuo_diagnostico_va"], 280)
        self.assertEqual(m["Residual_va"], 0)

    def test_measurement_error_pct(self):
        err = measurement_error_pct(0.96, 0.98)
        self.assertAlmostEqual(err, (0.02 / 0.98) * 100.0, places=4)

    def test_bank_combinations_generated(self):
        combos = simulate_all_bank_combinations(WEG_MOTOR_POINTS[0])
        self.assertEqual(len(combos), 8)
        self.assertEqual({c["C_total_uf"] for c in combos}, {0, 60, 120, 180})
        self.assertEqual(combos[-1]["tipo"], "ADELANTADO")
        self.assertTrue(combos[-1]["en_rango"])
        self.assertAlmostEqual(combos[-1]["FP_pred"], 0.9990, places=4)
        quarter_load = simulate_all_bank_combinations(WEG_MOTOR_POINTS[1], v_rms=REFERENCE_VOLTAGE_V)
        self.assertFalse(quarter_load[-1]["en_rango"])
        half_load_at_120 = simulate_all_bank_combinations(WEG_MOTOR_POINTS[2], v_rms=120)
        self.assertFalse(any(combo["en_rango"] for combo in half_load_at_120))

    def test_validator_compares_raw_fp_and_keeps_legacy_reference(self):
        measured = {
            "source": "arduino", "bank_simulated": "true", "vrms": "118",
            "fp_measured": "0.20", "fp_compensated": "0.9990", "p_w": "191.2", "cap_uf": "180",
        }
        source, fp, reference = compare_csv_row(measured)
        self.assertEqual(source, "arduino")
        self.assertEqual(fp, 0.2)
        self.assertAlmostEqual(reference, 0.2)
        legacy = {"fp_measured": "0.96", "fp_model": "0.97"}
        self.assertEqual(compare_csv_row(legacy), ("legacy", 0.96, 0.97))

    def test_simulator_compensates_to_target(self):
        sim = PowerGuardSimulator()
        # Avanzar 20 segundos en carga del 50% (t=36s..56s -> load_idx=2)
        last_row = None
        for t in range(36, 56):
            last_row = sim.step(float(t), noise_std=0.0)
        self.assertIsNotNone(last_row)
        self.assertGreaterEqual(last_row["fp_compensated"], 0.95)
        self.assertEqual(last_row["fp_measured"], "")

    def test_simulator_keeps_good_leading_fp_and_removes_excess(self):
        sim = PowerGuardSimulator()
        rows = [sim.step(float(t), noise_std=0.0) for t in range(29)]
        self.assertEqual(rows[6]["status"], "STEP_ADDED")
        self.assertEqual(rows[15]["status"], "IN_RANGE")
        self.assertLess(rows[15]["q_var"], 0)
        self.assertEqual(rows[18]["status"], "STEP_REMOVED")
        for row in rows:
            self.assertEqual(row["cap_uf"], 60 * sum(row[f"step_{i}"] for i in (1, 2, 3)))


if __name__ == "__main__":
    unittest.main()
