import math
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from powerguard_model import (
    WEG_MOTOR_POINTS,
    discharge_voltage,
    fp_from_phase_delay,
    measurement_error_pct,
    power_triangle,
    predict_compensated_state,
    required_compensation,
    simulate_all_bank_combinations,
    time_to_safe_voltage,
)
from serial_bridge import PowerGuardSimulator


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

    def test_measurement_error_pct(self):
        err = measurement_error_pct(0.96, 0.98)
        self.assertAlmostEqual(err, (0.02 / 0.98) * 100.0, places=4)

    def test_bank_combinations_generated(self):
        combos = simulate_all_bank_combinations(WEG_MOTOR_POINTS[2])  # 50% carga
        self.assertEqual(len(combos), 8)
        # Debe existir al menos una combinación cercana o dentro de 0.95-0.99
        best = min(combos, key=lambda c: abs(c["FP_pred"] - 0.97))
        self.assertGreaterEqual(best["FP_pred"], 0.93)

    def test_simulator_compensates_to_target(self):
        sim = PowerGuardSimulator()
        # Avanzar 20 segundos en carga del 50% (t=36s..56s -> load_idx=2)
        last_row = None
        for t in range(36, 56):
            last_row = sim.step(float(t), noise_std=0.0)
        self.assertIsNotNone(last_row)
        self.assertGreaterEqual(last_row["fp_measured"], 0.94)


if __name__ == "__main__":
    unittest.main()
