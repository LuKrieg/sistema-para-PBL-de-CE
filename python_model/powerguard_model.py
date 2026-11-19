"""
POWERGuard - Modelo Matemático del Compensador de Factor de Potencia
Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute

Implementa las fórmulas de clase y de la propuesta (Semana 14):
- Potencias: S = V * I, P = S * cos(phi), Q = S * sin(phi), FP = P / S
- Desfase: phi = 360 * f * dt, FP = cos(phi)
- Compensación reactiva: Qc = P * (tan(phi_1) - tan(phi_2)), C = Qc / (w * V^2)
- Descarga RC de capacitores: V(t) = V0 * exp(-t / (R * C))
- Error porcentual: error(%) = |FP_medido - FP_calculado| / FP_calculado * 100
- Predicción del FP para las 16 combinaciones del banco de 4 pasos (J1-J4).
"""

from dataclasses import dataclass
from itertools import product
import math
from typing import Dict, List, Tuple


DEFAULT_VOLTAGE_V = 120.0
DEFAULT_FREQ_HZ = 60.0
DEFAULT_FP_TARGET = 0.95
DEFAULT_FP_MAX = 0.99
DEFAULT_DISCHARGE_R_OHMS = 100_000.0  # R9-R12 = 100 kOhm

# Pasos del banco físico validado (3 pasos: 22 uF, 47 uF, 69 uF -> total 138 uF)
DEFAULT_BANK_STEPS_UF: Tuple[float, float, float] = (22.0, 47.0, 69.0)


@dataclass(frozen=True)
class MotorOperatingPoint:
    """Punto de operación del motor monofásico WEG 0.5 HP (Tabla 6 de la propuesta)."""
    load_pct: int
    p_out_w: float
    efficiency_pct: float
    fp_base: float
    p_in_w: float


# Datos de placa (WEG 0.5 HP, 4 polos, 60 Hz, 110/220 V) y Tabla 6 de la propuesta
WEG_MOTOR_POINTS: List[MotorOperatingPoint] = [
    MotorOperatingPoint(load_pct=0,   p_out_w=60.00,  efficiency_pct=0.0,  fp_base=0.27, p_in_w=60.0),
    MotorOperatingPoint(load_pct=25,  p_out_w=93.25,  efficiency_pct=38.0, fp_base=0.35, p_in_w=245.4),
    MotorOperatingPoint(load_pct=50,  p_out_w=186.50, efficiency_pct=47.2, fp_base=0.43, p_in_w=395.1),
    MotorOperatingPoint(load_pct=75,  p_out_w=279.75, efficiency_pct=55.3, fp_base=0.51, p_in_w=505.9),
    MotorOperatingPoint(load_pct=100, p_out_w=373.00, efficiency_pct=59.5, fp_base=0.59, p_in_w=626.9),
]


def fp_from_phase_delay(dt_seconds: float, freq_hz: float = DEFAULT_FREQ_HZ) -> Tuple[float, float]:
    """Calcula el ángulo de desfase (grados) y el FP a partir de dt (método osciloscopio)."""
    phi_deg = 360.0 * freq_hz * dt_seconds
    fp = math.cos(math.radians(phi_deg))
    return phi_deg, fp


def power_triangle(v_rms: float, i_rms: float, fp: float) -> Dict[str, float]:
    """Calcula S (VA), P (W) y Q (var) a partir de Vrms, Irms y FP."""
    fp_clamped = max(0.0, min(1.0, fp))
    s = v_rms * i_rms
    p = s * fp_clamped
    q = math.sqrt(max(0.0, s * s - p * p))
    return {"S_va": s, "P_w": p, "Q_var": q, "FP": fp_clamped}


def required_compensation(
    p_w: float,
    fp_initial: float,
    fp_target: float = DEFAULT_FP_TARGET,
    v_rms: float = DEFAULT_VOLTAGE_V,
    freq_hz: float = DEFAULT_FREQ_HZ,
) -> Dict[str, float]:
    """
    Calcula la potencia reactiva Qc (var) y capacitancia C (uF) necesarias
    para llevar una carga de FP inicial a FP objetivo.
    """
    phi1 = math.acos(max(0.01, min(1.0, fp_initial)))
    phi2 = math.acos(max(0.01, min(1.0, fp_target)))
    qc_var = p_w * (math.tan(phi1) - math.tan(phi2))
    omega = 2.0 * math.pi * freq_hz
    c_farads = qc_var / (omega * (v_rms ** 2))
    return {
        "phi1_deg": math.degrees(phi1),
        "phi2_deg": math.degrees(phi2),
        "Qc_var": qc_var,
        "C_uf": c_farads * 1e6,
    }


def capacitor_reactive_power(
    c_uf: float,
    v_rms: float = DEFAULT_VOLTAGE_V,
    freq_hz: float = DEFAULT_FREQ_HZ,
) -> float:
    """Potencia reactiva capacitiva Qc (var) aportada por C (uF) a tensión Vrms."""
    omega = 2.0 * math.pi * freq_hz
    c_farads = c_uf * 1e-6
    return omega * c_farads * (v_rms ** 2)


def predict_compensated_state(
    p_in_w: float,
    fp_initial: float,
    c_uf: float,
    v_rms: float = DEFAULT_VOLTAGE_V,
    freq_hz: float = DEFAULT_FREQ_HZ,
) -> Dict[str, float]:
    """
    Predice el estado eléctrico resultante tras conectar una capacitancia C (uF)
    en paralelo con una carga inductiva de potencia activa P_in y FP inicial.
    """
    phi1 = math.acos(max(0.01, min(1.0, fp_initial)))
    q_load = p_in_w * math.tan(phi1)
    q_cap = capacitor_reactive_power(c_uf, v_rms, freq_hz)
    q_net = q_load - q_cap  # >0 inductivo (atrasado), <0 capacitivo (adelantado)
    s_net = math.hypot(p_in_w, q_net)
    fp_new = p_in_w / s_net if s_net > 0 else 1.0
    i_new = s_net / v_rms if v_rms > 0 else 0.0
    return {
        "P_w": p_in_w,
        "Q_load_var": q_load,
        "Q_cap_var": q_cap,
        "Q_net_var": q_net,
        "S_net_va": s_net,
        "I_rms_a": i_new,
        "FP_new": fp_new,
        "tipo": "ATRASADO" if q_net >= 0 else "ADELANTADO",
    }


def measurement_error_pct(fp_measured: float, fp_calculated: float) -> float:
    """Calcula el error porcentual según la ecuación (2) de la propuesta."""
    if fp_calculated == 0:
        return 0.0
    return abs(fp_measured - fp_calculated) / abs(fp_calculated) * 100.0


def discharge_voltage(
    t_seconds: float,
    c_uf: float,
    v0_peak: float = DEFAULT_VOLTAGE_V * math.sqrt(2.0),
    r_ohms: float = DEFAULT_DISCHARGE_R_OHMS,
) -> float:
    """Voltaje instantáneo V(t) = V0 * exp(-t / (R * C)) durante la descarga."""
    tau = r_ohms * (c_uf * 1e-6)
    if tau <= 0:
        return 0.0
    return v0_peak * math.exp(-t_seconds / tau)


def time_to_safe_voltage(
    c_uf: float,
    v_safe: float = 50.0,
    v0_peak: float = DEFAULT_VOLTAGE_V * math.sqrt(2.0),
    r_ohms: float = DEFAULT_DISCHARGE_R_OHMS,
) -> float:
    """Tiempo en segundos para que el capacitor baje de V0_peak a V_safe."""
    if v_safe >= v0_peak or v_safe <= 0:
        return 0.0
    tau = r_ohms * (c_uf * 1e-6)
    return -tau * math.log(v_safe / v0_peak)


def simulate_all_bank_combinations(
    point: MotorOperatingPoint,
    bank_steps_uf: Tuple[float, ...] = DEFAULT_BANK_STEPS_UF,
    v_rms: float = DEFAULT_VOLTAGE_V,
    freq_hz: float = DEFAULT_FREQ_HZ,
) -> List[Dict[str, object]]:
    """
    Predice el FP para cada una de las 2^N combinaciones posibles (000 a 111)
    del banco de capacitores en un punto de operación del motor.
    """
    results = []
    for bits in product([0, 1], repeat=len(bank_steps_uf)):
        c_total = sum(b * c for b, c in zip(bits, bank_steps_uf))
        state = predict_compensated_state(point.p_in_w, point.fp_base, c_total, v_rms, freq_hz)
        in_range = (
            DEFAULT_FP_TARGET <= state["FP_new"] <= DEFAULT_FP_MAX
            and state["tipo"] == "ATRASADO"
        )
        results.append({
            "steps": "".join(str(b) for b in bits),
            "C_total_uf": round(c_total, 2),
            "Q_net_var": round(state["Q_net_var"], 2),
            "I_rms_a": round(state["I_rms_a"], 3),
            "FP_pred": round(state["FP_new"], 4),
            "tipo": state["tipo"],
            "en_rango_095_099": in_range,
        })
    results.sort(key=lambda r: r["C_total_uf"])
    return results


def print_report() -> None:
    """Imprime un resumen completo del modelo teórico para verificación del equipo."""
    print("=" * 78)
    print("POWERGuard - Modelo Teórico del Motor WEG 0.5 HP (120 V, 60 Hz)")
    print("=" * 78)
    print(f"{'Carga':>6} | {'Pin(W)':>7} | {'FP base':>7} | {'Qc req(var)':>11} | {'C req(uF)':>9}")
    print("-" * 55)
    for pt in WEG_MOTOR_POINTS:
        comp = required_compensation(pt.p_in_w, pt.fp_base, DEFAULT_FP_TARGET)
        print(
            f"{pt.load_pct:>5}% | {pt.p_in_w:>7.1f} | {pt.fp_base:>7.2f} | "
            f"{comp['Qc_var']:>11.1f} | {comp['C_uf']:>9.1f}"
        )

    print("\nTiempos de descarga RC (R = 100 kOhm, V0 = 169.7 V -> 50 V):")
    for idx, c_uf in enumerate(DEFAULT_BANK_STEPS_UF, start=1):
        t_safe = time_to_safe_voltage(c_uf)
        tau = DEFAULT_DISCHARGE_R_OHMS * c_uf * 1e-6
        print(f"  Paso J{idx} ({c_uf:>4.1f} uF): tau = {tau:.2f} s, t(<50V) = {t_safe:.2f} s")

    print("\nCombinaciones válidas en rango [0.95, 0.99] (banco 22 / 47 / 69 uF):")
    for pt in WEG_MOTOR_POINTS:
        combos = simulate_all_bank_combinations(pt)
        valid = [c for c in combos if c["en_rango_095_099"]]
        best = valid[0] if valid else min(combos, key=lambda x: abs(x["FP_pred"] - 0.97))
        print(
            f"  Carga {pt.load_pct:>3}% -> Mejor combinación [{best['steps']}] "
            f"({best['C_total_uf']:>5.1f} uF): FP = {best['FP_pred']:.3f} ({best['tipo']}), "
            f"I = {best['I_rms_a']:.2f} A"
        )


if __name__ == "__main__":
    print_report()
