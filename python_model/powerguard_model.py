"""
POWERGuard - Modelo Matemático del Compensador de Factor de Potencia
Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute

Implementa las fórmulas de clase y de la propuesta (Semana 14):
- Potencias: S = V * I, P = S * cos(phi), Q = S * sin(phi), FP = P / S
- Desfase: phi = 360 * f * dt, FP = cos(phi)
- Compensación reactiva: Qc = P * (tan(phi_1) - tan(phi_2)), C = Qc / (w * V^2)
- Descarga RC de capacitores: V(t) = V0 * exp(-t / (R * C))
- Error porcentual: error(%) = |FP_medido - FP_calculado| / FP_calculado * 100
- Predicción del FP para las ocho combinaciones del banco de tres pasos.
"""

from dataclasses import dataclass
from itertools import product
import math
from typing import Dict, List, Tuple


DEFAULT_VOLTAGE_V = 120.0
DEFAULT_FREQ_HZ = 60.0
DEFAULT_FP_TARGET = 0.95
DEFAULT_FP_MAX = 1.0
DEFAULT_DISCHARGE_R_OHMS = 100_000.0  # R9-R12 = 100 kOhm
REFERENCE_VOLTAGE_V = 118.0
MOTOR_RATED_OUTPUT_W = 367.75  # 0.5 CV, dato aportado para el proyecto
FP_MIN_IMPROVEMENT = 0.005

# Banco simulado por los tres LEDs del prototipo.
DEFAULT_BANK_STEPS_UF: Tuple[float, float, float] = (60.0, 60.0, 60.0)


@dataclass(frozen=True)
class MotorOperatingPoint:
    """Escenario de simulación del motor WEG."""
    load_pct: int
    p_out_w: float
    fp_base: float
    p_in_w: float

    @property
    def efficiency_pct(self) -> float | None:
        return motor_efficiency_pct(self.p_in_w, self.p_out_w)

    @property
    def losses_w(self) -> float:
        return self.p_in_w - self.p_out_w


def motor_efficiency_pct(p_in_w: float, p_out_w: float | None) -> float | None:
    """Calcula la eficiencia cuando se conoce la potencia útil del eje."""
    if p_out_w is None or not 0 <= p_out_w <= p_in_w or p_in_w <= 0:
        return None
    return 100.0 * p_out_w / p_in_w


# Vacío: escenario de referencia a 118 V, 8.10 A y FP supuesto de 0.20.
# Carga: Pin y FP de la propuesta; Pout proporcional a carga para la simulación.
WEG_MOTOR_POINTS: List[MotorOperatingPoint] = [
    MotorOperatingPoint(0, 0.0, 0.20, REFERENCE_VOLTAGE_V * 8.10 * 0.20),
    MotorOperatingPoint(25, MOTOR_RATED_OUTPUT_W * 0.25, 0.35, 245.4),
    MotorOperatingPoint(50, MOTOR_RATED_OUTPUT_W * 0.50, 0.43, 395.1),
    MotorOperatingPoint(75, MOTOR_RATED_OUTPUT_W * 0.75, 0.51, 505.9),
    MotorOperatingPoint(100, MOTOR_RATED_OUTPUT_W, 0.59, 626.9),
]


def fp_from_phase_delay(dt_seconds: float, freq_hz: float = DEFAULT_FREQ_HZ) -> Tuple[float, float]:
    """Calcula el ángulo y el factor de desplazamiento a partir de dt."""
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
    *,
    inductive: bool = True,
) -> Dict[str, float]:
    """
    Calcula la potencia reactiva Qc (var) y capacitancia C (uF) necesarias
    para llevar una carga de FP inicial a FP objetivo.
    """
    phi1 = math.acos(fp_initial) * (1 if inductive else -1)
    phi2 = math.acos(fp_target)
    q_load = p_w * math.tan(phi1)
    qc_var = reactive_adjustment(q_load, p_w * math.tan(phi2)) or 0.0
    omega = 2.0 * math.pi * freq_hz
    c_farads = qc_var / (omega * v_rms ** 2) if v_rms > 0 and omega > 0 else 0.0
    return {
        "phi1_deg": math.degrees(phi1),
        "phi2_deg": math.degrees(phi2),
        "Qc_var": qc_var,
        "C_uf": c_farads * 1e6,
    }


def reactive_adjustment(q_var: float, q_limit_var: float | None) -> float | None:
    """Reactiva que hay que cancelar para entrar en la banda en atraso o adelanto."""
    if q_limit_var is None:
        return None
    return q_var - max(-q_limit_var, min(q_limit_var, q_var))


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
    *,
    inductive: bool = True,
) -> Dict[str, float]:
    """
    Predice el estado eléctrico resultante tras conectar una capacitancia C (uF)
    en paralelo con una carga inductiva de potencia activa P_in y FP inicial.
    """
    phi1 = math.acos(fp_initial)
    q_load = p_in_w * math.tan(phi1) * (1 if inductive else -1)
    return compensate_reactive_load(p_in_w, q_load, c_uf, v_rms, freq_hz)


def compensate_reactive_load(
    p_in_w: float, q_load: float, c_uf: float, v_rms: float,
    freq_hz: float = DEFAULT_FREQ_HZ, *, residual_va: float = 0.0,
) -> Dict[str, float | str]:
    """Predicción con tensión casi sinusoidal y residuo no activo constante."""
    q_cap = capacitor_reactive_power(c_uf, v_rms, freq_hz)
    q_net = q_load - q_cap  # >0 inductivo (atrasado), <0 capacitivo (adelantado)
    s_net = math.hypot(p_in_w, q_net, residual_va)
    fp_new = p_in_w / s_net if s_net > 0 else 0.0
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


def memoria_de_calculo(
    v_rms: float,
    i_rms: float,
    p_w: float,
    inductiva: bool = True,
    c_uf: float = 0.0,
    fp_target: float = DEFAULT_FP_TARGET,
    freq_hz: float = DEFAULT_FREQ_HZ,
    *,
    q_var: float | None = None,
    sinusoidal_model: bool = False,
) -> Dict[str, float | None]:
    """Reproduce el modelo del firmware para el banco simulado.

    El modo sinusoidal reproduce el banco simulado del firmware: usa P y S
    para la magnitud de Q y la cuadratura, si está disponible, para el signo.
    Sin ese modo, una Q suministrada conserva el residuo en la predicción.
    """
    if v_rms <= 0 or i_rms <= 0 or freq_hz <= 0:
        raise ValueError("Se requieren tensión, corriente y frecuencia positivas.")
    omega = 2.0 * math.pi * freq_hz
    s = v_rms * i_rms
    p_w = max(-s, min(s, p_w))
    fp = p_w / s
    nonactive = math.sqrt(max(0.0, s ** 2 - p_w ** 2))
    q = nonactive * (1 if inductiva else -1) if q_var is None else max(-nonactive, min(nonactive, q_var))
    measured_residual = math.sqrt(max(0.0, (nonactive - abs(q)) * (nonactive + abs(q))))
    if sinusoidal_model:
        q = math.copysign(nonactive, q)
    residual = 0.0 if sinusoidal_model else measured_residual
    theta = math.atan2(q, p_w)
    s_displacement = math.hypot(p_w, q)
    z = v_rms ** 2 / s_displacement if s_displacement else None
    r = z * math.cos(theta) if z is not None else None
    x = z * math.sin(theta) if z is not None else None

    g = p_w / v_rms ** 2
    b = q / v_rms ** 2

    bc = omega * c_uf * 1e-6
    bn = b - bc
    state = compensate_reactive_load(p_w, q, c_uf, v_rms, freq_hz, residual_va=residual)

    tan1 = q / p_w if p_w else None
    tan2 = math.tan(math.acos(fp_target))
    q_limit_sq = (p_w * tan2) ** 2 - residual ** 2
    q_limit = math.sqrt(q_limit_sq) if p_w > 0 and q_limit_sq >= 0 else None
    q_req = reactive_adjustment(state["Q_net_var"], q_limit)
    c_req_uf = q_req / (omega * v_rms ** 2) * 1e6 if q_req is not None else None
    c_ideal_uf = b / omega * 1e6

    return {
        "S_va": s, "FP": fp, "theta_deg": math.degrees(theta), "Q_var": q,
        "Residual_va": residual,
        "Residuo_diagnostico_va": measured_residual,
        "Z_ohm": z, "R_ohm": r, "X_ohm": x, "FP_R_sobre_Z": math.cos(theta) if z else None,
        "G_S": g, "B_S": b,
        "Bc_S": bc, "Qc_var": state["Q_cap_var"],
        "Bn_S": bn, "FP_compensado": state["FP_new"], "theta_compensado_deg": math.degrees(math.atan2(bn, g)),
        "I_compensada_a": state["I_rms_a"], "S_compensada_va": state["S_net_va"], "Q_compensada_var": state["Q_net_var"],
        "P_desde_G_w": v_rms ** 2 * g,
        "tan1": tan1, "tan2": tan2, "Qc_requerida_var": q_req,
        "C_requerida_uf": c_req_uf, "C_ideal_fp1_uf": c_ideal_uf,
    }


def correccion_por_admitancia(g_s: float, b_s: float, v_rms: float, freq_hz: float = DEFAULT_FREQ_HZ) -> Dict[str, float]:
    """
    Corrección con un capacitor (método de admitancia, caso 'La celda que se calienta'):
    Y = G − jB; el capacitor suma +jωC, así que para cancelar la parte imaginaria ωC = B.
    """
    omega = 2.0 * math.pi * freq_hz
    c = b_s / omega
    return {
        "FP_antes": g_s / math.hypot(g_s, b_s),   # FP = G/|Y| = cos θ
        "C_uf": c * 1e6,
        "Qc_var": omega * c * v_rms ** 2,
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
    v_rms: float = REFERENCE_VOLTAGE_V,
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
        in_range = DEFAULT_FP_TARGET <= state["FP_new"] <= DEFAULT_FP_MAX
        results.append({
            "steps": "".join(str(b) for b in bits),
            "C_total_uf": round(c_total, 2),
            "Q_net_var": round(state["Q_net_var"], 2),
            "I_rms_a": round(state["I_rms_a"], 3),
            "FP_pred": round(state["FP_new"], 4),
            "tipo": state["tipo"],
            "en_rango": in_range,
        })
    results.sort(key=lambda r: r["C_total_uf"])
    return results


def best_bank_combination(combos: List[Dict[str, object]]) -> Dict[str, object]:
    """Elige la menor capacitancia que alcanza la meta, o el mayor FP disponible."""
    return next((combo for combo in combos if combo["en_rango"]), None) or max(
        combos, key=lambda combo: combo["FP_pred"]
    )


def print_report() -> None:
    """Imprime un resumen completo del modelo teórico para verificación del equipo."""
    print("=" * 78)
    print(f"POWERGuard - Escenarios del motor WEG 0.5 CV ({REFERENCE_VOLTAGE_V:g} V, 60 Hz)")
    print("=" * 78)
    print(f"{'Carga':>6} | {'Pin(W)':>7} | {'FP base':>7} | {'Qc req(var)':>11} | {'C req(uF)':>9}")
    print("-" * 55)
    for pt in WEG_MOTOR_POINTS:
        comp = required_compensation(pt.p_in_w, pt.fp_base, v_rms=REFERENCE_VOLTAGE_V)
        print(
            f"{pt.load_pct:>5}% | {pt.p_in_w:>7.1f} | {pt.fp_base:>7.2f} | "
            f"{comp['Qc_var']:>11.1f} | {comp['C_uf']:>9.1f}"
        )

    print("\nTiempos de descarga RC (R = 100 kOhm, V0 = 169.7 V -> 50 V):")
    for idx, c_uf in enumerate(DEFAULT_BANK_STEPS_UF, start=1):
        t_safe = time_to_safe_voltage(c_uf)
        tau = DEFAULT_DISCHARGE_R_OHMS * c_uf * 1e-6
        print(f"  Paso J{idx} ({c_uf:>4.1f} uF): tau = {tau:.2f} s, t(<50V) = {t_safe:.2f} s")

    print("\nOcho combinaciones por carga (60 / 60 / 60 uF, meta 0.95–1.00):")
    for pt in WEG_MOTOR_POINTS:
        combos = simulate_all_bank_combinations(pt)
        best = best_bank_combination(combos)
        print(
            f"  Carga {pt.load_pct:>3}%: Pout = {pt.p_out_w:.1f} W, "
            f"pérdidas = {pt.losses_w:.1f} W, eficiencia = {pt.efficiency_pct:.1f}%"
        )
        for combo in combos:
            chosen = " <- selección" if combo is best else ""
            print(f"    {combo['steps']}  {combo['C_total_uf']:5.1f} uF  "
                  f"FP={combo['FP_pred']:.4f}  {combo['tipo']}  I={combo['I_rms_a']:.2f} A{chosen}")


if __name__ == "__main__":
    print_report()
