"""
POWERGuard - Puente USB Serial / Simulador del motor WEG
Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute

Permite:
1) Leer tramas JSON en vivo desde el Arduino Nano por puerto COM/USB (--port COM3).
2) Simular el motor WEG con variación de carga y el banco de tres pasos.
3) Guardar todas las lecturas en un archivo CSV (--log mediciones.csv).
"""

import argparse
import csv
import json
import random
import time
from pathlib import Path
from typing import Dict, List, Optional

from powerguard_model import (
    DEFAULT_BANK_STEPS_UF,
    DEFAULT_FP_TARGET,
    FP_MIN_IMPROVEMENT,
    REFERENCE_VOLTAGE_V,
    WEG_MOTOR_POINTS,
    predict_compensated_state,
    time_to_safe_voltage,
)


CSV_HEADERS = [
    "timestamp_s",
    "load_pct",
    "vrms",
    "irms",
    "p_w",
    "q_var",
    "s_va",
    "fp_measured",
    "fp_model",
    "error_pct",
    "cap_uf",
    "step_1",
    "step_2",
    "step_3",
    "status",
    "source",
    "bank_simulated",
    "irms_measured",
    "q_estimated",
    "s_measured",
    "irms_compensated",
    "fp_compensated",
]


class PowerGuardSimulator:
    """Simula los escenarios de carga y el control del Arduino Nano."""

    def __init__(self, steps_uf=DEFAULT_BANK_STEPS_UF):
        self.steps_uf = list(steps_uf)
        self.num_steps = len(self.steps_uf)
        self.step_state = [0] * self.num_steps
        self.locked_until = [0.0] * self.num_steps
        self.last_switch_t = -10.0

    def step(self, t_sec: float, noise_std: float = 0.004) -> Dict[str, object]:
        # Cambia el nivel de carga cada 18 segundos para demostrar la respuesta automática
        pt = WEG_MOTOR_POINTS[int(t_sec // 18) % len(WEG_MOTOR_POINTS)]
        v_rms = REFERENCE_VOLTAGE_V + random.gauss(0.0, 0.35) if noise_std else REFERENCE_VOLTAGE_V
        fp_base = max(0.01, min(1.0, pt.fp_base + random.gauss(0.0, noise_std)))
        c_total = sum(s * c for s, c in zip(self.step_state, self.steps_uf))
        base = predict_compensated_state(pt.p_in_w, fp_base, 0, v_rms)
        state = predict_compensated_state(pt.p_in_w, fp_base, c_total, v_rms)
        fp = state["FP_new"]
        if t_sec - self.last_switch_t < 3.0:
            status = "SETTLING"
        elif fp >= DEFAULT_FP_TARGET:
            status = "IN_RANGE"
        else:
            add = state["Q_net_var"] > 0
            best = None
            best_fp = fp
            locked = False
            for i, step in enumerate(self.steps_uf):
                if bool(self.step_state[i]) == add:
                    continue
                candidate_cap = c_total + (step if add else -step)
                candidate = predict_compensated_state(pt.p_in_w, fp_base, candidate_cap, v_rms)
                candidate_fp = candidate["FP_new"]
                if candidate_fp <= best_fp or (
                    candidate_fp < DEFAULT_FP_TARGET and candidate_fp - fp < FP_MIN_IMPROVEMENT
                ):
                    continue
                if add and t_sec < self.locked_until[i]:
                    locked = True
                    continue
                best = i, candidate_cap, candidate
                best_fp = candidate_fp
            status = "STEP_LOCKED" if locked else "TARGET_UNREACHABLE"
            if best is not None:
                i, c_total, state = best
                self.step_state[i] = int(add)
                self.last_switch_t = t_sec
                if not add:
                    self.locked_until[i] = t_sec + max(2.0, time_to_safe_voltage(self.steps_uf[i]))
                status = "STEP_ADDED" if add else "STEP_REMOVED"

        model = predict_compensated_state(pt.p_in_w, pt.fp_base, c_total, v_rms)
        fp_model = model["FP_new"]
        err_pct = abs(state["FP_new"] - fp_model) / fp_model * 100.0
        steps = (self.step_state + [0, 0, 0])[:3]
        return {
            "timestamp_s": round(t_sec, 2),
            "load_pct": pt.load_pct,
            "vrms": round(v_rms, 2),
            "irms": round(state["I_rms_a"], 3),
            "p_w": round(pt.p_in_w, 1),
            "q_var": round(state["Q_net_var"], 1),
            "s_va": round(state["S_net_va"], 1),
            "fp_measured": "",
            "fp_model": round(fp_model, 4),
            "error_pct": round(err_pct, 2),
            "cap_uf": round(c_total, 1),
            "step_1": steps[0],
            "step_2": steps[1],
            "step_3": steps[2],
            "status": status,
            "source": "simulation",
            "bank_simulated": "true",
            "irms_measured": "",
            "q_estimated": round(base["Q_load_var"], 1),
            "s_measured": "",
            "irms_compensated": round(state["I_rms_a"], 3),
            "fp_compensated": round(state["FP_new"], 4),
        }


def run_simulation(duration_s: float, interval_s: float, log_path: Optional[Path]) -> None:
    sim = PowerGuardSimulator()
    rows: List[Dict[str, object]] = []
    t = 0.0
    print(f"Iniciando simulación POWERGuard ({duration_s:.0f} s)...")
    while t <= duration_s:
        row = sim.step(t)
        rows.append(row)
        print(
            f"[{row['timestamp_s']:>5.1f}s] Carga={row['load_pct']:>3}% | "
            f"V={row['vrms']:>6.2f}V I={row['irms']:>5.2f}A | "
            f"FP simulado={row['fp_compensated']:.3f} (ref={row['fp_model']:.3f}) | "
            f"C={row['cap_uf']:>4.1f}uF [{row['step_1']}{row['step_2']}{row['step_3']}] {row['status']}"
        )
        t += interval_s
        if interval_s > 0 and duration_s > 0:
            time.sleep(min(interval_s, 0.05))  # Acelerado para pruebas rápidas

    if log_path:
        with log_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_HEADERS)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nRegistro guardado en: {log_path}")


def run_serial_reader(port: str, baud: int, log_path: Optional[Path]) -> None:
    import serial  # Importación diferida para no exigir pyserial en modo simulación

    print(f"Conectando a Arduino Nano en {port} ({baud} baud)... Ctrl+C para detener.")
    writer = None
    csv_file = None
    if log_path:
        csv_file = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADERS)
        writer.writeheader()

    try:
        with serial.Serial(port, baud, timeout=2.0) as ser:
            t0 = time.time()
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line.startswith("{"):
                    continue
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue
                steps = data.get("steps", [0, 0, 0])
                modern = "bank_simulated" in data
                row = {
                    "timestamp_s": round(time.time() - t0, 2),
                    "load_pct": "",
                    "vrms": data.get("vrms", 0.0),
                    "irms": data.get("irms", 0.0),
                    "p_w": data.get("p", 0.0),
                    "q_var": data.get("q", 0.0),
                    "s_va": data.get("s", 0.0),
                    "fp_measured": data.get("fp_measured") if modern else data.get("fp"),
                    "fp_model": "",
                    "error_pct": "",
                    "cap_uf": data.get("cap_uf", 0.0),
                    "step_1": steps[0] if len(steps) > 0 else 0,
                    "step_2": steps[1] if len(steps) > 1 else 0,
                    "step_3": steps[2] if len(steps) > 2 else 0,
                    "status": data.get("status", ""),
                    "source": "arduino" if modern else "legacy",
                    "bank_simulated": str(data["bank_simulated"]).lower() if modern else "",
                    "irms_measured": data.get("irms_measured"),
                    "q_estimated": data.get("q_estimated"),
                    "s_measured": data.get("s_measured"),
                    "irms_compensated": data.get("irms_compensated"),
                    "fp_compensated": data.get("fp_compensated"),
                }
                print(
                    f"[{row['timestamp_s']:>6.1f}s] V={row['vrms']}V I={row['irms']}A "
                    f"P={row['p_w']}W FP={row['fp_measured'] if modern else '—'} "
                    f"FP {'estimado' if modern else 'histórico'}="
                    f"{row['fp_compensated'] if modern else row['fp_measured']} C={row['cap_uf']}uF {row['status']}"
                )
                if writer:
                    writer.writerow(row)
                    csv_file.flush()
    except KeyboardInterrupt:
        print("\nLectura detenida por el usuario.")
    finally:
        if csv_file:
            csv_file.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="POWERGuard Puente Serial y Simulador")
    parser.add_argument("--port", type=str, help="Puerto serial del Arduino Nano (ej. COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--simulate", action="store_true", help="Ejecutar en modo simulación")
    parser.add_argument("--duration", type=float, default=90.0, help="Duración simulación (s)")
    parser.add_argument("--interval", type=float, default=1.0, help="Paso de tiempo simulación (s)")
    parser.add_argument("--log", type=str, default=None, help="Ruta de archivo CSV de salida")
    args = parser.parse_args()

    log_path = Path(args.log) if args.log else None
    if args.simulate or not args.port:
        run_simulation(args.duration, args.interval, log_path)
    else:
        run_serial_reader(args.port, args.baud, log_path)


if __name__ == "__main__":
    main()
