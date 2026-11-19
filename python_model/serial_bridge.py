"""
POWERGuard - Puente USB Serial / Simulador de Motor WEG 0.5 HP
Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute

Permite:
1) Leer tramas JSON en vivo desde el Arduino Nano por puerto COM/USB (--port COM3).
2) Simular el motor WEG 0.5 HP con variación de carga y el controlador automático
   del banco de 4 pasos (--simulate) para pruebas sin hardware.
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
    DEFAULT_FREQ_HZ,
    DEFAULT_VOLTAGE_V,
    WEG_MOTOR_POINTS,
    predict_compensated_state,
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
    "step_4",
    "status",
]


class PowerGuardSimulator:
    """Simula la física del motor WEG 0.5 HP y el algoritmo del Arduino Nano."""

    def __init__(self, steps_uf=DEFAULT_BANK_STEPS_UF):
        self.steps_uf = list(steps_uf)
        self.step_state = [0, 0, 0, 0]
        self.last_disconnect_t = [0.0, 0.0, 0.0, 0.0]
        self.last_switch_t = -10.0
        self.awaiting_check = False
        self.last_added_step = -1
        self.fp_before_step = 0.0
        self.status = "READY"
        self.load_idx = 2  # Inicia en 50% de carga

    def step(self, t_sec: float, noise_std: float = 0.004) -> Dict[str, object]:
        # Cambia el nivel de carga cada 18 segundos para demostrar la respuesta automática
        self.load_idx = int(t_sec // 18) % len(WEG_MOTOR_POINTS)
        pt = WEG_MOTOR_POINTS[self.load_idx]

        v_rms = DEFAULT_VOLTAGE_V + random.gauss(0.0, 0.35)
        c_total = sum(s * c for s, c in zip(self.step_state, self.steps_uf))
        pred = predict_compensated_state(pt.p_in_w, pt.fp_base, c_total, v_rms, DEFAULT_FREQ_HZ)

        fp_meas = max(0.1, min(1.0, pred["FP_new"] + random.gauss(0.0, noise_std)))
        i_meas = max(0.05, pred["I_rms_a"] + random.gauss(0.0, 0.03))
        s_meas = v_rms * i_meas
        p_meas = s_meas * fp_meas
        q_meas = (max(0.0, s_meas ** 2 - p_meas ** 2)) ** 0.5

        # Lógica de control idéntica a powerguard_nano.ino
        if (t_sec - self.last_switch_t) >= 3.0:
            if self.awaiting_check and self.last_added_step >= 0:
                self.awaiting_check = False
                if fp_meas < self.fp_before_step:
                    idx = self.last_added_step
                    self.step_state[idx] = 0
                    self.last_disconnect_t[idx] = t_sec
                    self.last_switch_t = t_sec
                    self.status = "OVERCOMP_REVERT"
                self.last_added_step = -1
            elif fp_meas < 0.95:
                avail = [
                    i for i in range(4)
                    if self.step_state[i] == 0 and (t_sec - self.last_disconnect_t[i] >= 6.0 or self.last_disconnect_t[i] == 0.0)
                ]
                if avail:
                    best = min(avail, key=lambda i: self.steps_uf[i])
                    self.fp_before_step = fp_meas
                    self.step_state[best] = 1
                    self.last_added_step = best
                    self.awaiting_check = True
                    self.last_switch_t = t_sec
                    self.status = "STEP_ADDED"
                else:
                    self.status = "MAX_CAP_OR_LOCKED"
            elif fp_meas > 0.99:
                active = [i for i in range(4) if self.step_state[i] == 1]
                if active:
                    rem = min(active, key=lambda i: self.steps_uf[i])
                    self.step_state[rem] = 0
                    self.last_disconnect_t[rem] = t_sec
                    self.last_switch_t = t_sec
                    self.status = "STEP_REMOVED"
                else:
                    self.status = "IN_RANGE"
            else:
                self.status = "IN_RANGE"
        else:
            self.status = "SETTLING"

        err_pct = abs(fp_meas - pred["FP_new"]) / pred["FP_new"] * 100.0
        return {
            "timestamp_s": round(t_sec, 2),
            "load_pct": pt.load_pct,
            "vrms": round(v_rms, 2),
            "irms": round(i_meas, 3),
            "p_w": round(p_meas, 1),
            "q_var": round(q_meas, 1),
            "s_va": round(s_meas, 1),
            "fp_measured": round(fp_meas, 4),
            "fp_model": round(pred["FP_new"], 4),
            "error_pct": round(err_pct, 2),
            "cap_uf": round(c_total, 1),
            "step_1": self.step_state[0],
            "step_2": self.step_state[1],
            "step_3": self.step_state[2],
            "step_4": self.step_state[3],
            "status": self.status,
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
            f"FP={row['fp_measured']:.3f} (mod={row['fp_model']:.3f}, err={row['error_pct']:.2f}%) | "
            f"C={row['cap_uf']:>4.1f}uF [{row['step_1']}{row['step_2']}{row['step_3']}{row['step_4']}] {row['status']}"
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
                steps = data.get("steps", [0, 0, 0, 0])
                row = {
                    "timestamp_s": round(time.time() - t0, 2),
                    "load_pct": "",
                    "vrms": data.get("vrms", 0.0),
                    "irms": data.get("irms", 0.0),
                    "p_w": data.get("p", 0.0),
                    "q_var": data.get("q", 0.0),
                    "s_va": data.get("s", 0.0),
                    "fp_measured": data.get("fp", 0.0),
                    "fp_model": "",
                    "error_pct": "",
                    "cap_uf": data.get("cap_uf", 0.0),
                    "step_1": steps[0] if len(steps) > 0 else 0,
                    "step_2": steps[1] if len(steps) > 1 else 0,
                    "step_3": steps[2] if len(steps) > 2 else 0,
                    "step_4": steps[3] if len(steps) > 3 else 0,
                    "status": data.get("status", ""),
                }
                print(
                    f"[{row['timestamp_s']:>6.1f}s] V={row['vrms']}V I={row['irms']}A "
                    f"P={row['p_w']}W FP={row['fp_measured']} C={row['cap_uf']}uF {row['status']}"
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
