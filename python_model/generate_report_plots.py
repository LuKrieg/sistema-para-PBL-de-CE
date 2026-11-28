"""
POWERGuard - Generador de Gráficas PNG para el Informe Técnico
Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute

Genera 4 gráficas en alta resolución (300 DPI) numeradas secuencialmente
(Prueba_1, Prueba_2, ...) para no confundir los registros entre pruebas:
  1. Prueba_N_fp_y_corriente.png      -> FP, Irms y capacitancia activa en el tiempo
  2. Prueba_N_modelo_vs_medicion.png  -> FP medido vs modelo Python y error (%) < 4%
  3. Prueba_N_compensacion_carga.png  -> Escenarios WEG (0%-100%): sin compensar vs compensado
  4. Prueba_N_descarga_rc.png         -> Curvas de descarga RC (100 kOhm) hasta < 50 V

Uso:
  python generate_report_plots.py --input Prueba_1.csv
  python generate_report_plots.py --input Prueba_1.csv --name Prueba_1
"""

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from powerguard_model import (
    DEFAULT_BANK_STEPS_UF,
    DEFAULT_DISCHARGE_R_OHMS,
    DEFAULT_FP_MAX,
    DEFAULT_FP_TARGET,
    DEFAULT_VOLTAGE_V,
    REFERENCE_VOLTAGE_V,
    WEG_MOTOR_POINTS,
    best_bank_combination,
    discharge_voltage,
    measurement_error_pct,
    simulate_all_bank_combinations,
    time_to_safe_voltage,
)
from validate_measurements import compare_csv_row


def detect_next_prueba_name(out_dir: Path) -> str:
    """Busca carpetas/archivos Prueba_N existentes y devuelve el siguiente nombre."""
    max_n = 0
    pattern = re.compile(r"^Prueba_(\d+)")
    if out_dir.exists():
        for p in out_dir.iterdir():
            m = pattern.match(p.name)
            if m:
                max_n = max(max_n, int(m.group(1)))
    return f"Prueba_{max_n + 1}"


def load_csv_rows(csv_path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for idx, r in enumerate(reader):
            vrms = float(r.get("vrms") or DEFAULT_VOLTAGE_V)
            source, fp, reference = compare_csv_row(r)
            raw_current = r.get("irms_measured") if source == "arduino" else r.get("irms")
            irms = float(raw_current) if raw_current not in (None, "") else math.nan
            p_w = float(r.get("p_w") or 0.0)
            cap_uf = float(r.get("cap_uf") or 0.0)
            t_s = float(r.get("timestamp_s") or idx * 0.5)
            err = measurement_error_pct(fp, reference) if fp is not None and reference else math.nan
            fp_est = r.get("fp_compensated") if source == "arduino" else None
            i_est = r.get("irms_compensated") if source == "arduino" else None

            rows.append({
                "t": t_s,
                "vrms": vrms,
                "irms": irms,
                "p_w": p_w,
                "fp_meas": fp if fp is not None else math.nan,
                "fp_mod": reference if reference is not None else math.nan,
                "fp_est": float(fp_est) if fp_est not in (None, "") else math.nan,
                "i_est": float(i_est) if i_est not in (None, "") else math.nan,
                "error_pct": err,
                "cap_uf": cap_uf,
            })
    return rows


def plot_fp_and_current(rows: List[Dict[str, float]], out_path: Path, title_prefix: str) -> None:
    t = [r["t"] - rows[0]["t"] for r in rows]
    fp = [r["fp_meas"] for r in rows]
    irms = [r["irms"] for r in rows]
    cap = [r["cap_uf"] for r in rows]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    ax1.axhspan(DEFAULT_FP_TARGET, DEFAULT_FP_MAX, color="#10b981", alpha=0.18, label="Meta FP (0.95–1.00)")
    ax1.axhline(DEFAULT_FP_TARGET, color="#10b981", linestyle="--", linewidth=1)
    ax1.plot(t, fp, color="#0284c7", linewidth=2, label="FP registrado según el origen del CSV")
    if any(math.isfinite(r["fp_est"]) for r in rows):
        ax1.plot(t, [r["fp_est"] for r in rows], color="#7c3aed", linestyle="--", label="FP estimado con banco")
    ax1.set_ylabel("Factor de Potencia")
    ax1.set_ylim(0.2, 1.05)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right")
    ax1.set_title(f"{title_prefix} · Evolución de Factor de Potencia, Corriente y Capacitancia")

    ax2.plot(t, irms, color="#d97706", linewidth=2, label="Corriente Irms (A)")
    if any(math.isfinite(r["i_est"]) for r in rows):
        ax2.plot(t, [r["i_est"] for r in rows], color="#7c3aed", linestyle="--", label="Corriente estimada con banco")
    ax2.set_ylabel("Corriente RMS (A)", color="#d97706")
    ax2.set_xlabel("Tiempo (s)")
    ax2.grid(True, alpha=0.3)

    ax2_r = ax2.twinx()
    ax2_r.step(t, cap, where="post", color="#6366f1", linewidth=1.6, linestyle="-.", label="Capacitancia (µF)")
    ax2_r.set_ylabel("Capacitancia conectada (µF)", color="#6366f1")

    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_r.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def plot_model_vs_measured(rows: List[Dict[str, float]], out_path: Path, title_prefix: str) -> None:
    t = [r["t"] - rows[0]["t"] for r in rows]
    fp_meas = [r["fp_meas"] for r in rows]
    fp_mod = [r["fp_mod"] for r in rows]
    err = [r["error_pct"] for r in rows]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    ax1.plot(t, fp_mod, color="#0f172a", linewidth=2, linestyle="--", label="FP Modelo Python")
    ax1.plot(t, fp_meas, color="#0284c7", linewidth=1.8, label="FP registrado según el origen del CSV")
    ax1.set_ylabel("Factor de Potencia")
    ax1.set_ylim(0.2, 1.05)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right")
    ax1.set_title(f"{title_prefix} · Validación Modelo vs Medición (Meta error < 4%)")

    ax2.plot(t, err, color="#dc2626", linewidth=1.8, label="Error (%)")
    ax2.axhline(4.0, color="#16a34a", linestyle="--", linewidth=1.4, label="Límite meta (4.0%)")
    ax2.set_ylabel("Error (%)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylim(0, max(5.0, max(err) * 1.15))
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def plot_load_compensation(out_path: Path, title_prefix: str) -> None:
    loads = [f"{pt.load_pct}%" for pt in WEG_MOTOR_POINTS]
    fp_base = [pt.fp_base for pt in WEG_MOTOR_POINTS]
    i_base = [(pt.p_in_w / pt.fp_base) / REFERENCE_VOLTAGE_V for pt in WEG_MOTOR_POINTS]

    fp_comp = []
    i_comp = []
    for pt in WEG_MOTOR_POINTS:
        combos = simulate_all_bank_combinations(pt, DEFAULT_BANK_STEPS_UF)
        best = best_bank_combination(combos)
        fp_comp.append(best["FP_pred"])
        i_comp.append(best["I_rms_a"])

    x = np.arange(len(loads))
    width = 0.35

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.8))
    ax1.bar(x - width / 2, fp_base, width, color="#94a3b8", label="Sin compensar")
    ax1.bar(x + width / 2, fp_comp, width, color="#10b981", label="Compensado (Banco)")
    ax1.axhspan(DEFAULT_FP_TARGET, DEFAULT_FP_MAX, color="#10b981", alpha=0.15)
    ax1.set_xticks(x)
    ax1.set_xticklabels(loads)
    ax1.set_xlabel("Escenario de carga del motor WEG a 118 V")
    ax1.set_ylabel("Factor de Potencia")
    ax1.set_ylim(0, 1.08)
    ax1.set_title("Factor de Potencia por Carga")
    ax1.grid(True, axis="y", alpha=0.3)
    ax1.legend()

    ax2.bar(x - width / 2, i_base, width, color="#f59e0b", label="Sin compensar")
    ax2.bar(x + width / 2, i_comp, width, color="#0284c7", label="Compensado (Banco)")
    ax2.set_xticks(x)
    ax2.set_xticklabels(loads)
    ax2.set_xlabel("Escenario de carga del motor WEG a 118 V")
    ax2.set_ylabel("Corriente RMS (A)")
    ax2.set_title("Reducción de Corriente en Conductores")
    ax2.grid(True, axis="y", alpha=0.3)
    ax2.legend()

    fig.suptitle(f"{title_prefix} · Desempeño del Banco ({'/'.join(str(int(c)) for c in DEFAULT_BANK_STEPS_UF)} µF)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def plot_rc_discharge(out_path: Path, title_prefix: str) -> None:
    t = np.linspace(0, 25, 250)
    fig, ax = plt.subplots(figsize=(9, 4.8))
    colors = ["#0284c7", "#d97706", "#7c3aed"]
    for idx, c_uf in enumerate(DEFAULT_BANK_STEPS_UF):
        v = [discharge_voltage(float(ti), c_uf) for ti in t]
        t_safe = time_to_safe_voltage(c_uf)
        ax.plot(t, v, color=colors[idx % len(colors)], linewidth=2,
                label=f"Paso {idx + 1} ({c_uf:.0f} µF) · <50 V en {t_safe:.1f} s")

    ax.axhline(50.0, color="#dc2626", linestyle="--", linewidth=1.4, label="Umbral seguro (50 V)")
    ax.set_xlabel("Tiempo tras desconexión (s)")
    ax.set_ylabel("Voltaje en el capacitor (V)")
    ax.set_title(f"{title_prefix} · Curva de Descarga RC (R = {DEFAULT_DISCHARGE_R_OHMS/1000:.0f} kΩ)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)


def generate_all(csv_path: Path, out_dir: Path, name: Optional[str] = None) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    prueba_name = name or detect_next_prueba_name(out_dir)
    target_dir = out_dir / prueba_name
    target_dir.mkdir(parents=True, exist_ok=True)

    rows = load_csv_rows(csv_path)
    if not rows:
        raise ValueError(f"El archivo {csv_path} no contiene filas válidas.")

    plot_fp_and_current(rows, target_dir / f"{prueba_name}_fp_y_corriente.png", prueba_name)
    plot_model_vs_measured(rows, target_dir / f"{prueba_name}_modelo_vs_medicion.png", prueba_name)
    plot_load_compensation(target_dir / f"{prueba_name}_compensacion_carga.png", prueba_name)
    plot_rc_discharge(target_dir / f"{prueba_name}_descarga_rc.png", prueba_name)
    return target_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Genera gráficas PNG numeradas para el informe técnico")
    parser.add_argument(
        "--input",
        type=str,
        default=str(Path(__file__).resolve().parent / "sample_measurements.csv"),
        help="Ruta al CSV de mediciones (ej. Prueba_1.csv)",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=str(Path(__file__).resolve().parent / "reportes"),
        help="Carpeta destino para guardar las gráficas",
    )
    parser.add_argument(
        "--name",
        type=str,
        default=None,
        help="Nombre base de la prueba (ej. Prueba_1). Si se omite, se autogenera secuencialmente.",
    )
    args = parser.parse_args()

    out_folder = generate_all(Path(args.input), Path(args.out), args.name)
    print(f"Gráficas generadas exitosamente en: {out_folder}")
    for png in sorted(out_folder.glob("*.png")):
        print(f"  - {png.name}")


if __name__ == "__main__":
    main()
