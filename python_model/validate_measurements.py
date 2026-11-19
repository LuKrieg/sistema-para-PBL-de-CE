"""
POWERGuard - Validador de Mediciones vs Modelo Teórico
Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute

Lee un archivo CSV de mediciones reales (o simuladas) y verifica:
1) Error porcentual respecto al modelo teórico (< 4% según sección 5.3 de la propuesta).
2) Porcentaje de muestras en estado estacionario dentro del rango objetivo [0.95, 0.99].
"""

import argparse
import csv
from pathlib import Path
from typing import Dict, List

from powerguard_model import (
    DEFAULT_FREQ_HZ,
    DEFAULT_VOLTAGE_V,
    WEG_MOTOR_POINTS,
    measurement_error_pct,
    predict_compensated_state,
)


def find_closest_motor_point(load_pct_raw: str, p_w: float):
    if load_pct_raw != "" and load_pct_raw is not None:
        target = int(float(load_pct_raw))
        return min(WEG_MOTOR_POINTS, key=lambda pt: abs(pt.load_pct - target))
    return min(WEG_MOTOR_POINTS, key=lambda pt: abs(pt.p_in_w - p_w))


def validate_csv(csv_path: Path, max_error_pct: float = 4.0) -> Dict[str, object]:
    errors: List[float] = []
    in_range_count = 0
    total_rows = 0

    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            total_rows += 1
            vrms = float(row.get("vrms") or DEFAULT_VOLTAGE_V)
            p_w = float(row.get("p_w") or 0.0)
            fp_meas = float(row.get("fp_measured") or 0.0)
            cap_uf = float(row.get("cap_uf") or 0.0)

            pt = find_closest_motor_point(row.get("load_pct", ""), p_w)
            pred = predict_compensated_state(pt.p_in_w, pt.fp_base, cap_uf, vrms, DEFAULT_FREQ_HZ)
            err = measurement_error_pct(fp_meas, pred["FP_new"])
            errors.append(err)

            if 0.95 <= fp_meas <= 0.99:
                in_range_count += 1

    if total_rows == 0:
        raise ValueError("El archivo CSV no contiene registros.")

    avg_err = sum(errors) / len(errors)
    max_err = max(errors)
    within_4pct = sum(1 for e in errors if e <= max_error_pct)

    summary = {
        "total_rows": total_rows,
        "avg_error_pct": round(avg_err, 3),
        "max_error_pct": round(max_err, 3),
        "pct_within_4pct_target": round(within_4pct / total_rows * 100.0, 1),
        "pct_in_fp_range_095_099": round(in_range_count / total_rows * 100.0, 1),
        "passed_4pct_goal": avg_err < max_error_pct,
    }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Valida mediciones CSV contra el modelo POWERGuard")
    parser.add_argument("--input", required=True, help="Ruta al archivo CSV de mediciones")
    args = parser.parse_args()

    res = validate_csv(Path(args.input))
    print("=" * 60)
    print("POWERGuard - Reporte de Validación (Modelo vs Mediciones)")
    print("=" * 60)
    print(f"Muestras analizadas            : {res['total_rows']}")
    print(f"Error promedio                 : {res['avg_error_pct']} %")
    print(f"Error máximo observado         : {res['max_error_pct']} %")
    print(f"Muestras con error <= 4%       : {res['pct_within_4pct_target']} %")
    print(f"Muestras con FP en [0.95, 0.99]: {res['pct_in_fp_range_095_099']} %")
    print(f"Meta propuesta (< 4% error)    : {'CUMPLE' if res['passed_4pct_goal'] else 'NO CUMPLE'}")


if __name__ == "__main__":
    main()
