"""
POWERGuard - Validador de Mediciones vs Modelo Teórico
Equipo PowerGuard 12 | Circuitos Eléctricos | KEY Institute

Lee un archivo CSV de mediciones reales (o simuladas) y verifica:
1) Error porcentual respecto al modelo teórico (< 4% según sección 5.3 de la propuesta).
2) Porcentaje en [0.95, 1.00] por origen de los datos.
"""

import argparse
import csv
from pathlib import Path
from typing import Dict, List

from powerguard_model import (
    DEFAULT_FP_MAX,
    DEFAULT_FP_TARGET,
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


def compare_csv_row(row: Dict[str, str]) -> tuple[str, float | None, float | None]:
    """Devuelve origen, FP registrado y referencia comparable del mismo estado."""
    source = row.get("source") or "legacy"
    raw_fp = row.get("fp_compensated") if source == "simulation" else row.get("fp_measured")
    if raw_fp in (None, ""):
        return source, None, None
    fp = float(raw_fp)
    if fp <= 0.0:
        return source, None, None
    if source != "arduino" and row.get("fp_model") not in (None, ""):
        return source, fp, float(row["fp_model"])

    vrms = float(row.get("vrms") or DEFAULT_VOLTAGE_V)
    p_w = float(row.get("p_w") or 0.0)
    cap_uf = float(row.get("cap_uf") or 0.0)
    if source == "arduino" and row.get("bank_simulated") == "true":
        cap_uf = 0.0
    pt = find_closest_motor_point(row.get("load_pct", ""), p_w)
    pred = predict_compensated_state(pt.p_in_w, pt.fp_base, cap_uf, vrms)
    return source, fp, pred["FP_new"]


def validate_csv(csv_path: Path, max_error_pct: float = 4.0) -> Dict[str, object]:
    errors: Dict[str, List[float]] = {}
    fp_by_source: Dict[str, List[float]] = {}
    total_rows = 0

    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            total_rows += 1
            source, fp, reference = compare_csv_row(row)
            if fp is not None:
                fp_by_source.setdefault(source, []).append(fp)
            if fp is not None and reference:
                errors.setdefault(source, []).append(measurement_error_pct(fp, reference))
            if source == "arduino" and row.get("fp_compensated") not in (None, ""):
                fp_by_source.setdefault("estimated", []).append(float(row["fp_compensated"]))

    if total_rows == 0:
        raise ValueError("El archivo CSV no contiene registros.")

    return {
        "total_rows": total_rows,
        "pct_in_target_by_source": {
            source: round(100 * sum(DEFAULT_FP_TARGET <= fp <= DEFAULT_FP_MAX for fp in values) / len(values), 1)
            for source, values in fp_by_source.items()
        },
        "error_by_source": {
            source: {
                "average_pct": round(sum(values) / len(values), 3),
                "max_pct": round(max(values), 3),
                "pct_within_limit": round(100 * sum(value <= max_error_pct for value in values) / len(values), 1),
                "passed": sum(values) / len(values) < max_error_pct,
            }
            for source, values in errors.items()
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Valida mediciones CSV contra el modelo POWERGuard")
    parser.add_argument("--input", required=True, help="Ruta al archivo CSV de mediciones")
    args = parser.parse_args()

    res = validate_csv(Path(args.input))
    print("=" * 60)
    print("POWERGuard - Reporte de Validación (Modelo vs Mediciones)")
    print("=" * 60)
    print(f"Filas analizadas: {res['total_rows']}")
    labels = {
        "arduino": "FP medido por Arduino",
        "estimated": "FP estimado con banco a partir de la medición",
        "simulation": "FP de simulación",
        "legacy": "FP histórico (según el archivo original)",
    }
    for source, pct in res["pct_in_target_by_source"].items():
        print(f"{labels.get(source, source)} en [0.95, 1.00]: {pct} %")
    for source, stats in res["error_by_source"].items():
        print(f"{labels.get(source, source)} frente al escenario de referencia")
        print(f"  Error promedio: {stats['average_pct']} %, máximo: {stats['max_pct']} %")
        print(f"  Muestras con error <= 4 %: {stats['pct_within_limit']} %")


if __name__ == "__main__":
    main()
