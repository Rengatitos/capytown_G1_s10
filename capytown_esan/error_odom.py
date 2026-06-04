#!/usr/bin/env python3
"""Analizador de odometria RC-1 para Yahboom / ROS2.

El script lee un bag real de Yahboom, extrae la trayectoria de /odom o /odom_raw,
la compara con un cuadrado ideal de 1 x 1 m, calcula error de cierre y genera
una figura resumen. Si el desvio supera el umbral, registra una sugerencia de
calibracion en calibration_log.csv y actualiza wheel_params.yaml con el nuevo
b_eff estimado.

En este entorno el modo real solo funciona si estan disponibles rosbag2_py,
rclpy y nav_msgs. Para pruebas locales sin ROS2 existe un modo demo opt-in con
--demo, pero no se usa automaticamente.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


try:
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    ROS2_AVAILABLE = True
except Exception:
    ROS2_AVAILABLE = False
    Odometry = None  # type: ignore[assignment]
    deserialize_message = None  # type: ignore[assignment]
    SequentialReader = None  # type: ignore[assignment]
    StorageOptions = None  # type: ignore[assignment]
    ConverterOptions = None  # type: ignore[assignment]


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAGS_DIR = Path(
    os.environ.get(
        "CAPYTOWN_BAGS_DIR",
        str(REPO_ROOT / "bags"),
    )
).resolve()
DEFAULT_PLOTS_DIR = Path(
    os.environ.get(
        "CAPYTOWN_PLOTS_DIR",
        str(REPO_ROOT / "plots"),
    )
).resolve()
DEFAULT_YAML_PATH = Path(
    os.environ.get(
        "CAPYTOWN_WHEEL_PARAMS",
        str(REPO_ROOT / "config" / "wheel_params.yaml"),
    )
).resolve()
DEFAULT_CALIBRATION_LOG = Path(
    os.environ.get(
        "CAPYTOWN_CALIBRATION_LOG",
        str(REPO_ROOT / "calibration_log.csv"),
    )
).resolve()

IDEAL_SQUARE: Sequence[Tuple[float, float]] = (
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),
)

TOPIC_PRIORITY: Sequence[str] = ("/odom", "/odom_raw")
CRITICAL_ERROR_CM = 30.0
DEFAULT_BEFF = 0.2145
BEFF_MIN = 0.05
BEFF_MAX = 0.40

CALIBRATION_HEADER = [
    "timestamp",
    "run",
    "b_eff_used",
    "error_cm",
    "suggested_b_eff",
    "error_pct",
    "note",
]


@dataclass
class PoseSample:
    x: float
    y: float
    yaw: float
    stamp_s: float


@dataclass
class TrackAnalysis:
    run_name: str
    topic: str
    samples: List[PoseSample]
    failure_index: Optional[int]
    failure_error_cm: Optional[float]
    max_error_cm: float
    final_error_cm: float
    dx_cm: float
    dy_cm: float
    dtheta_deg: float
    plot_path: Path
    mode: str
    note: str


@dataclass
class CalibrationDecision:
    current_b_eff: float
    suggested_b_eff: float
    was_updated: bool
    log_row_id: Optional[int]


def _normalize_path(value: str | Path) -> Path:
    return Path(os.path.expanduser(str(value))).resolve()


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom == 0.0:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    cx = ax + t * abx
    cy = ay + t * aby
    return math.hypot(px - cx, py - cy)


def _distance_to_square(x: float, y: float) -> float:
    edges = list(zip(IDEAL_SQUARE[:-1], IDEAL_SQUARE[1:]))
    return min(_segment_distance(x, y, ax, ay, bx, by) for (ax, ay), (bx, by) in edges)


def _path_error_series(samples: Sequence[PoseSample]) -> List[float]:
    return [_distance_to_square(sample.x, sample.y) * 100.0 for sample in samples]


def _closure_metrics(samples: Sequence[PoseSample]) -> Tuple[float, float, float]:
    if not samples:
        return 0.0, 0.0, 0.0
    dx_cm = (samples[-1].x - samples[0].x) * 100.0
    dy_cm = (samples[-1].y - samples[0].y) * 100.0
    dtheta_deg = math.degrees(samples[-1].yaw - samples[0].yaw)
    return dx_cm, dy_cm, dtheta_deg


def _closure_distance_cm(samples: Sequence[PoseSample]) -> float:
    if not samples:
        return 0.0
    return math.hypot(samples[-1].x - samples[0].x, samples[-1].y - samples[0].y) * 100.0


def _ensure_csv(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        with path.open("w", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerow(CALIBRATION_HEADER)


def _read_log_rows(path: Path) -> List[List[str]]:
    if not path.exists():
        return []
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    return [row for row in rows[1:] if row]


def _append_log_row(
    path: Path,
    run_name: str,
    b_eff_used: float,
    error_cm: float,
    suggested_b_eff: float,
    note: str,
) -> int:
    _ensure_csv(path)
    rows = _read_log_rows(path)
    row_id = len(rows) + 1
    error_pct = abs(suggested_b_eff - b_eff_used) / max(abs(b_eff_used), 1e-9) * 100.0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                f"run-{row_id}-{run_name}",
                run_name,
                f"{b_eff_used:.5f}",
                f"{error_cm:.2f}",
                f"{suggested_b_eff:.5f}",
                f"{error_pct:.2f}",
                note,
            ]
        )
    return row_id


def _extract_current_beff(yaml_path: Path) -> float:
    if not yaml_path.exists():
        return DEFAULT_BEFF
    text = yaml_path.read_text(encoding="utf-8")
    match = re.search(r"wheel_separation:\s*([0-9]*\.?[0-9]+)", text)
    return float(match.group(1)) if match else DEFAULT_BEFF


def _rewrite_wheel_yaml(yaml_path: Path, new_beff: float, history_lines: Sequence[str]) -> None:
    yaml_path.parent.mkdir(parents=True, exist_ok=True)

    if yaml_path.exists():
        raw = yaml_path.read_text(encoding="utf-8").splitlines()
    else:
        raw = []

    body = [line for line in raw if line.strip() and not line.lstrip().startswith("#")]
    if not body:
        body = [
            "/**:",
            "  ros__parameters:",
            "    wheel_radius: 0.0325",
            "    wheel_separation: 0.2145",
            "    encoder_ticks_per_rev: 1500",
        ]

    rebuilt: List[str] = [
        "# wheel_params.yaml - Parametros cinematicos calibrados",
        "# RC-1: La Manzana del Tambo - CapyTown ESAN 2026-I",
        "",
    ]
    rebuilt.extend(body)
    text = "\n".join(rebuilt) + "\n"
    text = re.sub(
        r"(wheel_separation:\s*)([0-9]*\.?[0-9]+)",
        lambda match: f"{match.group(1)}{new_beff:.5f}",
        text,
        count=1,
    )
    if not history_lines:
        history_lines = ["# Historial de calibracion b_eff: (sin cambios nuevos)"]
    else:
        history_lines = ["# Historial de calibracion b_eff:"] + list(history_lines)
    text = text.rstrip() + "\n\n" + "\n".join(history_lines) + "\n"
    yaml_path.write_text(text, encoding="utf-8")


def _history_lines_from_log(log_path: Path, adopted_value: Optional[float] = None) -> List[str]:
    lines: List[str] = []
    for row_index, row in enumerate(reversed(_read_log_rows(log_path)), start=1):
        if len(row) < 6:
            continue
        run_name = row[1]
        b_eff_used = float(row[2])
        error_cm = float(row[3])
        suggested = float(row[4])
        error_pct = float(row[5])
        marker = "  <- ADOPTADO" if adopted_value is not None and abs(suggested - adopted_value) < 1e-6 else ""
        lines.append(
            f"#   Run {row_index}: {run_name} | b_eff={b_eff_used:.5f} -> error {error_pct:.2f}% "
            f"-> sugerido {suggested:.5f} (error={error_cm:.2f} cm){marker}"
        )
    return lines


class RosbagTrajectoryReader:
    def __init__(self, bag_path: Path):
        self.bag_path = bag_path
        if not ROS2_AVAILABLE:
            raise RuntimeError(
                "ROS2 no esta disponible en este entorno. Instala rosbag2_py, rclpy y nav_msgs para leer bags reales."
            )

    def _open_reader(self) -> SequentialReader:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(self.bag_path), storage_id="sqlite3"),
            ConverterOptions("", ""),
        )
        return reader

    def _select_topic(self, reader: SequentialReader) -> str:
        topics = getattr(reader, "get_all_topics_and_types", None)
        if callable(topics):
            available = {topic.name for topic in topics()}
            for candidate in TOPIC_PRIORITY:
                if candidate in available:
                    return candidate
        return TOPIC_PRIORITY[0]

    def read(self) -> Tuple[str, List[PoseSample]]:
        reader = self._open_reader()
        topic_name = self._select_topic(reader)
        samples: List[PoseSample] = []

        while reader.has_next():
            topic, payload, stamp = reader.read_next()
            if topic not in TOPIC_PRIORITY:
                continue
            message = deserialize_message(payload, Odometry)
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            samples.append(
                PoseSample(
                    x=float(position.x),
                    y=float(position.y),
                    yaw=_yaw_from_quaternion(
                        float(orientation.x),
                        float(orientation.y),
                        float(orientation.z),
                        float(orientation.w),
                    ),
                    stamp_s=float(stamp) * 1e-9,
                )
            )
            topic_name = topic

        if not samples:
            raise ValueError(f"No se encontraron mensajes en {TOPIC_PRIORITY} dentro de {self.bag_path}")

        return topic_name, samples


class YahboomAnalyzer:
    def __init__(
        self,
        plots_dir: Path,
        log_path: Path,
        yaml_path: Path,
        threshold_cm: float,
    ):
        self.plots_dir = plots_dir
        self.log_path = log_path
        self.yaml_path = yaml_path
        self.threshold_cm = threshold_cm

    def analyze(self, run_name: str, bag_path: Path, samples: Sequence[PoseSample], source_topic: str, mode: str) -> TrackAnalysis:
        self.plots_dir.mkdir(parents=True, exist_ok=True)
        errors_cm = _path_error_series(samples)
        max_error_cm = max(errors_cm) if errors_cm else 0.0
        final_error_cm = errors_cm[-1] if errors_cm else 0.0
        failure_index = next((index for index, value in enumerate(errors_cm) if value > self.threshold_cm), None)
        failure_error_cm = errors_cm[failure_index] if failure_index is not None else None

        plot_path = self.plots_dir / f"trayectoria_{run_name}.png"
        self._plot(samples, plot_path, run_name, source_topic, mode, threshold_cm=self.threshold_cm)

        dx_cm, dy_cm, dtheta_deg = _closure_metrics(samples)
        note = "bag-real" if mode == "real" else mode
        result = TrackAnalysis(
            run_name=run_name,
            topic=source_topic,
            samples=list(samples),
            failure_index=failure_index,
            failure_error_cm=failure_error_cm,
            max_error_cm=max_error_cm,
            final_error_cm=final_error_cm,
            dx_cm=dx_cm,
            dy_cm=dy_cm,
            dtheta_deg=dtheta_deg,
            plot_path=plot_path,
            mode=mode,
            note=note,
        )

        if max_error_cm > self.threshold_cm:
            self._persist_calibration(run_name, bag_path, max_error_cm)

        return result

    def _persist_calibration(self, run_name: str, bag_path: Path, measured_error_cm: float) -> CalibrationDecision:
        current_beff = _extract_current_beff(self.yaml_path)
        correction = 1.0 + min(max(measured_error_cm / 150.0, 0.05), 0.20)
        suggested_beff = max(BEFF_MIN, min(BEFF_MAX, current_beff * correction))
        row_id = _append_log_row(
            self.log_path,
            run_name=run_name,
            b_eff_used=current_beff,
            error_cm=measured_error_cm,
            suggested_b_eff=suggested_beff,
            note=f"real-{bag_path.name}",
        )
        history = _history_lines_from_log(self.log_path, adopted_value=suggested_beff)
        _rewrite_wheel_yaml(self.yaml_path, suggested_beff, history)
        return CalibrationDecision(current_beff, suggested_beff, True, row_id)

    def _plot(
        self,
        samples: Sequence[PoseSample],
        output: Path,
        run_name: str,
        source_topic: str,
        mode: str,
        threshold_cm: float,
    ) -> None:
        xs = [sample.x for sample in samples]
        ys = [sample.y for sample in samples]
        errors_cm = _path_error_series(samples)
        dx_cm, dy_cm, dtheta_deg = _closure_metrics(samples)
        closure_cm = _closure_distance_cm(samples)

        fig, (ax_xy, ax_err) = plt.subplots(1, 2, figsize=(13.5, 6.0))
        fig.suptitle("RC-1 - Análisis de odometría Yahboom", fontsize=13)

        ideal_x = [point[0] for point in IDEAL_SQUARE]
        ideal_y = [point[1] for point in IDEAL_SQUARE]
        ax_xy.plot(ideal_x, ideal_y, "--", color="#4f6d7a", lw=2, label="Cuadrado ideal 1x1 m")
        ax_xy.plot(xs, ys, color="#c14953", lw=1.8, label=f"Trayectoria {source_topic}")
        ax_xy.scatter([xs[0]], [ys[0]], s=55, color="#2f4858", label="Inicio", zorder=4)
        ax_xy.scatter([xs[-1]], [ys[-1]], s=55, color="#f2545b", label="Fin", zorder=4)
        ax_xy.annotate(
            "",
            xy=(xs[-1], ys[-1]),
            xytext=(xs[0], ys[0]),
            arrowprops=dict(arrowstyle="->", color="#f2545b", lw=1.5),
        )
        ax_xy.set_aspect("equal", adjustable="box")
        ax_xy.set_xlim(-0.25, 1.35)
        ax_xy.set_ylim(-0.25, 1.35)
        ax_xy.set_xlabel("x [m]")
        ax_xy.set_ylabel("y [m]")
        ax_xy.set_title(f"Trayectoria XY ({mode})")
        ax_xy.grid(True, linestyle=":", alpha=0.4)
        ax_xy.legend(loc="upper right", fontsize=8)
        ax_xy.text(
            0.03,
            0.03,
            f"dx={dx_cm:+.2f} cm\ndy={dy_cm:+.2f} cm\n|d|={closure_cm:.2f} cm\nΔθ={dtheta_deg:+.2f}°",
            transform=ax_xy.transAxes,
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="#fff8d6", alpha=0.95),
        )

        ax_err.plot(errors_cm, color="#264653", lw=1.5)
        ax_err.axhline(threshold_cm, color="#e76f51", ls="--", lw=1.8, label=f"Umbral {threshold_cm:.1f} cm")
        ax_err.set_title("Error instantáneo respecto al cuadrado")
        ax_err.set_xlabel("Muestra")
        ax_err.set_ylabel("Distancia al borde [cm]")
        ax_err.grid(True, linestyle=":", alpha=0.4)
        ax_err.legend(loc="upper left", fontsize=8)

        plt.tight_layout(rect=(0, 0, 1, 0.95))
        fig.savefig(output, dpi=150, bbox_inches="tight")
        plt.close(fig)


def _detect_yahboom_source(bag_path: Path) -> str:
    bag_name = bag_path.name.lower()
    if any(token in bag_name for token in ("yahboom", "tambo", "capytown")):
        return "yahboom"
    return "generic-ros2"


def _default_bag_for_run(bags_dir: Path, run_name: str) -> Path:
    candidates = [
        bags_dir / f"tambo_G_{run_name}.bag",
        bags_dir / f"tambo_G1_{run_name}.bag",
        bags_dir / f"tambo_{run_name}.bag",
        bags_dir / f"{run_name}.bag",
        bags_dir / run_name,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _load_demo_trajectory() -> List[PoseSample]:
    samples: List[PoseSample] = []
    laps = 3
    per_side = 80
    t = 0.0
    for _ in range(laps):
        for (ax, ay), (bx, by) in zip(IDEAL_SQUARE[:-1], IDEAL_SQUARE[1:]):
            for step in range(per_side):
                alpha = step / max(per_side - 1, 1)
                x = ax + (bx - ax) * alpha
                y = ay + (by - ay) * alpha
                wobble = 0.02 * math.sin((t + alpha) * math.pi * 6.0)
                samples.append(PoseSample(x=x + wobble, y=y - wobble * 0.5, yaw=0.0, stamp_s=t))
                t += 0.05
    return samples


def _run_once(
    bag_path: Path,
    run_name: str,
    plots_dir: Path,
    yaml_path: Path,
    log_path: Path,
    threshold_cm: float,
    demo: bool,
) -> TrackAnalysis:
    analyzer = YahboomAnalyzer(plots_dir=plots_dir, log_path=log_path, yaml_path=yaml_path, threshold_cm=threshold_cm)

    if demo:
        samples = _load_demo_trajectory()
        demo_plot = plots_dir / f"trayectoria_{run_name}_demo.png"
        analyzer._plot(samples, demo_plot, run_name, "demo", "demo", threshold_cm)
        return TrackAnalysis(
            run_name=run_name,
            topic="demo",
            samples=samples,
            failure_index=None,
            failure_error_cm=None,
            max_error_cm=max(_path_error_series(samples)),
            final_error_cm=_path_error_series(samples)[-1],
            dx_cm=_closure_metrics(samples)[0],
            dy_cm=_closure_metrics(samples)[1],
            dtheta_deg=_closure_metrics(samples)[2],
            plot_path=demo_plot,
            mode="demo",
            note="modo-demo",
        )

    reader = RosbagTrajectoryReader(bag_path)
    topic, samples = reader.read()
    return analyzer.analyze(run_name, bag_path, samples, topic, mode="real")


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analisis de odometria RC-1 para Yahboom")
    parser.add_argument("--bag", help="Ruta a un bag de ROS2")
    parser.add_argument("--batch", action="store_true", help="Procesar run1, run2 y run3")
    parser.add_argument("--bags-dir", default=str(DEFAULT_BAGS_DIR), help="Carpeta base de bags")
    parser.add_argument("--plots-dir", default=str(DEFAULT_PLOTS_DIR), help="Carpeta de salida de plots")
    parser.add_argument("--yaml", dest="yaml_path", default=str(DEFAULT_YAML_PATH), help="Ruta de wheel_params.yaml")
    parser.add_argument("--threshold", type=float, default=CRITICAL_ERROR_CM, help="Umbral de error critico en cm")
    parser.add_argument("--demo", action="store_true", help="Genera una corrida demostrativa sin ROS2")
    return parser.parse_args(argv)


def _print_result(result: TrackAnalysis) -> None:
    print(f"[{result.run_name}] modo={result.mode} topic={result.topic}")
    print(f"  max_error={result.max_error_cm:.2f} cm | final_error={result.final_error_cm:.2f} cm")
    print(f"  cierre dx={result.dx_cm:+.2f} cm dy={result.dy_cm:+.2f} cm dtheta={result.dtheta_deg:+.2f}°")
    print(f"  plot={result.plot_path}")
    if result.failure_index is not None:
        print(f"  alerta: primer cruce del umbral en la muestra {result.failure_index}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    bags_dir = _normalize_path(args.bags_dir)
    plots_dir = _normalize_path(args.plots_dir)
    yaml_path = _normalize_path(args.yaml_path)
    log_path = _normalize_path(DEFAULT_CALIBRATION_LOG)
    threshold_cm = float(args.threshold)

    if args.demo:
        print("Modo demo activado: no se intentara leer ROS2/rosbag2.")

    if args.bag:
        bag_path = _normalize_path(args.bag)
        run_name = bag_path.stem.replace(".bag", "")
        if not bag_path.exists() and not args.demo:
            raise SystemExit(f"No se encontro el bag: {bag_path}")
        result = _run_once(bag_path, run_name, plots_dir, yaml_path, log_path, threshold_cm, demo=args.demo)
        _print_result(result)
        return 0

    run_names = ("run1", "run2", "run3") if args.batch or not args.demo else ("run1",)
    results: List[TrackAnalysis] = []
    for run_name in run_names:
        bag_path = _default_bag_for_run(bags_dir, run_name)
        if not bag_path.exists() and not args.demo:
            raise SystemExit(
                f"No se encontro el bag para {run_name}: {bag_path}. Usa --demo si solo quieres validar la grafica localmente."
            )
        result = _run_once(bag_path, run_name, plots_dir, yaml_path, log_path, threshold_cm, demo=args.demo)
        results.append(result)
        _print_result(result)

    if results:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        names = [result.run_name for result in results]
        values = [result.final_error_cm for result in results]
        bars = ax.bar(names, values, color="#c14953", edgecolor="#222")
        ax.axhline(threshold_cm, color="#264653", ls="--", lw=2, label="Umbral critico")
        ax.set_title("Resumen de error de cierre")
        ax.set_ylabel("Error final [cm]")
        ax.grid(True, axis="y", alpha=0.3)
        ax.legend()
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 0.25, f"{value:.1f}", ha="center", fontsize=8)
        plots_dir.mkdir(parents=True, exist_ok=True)
        fig.savefig(plots_dir / "resumen_error_cierre.png", dpi=150, bbox_inches="tight")
        plt.close(fig)

    current_beff = _extract_current_beff(yaml_path)
    print(f"wheel_separation actual: {current_beff:.5f} m")
    print(f"calibration_log.csv en : {log_path}")
    print(f"plots en               : {plots_dir}")
    if ROS2_AVAILABLE:
        print("deteccion: ROS2 disponible, script listo para bags reales de Yahboom")
    else:
        print("deteccion: ROS2 no disponible en este entorno; usa --demo solo para pruebas locales")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
