#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
informe_tecnico.py
==================
Genera el informe técnico RC-1 a partir de los archivos reales
producidos por el robot:

  - calibration_log.csv  → sección (b): calibración b_eff
  - bags/run1, run2, run3 → sección (c): resultados experimentales

Las predicciones iniciales (a) y el análisis de causa raíz (d/e)
son respuestas teóricas que complementan los datos del robot.

EJECUCIÓN
---------
    python3 informe_tecnico.py \\
        --cal calibration_log.csv \\
        --bags bags/tambo_run1 bags/tambo_run2 bags/tambo_run3 \\
        --grupo G01 --domain 1 \\
        --out-dir informe/

RC-1 · La Manzana del Tambo · CapyTown · Robótica 2026-I · ESAN
"""

import argparse
import math
import os
import sys
from typing import List

import matplotlib.pyplot as plt
import pandas as pd

# Importar clases de análisis del módulo 03_error_odom
_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_DIR, "..", "03_error_odom"))
from error_odom import LectorBag, AnalisisOdom, AnalisisMultiRun, ResultadoRun   # noqa: E402

# Importar el analizador de calibración del módulo 01
sys.path.insert(0, os.path.join(_DIR, "..", "01_calibracion_beff"))
from calibracion_beff import AnalizadorCalibracion   # noqa: E402


# ────────────────────────────────────────────────────────────────────
# Sección (a): Predicciones iniciales — respuestas teóricas
# ────────────────────────────────────────────────────────────────────

class PrediccionInicial:
    """
    Respuestas razonadas a la activación cognitiva.
    Estas predicciones se registran ANTES de encender el robot.
    Son cálculos basados en física/cinemática, no en mediciones del robot.
    """

    @staticmethod
    def calcular_prediccion_error(
        n_laps: int = 3,
        sigma_giro_deg: float = 1.0,
        side_m: float = 1.0,
    ) -> str:
        """
        Estima el error de posición esperado tras n_laps vueltas.

        El robot completa 4 giros por vuelta → n_giros = 4 × n_laps.
        El error angular acumulado crece como √N × σ_giro.
        La simetría del cuadrado cancela parcialmente los errores → factor ≈ 0.5.
        """
        n_giros       = 4 * n_laps
        sigma_theta   = math.sqrt(n_giros) * sigma_giro_deg
        L_rectas      = n_laps * 4 * side_m
        est_min_cm    = L_rectas * math.sin(math.radians(sigma_theta * 0.3)) * 100
        est_max_cm    = L_rectas * math.sin(math.radians(sigma_theta * 0.7)) * 100

        return (
            f"PREDICCIÓN 1 — Error de posición tras {n_laps} vueltas\n"
            "────────────────────────────────────────────────────────────\n"
            f"  Giros totales     : {n_giros}  ({n_laps} vueltas × 4 giros)\n"
            f"  σ_giro estimado   : {sigma_giro_deg:.1f}° post-calibración\n"
            f"  σ_θ acumulado     : √{n_giros} × {sigma_giro_deg:.1f}° = {sigma_theta:.1f}°\n"
            f"  L total rectas    : {L_rectas:.0f} m\n"
            f"  Estimación error  : {est_min_cm:.0f}–{est_max_cm:.0f} cm\n"
            "  (El cuadrado cancela parcialmente los errores por simetría)\n"
            "\n"
            f"  RESPUESTA: Estimamos entre {est_min_cm:.0f} y {est_max_cm:.0f} cm de error promedio."
        )

    @staticmethod
    def rectas_vs_esquinas() -> str:
        return (
            "PREDICCIÓN 2 — ¿Rectas o esquinas producen más error?\n"
            "────────────────────────────────────────────────────────────\n"
            "  RESPUESTA: Las ESQUINAS dominan el error.\n"
            "\n"
            "  En las RECTAS: las 4 ruedas ruedan hacia adelante.\n"
            "  El slip lateral es mínimo. El error se debe a la\n"
            "  cuantización del encoder (~0.1% por metro).\n"
            "\n"
            "  En las ESQUINAS (giro en sitio): las ruedas izquierdas\n"
            "  y derechas giran en sentidos opuestos. Las 4 ruedas\n"
            "  PATINAN lateralmente de forma simultánea, desplazando\n"
            "  el ICR real respecto al ICR modelado.\n"
            "  Con 12 giros y solo 12 m de rectas, los giros aportan\n"
            "  ~5–10× más error que las rectas."
        )

    @staticmethod
    def encoders_mejores() -> str:
        return (
            "PREDICCIÓN 3 — ¿Encoders de mayor resolución eliminan la deriva?\n"
            "────────────────────────────────────────────────────────────────\n"
            "  RESPUESTA: NO. La deriva continuaría (aunque sería menor).\n"
            "\n"
            "  Los encoders miden la rotación de los ejes, no el\n"
            "  desplazamiento del chasis sobre el piso. El slip lateral\n"
            "  (patinaje de goma sobre superficie) hace que el chasis se\n"
            "  mueva de forma diferente a lo que los encoders reportan.\n"
            "  Esta discrepancia es FÍSICA, no del sensor.\n"
            "\n"
            "  Para eliminar la deriva se necesita una referencia absoluta:\n"
            "  cámara de flujo óptico, GPS, landmarks, LiDAR + mapa."
        )

    def imprimir_todas(self, n_laps: int = 3) -> None:
        print("=" * 66)
        print("  (a) PREDICCIÓN INICIAL — Activación cognitiva")
        print("  (Se registra ANTES de encender el robot)")
        print("=" * 66)
        print()
        print(self.calcular_prediccion_error(n_laps=n_laps))
        print()
        print(self.rectas_vs_esquinas())
        print()
        print(self.encoders_mejores())


# ────────────────────────────────────────────────────────────────────
# Sección (d): Análisis de causa raíz — interpretación de datos reales
# ────────────────────────────────────────────────────────────────────

class AnalisisCausaRaiz:
    """
    Interpreta los resultados reales para identificar las fuentes de error.
    El diagnóstico se construye a partir de los ResultadoRun del robot.
    """

    def __init__(self, resultados: List[ResultadoRun], b_eff_final: float):
        self.resultados  = resultados
        self.b_eff_final = b_eff_final

    def diagnostico(self) -> str:
        if not self.resultados:
            return "Sin datos de runs disponibles."

        errores  = [r.err_cm for r in self.resultados]
        prom     = sum(errores) / len(errores)
        dthetas  = [abs(r.dtheta_deg) for r in self.resultados]
        prom_ang = sum(dthetas) / len(dthetas)

        lineas = [
            "Diagnóstico basado en datos reales del robot",
            "─" * 52,
            f"  b_eff calibrado  : {self.b_eff_final:.5f} m",
            f"  Error pos prom   : {prom:.2f} cm",
            f"  Error ang prom   : {prom_ang:.2f}°",
            "",
            "  Fuentes de error identificadas:",
        ]

        # Diagnóstico automático basado en los datos reales
        if prom_ang > 3.0:
            lineas.append(
                f"  [ALTO] Drift angular: {prom_ang:.1f}° promedio → "
                "slip lateral en giros domina el error."
            )
        elif prom_ang > 1.0:
            lineas.append(
                f"  [MEDIO] Drift angular: {prom_ang:.1f}° promedio → "
                "slip lateral presente, b_eff bien calibrado."
            )
        else:
            lineas.append(
                f"  [BAJO] Drift angular: {prom_ang:.1f}° promedio → "
                "calibración b_eff excelente."
            )

        if prom > 15.0:
            lineas.append(
                "  [CRITICO] Error posicional > 15 cm → "
                "recalibrar b_eff en esta superficie."
            )
        elif prom > 5.0:
            lineas.append(
                "  [NORMAL] Error posicional 5–15 cm → "
                "dentro del rango típico del skid-steer."
            )
        else:
            lineas.append(
                f"  [EXCELENTE] Error posicional {prom:.1f} cm ≤ 5 cm → "
                "calibración fina de b_eff demostrada. BONUS ✓"
            )

        lineas += [
            "",
            "  ¿Coincide con la predicción (esquinas > rectas)?",
            f"  Error angular promedio {prom_ang:.1f}° en {len(self.resultados) * 12} giros",
            f"  → ~{prom_ang / 12:.2f}° por giro → confirma que el slip en giros domina.",
            "",
            "  La odometría es 'continua pero deriva' porque integra velocidades",
            "  de rueda sin realimentación de posición absoluta.",
        ]
        return "\n".join(lineas)

    def graficar_errores(self, out_png: str) -> None:
        """Gráfica de barras con los errores reales de cada run."""
        nombres = [r.nombre for r in self.resultados]
        errores = [r.err_cm for r in self.resultados]
        dxs     = [r.dx_cm  for r in self.resultados]
        dys     = [r.dy_cm  for r in self.resultados]
        prom    = sum(errores) / len(errores) if errores else 0

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

        # Error de posición total
        colores = ["#2E86AB", "#E9B44C", "#B85042", "#3BB273"][:len(nombres)]
        bars = ax1.bar(nombres, errores, color=colores, edgecolor="black", lw=0.8)
        ax1.axhline(5.0,  color="#3BB273", ls="--", lw=2, label="Bonus (5 cm)")
        ax1.axhline(15.0, color="#E84855", ls="--", lw=2, label="Mínimo (15 cm)")
        ax1.axhline(prom, color="#555",    ls=":",  lw=1.5, label=f"Promedio ({prom:.1f} cm)")
        for bar, val in zip(bars, errores):
            ax1.text(
                bar.get_x() + bar.get_width() / 2, val + 0.1,
                f"{val:.2f} cm", ha="center", fontsize=11, fontweight="bold",
            )
        ax1.set_ylabel("Error de posición [cm]")
        ax1.set_title("Error de cierre por run (datos reales)")
        ax1.legend(fontsize=9)
        ax1.set_ylim(0, max(max(errores, default=20), 20) * 1.25)
        ax1.grid(True, alpha=0.35, axis="y")

        # Componentes dx y dy
        x_pos = range(len(nombres))
        w = 0.35
        b_dx = ax2.bar(
            [x - w/2 for x in x_pos], dxs, w,
            label="dx", color="#2E86AB", edgecolor="black",
        )
        b_dy = ax2.bar(
            [x + w/2 for x in x_pos], dys, w,
            label="dy", color="#E9B44C", edgecolor="black",
        )
        ax2.axhline(0, color="black", lw=0.8)
        for bars_g, vals in [(b_dx, dxs), (b_dy, dys)]:
            for bar, val in zip(bars_g, vals):
                offset = 0.1 if val >= 0 else -0.4
                ax2.text(
                    bar.get_x() + bar.get_width() / 2, val + offset,
                    f"{val:+.1f}", ha="center", fontsize=9,
                )
        ax2.set_xticks(list(x_pos))
        ax2.set_xticklabels(nombres)
        ax2.set_ylabel("Error [cm]")
        ax2.set_title("Componentes dx y dy (datos reales)")
        ax2.legend()
        ax2.grid(True, alpha=0.35, axis="y")

        plt.suptitle(
            f"RC-1 La Manzana del Tambo — Errores odométricos reales\n"
            f"b_eff = {self.b_eff_final:.5f} m | v = 0.10 m/s | 3 vueltas × 1×1 m",
            fontsize=12, fontweight="bold",
        )
        plt.tight_layout()
        plt.savefig(out_png, dpi=150, bbox_inches="tight")
        plt.close()
        print(f"Gráfica errores: {out_png}")


# ────────────────────────────────────────────────────────────────────
# Sección (e): Propuesta de corrección
# ────────────────────────────────────────────────────────────────────

class PropuestaCorreccion:
    """Plan de corrección del error odométrico a lo largo del semestre."""

    PLAN = [
        ("11", "lane_detector.py",       "Deriva lateral en rectas",     "PID sobre detección de carril"),
        ("12", "sign_detector.py",        "Señales PARE y semáforo",      "YOLOv8 / color-blob + FSM"),
        ("13", "obstacle_detector.py",    "Karpinchus (RC-4)",            "LiDAR MS200 → evasión"),
        ("14", "route_planner.py",        "Navegación por manzanas",      "A* sobre tile_graph.yaml"),
        ("14-15", "AMCL / EKF",           "Deriva posicional absoluta",   "Fusión odom + LiDAR en mapa"),
    ]

    def mostrar(self) -> None:
        print("  (e) PROPUESTA DE CORRECCIÓN — Plan semanas 11–15")
        print("  ─" * 34)
        df = pd.DataFrame(self.PLAN, columns=["Semana", "Módulo", "Corrige", "Mecanismo"])
        print(df.to_string(index=False))
        print()
        print("  La calibración b_eff de RC-1 es la BASE de todos los módulos.")
        print("  Un b_eff preciso reduce el error de punto de partida para")
        print("  el lane_controller de la semana 11 en adelante.")


# ────────────────────────────────────────────────────────────────────
# Informe completo
# ────────────────────────────────────────────────────────────────────

class InformeTecnico:
    """
    Genera el informe técnico RC-1 completo a partir de archivos reales.

    Fuentes de datos:
        - calibration_log.csv  → calibración b_eff (datos del robot)
        - bags/run1, run2, run3 → errores de cierre (datos del robot)

    No contiene valores hardcodeados de mediciones del robot.
    """

    def __init__(
        self,
        cal_path:  str,
        bag_paths: List[str],
        grupo:     str = "G__",
        domain_id: int = 0,
        out_dir:   str = "informe",
    ):
        self.grupo     = grupo
        self.domain_id = domain_id
        self.out_dir   = out_dir
        os.makedirs(out_dir, exist_ok=True)

        # Cargar datos reales de calibración
        self.cal = AnalizadorCalibracion(cal_path)

        # Procesar bags reales
        self.multi = AnalisisMultiRun(out_dir=os.path.join(out_dir, "plots"))
        for bp in bag_paths:
            self.multi.procesar_bag(bp)

        self.prediccion = PrediccionInicial()
        self.causa_raiz = AnalisisCausaRaiz(
            resultados=self.multi.resultados,
            b_eff_final=self.cal.b_eff_final,
        )
        self.propuesta = PropuestaCorreccion()

    def _cabecera(self) -> None:
        sep = "═" * 66
        print("╔" + sep + "╗")
        print("║  INFORME TÉCNICO — RC-1 La Manzana del Tambo" + " " * 20 + "║")
        print("║  CapyTown · Robótica 2026-I · Universidad ESAN" + " " * 18 + "║")
        print(f"║  Grupo: {self.grupo:<10}  ROS_DOMAIN_ID: {self.domain_id:<4}" + " " * 30 + "║")
        print("╚" + sep + "╝\n")

    def generar(self, con_graficas: bool = True) -> None:
        """Genera el informe completo."""
        self._cabecera()

        # ── (a) Predicciones ─────────────────────────────────────────
        n_laps = 3
        self.prediccion.imprimir_todas(n_laps=n_laps)

        # ── (b) Calibración ──────────────────────────────────────────
        print("\n" + "─" * 66)
        print("  (b) CALIBRACIÓN DE b_eff — datos reales del robot")
        print("─" * 66)
        self.cal.mostrar_resumen()
        print()
        print(self.cal.analisis_cinematico())
        if con_graficas:
            self.cal.graficar_convergencia(
                os.path.join(self.out_dir, "plots", "convergencia_beff.png")
            )
        yaml_path = self.cal.guardar_wheel_params_yaml(
            os.path.join(self.out_dir, "wheel_params.yaml")
        )
        print(f"\nwheel_params.yaml guardado: {yaml_path}")

        # ── (c) Resultados ───────────────────────────────────────────
        print("\n" + "─" * 66)
        print("  (c) RESULTADOS EXPERIMENTALES — datos reales del robot")
        print("─" * 66)
        self.multi.mostrar_resumen()
        if con_graficas and len(self.multi.resultados) > 1:
            self.multi.graficar_comparativa()
            self.causa_raiz.graficar_errores(
                os.path.join(self.out_dir, "plots", "errores_experimentales.png")
            )

        # ── (d) Causa raíz ───────────────────────────────────────────
        print("\n" + "─" * 66)
        print("  (d) ANÁLISIS DE CAUSA RAÍZ")
        print("─" * 66)
        print(self.causa_raiz.diagnostico())

        # ── Comparación predicción vs resultado real ──────────────────
        if self.multi.resultados:
            errores = [r.err_cm for r in self.multi.resultados]
            prom    = sum(errores) / len(errores)
            print("\n  Predicción vs. resultado real:")
            print(f"    Predicción     : {self.prediccion.calcular_prediccion_error(n_laps).split(chr(10))[-1].strip()}")
            print(f"    Resultado real : {prom:.2f} cm promedio")

        # ── (e) Propuesta ─────────────────────────────────────────────
        print("\n" + "─" * 66)
        self.propuesta.mostrar()

        print("\n" + "=" * 66)
        print("  Informe generado correctamente.")
        print(f"  Archivos en: {os.path.abspath(self.out_dir)}/")
        print("=" * 66)


# ────────────────────────────────────────────────────────────────────
# Entry point
# ────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Genera el informe técnico RC-1 a partir de archivos reales del robot.\n"
            "\n"
            "Ejemplo:\n"
            "  python3 informe_tecnico.py \\\n"
            "    --cal calibration_log.csv \\\n"
            "    --bags bags/run1 bags/run2 bags/run3 \\\n"
            "    --grupo G01 --domain 1"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--cal", required=True, metavar="CSV",
        help="Ruta a calibration_log.csv (generado por calibrate_beff.py)",
    )
    parser.add_argument(
        "--bags", required=True, nargs="+", metavar="BAG",
        help="Paths a los bags grabados durante el experimento (run1 run2 run3)",
    )
    parser.add_argument(
        "--grupo",  default="G__", help="Nombre del grupo (p.ej. G01)"
    )
    parser.add_argument(
        "--domain", default=0, type=int, help="ROS_DOMAIN_ID del grupo"
    )
    parser.add_argument(
        "--out-dir", default="informe",
        help="Directorio de salida (default: informe/)",
    )
    parser.add_argument(
        "--sin-graficas", action="store_true",
        help="Generar solo texto sin guardar PNGs",
    )
    args = parser.parse_args()

    try:
        informe = InformeTecnico(
            cal_path=args.cal,
            bag_paths=args.bags,
            grupo=args.grupo,
            domain_id=args.domain,
            out_dir=args.out_dir,
        )
        informe.generar(con_graficas=not args.sin_graficas)
    except (FileNotFoundError, ValueError, RuntimeError) as e:
        print(f"ERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
