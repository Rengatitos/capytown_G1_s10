#!/usr/bin/env python3
"""
Generador formal del informe RC-1 en estilo academico simple (APA simplificado).
Produce un PDF de una pagina usando los datos de calibration_log.csv y los plots
mas recientes del repositorio.
"""

from __future__ import annotations

import csv
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm
from reportlab.platypus import (
    Image,
    KeepTogether,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

ROOT = Path(__file__).resolve().parents[1]
PDF_PATH = ROOT / "informe.pdf"
CAL_LOG = ROOT / "calibration_log.csv"
PLOTS = ROOT / "plots"

TITLE = "Informe tecnico RC-1: La Manzana del Tambo"
SUBTITLE = "CapyTown ESAN 2026-I | Grupo G1"

INTEGRANTES = [
    "AGUILAR CONTRERAS Angel Jesus",
    "CABALLERO SALAZAR Mattias Lincoln",
    "NECIOSUP SAAVEDRA Leslie Jazmin",
    "TICONA SANCHEZ Camila Danna",
]


def read_calibration_rows(path: Path):
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        return list(reader)


def latest_value(rows, key):
    return rows[-1][key] if rows else ""


def build_history_lines(rows):
    lines = []
    for index, row in enumerate(rows, start=1):
        lines.append(
            f"Run {index}: b_eff={float(row['b_eff_used']):.5f} "
            f"-> sugerido {float(row['suggested_b_eff']):.5f} "
            f"(error={float(row['error_cm']):.2f} cm, {float(row['error_pct']):.2f}%)"
        )
    return lines


def make_style_sheet():
    styles = getSampleStyleSheet()
    styles.add(
        ParagraphStyle(
            name="TitleAPA",
            parent=styles["Title"],
            fontName="Times-Bold",
            fontSize=14,
            leading=16,
            alignment=TA_CENTER,
            spaceAfter=4,
        )
    )
    styles.add(
        ParagraphStyle(
            name="SubtitleAPA",
            parent=styles["Normal"],
            fontName="Times-Italic",
            fontSize=9,
            leading=11,
            alignment=TA_CENTER,
            spaceAfter=6,
        )
    )
    styles.add(
        ParagraphStyle(
            name="BodyAPA",
            parent=styles["Normal"],
            fontName="Times-Roman",
            fontSize=8.3,
            leading=9.4,
            alignment=TA_JUSTIFY,
            spaceAfter=2,
        )
    )
    styles.add(
        ParagraphStyle(
            name="SectionAPA",
            parent=styles["Normal"],
            fontName="Times-Bold",
            fontSize=9.2,
            leading=10.2,
            alignment=TA_LEFT,
            spaceBefore=2,
            spaceAfter=2,
        )
    )
    styles.add(
        ParagraphStyle(
            name="MiniAPA",
            parent=styles["Normal"],
            fontName="Times-Roman",
            fontSize=7.4,
            leading=8.2,
            alignment=TA_JUSTIFY,
            spaceAfter=1,
        )
    )
    return styles


def build_pdf():
    rows = read_calibration_rows(CAL_LOG)
    final_b_eff = float(rows[-1]["suggested_b_eff"]) if rows else 0.0
    final_error_cm = float(rows[-1]["error_cm"]) if rows else 0.0

    styles = make_style_sheet()
    story = []

    story.append(Paragraph(TITLE, styles["TitleAPA"]))
    story.append(Paragraph(SUBTITLE, styles["SubtitleAPA"]))
    summary = Table(
        [
            ["Grupo", "G1"],
            ["ROS_DOMAIN_ID", "1"],
            ["b_eff final", f"{final_b_eff:.5f} m"],
        ],
        colWidths=[3 * cm, 4 * cm],
    )

    summary.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (0, -1),
                 colors.HexColor("#e9eef5")),
                ("GRID", (0, 0), (-1, -1), 0.3, colors.grey),
                ("FONTNAME", (0, 0), (0, -1), "Times-Bold"),
                ("FONTSIZE", (0, 0), (-1, -1), 8),
            ]
        )
    )

    story.append(summary)
    story.append(Spacer(1, 0.1 * cm))

    story.append(Paragraph("(a) Prediccion inicial", styles["SectionAPA"]))
    story.append(
        Paragraph(
            "Se preveía que la odometría del robot skid-steer acumularía errores por deslizamiento lateral en giros, haciendo que la separación efectiva de ruedas (b_eff) fuera mayor que el ancho físico. Una sobreestimación del giro por odometría indicaría un b_eff pequeño. Se confirmó que, aun con calibración, el error se acumula en lazo abierto. Nuestro ejercicio simulado, tras una calibración iterativa, estableció un b_eff final de 0.23000 m con un error de cierre de 0.01-0.02 cm en un cuadrado de 1x1m.",
            styles["BodyAPA"],
        )
    )


    story.append(Paragraph("(b) Calibracion de b_eff", styles["SectionAPA"]))

    story.append(
        Paragraph(
            f"Se realizaron {len(rows)} iteraciones de calibracion mediante "
            f"UMBmark simplificado. El valor final adoptado fue "
            f"{final_b_eff:.5f} m con un error residual de "
            f"{final_error_cm:.2f} cm.",
            styles["BodyAPA"],
        )
    )

    # Mostrar solo las ultimas 5 iteraciones para ahorrar espacio
    cal_data = [
        ["Iter", "b_eff usado", "b_eff sugerido", "Error cm", "Error %"]
    ]

    for idx, row in enumerate(rows[-5:], start=len(rows)-4):
        cal_data.append([
            str(idx),
            f"{float(row['b_eff_used']):.5f}",
            f"{float(row['suggested_b_eff']):.5f}",
            f"{float(row['error_cm']):.2f}",
            f"{float(row['error_pct']):.2f}",
        ])

    cal_table = Table(
        cal_data,
        colWidths=[
            1.0 * cm,
            2.3 * cm,
            2.5 * cm,
            1.8 * cm,
            1.8 * cm,
        ],
    )

    cal_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1F4E79")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("FONTNAME", (0, 0), (-1, 0), "Times-Bold"),
                ("FONTSIZE", (0, 0), (-1, -1), 7),
                ("GRID", (0, 0), (-1, -1), 0.3, colors.grey),
                ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1),
                 [colors.white, colors.HexColor("#f6f8fb")]),
            ]
        )
    )

    story.append(cal_table)
    story.append(Spacer(1, 0.10 * cm))

    story.append(Paragraph("(c) Resultados experimentales", styles["SectionAPA"]))
    story.append(
        Paragraph(
            "El peor intento visual corresponde a trayectoria_run1_intento_fallido.png. "
            "El mejor cierre corresponde a trayectoria_corregida_final.png. "
            "Como no existen mediciones de orientacion registradas, dtheta queda documentado como pendiente.",
            styles["BodyAPA"],
        )
    )

    res_table = Table(
        [
            ["Run", "dx [cm]", "dy [cm]", "dtheta"],
            ["1", "+0.15", "-0.11", "[PENDIENTE: completar Δθ run1]"],
            ["2", "-0.11", "+0.16", "[PENDIENTE: completar Δθ run2]"],
            ["3", "+0.13", "+0.12", "[PENDIENTE: completar Δθ run3]"],
        ],
        colWidths=[1.1 * cm, 1.4 * cm, 1.4 * cm, 6.2 * cm],
    )
    res_table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e9eef5")),
                ("FONTNAME", (0, 0), (-1, 0), "Times-Bold"),
                ("FONTNAME", (0, 1), (-1, -1), "Times-Roman"),
                ("FONTSIZE", (0, 0), (-1, -1), 7.0),
                ("LEADING", (0, 0), (-1, -1), 8.0),
                ("GRID", (0, 0), (-1, -1), 0.35, colors.grey),
                ("ALIGN", (1, 1), (2, -1), "CENTER"),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f9fbfd")]),
            ]
        )
    )
    story.append(res_table)

    story.append(Paragraph("(d) Analisis de causa raiz", styles["SectionAPA"]))
    story.append(
        Paragraph(
            "La fuente dominante de error es el slip lateral en las esquinas del skid-steer. "
            "La integracion de velocidades de rueda no coincide con el desplazamiento real del chasis, "
            "por lo que la trayectoria se abre antes de cerrar el cuadrado.",
            styles["BodyAPA"],
        )
    )

    story.append(Paragraph("(e) Propuesta de correccion", styles["SectionAPA"]))
    story.append(
        Paragraph(
            "Semana 10: calibracion fina de b_eff y validacion de /odom. "
            "Semana 11: correccion de error lateral en seguimiento de carril. "
            "Semana 12: manejo de señales y parada segura. "
            "Semana 13: navegacion con obstaculos. "
            "Semana 14-15: fusion sensorial para reducir deriva posicional.",
            styles["BodyAPA"],
        )
    )

    img1 = Image(str(PLOTS / "trayectoria_run1_intento_fallido.png"))
    img2 = Image(str(PLOTS / "trayectoria_run3_iter2.png"))
    img1.drawWidth = 7.8 * cm
    img1.drawHeight = 3.9 * cm
    img2.drawWidth = 7.8 * cm
    img2.drawHeight = 3.9 * cm
    images = Table([[img1, img2]], colWidths=[8.7 * cm, 8.7 * cm])
    images.setStyle(TableStyle([("ALIGN", (0, 0), (-1, -1), "CENTER"), ("VALIGN", (0, 0), (-1, -1), "TOP")]))
    story.append(images)

    story.append(
        Paragraph(
            f"El valor final de b_eff es {final_b_eff:.5f} m y el error final reportado por la ultima iteracion es {final_error_cm:.2f} cm.",
            styles["MiniAPA"],
        )
    )

    doc = SimpleDocTemplate(
        str(PDF_PATH),
        pagesize=A4,
        leftMargin=1.0 * cm,
        rightMargin=1.0 * cm,
        topMargin=0.9 * cm,
        bottomMargin=0.9 * cm,
    )
    doc.build(story)
    print(PDF_PATH)


if __name__ == "__main__":
    build_pdf()
