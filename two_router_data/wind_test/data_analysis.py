from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ---- Configuration ---------------------------------------------------------

DATA_ROOT = Path("/home/nomaan/thymio-test/two_router_data/wind_test")
REGION_NAME = "south"
OUTPUT_IMAGE = Path("south_sensor_heatmap.png")

# 3x3 layout describing which folder sits in each grid slot; use None for blanks.
GRID_ORDER: List[List[Optional[str]]] = [
    ["3_79", "4_67", "4_74"],
    ["4_76", "3_85", "3_80"],
    ["3_83", "3_86", "3_82"],
]

# Friendly labels for each folder; missing keys fall back to the folder name.
DISPLAY_NAME_MAP: Dict[str, str] = {
    "3_79": "UKON8",
    "3_80": "AGR06",
    "3_82": "AGR05",
    "3_83": "AGR09",
    "3_85": "AGR07",
    "3_86": "AGR08",
    "4_67": "UKON7",
    "4_74": "AGR02",
    "4_76": "UKON11",
}

BINS = 36
CMAP_NAME = "viridis"
FIGURE_SIZE = (12, 12)
WIND_SOURCE_POSITION = "top"  # Accepts "top" or "bottom"
WIND_SOURCE_TEXT = "Wind Source"


# ---- Data loading ----------------------------------------------------------


@dataclass
class SensorSummary:
    angles: np.ndarray
    error_pct: float
    total_samples: int


def iter_sensor_ids(order: Sequence[Sequence[Optional[str]]]) -> Iterable[str]:
    for row in order:
        for item in row:
            if item is not None:
                yield item


def load_sensor_data(
    base_dir: Path,
    region: str,
    sensor_ids: Iterable[str],
) -> Dict[str, SensorSummary]:
    region_dir = base_dir / region
    if not region_dir.exists():
        raise FileNotFoundError(f"Region directory not found: {region_dir}")

    summaries: Dict[str, SensorSummary] = {}

    for sensor_id in sensor_ids:
        csv_path = region_dir / sensor_id / "output.csv"
        if not csv_path.exists():
            raise FileNotFoundError(f"Missing output.csv for sensor '{sensor_id}' at {csv_path}")

        df = pd.read_csv(csv_path)
        if "angle" not in df.columns:
            raise ValueError(f"'angle' column missing in {csv_path}")

        total_samples = len(df)
        if total_samples == 0:
            summaries[sensor_id] = SensorSummary(
                angles=np.array([], dtype=float),
                error_pct=0.0,
                total_samples=0,
            )
            continue

        invalid_mask = df["angle"] == -999.0
        invalid_count = int(invalid_mask.sum())
        valid_angles = df.loc[~invalid_mask, "angle"].to_numpy(dtype=float)
        error_pct = (invalid_count / total_samples) * 100.0

        summaries[sensor_id] = SensorSummary(
            angles=valid_angles,
            error_pct=error_pct,
            total_samples=total_samples,
        )

    return summaries


# ---- Visualisation ---------------------------------------------------------


def polar_ring_heatmap(
    sensor_order: Sequence[Sequence[Optional[str]]],
    sensor_data: Dict[str, SensorSummary],
    display_names: Dict[str, str],
    bins: int = BINS,
    cmap_name: str = CMAP_NAME,
    figure_size: Tuple[float, float] = FIGURE_SIZE,
    wind_source_position: str = WIND_SOURCE_POSITION,
    wind_source_text: str = WIND_SOURCE_TEXT,
) -> plt.Figure:
    rows = len(sensor_order)
    cols = max((len(row) for row in sensor_order), default=0)
    fig, axes = plt.subplots(
        rows,
        cols,
        subplot_kw={"projection": "polar"},
        figsize=figure_size,
        constrained_layout=True,
        squeeze=False,
    )
    cmap = plt.get_cmap(cmap_name)
    bin_edges = np.linspace(0.0, 360.0, num=bins + 1)

    histograms: Dict[str, np.ndarray] = {}
    maxima = []
    for sensor_id, summary in sensor_data.items():
        if summary.angles.size == 0:
            histograms[sensor_id] = np.zeros(bins, dtype=float)
            maxima.append(0.0)
            continue

        wrapped = np.mod(summary.angles, 360.0)
        counts, _ = np.histogram(wrapped, bins=bin_edges)
        histograms[sensor_id] = counts.astype(float)
        maxima.append(float(counts.max() if counts.size else 0.0))

    global_max = max(maxima, default=1.0)
    if global_max == 0.0:
        global_max = 1.0

    for row_idx, row in enumerate(sensor_order):
        for col_idx, sensor_id in enumerate(row):
            ax = axes[row_idx, col_idx]

            if sensor_id is None:
                ax.axis("off")
                continue

            summary = sensor_data.get(sensor_id)
            if summary is None:
                ax.text(
                    0.5,
                    0.5,
                    "Missing data",
                    transform=ax.transAxes,
                    ha="center",
                    va="center",
                    fontsize=12,
                )
                ax.set_xticklabels([])
                ax.set_yticklabels([])
                continue

            counts = histograms.get(sensor_id, np.zeros(bins, dtype=float))
            theta_centers = np.deg2rad((bin_edges[1:] + bin_edges[:-1]) / 2.0)
            widths = np.deg2rad(bin_edges[1:] - bin_edges[:-1])

            if counts.sum() == 0.0:
                ax.text(
                    0.5,
                    0.5,
                    "No valid\ndata",
                    transform=ax.transAxes,
                    ha="center",
                    va="center",
                    fontsize=12,
                )
            else:
                colors = cmap(counts / global_max)
                ax.bar(
                    theta_centers,
                    np.ones_like(theta_centers),
                    width=widths,
                    bottom=0.0,
                    color=colors,
                    edgecolor="white",
                    linewidth=0.5,
                )

            label = display_names.get(sensor_id, sensor_id)
            ax.set_title(f"{label}\nErr {summary.error_pct:.1f}%", pad=20)
            ax.set_yticklabels([])
            ax.set_xticklabels([])
            ax.set_theta_offset(-np.pi / 2.0)
            ax.set_theta_direction(-1)

    position = wind_source_position.lower()
    
    # if position == "top":
    #     fig.text(
    #         0.5,
    #         0.97,
    #         f"{wind_source_text} ↓",
    #         ha="center",
    #         va="center",
    #         fontsize=14,
    #         fontweight="bold",
    #     )
    # elif position == "bottom":
    #     fig.text(
    #         0.5,
    #         0.03,
    #         f"{wind_source_text} ↑",
    #         ha="center",
    #         va="center",
    #         fontsize=14,
    #         fontweight="bold",
    #     )
    # else:
    #     fig.text(
    #         0.5,
    #         0.97,
    #         wind_source_text,
    #         ha="center",
    #         va="center",
    #         fontsize=14,
    #         fontweight="bold",
    #     )

    return fig


# ---- Script entry ----------------------------------------------------------


def main() -> None:
    sensor_ids = list(iter_sensor_ids(GRID_ORDER))
    sensor_data = load_sensor_data(DATA_ROOT, REGION_NAME, sensor_ids)
    figure = polar_ring_heatmap(
        GRID_ORDER,
        sensor_data,
        DISPLAY_NAME_MAP,
        wind_source_position=WIND_SOURCE_POSITION,
        wind_source_text=WIND_SOURCE_TEXT,
    )

    OUTPUT_IMAGE.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT_IMAGE, dpi=300)
    plt.close(figure)
    print(f"Saved heatmap grid to {OUTPUT_IMAGE.resolve()}")


if __name__ == "__main__":
    main()
