from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ---- Configuration ---------------------------------------------------------

DATA_ROOT = Path("/home/nomaan/thymio-test/two_router_data/wind_test")
REGION = "south"
OUTPUT_PATH = Path(f"{REGION}_sensor_timeseries_grid.png")
SENSOR_IDS: Sequence[str] = (
    "3_79",
    "4_67",
    "4_74",
    "4_76",
    "3_85",
    "3_80",
    "3_83",
    "3_86",
    "3_82",
)
SUBPLOT_ROWS = 3
SUBPLOT_COLS = 3
FIGURE_SIZE = (14, 10)
LINE_COLOR = "#1f77b4"
ERROR_COLOR = "#d62728"
BASELINE_COLOR = "#ff7f0e"
ERROR_VALUE = -999.0
ERROR_MARKER_SIZE = 18
USE_ELAPSED_SECONDS = True  # If True, x-axis is seconds since first reading; else raw Unix time.
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
ERROR_BASELINE: Dict[str, float] = {
    "north": 0.0,
    "south": -180.0,
}


# ---- Data handling ---------------------------------------------------------


@dataclass
class SensorFrame:
    timestamps: np.ndarray
    angles: np.ndarray
    error_mask: np.ndarray


def load_sensor_ids(region_path: Path, fallback: Sequence[str]) -> List[str]:
    """
    Return sensor IDs from fallback list if provided; otherwise enumerate directories.
    """
    if fallback:
        return list(fallback)
    return sorted(p.name for p in region_path.iterdir() if p.is_dir())


def load_sensor_frame(sensor_dir: Path) -> SensorFrame:
    csv_path = sensor_dir / "output.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing output.csv at {csv_path}")

    df = pd.read_csv(csv_path)
    required = {"timestamp", "angle"}
    missing = required.difference(df.columns)
    if missing:
        missing_fmt = ", ".join(f"'{col}'" for col in sorted(missing))
        raise ValueError(f"Columns {missing_fmt} missing in {csv_path}")

    df = df.sort_values("timestamp").reset_index(drop=True)
    timestamps = pd.to_numeric(df["timestamp"], errors="coerce").to_numpy(dtype=float)
    angles = pd.to_numeric(df["angle"], errors="coerce").to_numpy(dtype=float)
    error_mask = np.isclose(angles, ERROR_VALUE, atol=1e-12)

    if USE_ELAPSED_SECONDS and timestamps.size:
        timestamps = timestamps - timestamps[0]

    return SensorFrame(
        timestamps=timestamps,
        angles=angles,
        error_mask=error_mask,
    )


# ---- Plotting --------------------------------------------------------------


def plot_sensor_timeseries(
    frames: Iterable[SensorFrame],
    sensor_ids: Sequence[str],
    display_names: Dict[str, str],
    region: str,
) -> plt.Figure:
    if len(sensor_ids) != SUBPLOT_ROWS * SUBPLOT_COLS:
        raise ValueError(
            "SUBPLOT_ROWS * SUBPLOT_COLS must match number of sensor IDs"
        )

    fig, axes = plt.subplots(
        SUBPLOT_ROWS,
        SUBPLOT_COLS,
        sharex=True,
        figsize=FIGURE_SIZE,
        constrained_layout=True,
    )
    axes_flat = axes.flatten()

    for idx, (frame, sensor_id) in enumerate(zip(frames, sensor_ids)):
        ax = axes_flat[idx]
        timestamps = frame.timestamps
        angles = frame.angles
        error_mask = frame.error_mask

        clean_series = angles.astype(float)
        clean_series[error_mask] = np.nan
        clean_series = pd.Series(clean_series).ffill().to_numpy()

        ax.plot(timestamps, clean_series, color=LINE_COLOR, linewidth=1.2)

        if error_mask.any():
            error_times = timestamps[error_mask]
            error_values = clean_series[error_mask]
            fallback_value = ERROR_BASELINE.get(region.lower(), 0.0)
            error_plot_values = np.where(np.isnan(error_values), fallback_value, error_values)
            valid_points = ~np.isnan(error_plot_values)
            if valid_points.any():
                ax.scatter(
                    error_times[valid_points],
                    error_plot_values[valid_points],
                    color=ERROR_COLOR,
                    s=ERROR_MARKER_SIZE,
                    zorder=3,
                )

        label = display_names.get(sensor_id, sensor_id)
        ax.set_title(label)
        ax.grid(alpha=0.25)
        if idx % SUBPLOT_COLS == 0:
            ax.set_ylabel("Angle (°)")

        baseline_value = ERROR_BASELINE.get(region.lower(), 0.0)
        ax.axhline(
            baseline_value,
            color=BASELINE_COLOR,
            linewidth=1.1,
            linestyle="-",
        )

    for ax in axes_flat[-SUBPLOT_COLS:]:
        ax.set_xlabel("Time (s)" if USE_ELAPSED_SECONDS else "Unix Time (s)")

    figure_title = region.capitalize()
    plt.suptitle(figure_title, fontsize=18, fontweight="bold")

    return fig


# ---- Script entry ----------------------------------------------------------


def main() -> None:
    region_path = DATA_ROOT / REGION
    if not region_path.exists():
        raise FileNotFoundError(f"Region directory not found: {region_path}")

    sensor_ids = load_sensor_ids(region_path, SENSOR_IDS)
    frames = [load_sensor_frame(region_path / sensor_id) for sensor_id in sensor_ids]

    figure = plot_sensor_timeseries(frames, sensor_ids, DISPLAY_NAME_MAP, REGION)
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    plt.show()
    figure.savefig(OUTPUT_PATH, dpi=300)
    plt.close(figure)
    print(f"Saved time series grid to {OUTPUT_PATH.resolve()}")


if __name__ == "__main__":
    main()
