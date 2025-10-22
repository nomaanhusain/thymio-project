import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# --- Define paths ---
csv1 = "truly_successful_runs/timewise_avg_data.csv"
csv2 = "truly_successful_runs/ir0.8/timewise_avg_data.csv"

label1 = "Condition 1"
label2 = "Condition 2"

LINE_COLORS = {
    "N": "tab:red",
    "S": "tab:blue",
    "E": "tab:green",
    "W": "tab:orange",
    "X": "gray",
}

# def load_clean(path):
#     df = pd.read_csv(path)

#     # Fix accidental index column like "Unnamed: 0"
#     if "counter" not in df.columns:
#         for c in df.columns:
#             if str(c).lower().startswith("unnamed"):
#                 df = df.rename(columns={c: "counter"})
#                 break

#     # Ensure numeric
#     df["counter"] = pd.to_numeric(df["counter"], errors="coerce")
#     for c in df.columns:
#         if c != "counter":
#             df[c] = pd.to_numeric(df[c], errors="coerce")

#     # Sort & drop rows without counter
#     df = df.dropna(subset=["counter"]).sort_values("counter")
#     return df

# df1 = load_clean(csv1)
# df2 = load_clean(csv2)

# plt.figure(figsize=(10, 5))
# plt.title("Comparative Study Across Two Conditions")
# plt.xlabel("Timestep")
# plt.ylabel("Robot opinion count")
# plt.grid(True, alpha=0.3)

# for col in ["X", "N", "S", "E", "W"]:
#     if col in df1.columns and col in df2.columns:
#         x1 = df1["counter"].to_numpy().ravel()
#         y1 = df1[col].to_numpy().ravel()
#         x2 = df2["counter"].to_numpy().ravel()
#         y2 = df2[col].to_numpy().ravel()

#         plt.plot(x1, y1,
#                  label=f"{label1} - {col}",
#                  color=LINE_COLORS[col],
#                  linestyle='-',
#                  alpha=0.9 if col != "X" else 0.4)

#         plt.plot(x2, y2,
#                  label=f"{label2} - {col}",
#                  color=LINE_COLORS[col],
#                  linestyle='--',
#                  alpha=0.9 if col != "X" else 0.4)

# plt.legend(ncol=2)
# plt.tight_layout()
# plt.show()

KEEP_HARMONICS = 5  # increase to keep more wiggle; decrease for smoother trends

PLOT_ORDER = ["X", "N", "S", "E", "W"]  # order in legend/plot

def load_clean(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)

    # Fix accidental index column
    if "counter" not in df.columns:
        for c in df.columns:
            if str(c).lower().startswith("unnamed"):
                df = df.rename(columns={c: "counter"})
                break

    # Ensure numeric
    df["counter"] = pd.to_numeric(df["counter"], errors="coerce")
    for c in df.columns:
        if c != "counter":
            df[c] = pd.to_numeric(df[c], errors="coerce")

    # Sort & drop invalid rows
    df = df.dropna(subset=["counter"]).sort_values("counter").reset_index(drop=True)
    return df

def fft_lowpass_trend(y: np.ndarray, keep_n: int) -> np.ndarray:
    """Return low-frequency trend by keeping first keep_n real-FFT bins (incl. DC)."""
    y = np.asarray(y, dtype=float)
    # Fill NaNs with series mean (so FFT works)
    if np.isnan(y).any():
        y = np.where(np.isnan(y), np.nanmean(y), y)

    Y = np.fft.rfft(y)  # real FFT (one-sided)
    keep_n = max(1, min(keep_n, Y.shape[0]))  # guard
    Y[keep_n:] = 0
    trend = np.fft.irfft(Y, n=y.shape[0])
    return trend

# --- Load data ---
df1 = load_clean(csv1)
df2 = load_clean(csv2)

# --- Plot trends ---
plt.figure(figsize=(10, 5))
plt.title(f"FFT Low-Pass Trend Comparison (keep {KEEP_HARMONICS} harmonics)")
plt.xlabel("Counter")
plt.ylabel("Smoothed Trend")
plt.ylim(0,20)
plt.grid(True, alpha=0.3)

for col in PLOT_ORDER:
    if col in df1.columns:
        x1 = df1["counter"].to_numpy().ravel()
        y1 = df1[col].to_numpy().ravel()
        t1 = fft_lowpass_trend(y1, KEEP_HARMONICS)
        if(col != 'X'):
            plt.plot(
                x1, t1,
                label=f"{label1} - {col}",
                color=LINE_COLORS[col],
                linestyle='-',
                alpha=0.9 if col != "X" else 0.45
            )
        plt.plot(
            x1, y1,
            color=LINE_COLORS[col],
            linestyle='-',
            alpha=0.2 if col != "X" else 0.15
        )

    if col in df2.columns:
        x2 = df2["counter"].to_numpy().ravel()
        y2 = df2[col].to_numpy().ravel()
        t2 = fft_lowpass_trend(y2, KEEP_HARMONICS)
        if(col != 'X'):
            plt.plot(
                x2, t2,
                label=f"{label2} - {col}",
                color=LINE_COLORS[col],
                linestyle='--',
                alpha=0.9 if col != "X" else 0.45
            )
        plt.plot(
            x2, y2,
            label=f"{label2} - {col}",
            color=LINE_COLORS[col],
            linestyle='--',
            alpha=0.2 if col != "X" else 0.15
        )

plt.legend(ncol=2)
plt.tight_layout()
plt.show()