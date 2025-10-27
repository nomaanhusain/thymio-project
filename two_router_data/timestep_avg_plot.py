import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
# --- Define paths ---
csv1_ir1 = "truly_successful_runs/timewise_avg_data.csv"
csv2_ir08 = "truly_successful_runs/ir0.8/timewise_avg_data.csv"

label1 = r"$\rho=1.0$"
label2 = r"$\rho=0.8$"

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

# PLOT_ORDER = ["X", "N", "S", "E", "W"]  # order in legend/plot
PLOT_ORDER = ["X", "E", "W"]  # order in legend/plot

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
df1_ir1 = load_clean(csv1_ir1)
df2_ir08 = load_clean(csv2_ir08)

# --- Plot trends ---
plt.figure(figsize=(10, 5))
# plt.title(f"Swarm Performance")
plt.xlabel("Time (Sec.)",size=16)
plt.ylabel("Opinion Count",size=16)
plt.ylim(0,20)
plt.grid(True, alpha=0.3)

west_ir1 = None
west_ir08 = None
east_ir1 = None
east_ir08 = None
west_ir1_fft = None
west_ir08_fft = None
east_ir1_fft = None
east_ir08_fft = None
for col in PLOT_ORDER:
    if col in df1_ir1.columns:
        if col == 'X': continue
        x1 = df1_ir1["counter"].to_numpy().ravel()
        y1 = df1_ir1[col].to_numpy().ravel()
        t_sec_1 = x1 * 5
        t1 = fft_lowpass_trend(y1, KEEP_HARMONICS)
        if col == 'W':
            west_ir1 = y1
            west_ir1_fft = t1
        if col == 'E':
            east_ir1 = y1
            east_ir1_fft = t1
        if(col != 'X'):
            plt.plot(
                t_sec_1, t1,
                label=fr"{label1}, {col}, FT",
                color=LINE_COLORS[col],
                linestyle='-',
                alpha=0.9 if col != "X" else 0.45
            )
        plt.plot(
            t_sec_1, y1,
            label = fr"{label1} : {col}" if col != 'X' else None,
            color=LINE_COLORS[col],
            linestyle='-',
            alpha=0.2 if col != "X" else 0.15
        )

    if col in df2_ir08.columns:
        if col == 'X': continue
        x2 = df2_ir08["counter"].to_numpy().ravel()
        y2 = df2_ir08[col].to_numpy().ravel()
        print(x2)
        t_sec_08 = x2 * 5
        t2 = fft_lowpass_trend(y2, KEEP_HARMONICS)
        if col == 'W':
            west_ir08 = y2
            west_ir08_fft = t2
        if col == 'E':
            east_ir08 = y2
            east_ir08_fft = t2
        if(col != 'X'):
            plt.plot(
                t_sec_08, t2,
                label=fr"{label2}, {col}, FT",
                color=LINE_COLORS[col],
                linestyle='--',
                alpha=0.9 if col != "X" else 0.45
            )
        plt.plot(
            t_sec_08, y2,
            label = fr"{label2} : {col}" if col != 'X' else None,
            color=LINE_COLORS[col],
            linestyle='--',
            alpha=0.2 if col != "X" else 0.15
        )
ir1 = np.append(west_ir1[0:int(len(west_ir1)/2)], east_ir1[int(len(east_ir1)/2):])
ir08 = np.append(west_ir08[0:int(len(west_ir08)/2)], east_ir08[int(len(east_ir08)/2):])
# print(ir1)
# print(len(ir1))
# print(ir08)
# print(len(ir08))
mean_ir1 = np.mean(ir1)
mean_ir08 = np.mean(ir08)
print(f"ir1.0 mean = {mean_ir1}\nir08 mean = {mean_ir08}\n perf. gain = {mean_ir08 - mean_ir1}")
print("-------Fourier Transform--------")
fft_ir1 = np.append(west_ir1_fft[0:int(len(west_ir1_fft)/2)], east_ir1_fft[int(len(east_ir1_fft)/2):])
fft_ir08 = np.append(west_ir08_fft[0:int(len(west_ir08_fft)/2)], east_ir08_fft[int(len(east_ir08_fft)/2):])

fft_mean_ir1 = np.mean(fft_ir1)
fft_mean_ir08 = np.mean(fft_ir08)
print(f"ir1.0 mean = {fft_mean_ir1}\nir08 mean = {fft_mean_ir08}\n perf. gain = {fft_mean_ir08 - fft_mean_ir1}")

plt.tick_params(axis='x', labelsize=16)  # x-axis ticks
plt.tick_params(axis='y', labelsize=16)  # y-axis ticks
plt.legend(ncol=2)
plt.tight_layout()
plt.savefig("truly_successful_runs/real_world_comp_ir08.png", dpi=300, transparent=True)
plt.show()