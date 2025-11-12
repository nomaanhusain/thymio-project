import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- Inputs ---
csv_path_1 = "truly_successful_runs/timewise_avg_data.csv"
csv_path_2 = "truly_successful_runs/ir0.8/timewise_avg_data.csv"
column = "W"          # Choose one of ["N", "S", "E", "W", "X"]
title_label = f"{column}"

# --- Load data ---
def load_data_get_ftt(csv_path, column):
    df = pd.read_csv(csv_path)

    if "counter" not in df.columns:
        df = df.rename(columns={df.columns[0]: "counter"})

    # Extract numeric series
    y = df[column].to_numpy(dtype=float)
    y = np.where(np.isnan(y), np.nanmean(y), y)  # handle NaNs

    # --- FFT ---
    Y = np.fft.rfft(y)
    N = len(y)
    freqs = np.fft.rfftfreq(N, d=1)  # assuming one sample per counter step
    magnitude = np.abs(Y) / N
    return freqs,magnitude

freqs, magnitude = load_data_get_ftt(csv_path_1, column)
freqs2, magnitude2 = load_data_get_ftt(csv_path_2, column)

# --- Plot magnitude spectrum ---
plt.figure(figsize=(8, 4))
plt.plot(freqs, magnitude, marker='o', linewidth=1)
plt.plot(freqs2, magnitude2, marker='x', linewidth=1)
plt.title(f"FFT Magnitude Spectrum - {title_label}")
plt.xlabel("Normalized Frequency (cycles per timestep)")
plt.ylabel("Magnitude (Amplitude)")
plt.grid(True, alpha=0.3)

# Optional: highlight first few harmonics visually
K = 10  # e.g., show first 10 vertical markers
# plt.axvline(freqs[K], color="red", linestyle="--", label=f"K={K}")
plt.legend()
plt.tight_layout()
plt.show()
