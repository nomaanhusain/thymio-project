import os
import pandas as pd
import numpy as np
from glob import glob
import matplotlib.pyplot as plt
import seaborn as sns

# base_path = "truly_successful_runs/run23/data"
# true_direction = "W"

# def compute_noise(df, true_direction):
#     # Filter up to the '*' row
#     star_idx = df.index[df.iloc[:, 0].astype(str).str.contains(r'\*')].tolist()
#     if star_idx:
#         df = df.iloc[:star_idx[0]]
    
#     if 'cam_wind_direction' not in df.columns:
#         raise ValueError("Missing 'cam_wind_direction' column.")
    
#     # Clean NaNs or invalid entries
#     df = df.dropna(subset=['cam_wind_direction'])
    
#     # Error Rate
#     total = len(df)
#     errors = np.sum(df['cam_wind_direction'] != true_direction)
#     error_rate = errors / total if total > 0 else np.nan

#     # Shannon Entropy
#     probs = df['cam_wind_direction'].value_counts(normalize=True)
#     entropy = -np.sum(probs * np.log2(probs))

#     # Transition Noise (instability)
#     transitions = np.sum(df['cam_wind_direction'].values[:-1] != df['cam_wind_direction'].values[1:])
#     transition_rate = transitions / (total - 1) if total > 1 else np.nan

#     return error_rate, entropy, transition_rate

# results = []

# for folder in os.listdir(base_path):
#     folder_path = os.path.join(base_path, folder)
#     if not os.path.isdir(folder_path):
#         continue
    
#     csv_files = glob(os.path.join(folder_path, "log*.csv"))
#     for f in csv_files:
#         try:
#             df = pd.read_csv(f)
#             e_rate, ent, t_rate = compute_noise(df, true_direction)
#             results.append({
#                 "sensor": folder,
#                 "file": os.path.basename(f),
#                 "error_rate": e_rate,
#                 "entropy": ent,
#                 "transition_rate": t_rate
#             })
#         except Exception as e:
#             print(f"Error reading {f}: {e}")

# # Summarize by run (average of all sensors)
# df_res = pd.DataFrame(results)
# # run_summary = df_res.groupby("sensor")[["error_rate", "entropy", "transition_rate"]].mean()
# # print(run_summary)
# print(df_res["error_rate"].mean())

# RUN_DIRS = [
#     "truly_successful_runs/run18",
#     "truly_successful_runs/run19",
#     "truly_successful_runs/run20",
#     "truly_successful_runs/run21",
#     "truly_successful_runs/run23"
# ]

RUN_DIRS = [
    "truly_successful_runs/run20"
]

TRUE_DIRECTION = "W"  # correct opinion before the '*' row


def trim_to_star(df: pd.DataFrame) -> pd.DataFrame:
    """Keep rows strictly before the row that is exactly '*,*,*,*,*' (all columns '*')."""
    if df.empty:
        return df
    mask_star = df.apply(lambda row: all(str(x).strip() == '*' for x in row), axis=1)
    star_idx = mask_star[mask_star].index
    if len(star_idx) > 0:
        # keep rows before the first star row
        df = df.loc[:star_idx[0] - 1]
    return df


def compute_file_error_rate(csv_path: str, true_dir: str) -> float:
    """Compute the error rate for a single log file up to the '*' separator."""
    df = pd.read_csv(csv_path)
    df = trim_to_star(df)

    if 'cam_wind_direction' not in df.columns:
        raise ValueError(f"Missing 'cam_wind_direction' in {csv_path}")

    df = df.dropna(subset=['cam_wind_direction'])

    total = len(df)
    if total == 0:
        return np.nan

    errors = np.sum(df['cam_wind_direction'] != true_dir)
    return errors / total


def summarize_runs(run_dirs, true_dir="W"):
    """
    Returns:
      df_files: one row per file with file-level error_rate
      df_runs: per-run mean error_rate (averaging sensors equally)
    """
    file_rows = []
    run_index = 0
    for run_root in run_dirs:
        run_index += 1
        data_dir = os.path.join(run_root, "data")
        if not os.path.isdir(data_dir):
            print(f"Skipping {run_root}: no 'data/' directory found.")
            continue

        # sensor folders directly under runX/data
        for sensor_folder in sorted(os.listdir(data_dir)):
            sensor_path = os.path.join(data_dir, sensor_folder)
            if not os.path.isdir(sensor_path):
                continue

            csv_files = glob(os.path.join(sensor_path, "log*.csv"))
            if not csv_files:
                continue

            for f in csv_files:
                try:
                    er = compute_file_error_rate(f, true_dir)
                except Exception as e:
                    print(f"Error processing {f}: {e}")
                    er = np.nan
                file_rows.append({
                    "run": run_index,
                    "sensor": sensor_folder,
                    "file": os.path.basename(f),
                    "file_path": f,
                    "error_rate": er
                })

    df_files = pd.DataFrame(file_rows)

    if df_files.empty:
        print("No files found/processed.")
        return df_files, pd.DataFrame()

    # 1) Average over files per sensor (so each sensor counts once)
    df_sensor = (
        df_files.groupby(["run", "sensor"], as_index=False)["error_rate"].mean()
        .rename(columns={"error_rate": "sensor_error_rate"})
    )

    # 2) Average over sensors per run (representative noise per run)
    
    df_runs = (
        df_sensor.groupby("run", as_index=False)["sensor_error_rate"].mean()
        .rename(columns={"sensor_error_rate": "run_error_rate"})
        .sort_values("run")
    )

    print(df_sensor)

    return df_files, df_runs, df_sensor


# ----- Run the summary and print results -----
df_files, df_runs, df_sensor = summarize_runs(RUN_DIRS, true_dir=TRUE_DIRECTION)

if not df_runs.empty:
    # Pretty print per-run values
    print("\nPer-run noise:")
    for _, row in df_runs.iterrows():
        print(f"  {row['run']}: {row['run_error_rate']:.4f}")

    # Overall mean across runs
    overall_mean = df_runs["run_error_rate"].mean()
    print(f"\nOverall mean noise across runs: {overall_mean:.4f}")
else:
    print("No per-run summary produced.")

sns.set(style="whitegrid", font_scale=1.2)

plt.figure(figsize=(8, 5))
ax = sns.boxplot(
    x="run",
    y="sensor_error_rate",
    data=df_sensor,
    color="skyblue",
    showmeans=True,
    meanprops={"marker": "o", "markerfacecolor": "black", "markeredgecolor": "black"}
)

sns.stripplot(
    x="run",
    y="sensor_error_rate",
    data=df_sensor,
    color="black",
    alpha=0.4,
    size=4,
    jitter=True
)

overall_mean = df_sensor["sensor_error_rate"].mean()

# Add horizontal dashed line
plt.axhline(
    overall_mean,
    color="red",
    linestyle="--",
    linewidth=1.8,
    label=f"Average Sensor Noise = {overall_mean:.3f}"
)


# plt.title("Sensor Noise per Run", fontsize=16)
plt.ylabel(r"Sensor Noise ($\zeta$)")
plt.xlabel("Experiment")
plt.ylim(0.33, 0.9)  # since error_rate ∈ [0,1]
plt.legend(loc="lower left", frameon=True)
plt.grid(axis='y', alpha=0.3)
plt.tight_layout()
# plt.savefig("truly_successful_runs/sensor_noise_boxplot_trns.png", dpi=300, transparent=True)
plt.show()