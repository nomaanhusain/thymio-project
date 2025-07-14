import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np

# Set path to root directory
root_dir = 'run_experiment_data/ir1.0/'
# root_dir = 'run_experiment_data/ir_4_uninf/'

selected_runs = ['run_22', 'run_23', 'run_24', 'run_1_1', 'run_3_3']
save_data = False

# counts[index][direction] = list of counts across runs
counts = defaultdict(lambda: defaultdict(list))

right_option = 'N'
tot_cnt = 0
right_cnt = 0

# Loop through selected runs
for run in selected_runs:
    run_path = os.path.join(root_dir, run)
    if not os.path.isdir(run_path):
        print(f"Warning: {run_path} does not exist or is not a directory.")
        continue

    temp_counts = defaultdict(lambda: defaultdict(int))

    # Loop through nested folders inside the run directory
    for subfolder in os.listdir(run_path):
        subfolder_path = os.path.join(run_path, subfolder)
        if not os.path.isdir(subfolder_path):
            continue

        for file in os.listdir(subfolder_path):
            if file.endswith(".csv"):
                file_path = os.path.join(subfolder_path, file)
                df = pd.read_csv(file_path)
                for _, row in df.iterrows():
                    idx = int(row['index'])
                    msg = row['message']
                    #msg = row['cam_wind_direction']
                    tot_cnt += 1
                    if msg == right_option:
                        right_cnt += 1
                    temp_counts[idx][msg] += 1

    # Merge this run's data into the global structure
    for idx in temp_counts:
        for d in ['N', 'S', 'E', 'W']:
            counts[idx][d].append(temp_counts[idx].get(d, 0))

# Prepare average data
avg_plot_data = {d: [] for d in ['N', 'S', 'E', 'W']}
indices = sorted(counts.keys())

avg_plot_data = {d: [] for d in ['N', 'S', 'E', 'W']}
std_plot_data = {d: [] for d in ['N', 'S', 'E', 'W']}  # for noise/variation

for idx in indices:
    for d in ['N', 'S', 'E', 'W']:
        vals = counts[idx][d]
        avg = np.mean(vals) if vals else 0
        std = np.std(vals) if vals else 0
        avg_plot_data[d].append(avg)
        std_plot_data[d].append(std)

print(f"Ratio of right opinion over total opinions ({right_cnt}/{tot_cnt}): {right_cnt/tot_cnt:.3f}")

plt.figure(figsize=(12, 6))
# colors = {'N': 'blue', 'S': 'red', 'E': 'green', 'W': 'orange'}
colors = {'N': 'blue', 'S': 'red', 'E': 'green', 'W': 'orange'}

for d in ['N', 'S', 'E', 'W']:
    mean_vals = avg_plot_data[d]
    std_vals = std_plot_data[d]
    plt.plot(indices, mean_vals, label=f"{d} (avg)", color=colors[d])
    plt.fill_between(indices,
                     np.array(mean_vals) - np.array(std_vals),
                     np.array(mean_vals) + np.array(std_vals),
                     alpha=0.2, color=colors[d], label=f"{d} (std dev.)")

plt.xlabel("Index")
plt.ylabel("Average Count")
plt.title("Wind Direction Opinions and Standard Deviation")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
