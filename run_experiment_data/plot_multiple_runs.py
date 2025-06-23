import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict

# Set path to root directory
root_dir = 'run_experiment_data/ir1.0/'


selected_runs = ['run_20', 'run_21']

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

for idx in indices:
    for d in ['N', 'S', 'E', 'W']:
        vals = counts[idx][d]
        avg = sum(vals) / len(vals) if vals else 0
        avg_plot_data[d].append(avg)

print(f"Ratio of right opinion over total opinions ({right_cnt}/{tot_cnt}): {right_cnt/tot_cnt:.3f}")

# Plotting
plt.figure(figsize=(12, 6))
colors = {'N': 'blue', 'S': 'red', 'E': 'green', 'W': 'orange'}
for d in ['N', 'S', 'E', 'W']:
    plt.plot(indices, avg_plot_data[d], label=f"{d} (avg)", color=colors[d])

plt.xlabel("Index")
plt.ylabel("Average Count")
plt.title("Average Wind Direction Opinion by Index (Selected Runs)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
