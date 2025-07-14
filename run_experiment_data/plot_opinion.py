import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict

# Set path to your root directory
# root_dir = 'run_experiment_data/ir0.7/run_1/'
# root_dir = 'run_experiment_data/ir1.0/run_23/'
# root_dir = 'run_experiment_data/ir_4_uninf/run_6/'
root_dir = 'ir_4_uninf/run_6/'

# Nested dict: counts[index][message] = count
counts = defaultdict(lambda: defaultdict(int))
right_option = 'W'
tot_cnt=0
right_cnt=0
# Loop through folders
for folder in os.listdir(root_dir):
    folder_path = os.path.join(root_dir, folder)
    if not os.path.isdir(folder_path):
        continue
    for file in os.listdir(folder_path):
        if file.endswith(".csv"):
            file_path = os.path.join(folder_path, file)
            df = pd.read_csv(file_path)
            for _, row in df.iterrows():
                idx = int(row['index'])
                msg = row['cam_wind_direction']
                # msg = row['message']
                tot_cnt+=1
                if msg == right_option: right_cnt += 1
                counts[idx][msg] += 1

# Convert to a DataFrame
directions = ['N', 'S', 'E', 'W']
plot_data = {d: [] for d in directions}
indices = sorted(counts.keys())

for idx in indices:
    for d in directions:
        plot_data[d].append(counts[idx].get(d, 0))

print(f"Ratio of right opinion over total opinions ({right_cnt}/{tot_cnt}): {right_cnt/tot_cnt}")
# Plotting
plt.figure(figsize=(12, 6))
colors = {'N': 'blue', 'S': 'red', 'E': 'green', 'W': 'orange'}
for d in directions:
    plt.plot(indices, plot_data[d], label=d, color=colors[d])

plt.xlabel("Index")
plt.ylabel("Count")
plt.title("Wind Direction Opinion by Index")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()