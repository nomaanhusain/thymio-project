import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict

# Set path to your root directory
root_dir = 'ir1.0/run_8_piw_0.6/'

# Nested dict: counts[index][message] = count
counts = defaultdict(lambda: defaultdict(int))

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
                msg = row['message']
                counts[idx][msg] += 1

# Convert to a DataFrame
directions = ['N', 'S', 'E', 'W']
plot_data = {d: [] for d in directions}
indices = sorted(counts.keys())

for idx in indices:
    for d in directions:
        plot_data[d].append(counts[idx].get(d, 0))

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