
import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict
import glob

# Define base path and parameters
base_path = "run_experiment_data/5_run_test/"  # <-- CHANGE THIS to the actual path
run_folders = [f"run_{i}" for i in range(22, 27)]
robot_ids = [f"3_{i}" for i in range(68, 87)]
correct_direction = "N"

print(run_folders)
print(robot_ids)

# Initialize data structures
observation_ratios = defaultdict(list)
observation_counts = defaultdict(list)

# Iterate through runs and robots
for run in run_folders:
    run_path = os.path.join(base_path, run)
    if not os.path.isdir(run_path):
        print(f"Run folder missing: {run_path}")
        continue
    for robot_id in robot_ids:
        robot_path = os.path.join(run_path, robot_id)
        if not os.path.isdir(robot_path):
            continue
        csv_files = glob.glob(os.path.join(robot_path, "log_*.csv"))
        if not csv_files:
            continue
        # Assuming only one log_*.csv file per robot
        csv_file = csv_files[0]
        try:
            df = pd.read_csv(csv_file)
            if "cam_wind_direction" in df.columns:
                total = len(df)
                correct = (df["cam_wind_direction"] == correct_direction).sum()
                ratio = correct / total if total > 0 else 0
                observation_ratios[robot_id].append(ratio)
                observation_counts[robot_id].append(total)
        except Exception as e:
            print(f"Error reading {csv_file}: {e}")
            continue

# Create plots
fig, axs = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

# Plot ratio of correct observations with annotations
for robot_id, ratios in observation_ratios.items():
    x_positions = [robot_id] * len(ratios)
    axs[0].scatter(x_positions, ratios)

    # Annotate min and max values
    if ratios:
        min_val = min(ratios)
        max_val = max(ratios)
        axs[0].text(robot_id, min_val, f"{min_val:.2f}", ha='right', va='bottom', fontsize=8, color='blue')
        axs[0].text(robot_id, max_val, f"{max_val:.2f}", ha='right', va='top', fontsize=8, color='blue')

axs[0].set_ylabel("Correct Observations Ratio")
axs[0].set_title("Correct Wind Direction Observation Ratio per Robot (over 5 Runs)")
axs[0].grid(True)

# Plot average observation count
average_counts = {robot_id: sum(counts)/len(counts) for robot_id, counts in observation_counts.items()}
axs[1].bar(average_counts.keys(), average_counts.values())
axs[1].set_ylabel("Average Observation Count")
axs[1].set_title("Average Number of Observations per Robot")
axs[1].set_xlabel("Robot ID")
axs[1].grid(True)

plt.xticks(rotation=45)
plt.tight_layout()
plt.show()

# # Plot ratio of correct observations
# for robot_id, ratios in observation_ratios.items():
#     axs[0].scatter([robot_id] * len(ratios), ratios, label=robot_id)
# axs[0].set_ylabel("Correct Observations Ratio")
# axs[0].set_title("Correct Wind Direction Observation Ratio per Robot (over 5 Runs)")
# axs[0].grid(True)

# # Plot average observation count
# average_counts = {robot_id: sum(counts)/len(counts) for robot_id, counts in observation_counts.items()}
# axs[1].bar(average_counts.keys(), average_counts.values())
# axs[1].set_ylabel("Average Observation Count")
# axs[1].set_title("Average Number of Observations per Robot")
# axs[1].set_xlabel("Robot ID")
# axs[1].grid(True)

# plt.xticks(rotation=45)
# plt.tight_layout()
# plt.show()
