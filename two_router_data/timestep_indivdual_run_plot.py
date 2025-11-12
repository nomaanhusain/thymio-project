import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict, Counter

# "run18", "run19", "run20", "run21", "run23" for ir=1.0
# "run8", "run9", "run10", "run11","run12" for ir=0.8

# run = "run18"
# # root_dir = f"truly_successful_runs/ir0.8/{run}/data"
# root_dir = f"truly_successful_runs/{run}/data/timespet_ds_{run}.csv"

# # Load the CSV
# df = pd.read_csv(root_dir)

# print(df)

# # Define colors for each column
# colors = {
#     'X': 'grey',
#     'S': 'blue',
#     'W': 'orange',
#     'E': 'green',
#     'N': 'red'
# }

# # Plot each column except 'counter'
# plt.figure(figsize=(8, 5))
# for col in ['X', 'S', 'W', 'E', 'N']:
#     plt.plot(df['counter'].to_numpy(), df[col].to_numpy(), label=col, color=colors[col])


# plt.xlabel("Counter")
# plt.ylabel("Value")
# plt.title("Counts per Direction")
# plt.grid(True, alpha=0.3)
# plt.legend()
# plt.tight_layout()
# plt.show()



# --- Multi_subplots ---
# csv_paths = [
#     "truly_successful_runs/run18/data/timespet_ds_run18.csv",
#     "truly_successful_runs/run19/data/timespet_ds_run19.csv",
#     "truly_successful_runs/run20/data/timespet_ds_run20.csv",
#     "truly_successful_runs/run21/data/timespet_ds_run21.csv",
#     "truly_successful_runs/run23/data/timespet_ds_run23.csv",
# ]

csv_paths = [
    "truly_successful_runs/run18/data/timespet_cam_reading_run18.csv",
    "truly_successful_runs/run19/data/timespet_cam_reading_run19.csv",
    "truly_successful_runs/run20/data/timespet_cam_reading_run20.csv",
    "truly_successful_runs/run21/data/timespet_cam_reading_run21.csv",
    "truly_successful_runs/run23/data/timespet_cam_reading_run23.csv",
]

# csv_paths = [
#     "truly_successful_runs/ir0.8/run8/data/timespet_ds_run8.csv",
#     "truly_successful_runs/ir0.8/run9/data/timespet_ds_run9.csv",
#     "truly_successful_runs/ir0.8/run10/data/timespet_ds_run10.csv",
#     "truly_successful_runs/ir0.8/run11/data/timespet_ds_run11.csv",
#     "truly_successful_runs/ir0.8/run12/data/timespet_ds_run12.csv",
# ]

# --- Common colors for all plots ---
colors = {
    'X': 'grey',
    'S': 'blue',
    'W': 'orange',
    'E': 'green',
    'N': 'red'
}

# --- Create subplots ---
fig, axes = plt.subplots(nrows=5, ncols=1, figsize=(8, 12), sharex=True)

correct_op = 'W'
correct_op_cnt=0
tot_cnt = 0
# --- Loop through CSVs and plot each in its own subplot ---
for i, (path, ax) in enumerate(zip(csv_paths, axes)):
    df = pd.read_csv(path)
    for col in ['X', 'S', 'W', 'E', 'N']:
        ax.plot(df['counter'].to_numpy(), df[col].to_numpy(), label=col, color=colors[col])
    ax.set_title(f"Run {i+1}")
    ax.grid(True, alpha=0.3)
    if i == len(csv_paths) - 1:
        ax.set_xlabel("Counter")
    ax.set_ylabel("Value")
    ax.set_ylim(0,20)

# --- Add a single legend for all subplots ---
handles, labels = axes[0].get_legend_handles_labels()
fig.legend(handles, labels, loc='upper right')

plt.tight_layout()
plt.show()
