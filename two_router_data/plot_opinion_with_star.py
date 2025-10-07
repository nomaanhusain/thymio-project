import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict

# Set path to your root directory
# root_dir = 'run_experiment_data/ir0.7/run_1/'
# root_dir = 'run_experiment_data/ir1.0/run_23/'
# root_dir = 'run_experiment_data/ir_4_uninf/run_6/'
# root_dir = 'run_13/data/'
root_dir = "truly_successful_runs/run21/data"

# Nested dict: counts[index][message] = count
counts = defaultdict(lambda: defaultdict(int))
right_option = 'W'
tot_cnt=0
right_cnt=0
right_cnt_sensor=0
# ----- after * introduction -----
env_switch = False
before_right_cnt = 0
before_right_cnt_sensor = 0
after_right_cnt = 0
after_right_cnt_sensor = 0
tot_cnt_before = 0
tot_cnt_after = 0
tot_files = 0
files_skipped = 0

# prev_global_frame_no=-1
# Loop through folders
for folder in os.listdir(root_dir):
    folder_path = os.path.join(root_dir, folder)
    if not os.path.isdir(folder_path):
        continue
    for file in os.listdir(folder_path):
        if file.endswith(".csv"):
            file_path = os.path.join(folder_path, file)
            print(f"reading file at path: {file_path}")
            df = pd.read_csv(file_path)
            print(f"file size: {len(df)}")
            tot_files += 1
            if len(df) < 180:
                files_skipped += 1
                continue
            env_switch = False
            right_option = 'W'
            for _, row in df.iterrows():
                if row['index'] == '*':
                    env_switch = True
                    right_option = 'E'
                    continue
                idx = int(row['index'])
                msg_sensor = row['cam_wind_direction']
                msg = row['message']
                # if prev_global_frame_no == -1:
                #     prev_global_frame_no = int(row['global_frame_number'])
                # else:
                #     print(f"idx= {idx} | frame_no_diff={int(row['global_frame_number']) - prev_global_frame_no}")
                #     prev_global_frame_no = int(row['global_frame_number'])
                # tot_cnt+=1
                # if msg == right_option: right_cnt += 1
                # if msg_sensor == right_option: right_cnt_sensor += 1
                counts[idx][msg] += 1
                if not env_switch:
                    tot_cnt_before += 1
                    if msg == right_option: before_right_cnt += 1
                    if msg_sensor == right_option: before_right_cnt_sensor += 1
                else:
                    tot_cnt_after += 1
                    if msg == right_option: after_right_cnt += 1
                    if msg_sensor == right_option: after_right_cnt_sensor += 1

print(f"Files skipped: {files_skipped}/{tot_files}")
print(f"Correct opinion before env. change: ({before_right_cnt}/{tot_cnt_before}): {before_right_cnt/tot_cnt_before}")
print(f"Correct opinion after env. change: ({after_right_cnt}/{tot_cnt_after}): {after_right_cnt/tot_cnt_after}")
# Convert to a DataFrame
directions = ['N', 'S', 'E', 'W']
plot_data = {d: [] for d in directions}
indices = sorted(counts.keys())

for idx in indices:
    for d in directions:
        plot_data[d].append(counts[idx].get(d, 0))

# print(f"Ratio of right observation (Sensor) ({right_cnt_sensor}/{tot_cnt}): {right_cnt_sensor/tot_cnt}")
# print(f"Ratio of right opinion (Opinion) ({right_cnt}/{tot_cnt}): {right_cnt/tot_cnt}")
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