import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict, Counter

# Set path to your root directory
# root_dir = 'run_experiment_data/ir0.7/run_1/'
# root_dir = 'run_experiment_data/ir1.0/run_23/'
# root_dir = 'run_experiment_data/ir_4_uninf/run_6/'
# root_dir = 'run_13/data/'
run = "run23"
root_dir = f"truly_successful_runs/{run}/data"

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

smallest_global_frame = -1
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
                    continue
                if smallest_global_frame == -1 and int(row['global_frame_number']) != 0:
                    smallest_global_frame = int(row['global_frame_number'])
                if int(row['global_frame_number']) < smallest_global_frame and int(row['global_frame_number']) != 0:
                    smallest_global_frame = int(row['global_frame_number'])

print(f"Smallest frame no: {smallest_global_frame}")
frame_increments = 750 # 5 decisions respresented in one entry in the final ds, decision node executes every 5 secs, hence 5 decisions * 5 seconds =25 secs, as vicon outputs at 30 Hz, 25 * 30 = 750 frames.
data = defaultdict(Counter)
for folder in os.listdir(root_dir):
    folder_path = os.path.join(root_dir, folder)
    if not os.path.isdir(folder_path):
        continue
    for file in os.listdir(folder_path):
        if file.endswith(".csv"):
            file_path = os.path.join(folder_path, file)
            df = pd.read_csv(file_path)
            if len(df) < 180:
                continue
            env_switch = False
            right_option = 'W'
            local_frame_number = smallest_global_frame + frame_increments
            bin_idx = 0
            for _, row in df.iterrows():
                if row['index'] == '*':
                    env_switch = True
                    right_option = 'E'
                    continue
                if int(row['global_frame_number']) >= local_frame_number:
                    local_frame_number += frame_increments
                    bin_idx += 1
                    n_idx=0
                    continue
                msg_sensor = row['cam_wind_direction']
                msg = row['message']
                # added
                data[bin_idx][msg] += 1
            
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
print("----------------")
print(data)
df = pd.DataFrame.from_dict({k: dict(v) for k, v in data.items()}, orient="index").fillna(0).astype(int)
df.index.name = "counter"
df.to_csv(f"truly_successful_runs/{run}/data/results_{run}.csv")