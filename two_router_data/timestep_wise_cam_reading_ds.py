import os
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict, Counter

# Set path to your root directory
# root_dir = 'run_experiment_data/ir0.7/run_1/'
# root_dir = 'run_experiment_data/ir1.0/run_23/'
# root_dir = 'run_experiment_data/ir_4_uninf/run_6/'
# root_dir = 'run_13/data/'

# "run18", "run19", "run20", "run21", "run23" for ir=1.0
# "run8", "run9", "run10", "run11","run12" for ir=0.8

run = "run23"
# root_dir = f"truly_successful_runs/ir0.8/{run}/data"
root_dir = f"truly_successful_runs/{run}/data"

right_option = 'W'
tot_cnt=0
right_cnt=0
right_cnt_sensor=0
# ----- after * introduction -----
before_right_cnt = 0
before_right_cnt_sensor = 0
after_right_cnt = 0
after_right_cnt_sensor = 0
tot_cnt_before = 0
tot_cnt_after = 0
tot_files = 0
files_skipped = 0

smallest_global_frame = -1
largest_global_frame = -1
# prev_global_frame_no=-1
# Loop through folders
for folder in os.listdir(root_dir):
    folder_path = os.path.join(root_dir, folder)
    if not os.path.isdir(folder_path):
        continue
    for file in os.listdir(folder_path):
        if file.endswith(".csv"):
            file_path = os.path.join(folder_path, file)
            # print(f"reading file at path: {file_path}")
            df = pd.read_csv(file_path)
            # print(f"file size: {len(df)}")
            # tot_files += 1
            if len(df) < 180:
                # files_skipped += 1
                continue
            right_option = 'W'
            for _, row in df.iterrows():
                if row['index'] == '*':
                    continue
                if smallest_global_frame == -1 and int(row['global_frame_number']) != 0:
                    smallest_global_frame = int(row['global_frame_number'])
                if int(row['global_frame_number']) < smallest_global_frame and int(row['global_frame_number']) != 0:
                    smallest_global_frame = int(row['global_frame_number'])
                if largest_global_frame == -1 and int(row['global_frame_number']) != 0:
                    largest_global_frame = int(row['global_frame_number'])
                if int(row['global_frame_number']) > largest_global_frame and int(row['global_frame_number']) != 0:
                    largest_global_frame = int(row['global_frame_number'])
print(f"Smallest frame no: {smallest_global_frame}")
print(f"largest frame no: {largest_global_frame}")
run_data = defaultdict(Counter)
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
            for timestep in range(smallest_global_frame,largest_global_frame,150):
                needed_timestep = -1
                message_at_timestep = 'X'
                for _, row in df.iterrows():
                    if row['index'] == '*':
                        continue
                    if int(row['global_frame_number']) < timestep:
                        needed_timestep = int(row['global_frame_number'])
                        message_at_timestep =  row['cam_wind_direction']
                        continue
                    break
                
                run_data[timestep][message_at_timestep] += 1
print(f"files skipped: {files_skipped}/{tot_files}")
print(run_data)

# df = pd.DataFrame.from_dict({k: dict(v) for k, v in run_data.items()}, orient="index").fillna(0).astype(int)
# df.index.name = "counter"
# df.to_csv(f"{root_dir}/timespet_ds_{run}.csv")

df = pd.DataFrame.from_dict(
    {k: dict(v) for k, v in run_data.items()}, 
    orient="index"
).fillna(0).astype(int)

# when you want the actual timestep rather than the counter like 1,2....N, comm out these lines.
df.reset_index(drop=True, inplace=True)
df.insert(0, "counter", range(1, len(df) + 1))

# Save to CSV
df.to_csv(f"{root_dir}/timespet_cam_reading_{run}.csv", index=False)

