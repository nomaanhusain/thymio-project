import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict

# --- Configuration ---
runs = ["run18", "run19", "run20", "run21", "run23"]
# runs_ir08 = ["run8", "run9", "run10", "run11","run12"]
base_dir = Path("data")                   
filename = "results_" # prefix before run name
dirs = ["N", "E", "S", "W"]
treat_missing_as_zero = False

# --- Load CSVs ---
dfs = []
for run in runs:
    # construct path like: run15/data/results_run15.csv
    # path = "truly_successful_runs" / Path("ir0.8") / Path(run) / "data" / f"1500_results_{run}.csv"
    path = "truly_successful_runs" / Path(run) / "data" / f"1500_results_{run}.csv"
    if not path.exists():  
        print(f"Warning: {path} not found, skipping.")
        continue
    
    df = pd.read_csv(path)
    # normalize column naming
    if "counter" not in df.columns:
        df = df.rename(columns={df.columns[0]: "counter"})
    keep = ["counter"] + dirs
    df = df[keep].copy()
    df[dirs] = df[dirs].apply(pd.to_numeric, errors="coerce")
    df = df.set_index("counter").sort_index()
    dfs.append(df)

if not dfs:
    raise RuntimeError("No valid CSV files found.")

# --- Align counters across files ---
if treat_missing_as_zero:
    all_idx = sorted(set().union(*[d.index for d in dfs]))
    dfs = [d.reindex(all_idx).fillna(0) for d in dfs]

# --- Average per counter across runs ---
combined = pd.concat(dfs)
avg = combined.groupby(level=0).mean().sort_index()

# --- Save averaged result ---
# output_path = "truly_successful_runs/ir0.8/1500_averaged_results.csv"
output_path = "truly_successful_runs/1500_averaged_results.csv"
avg.to_csv(output_path, index_label="counter")
print(f"Averaged table saved to {output_path}")

# --- Plot ---
avg.columns = [c[0] if isinstance(c, tuple) else c for c in avg.columns]
plt.figure(figsize=(10,5))
plt.ylim(0,200) # 5 Decisions in 1 counter, 20 robots, max possible is 5*20 = 100 counts favouring one decision
for d in dirs:
    plt.plot(avg.index.values, avg[d].to_numpy().flatten(), label=d)
plt.xlabel("counter")
plt.ylabel("average count")
plt.title("Average counts per direction (N/E/S/W)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
