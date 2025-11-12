import os, glob
import pandas as pd

def average_runs(base_dir: str, runs: list[str], output_csv: str):
    """
    base_dir: e.g. "truly_successful_runs"
    runs: e.g. ["run18", "run19", "run20"]
    output_csv: e.g. "truly_successful_runs/averaged_results.csv"
    """
    csv_files = []
    for r in runs:
        data_dir = os.path.join(base_dir, r, "data")
        # csv_files.extend(glob.glob(os.path.join(data_dir, "timespet_ds*.csv")))
        csv_files.extend(glob.glob(os.path.join(data_dir, "timespet_cam*.csv")))

    if not csv_files:
        raise FileNotFoundError("No CSVs found in the given run folders.")

    dfs = []
    for f in csv_files:
        print("csv files: ",csv_files)
        df = pd.read_csv(f)

        # Ensure 'counter' column exists (rename first unnamed column if needed)
        if "counter" not in df.columns:
            first_col = df.columns[0]
            if first_col.lower().startswith("unnamed"):
                df = df.rename(columns={first_col: "counter"})
            else:
                raise ValueError(f"'counter' column not found in {f}")

        # Make sure counter is integer-like and numeric columns are numeric
        df["counter"] = pd.to_numeric(df["counter"], errors="coerce").astype("Int64")
        for c in df.columns:
            if c != "counter":
                df[c] = pd.to_numeric(df[c], errors="coerce")

        dfs.append(df)

    combined = pd.concat(dfs, ignore_index=True)

    # Average numeric columns per counter
    avg = (combined
           .groupby("counter", as_index=False)
           .mean(numeric_only=True)
           .sort_values("counter"))

    # Optional: round to, say, 3 decimals (remove if you want full precision)
    avg = avg.round(3)

    avg.to_csv(output_csv, index=False)
    print(f"Averaged CSV saved to: {output_csv}")


base_fol = "truly_successful_runs/"
# base_fol = "truly_successful_runs/ir0.8/"
runs_list = ["run18", "run19", "run20", "run21", "run23"]
# runs_list = ["run8", "run9", "run10", "run11","run12"]
# csv_out = "truly_successful_runs/timewise_avg_data.csv"
csv_out = "truly_successful_runs/timewise_cam_reading_avg_data.csv"
# csv_out = "truly_successful_runs/ir0.8/timewise_avg_data.csv"
average_runs(base_fol, runs_list, csv_out)
