#!/usr/bin/env python3
"""
Plot normalized direction counts (N/E/S/W) vs counter from a CSV.

CSV format:
counter,N,E,S,W
0,44.75,25.0,16.5,30.5
...

Normalization:
normalized_value = (raw_value / MAX_YLIM) * 100
"""


import pandas as pd
import matplotlib.pyplot as plt

# --- Parameters ---
MAX_YLIM = 200  # raw maximum for normalization
CSV_PATH_1 = "truly_successful_runs/ir0.8/1500_averaged_results.csv"
CSV_PATH_2 = "truly_successful_runs/1500_averaged_results.csv"
LABEL_1 = r"$\rho=1.0$"
LABEL_2 = r"$\rho=0.8$"
PLT_SAVE = "truly_successful_runs/output_trans_plt_new.png"

# --- Direction colors (consistent across both plots) ---
LINE_COLORS = {
    "N": "tab:blue",
    "E": "tab:orange",
    "S": "tab:green",
    "W": "tab:red",
}

def normalize_and_plot_two(csv1, csv2, max_ylim, label1, label2):
    # Read both CSVs
    df1 = pd.read_csv(csv1)
    df2 = pd.read_csv(csv2)

    # Normalize both
    for df in [df1, df2]:
        for col in ["N", "E", "S", "W"]:
            df[col] = (df[col] / max_ylim) * 100.0

    # Plot
    plt.figure(figsize=(10, 5))
    
    # solid line for csv1
    for col in ["N", "E", "S", "W"]:
        plt.plot(df1["counter"].to_numpy(), df1[col].to_numpy(),
                label=f"{label1} - {col}", color=LINE_COLORS[col], linestyle='-')

    for col in ["N", "E", "S", "W"]:
        plt.plot(df2["counter"].to_numpy(), df2[col].to_numpy(),
                label=f"{label2} - {col}", color=LINE_COLORS[col], linestyle='--')
    # solid line for csv1
    # for col in ["E", "W"]:
    #     plt.plot(df1["counter"].to_numpy(), df1[col].to_numpy(),
    #             label=f"{label1} - {col}", color=LINE_COLORS[col], linestyle='-')

    # for col in ["E", "W"]:
    #     plt.plot(df2["counter"].to_numpy(), df2[col].to_numpy(),
    #             label=f"{label2} - {col}", color=LINE_COLORS[col], linestyle='--')

    plt.ylim(0, 100)
    plt.xlabel("counter")
    plt.ylabel(f"Swarm Performance [raw scaled by {max_ylim}]")
    plt.title("Comparison of Directional Counts")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(PLT_SAVE, dpi=300, transparent=True)
    plt.show()


# --- Run the function ---
normalize_and_plot_two(CSV_PATH_1, CSV_PATH_2, MAX_YLIM, LABEL_1, LABEL_2)




# from pathlib import Path
# import pandas as pd
# import matplotlib.pyplot as plt

# # --- Default normalization ceiling (can be edited here) ---
# MAX_YLIM = 200  # change this if your raw max changes

# # --- Default colors for lines (distinct per direction) ---
# LINE_COLORS = {
#     "N": "tab:blue",
#     "E": "tab:orange",
#     "S": "tab:green",
#     "W": "tab:red",
# }

# def plot_normalized(csv_path: Path, max_ylim: float, save_path: Path | None):
#     # Read CSV
#     df = pd.read_csv(csv_path)

#     # Basic validation
#     required_cols = {"counter", "N", "E", "S", "W"}
#     missing = required_cols.difference(df.columns)
#     if missing:
#         raise ValueError(f"CSV is missing required columns: {sorted(missing)}")

#     # Normalize N/E/S/W to [0, 100] based on max_ylim
#     norm_df = df.copy()
#     for col in ["N", "E", "S", "W"]:
#         norm_df[col] = (norm_df[col] / max_ylim) * 100.0

#     # Plot
#     plt.figure(figsize=(10, 5))
#     x = norm_df["counter"].to_numpy()

#     for col in ["N", "E", "S", "W"]:
#         plt.plot(x, norm_df[col].to_numpy(), label=col, linewidth=2, color=LINE_COLORS[col])

#     plt.ylim(0, 100)
#     plt.xlim(x.min(), x.max() if len(x) else 1)
#     plt.xlabel("counter")
#     plt.ylabel(f"normalized count (0–100)  [raw scaled by {max_ylim}]")
#     plt.title("Normalized counts per direction (N/E/S/W)")
#     plt.grid(True, alpha=0.3)
#     plt.legend()
#     plt.tight_layout()

#     if save_path:
#         plt.savefig(save_path, dpi=300, transparent=True)
#         print(f"Saved plot to {save_path}")
#     else:
#         plt.show()

# def main():
    
#     csv = "truly_successful_runs/ir0.8/1500_averaged_results.csv"
#     plt_savepath = "truly_successful_runs/ir0.8/ir0.8_plot.png"
#     plot_normalized(csv, MAX_YLIM, plt_savepath)

# if __name__ == "__main__":
#     main()
