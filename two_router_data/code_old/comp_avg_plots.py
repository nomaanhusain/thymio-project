import pickle
import matplotlib.pyplot as plt

# ====== Settings ======
file1 = 'run_experiment_data/avg_opinion_ir1.0.pkl'
file2 = 'run_experiment_data/avg_opinion_ir_4_uninf.pkl'
direction = 'N'  # the one you want to compare
label1 = 'IR=4'
label2 = 'IR=1'
# ======================

# Load files
with open(file1, 'rb') as f:
    data1 = pickle.load(f)

with open(file2, 'rb') as f:
    data2 = pickle.load(f)

indices1 = data1['indices']
values1 = data1['avg_plot_data'][direction]

indices2 = data2['indices']
values2 = data2['avg_plot_data'][direction]

# Plot
plt.figure(figsize=(10, 5))
plt.plot(indices1, values1, label=label1, linestyle='-')
plt.plot(indices2, values2, label=label2, linestyle='--')

plt.xlabel('Index')
plt.ylabel(f'Average count of {direction}')
plt.title(f'Comparison of Average {direction} Opinion')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
