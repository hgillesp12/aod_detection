import pandas as pd
import matplotlib.pyplot as plt

# Load the three CSV files into dataframes
augmented_path = '/Users/hannahgillespie/aod_detection/model/results/augmented_results_training.csv'
combined_path = '/Users/hannahgillespie/aod_detection/model/results/combined_results_training.csv'
real_path = '/Users/hannahgillespie/aod_detection/model/results/real_results_training.csv'

df1 = pd.read_csv(augmented_path)
df2 = pd.read_csv(combined_path)
df3 = pd.read_csv(real_path)

# Remove leading and trailing spaces from column names
df1.columns = df1.columns.str.strip()
df2.columns = df2.columns.str.strip()
df3.columns = df3.columns.str.strip()

# Define the list of columns to plot
columns_to_plot = [
    'metrics/precision', 'metrics/recall', 'metrics/mAP_0.5', 'metrics/mAP_0.5:0.95'
]

# columns_to_plot = [
#     'train/box_loss', 'train/obj_loss', 'train/cls_loss'
# ]

# Plot the data
plt.figure(figsize=(10, 8))

for i, column in enumerate(columns_to_plot, 1):
    plt.subplot(4, 1, i)
    plt.plot(df1['epoch'], df1[column], label='Augmented Results', marker='o')
    plt.plot(df2['epoch'], df2[column], label='Combined Results', marker='x')
    plt.plot(df3['epoch'], df3[column], label='Real Results', marker='s')
    plt.xlabel('Epoch')
    plt.ylabel(column)
    plt.title(column)
    plt.legend()

plt.tight_layout()

plt.savefig('/Users/hannahgillespie/aod_detection/model/results/training_results_metrics.png', dpi=300)

plt.show()
