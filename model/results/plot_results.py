import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import auc  # To calculate the area under the curve

# List of CSV file paths and their corresponding labels for the legend
csv_files = ['results_real.csv', 'results_combined.csv', 'results_augmented_combined.csv']
labels = ['Real', 'Real + Synthetic', 'Real + Synthetic + Augmented']

# Initialize a list to store dataframes
dataframes = []

# Read each CSV file and store the dataframe
for file in csv_files:
    df = pd.read_csv(file)
    dataframes.append(df)

# Recall values are assumed to be the same across all models
recall_values = dataframes[0]['Recall'].values  

# Initialize an array to store average precisions and mAP@0.5 for each model
average_precisions = []
map_values = []  # To store mAP@0.5 for each model

for df in dataframes:
    # Calculate mean precision for each recall level (across all classes)
    class_columns = [col for col in df.columns if 'Precision' in col]
    df['Average Precision'] = df[class_columns].mean(axis=1)
    average_precisions.append(df['Average Precision'].values)
    
    # Calculate mAP@0.5 (area under the precision-recall curve)
    map_value = auc(recall_values, df['Average Precision'].values)
    map_values.append(map_value)

# Plot the average precision for each model against recall
plt.figure(figsize=(10, 6))

for i, ap in enumerate(average_precisions):
    plt.plot(recall_values, ap, label=f'{labels[i]} (mAP@0.5: {map_values[i]:.3f})', linewidth=2)

# Set the axis labels and title
plt.xlabel('Recall')
plt.ylabel('Average Precision')
plt.title('Precision-Recall Curve Across All Classes (mAP@0.5)')
plt.grid(True)

# Add the legend to distinguish between different models
plt.legend()

# Save the figure to the specified file path
plt.savefig('/Users/hannahgillespie/aod_detection/model/results/pr_curve.png', dpi=300)

# Show the plot
plt.show()
