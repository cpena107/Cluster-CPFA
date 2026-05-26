import pandas as pd
import os

def find_best_configurations():
    input_file = "all_experiment_results_summary.csv"
    output_file = "best_experiment_configurations.csv"
    
    if not os.path.exists(input_file):
        print(f"Error: {input_file} not found.")
        return

    print(f"Reading {input_file}...")
    df = pd.read_csv(input_file)
    
    # We define the "Problem Scenario" by these columns
    group_cols = ['ArenaSize', 'ResourceCount', 'Distribution']
    
    # We define "Algorithm Configuration" by these columns (to keep in the output)
    config_cols = ['VisitedTolerance', 'RecordingFreq', 'MaxVisited', 'MaxRadius']
    
    # Other stats columns to keep
    stat_cols = ['NumTests', 'Mean', 'Std', 'Lowest', '25%', '50%', '75%', 'Max']
    
    # Check if necessary columns exist
    for col in group_cols + ['Mean']:
        if col not in df.columns:
            print(f"Error: Column '{col}' missing from input CSV.")
            return

    best_rows = []
    
    # Group by the problem scenario
    grouped = df.groupby(group_cols)
    
    print(f"Processing {len(grouped)} scenarios...")
    
    for name, group in grouped:
        # Find the row with the minimum Mean (assuming Time, so lower is better)
        # If the metric was 'Resources Collected', we would use idxmax()
        best_idx = group['Mean'].idxmin()
        best_row = group.loc[best_idx]
        best_rows.append(best_row)

    if best_rows:
        best_df = pd.DataFrame(best_rows)
        
        # Organize columns: Scenario -> Best Config -> Stats
        cols_ordered = group_cols + config_cols + stat_cols
        # Filter to only existing columns
        cols_ordered = [c for c in cols_ordered if c in best_df.columns]
        
        best_df = best_df[cols_ordered]
        
        best_df.to_csv(output_file, index=False)
        print(f"Successfully wrote {len(best_df)} best configurations to {output_file}.")
        print("Note: 'Best' is defined as the Lowest Mean value (assuming 'Time' metric).")
    else:
        print("No data processed.")

if __name__ == "__main__":
    find_best_configurations()
