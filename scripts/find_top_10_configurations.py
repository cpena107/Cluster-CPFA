import pandas as pd
import os

def find_top_10_configurations():
    input_file = "all_experiment_results_summary.csv"
    output_file = "top_10_experiment_configurations.csv"
    
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
    possible_stat_cols = ['NumTests', 'Mean', 'Std', 'Lowest', '2%', '25%', '50%', '75%', 'Max']
    stat_cols = [c for c in possible_stat_cols if c in df.columns]
    
    # Filter columns to keep, if they exist
    cols_to_keep = group_cols + config_cols + stat_cols
    df_filtered = df[[c for c in cols_to_keep if c in df.columns]]

    # Group by the problem scenario
    grouped = df_filtered.groupby(group_cols)
    
    print(f"Processing {len(grouped)} scenarios...")
    
    top_rows = []
    
    for name, group in grouped:
        # Sort by Mean ascending (lowest time is best)
        # Take the top 10
        top_10_group = group.sort_values(by='Mean', ascending=True).head(10)
        top_rows.append(top_10_group)

    if top_rows:
        result_df = pd.concat(top_rows)
        
        result_df.to_csv(output_file, index=False)
        print(f"Successfully wrote {len(result_df)} rows to {output_file}.")
        print("Note: Top 10 configurations per scenario, sorted by Lowest Mean time.")
    else:
        print("No data processed.")

if __name__ == "__main__":
    find_top_10_configurations()
