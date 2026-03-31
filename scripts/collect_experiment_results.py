import os
import re
import glob
import pandas as pd

def collect_results():
    experiments_dir = "experiments"
    output_file = "all_experiment_results_summary.csv"
    id = 0
    
    # Pattern for directory
    dir_pattern = re.compile(r"tol_([\d.]+)m_freq_(\d+)s_visited_(\d+)_radius_([\d.]+)m_arena_(\d+)_(\d+)")
    
    # Pattern for filename
    file_pattern = re.compile(r"Cluster_CPFA_(\d+)res_(random|clustered|semi_cluster)\.csv")
    
    summary_data = []
    
    print(f"Scanning {experiments_dir}...")
    
    # Process all directories matching the pattern
    # We iterate only directories first
    for root, dirs, files in os.walk(experiments_dir):
        dirname = os.path.basename(root)
        dir_match = dir_pattern.match(dirname)
        
        if dir_match:
            # Extract configuration
            visited_tolerance = float(dir_match.group(1))
            recording_freq = int(dir_match.group(2))
            max_visited = int(dir_match.group(3))
            max_radius = float(dir_match.group(4))
            arena_x = int(dir_match.group(5))
            arena_y = int(dir_match.group(6))
            arena_size = f"{arena_x}x{arena_y}"
            
            # Process CSV files in this directory
            for filename in files:
                file_match = file_pattern.match(filename)
                if file_match:
                    resource_count = int(file_match.group(1))
                    distribution = file_match.group(2)
                    if distribution == "cluster":
                        distribution = "powerlaw"
                    
                    file_path = os.path.join(root, filename)
                    
                    try:
                        df = pd.read_csv(file_path)
                        
                        # We need to decide which column to summarize.
                        # Based on typical CPFA analysis and existing summary stats, 'FinalTime' is the likely metric.
                        # If 'ResourcesCollected' is constant (target met), 'FinalTime' varies.
                        # If 'ResourcesCollected' varies (target not met), 'ResourcesCollected' is the metric.
                        # However, combined_summary_statistics.csv had 'FinalTime' metrics.
                        # We will compute stats for 'FinalTime'.
                        
                        target_column = 'cumulative_time'
                        
                        if target_column in df.columns:
                            values = df[target_column]
                            
                            stats = values.describe(percentiles=[0.25, 0.5, 0.75])
                            
                            # Row dictionary
                            row = {
                                'random_seed': id,
                                'ResourceCount': resource_count,
                                'Distribution': distribution,
                                'VisitedTolerance': visited_tolerance,
                                'RecordingFreq': recording_freq,
                                'MaxVisited': max_visited,
                                'MaxRadius': max_radius,
                                'ArenaSize': arena_size,
                                'NumTests': int(stats['count']),
                                'Mean': stats['mean'],
                                'Std': stats['std'],
                                'Lowest': stats['min'],
                                '25%': stats['25%'], # Included as standard Q1
                                '50%': stats['50%'],
                                '75%': stats['75%'],
                                'Max': stats['max']
                            }
                            summary_data.append(row)
                            id += 1
                            
                    except Exception as e:
                        print(f"Error reading {file_path}: {e}")

    # Create DataFrame
    if summary_data:
        summary_df = pd.DataFrame(summary_data)
        
        # Reorder columns to match request logic
        cols = [
            'random_seed', 'ResourceCount', 'Distribution', 
            'VisitedTolerance', 'RecordingFreq', 'MaxVisited', 'MaxRadius', 'ArenaSize',
            'NumTests', 'Mean', 'Std', 'Lowest', '25%', '50%', '75%', 'Max'
        ]
        # Only keep existing columns (in case 2% vs 25% logic changes)
        cols = [c for c in cols if c in summary_df.columns]
        
        summary_df = summary_df[cols]
        # Only keep rows with 100 milestone_percent
        summary_df = summary_df[summary_df['milestone_percent'] == 100]
        
        summary_df.to_csv(output_file, index=False)
        print(f"Successfully created {output_file} with {len(summary_df)} rows.")
    else:
        print("No matching data found.")

if __name__ == "__main__":
    collect_results()
