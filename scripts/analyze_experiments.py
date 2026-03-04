import os
import re
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import glob

def parse_directory_name(dirname):
    # Pattern: tol_0.25m_freq_1s_visited_25_radius_0.5m_arena_8_8
    pattern = r"tol_([\d.]+)m_freq_(\d+)s_visited_(\d+)_radius_([\d.]+)m_arena_(\d+)_(\d+)"
    match = re.search(pattern, dirname)
    if match:
        return {
            'VisitedTolerance': float(match.group(1)),
            'RecordingFreq': int(match.group(2)),
            'MaxVisited': int(match.group(3)),
            'MaxRadius': float(match.group(4)),
            'ArenaSize': int(match.group(5))  # Assuming square arena
        }
    return None

def parse_filename(filename):
    # Pattern: Cluster_CPFA_16res_14x14_clustered.csv
    # Note: 14x14 in filename seems hardcoded/ignored in favor of directory info, 
    # but the resource count and distribution are real.
    pattern = r"Cluster_CPFA_(\d+)res_.*_(.+)\.csv"
    match = re.search(pattern, filename)
    if match:
        return {
            'ResourceCount': int(match.group(1)),
            'Distribution': match.group(2)
        }
    return None

def load_data(experiments_dir):
    data = []
    
    # Walk through the directory structure
    for root, dirs, files in os.walk(experiments_dir):
        # Check if this is an experiment directory
        dir_params = parse_directory_name(os.path.basename(root))
        
        if dir_params:
            for file in files:
                if file.endswith(".csv") and "Cluster_CPFA" in file:
                    file_params = parse_filename(file)
                    if file_params:
                        # Read the CSV
                        file_path = os.path.join(root, file)
                        try:
                            df = pd.read_csv(file_path)
                            
                            # Add parameter columns
                            for k, v in dir_params.items():
                                df[k] = v
                            for k, v in file_params.items():
                                df[k] = v
                                
                            data.append(df)
                        except Exception as e:
                            print(f"Error reading {file_path}: {e}")

    if not data:
        return pd.DataFrame()
        
    return pd.concat(data, ignore_index=True)

def visualize_results(df, output_dir):
    if df.empty:
        print("No data found to visualize.")
        return

    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Get unique resource counts
    resource_counts = sorted(df['ResourceCount'].unique())
    
    sns.set_theme(style="whitegrid")

    for res_count in resource_counts:
        print(f"Generating plots for Resource Count: {res_count}")
        
        subset = df[df['ResourceCount'] == res_count]
        
        if subset.empty:
            continue
            
        # Plot 1: Resources Collected vs Radius, faceted by Arena Size and Distribution
        # Hue: VisitedTolerance
        g = sns.relplot(
            data=subset,
            x="MaxRadius", y="ResourcesCollected",
            hue="VisitedTolerance", style="MaxVisited",
            col="ArenaSize", row="Distribution",
            kind="line", marker="o",
            facet_kws={'sharey': False, 'sharex': True},
            height=3, aspect=1.2
        )
        g.fig.suptitle(f'Resources Collected vs Radius (Res: {res_count})', y=1.02)
        g.savefig(os.path.join(output_dir, f'resources_vs_radius_res{res_count}.png'))
        plt.close(g.fig)

        # Plot 2: Final Time vs Radius
        g = sns.relplot(
            data=subset,
            x="MaxRadius", y="FinalTime",
            hue="VisitedTolerance", style="MaxVisited",
            col="ArenaSize", row="Distribution",
            kind="line", marker="o",
            facet_kws={'sharey': False, 'sharex': True},
            height=3, aspect=1.2
        )
        g.fig.suptitle(f'Final Time vs Radius (Res: {res_count})', y=1.02)
        g.savefig(os.path.join(output_dir, f'time_vs_radius_res{res_count}.png'))
        plt.close(g.fig)
        
        # Plot 3: Heatmap of Efficiency (Resources / Time) for specific Arena/Distribution?
        # Maybe too complex for generic loop.
        
    print(f" visualizations saved to {output_dir}")

def main():
    # Use absolute path relative to the script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    experiments_dir = os.path.join(script_dir, "experiments")
    output_dir = os.path.join(script_dir, "analysis_results")
    
    print(f"Looking for experiments in: {experiments_dir}")
    print("Loading data...")
    df = load_data(experiments_dir)
    
    if df.empty:
        print("No matches found. Check directory structure.")
        return

    print(f"Loaded {len(df)} records.")
    
    # Save combined data
    os.makedirs(output_dir, exist_ok=True)
    output_csv = os.path.join(output_dir, "combined_experiment_data.csv")
    df.to_csv(output_csv, index=False)
    print(f"Saved combined data to {output_csv}")
    
    # Visualize
    visualize_results(df, output_dir)

if __name__ == "__main__":
    main()
