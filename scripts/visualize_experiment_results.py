import os
import glob
import re
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def parse_filename(filename):
    """
    Parses the filename to extract the number of sites/resources and the distribution type.
    Expected format examples: 
    - Cluster_CPFA_16res_14x14_clustered.csv
    - Cluster_GPFA_40sites_16res_14x14_clustered.csv
    """
    # Regex to find the number before 'res' and the type at the end
    match = re.search(r'(\d+)res_.*_(random|clustered|semi_cluster)\.csv$', filename)
    if match:
        num_res = int(match.group(1))
        dist_type = match.group(2)
        return num_res, dist_type
    return None, None

def parse_folder_sites(folder_name):
    """
    Extracts the updated sites count from the folder name.
    Expects pattern like: updated_algo_clear_{NUM}_sites
    """
    match = re.search(r'updated_algo_clear_(\d+)_sites', folder_name)
    if match:
        return int(match.group(1))
    return None

def main():
    parser = argparse.ArgumentParser(description="Visualize simulation results from multiple folders")
    parser.add_argument("--pattern", default="updated_algo_clear_*", help="Folder pattern to search for results (default: updated_algo_clear_*)")
    parser.add_argument("--output", "-o", default="productivity_combined_plot.png", help="Output filename for the plot")
    args = parser.parse_args()

    # Find all matching directories
    matching_dirs = glob.glob(args.pattern)
    matched_dirs = [d for d in matching_dirs if os.path.isdir(d)]
    
    if not matched_dirs:
        print(f"No directories found matching pattern: {args.pattern}")
        # Fallback to current dir if regex was empty/default but nothing found? 
        if os.path.isdir(args.pattern):
             matched_dirs = [args.pattern]
        else:
             return

    print(f"Found {len(matched_dirs)} directories: {matched_dirs}")

    data_frames = []

    # Iterate through all matched directories
    for input_dir in matched_dirs:
        # Extract site count from folder name
        folder_sites = parse_folder_sites(input_dir)
        if folder_sites is None:
            print(f"  Warning: Could not extract site count from folder name '{input_dir}'. Using 'Unknown' as label.")
            folder_sites_label = "Unknown"
        else:
            folder_sites_label = f"{folder_sites} Sites"

        csv_files = glob.glob(os.path.join(input_dir, "*.csv"))
        print(f"  Processing {input_dir} (Map Sites: {folder_sites_label}): found {len(csv_files)} CSV files.")
        
        for file_path in csv_files:
            filename = os.path.basename(file_path)
            
            # Skip summary files if they exist
            if filename == "summary_statistics.csv":
                continue

            num_res, dist_type = parse_filename(filename)
            
            if num_res is not None and dist_type is not None:
                try:
                    df = pd.read_csv(file_path)
                    if 'FinalTime' in df.columns:
                        df['NumSites'] = num_res
                        df['Distribution'] = dist_type
                        df['MapSites'] = folder_sites_label # Storing the per-folder site count
                        df['SourceFolder'] = input_dir 
                        data_frames.append(df)
                except Exception as e:
                    print(f"    Error reading {filename}: {e}")

    if not data_frames:
        print("No valid data collected across all folders.")
        return

    all_data = pd.concat(data_frames, ignore_index=True)
    all_data = all_data.sort_values(by='NumSites')

    # Define the distributions and order
    distributions = ['random', 'clustered', 'semi_cluster']
    titles = ['Random Distribution', 'Clustered Distribution', 'Semi-Clustered Distribution']
    
    # Create a figure with 3 subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), sharey=True)
    sns.set_style("whitegrid")
    
    # Get unique MapSites for hue ordering (numerically sorted)
    def get_site_count(label):
        match = re.search(r'(\d+)', str(label))
        return int(match.group(1)) if match else float('inf')

    try:
        unique_map_sites = sorted(all_data['MapSites'].unique(), key=get_site_count)
    except:
        unique_map_sites = sorted(all_data['MapSites'].unique())

    legend_handles = []
    legend_labels = []
        
    for i, dist in enumerate(distributions):
        ax = axes[i]
        subset = all_data[all_data['Distribution'] == dist]
        
        if subset.empty:
            ax.text(0.5, 0.5, 'No Data', horizontalalignment='center', verticalalignment='center', transform=ax.transAxes)
            ax.set_title(titles[i])
            continue

        # Plot Time vs NumSites, with hue based on MapSites (folder source)
        sns.lineplot(
            data=subset, 
            x='NumSites', 
            y='FinalTime',
            hue='MapSites',
            hue_order=unique_map_sites,
            style='MapSites', 
            marker='o',
            linewidth=2.5,
            ax=ax,
            palette="viridis"
        )
        
        ax.set_title(titles[i], fontsize=14, fontweight='bold')
        ax.set_xlabel('Number of Sites', fontsize=12)
        if i == 0:
            ax.set_ylabel('Completion Time (s)', fontsize=12)
        else:
            ax.set_ylabel('')
            
        # Capture handles/labels from the first valid plot we see
        if not legend_handles and ax.get_legend():
            handles, labels = ax.get_legend_handles_labels()
            legend_handles.extend(handles)
            legend_labels.extend(labels)
            
        # Use remove() if get_legend() exists to clear individual plot legends
        if ax.get_legend():
            ax.get_legend().remove()

    plt.suptitle('Simulation Completion Time vs Num Sites (by Distribution)', fontsize=16)
    
    # Add a unified figure legend if we collected handles
    if legend_handles:
        fig.legend(
            legend_handles, 
            legend_labels, 
            title='Total Sites on Map', 
            bbox_to_anchor=(0.99, 0.5), 
            loc='center left', 
            borderaxespad=0.
        )
        
        # Adjust layout to make room for the legend on the right
        plt.tight_layout(rect=[0, 0, 0.88, 0.95])
    else:
        plt.tight_layout()
    
    plt.savefig(args.output, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved to: {args.output}")

    # Save summary stats
    summary_path = "combined_summary_statistics.csv"
    summary = all_data.groupby(['MapSites', 'NumSites', 'Distribution'])['FinalTime'].describe()
    summary.to_csv(summary_path)
    print(f"Summary statistics saved to: {summary_path}")

if __name__ == "__main__":
    main()
