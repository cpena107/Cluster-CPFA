"""
Script to compare CPFA, GCFA, and CCPFA algorithms across different resource counts and distributions.
For each analysis directory (e.g. resource_collection_analysis_5_seconds), it looks for subfolders like 
cluster_distribution, powerlaw_distribution, and random_distribution. Within each, it finds resource-count
subfolders (e.g. 8_resources, 16_resources, etc.) and reads the relevant CSV files to extract mean cumulative 
times at milestone_percent == 100. It prints a summary table for each distribution type.
"""


import os
import glob
import re
import shutil
import pandas as pd

def _find_resource_counts(root_dir):
    """Discover all resource-count subfolders like '16_resources' and return sorted ints."""
    counts = []
    try:
        for name in os.listdir(root_dir):
            m = re.match(r"^(\d+)_resources$", name)
            if m:
                if int(m.group(1)) not in [8, 24, 40]:  # Exclude these counts
                    counts.append(int(m.group(1)))
    except FileNotFoundError:
        return []
    return sorted(counts)

def _pick_csv(files):
    """Prefer a '*details*.csv' file; otherwise return the first CSV found."""
    if not files:
        return None
    for f in files:
        if "details" in os.path.basename(f).lower():
            return f
    return files[0]

def calculate_means(root_dir):
    print(f"Analyzing {root_dir}")
    types = {'baseline': 'CPFA', 'algorithm': 'GCFA', 'new': 'CCPFA'}

    resources = _find_resource_counts(root_dir)
    if not resources:
        print("  (no resource folders found)\n" + "-" * 30)
        return

    results = []

    for res in resources:
        res_dir = os.path.join(root_dir, f"{res}_resources")
        if not os.path.isdir(res_dir):
            continue

        row = {'Resources': res}
        for type_dir, label in types.items():
            path = os.path.join(res_dir, type_dir)
            if not os.path.isdir(path):
                continue
            files = glob.glob(os.path.join(path, "*.csv"))
            target = _pick_csv(files)
            if not target:
                continue

            try:
                df = pd.read_csv(target)
                # Prefer milestone-filtered cumulative_time when available
                if 'milestone_percent' in df.columns and 'cumulative_time' in df.columns:
                    completed = df[df['milestone_percent'] == 100]
                    if not completed.empty:
                        row[label] = completed['cumulative_time'].mean()
                elif 'FinalTime' in df.columns:
                    row[label] = df['FinalTime'].mean()
            except Exception:
                # Skip unreadable/malformed files
                pass
        results.append(row)

    df_res = pd.DataFrame(results)
    print(df_res)
    print("-" * 30)

if __name__ == "__main__":
    experiments_path = "experiments"
    source_folder = "resource_collection_analysis"
    
    # Check if experiments folder exists
    if not os.path.exists(experiments_path):
        print(f"Error: {experiments_path} not found.")
        exit(1)
        
    # Get all subdirectories in experiments/
    exp_folders = [f for f in os.listdir(experiments_path) if os.path.isdir(os.path.join(experiments_path, f))]
    exp_folders.sort() # sort for consistency

    for folder_name in exp_folders:
        new_folder_name = os.path.join("resource_collection_all", f"{source_folder}_{folder_name}")
        
        # Copy folder
        if not os.path.exists(new_folder_name):
            print(f"Copying {source_folder} to {new_folder_name}")
            shutil.copytree(source_folder, new_folder_name)
        else:
            print(f"Folder {new_folder_name} already exists.")
            
        calculate_means(os.path.join(new_folder_name, "cluster_distribution"))
        calculate_means(os.path.join(new_folder_name, "random_distribution"))
        calculate_means(os.path.join(new_folder_name, "powerlaw_distribution"))