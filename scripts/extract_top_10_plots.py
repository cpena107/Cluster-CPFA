import csv
import os
import shutil

# Configuration
input_csv = 'top_10_random_48_resources.csv'
source_base_dir = 'resource_collection_arenas'
output_dir = 'top_10_plots'

# Ensure output directory exists
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

def get_folder_name(tolerance, freq, visited, radius):
    # Construct folder name based on the convention:
    # resource_collection_analysis_tol_{}m_freq_{}s_visited_{}_radius_{}m
    return f"resource_collection_analysis_tol_{tolerance}m_freq_{freq}s_visited_{visited}_radius_{radius}m"

def copy_plots():
    current_arena = None
    rank = 1

    with open(input_csv, 'r') as f:
        reader = csv.DictReader(f)
        
        for row in reader:
            arena_size = row['ArenaSize']
            
            # Reset rank if arena size changes
            if arena_size != current_arena:
                current_arena = arena_size
                rank = 1
            
            tolerance = row['VisitedTolerance']
            freq = row['RecordingFreq']
            visited = row['MaxVisited']
            radius = row['MaxRadius']
            
            folder_name = get_folder_name(tolerance, freq, visited, radius)
            source_path = os.path.join(source_base_dir, folder_name, 'arenas', 'arenas_summary.png')
            
            if os.path.exists(source_path):
                dest_filename = f"{arena_size}_{rank}.png"
                dest_path = os.path.join(output_dir, dest_filename)
                
                print(f"Copying {source_path} -> {dest_path}")
                shutil.copy2(source_path, dest_path)
            else:
                print(f"Warning: Source file not found: {source_path}")
            
            rank += 1

if __name__ == "__main__":
    copy_plots()
