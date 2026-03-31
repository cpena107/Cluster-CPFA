import os
import subprocess
import pandas as pd
from concurrent.futures import ThreadPoolExecutor

# Configuration
NUM_WORKERS = 3  # Based on 48 cores / 15 tasks per experiment
CSV_FILE = "experiment_configurations.csv"

def get_experiments_from_csv():
    if not os.path.exists(CSV_FILE):
        print(f"Error: {CSV_FILE} not found in current directory.")
        return []

    try:
        df = pd.read_csv(CSV_FILE)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return []

    required_cols = ['VisitedTolerance', 'RecordingFreq', 'MaxVisited', 'MaxRadius', 'ArenaSize', 'NumTests']
    for col in required_cols:
        if col not in df.columns:
            print(f"Error: Column {col} missing in CSV.")
            return []

    # Group by configuration parameters and take the maximum NumTests
    # We ignore ResourceCount and Distribution as they are handled by the shell script (runs all resources/distributions)
    grouped = df.groupby(['VisitedTolerance', 'RecordingFreq', 'MaxVisited', 'MaxRadius', 'ArenaSize'])['NumTests'].max().reset_index()

    tasks = []
    # Locate generation script once
    gen_script_path = "scripts/generate_experiments.py"
    if not os.path.exists(gen_script_path):
        if os.path.exists("generate_experiments.py"):
            gen_script_path = "generate_experiments.py"
        else:
             print(f"Warning: generate_experiments.py not found at {gen_script_path}")

    # Locate execution script once
    base_script_path = "sh/run_all_clustermap_experiments.sh"
    if not os.path.exists(base_script_path):
            if os.path.exists("run_all_clustermap_experiments.sh"):
                base_script_path = "./run_all_clustermap_experiments.sh"
            elif os.path.exists("scripts/run_all_clustermap_experiments.sh"):
                base_script_path = "scripts/run_all_clustermap_experiments.sh"
            else:
                print(f"Warning: execute script not found at {base_script_path}")

    for _, row in grouped.iterrows():
        visited_tolerance = row['VisitedTolerance']
        recording_freq = row['RecordingFreq']
        max_visited = row['MaxVisited']
        max_radius = row['MaxRadius']
        arena_size = row['ArenaSize']
        num_runs = int(row['NumTests'])

        # Parse arena size (e.g., "10x10" -> 10)
        try:
            arena_x = int(arena_size.split('x')[0])
            arena_y = int(arena_size.split('x')[1])
        except (ValueError, IndexError):
            print(f"Skipping invalid ArenaSize: {arena_size}")
            continue

        # Directory name logic matching the shell script expectation
        # Note: The shell script doesn't enforce directory names, but we need unique dirs
        dir_name = f"tol_{visited_tolerance}m_freq_{recording_freq}s_visited_{max_visited}_radius_{max_radius}m_arena_{arena_x}_{arena_y}"
        path = os.path.join("experiments", dir_name)

        # Check if directory exists
        if os.path.isdir(path):
            print(f"Skipping existing directory: {path}")
            continue

        # Shell script arguments
        shell_args = [
            base_script_path,
            str(num_runs),
            path,
            str(visited_tolerance),
            str(recording_freq),
            str(max_visited),
            str(max_radius),
            str(arena_x),
            str(arena_y)
        ]

        # Generation script arguments
        gen_args = [
            "python3",
            gen_script_path,
            "--visited-tolerance", str(visited_tolerance),
            "--recording-freq", str(recording_freq),
            "--max-visited", str(max_visited),
            "--max-radius", str(max_radius),
            "--arena-x", str(arena_x),
            "--arena-y", str(arena_y),
            "--output-dir", path
        ]

        tasks.append((path, shell_args, gen_args))

    return tasks

def run_task(task):
    path, shell_args, gen_args = task
    print(f"STARTING: {path}")
    try:
        # Create directory first
        os.makedirs(path, exist_ok=True)
        
        # Log 
        log_file = os.path.join(path, "experiment.log")
        with open(log_file, "w") as f:
            # Generate XML files
            f.write("Generating experiments...\n")
            f.flush()
            subprocess.run(gen_args, stdout=f, stderr=subprocess.STDOUT, check=True)

            # Run simulations
            f.write("\nRunning simulations...\n")
            f.flush()
            subprocess.run(shell_args, stdout=f, stderr=subprocess.STDOUT, check=True)
            
        print(f"COMPLETED: {path}")
    except subprocess.CalledProcessError as e:
        print(f"FAILED: {path} with error {e}")
    except Exception as e:
        print(f"ERROR: {path}: {e}")

def main():
    print(f"Reading configurations from {CSV_FILE}...")
    tasks = get_experiments_from_csv()
    
    if not tasks:
        print("No new experiments to run (or CSV/Script error).")
        return

    print(f"Found {len(tasks)} missing experiments. Running {NUM_WORKERS} in parallel...")
    
    with ThreadPoolExecutor(max_workers=NUM_WORKERS) as executor:
        executor.map(run_task, tasks)

if __name__ == "__main__":
    main()
