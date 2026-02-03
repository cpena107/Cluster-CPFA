import csv
import argparse
import sys
import os

def convert_csv(input_file, output_file, num_robots=16, algorithm_mode=1, food_distribution=0):
    """
    Converts a simulation result CSV to the analysis details format.
    
    Args:
        input_file (str): Path to the source CSV file (Run, FinalTime, ResourcesCollected).
        output_file (str): Path to the destination CSV file.
        num_robots (int): Number of robots used in the experiment.
        algorithm_mode (int): Code for the algorithm mode.
        food_distribution (int): Code for the food distribution.
    """
    try:
        if not os.path.exists(input_file):
            print(f"Error: Input file '{input_file}' not found.")
            return

        with open(input_file, 'r') as infile:
            reader = csv.DictReader(infile)
            
            # Check if required columns exist in input
            # Allow for potential whitespace in headers
            field_map = {name.strip(): name for name in reader.fieldnames} if reader.fieldnames else {}
            
            required_columns = ['Run', 'FinalTime', 'ResourcesCollected']
            missing = [col for col in required_columns if col not in field_map]
            
            if missing:
                print(f"Error: Input file must contain columns: {', '.join(missing)}")
                print(f"Found columns: {list(field_map.keys())}")
                sys.exit(1)

            with open(output_file, 'w', newline='') as outfile:
                fieldnames = [
                    'random_seed', 
                    'milestone_percent', 
                    'time_interval', 
                    'cumulative_time', 
                    'food_distribution', 
                    'algorithm_mode', 
                    'num_robots', 
                    'total_food'
                ]
                writer = csv.DictWriter(outfile, fieldnames=fieldnames)
                writer.writeheader()

                for row in reader:
                    # Parse input values
                    run_id = row[field_map['Run']].strip()
                    try:
                        final_time = float(row[field_map['FinalTime']])
                        resources_collected = float(row[field_map['ResourcesCollected']])
                    except ValueError:
                        print(f"Warning: Skipping row with invalid data: {row}")
                        continue

                    # Create output row
                    # Since we only have the 100% milestone, time_interval equals cumulative_time
                    out_row = {
                        'random_seed': run_id,
                        'milestone_percent': 100,
                        'time_interval': final_time,
                        'cumulative_time': final_time,
                        'food_distribution': food_distribution,
                        'algorithm_mode': algorithm_mode,
                        'num_robots': num_robots,
                        'total_food': int(resources_collected) 
                    }
                    writer.writerow(out_row)
                    
        print(f"Successfully converted '{input_file}' to '{output_file}'")
        
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert simulation results CSV to analysis format.')
    parser.add_argument('--input_file', help='Path to the input CSV file')
    parser.add_argument('--output_file', help='Path to the output CSV file')
    
    # Optional arguments with defaults
    parser.add_argument('--num_robots', type=int, default=16, help='Number of robots (default: 16)')
    parser.add_argument('--algorithm_mode', type=int, default=1, help='Algorithm mode code (default: 1)')
    parser.add_argument('--food_distribution', type=int, default=0, help='Food distribution code (default: 0)')

    args = parser.parse_args()

    convert_csv(args.input_file, args.output_file, args.num_robots, args.algorithm_mode, args.food_distribution)
