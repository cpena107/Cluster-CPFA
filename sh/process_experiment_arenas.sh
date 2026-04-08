#!/bin/bash

# Script to process experiment results, grouped by configuration and separated by arena size
# Usage: ./process_experiment_arenas.sh

EXPERIMENTS_DIR="experiments"

if [ ! -d "$EXPERIMENTS_DIR" ]; then
    echo "Error: Directory '$EXPERIMENTS_DIR' not found."
    exit 1
fi

echo "Processing results from $EXPERIMENTS_DIR..."

process_files() {
    local input_dir=$1
    local output_dir=$2
    local suffix_pattern=$3
    local dist_code=$4
    
    # Process files matching the distribution pattern
    count=0
    for filepath in "$input_dir"/Cluster_CPFA_*_"$suffix_pattern"; do
        if [ ! -f "$filepath" ]; then
            continue
        fi
        
        filename=$(basename "$filepath")
        
        # Extract resource count
        if [[ "$filename" =~ ([0-9]+)res ]]; then
            res_count="${BASH_REMATCH[1]}"
            
            # Create a specific folder for resources inside the arena folder
            # Structure: .../arenas/10x10/16_resources/
            target_file_dir="${output_dir}/${res_count}_resources"
            mkdir -p "$target_file_dir"
            
            output_file="${target_file_dir}/resource_collection_analysis_details.csv"
            summary_file="${target_file_dir}/resource_collection_analysis_summary.csv"

            # if the summary file already exists, skip processing
            if [ -f "$summary_file" ]; then
                echo "Skipping $filename; output already exists."
                continue
            fi
            
            echo "Converting $filename..."
            echo "  -> Target: $(basename "$output_dir") ($res_count resources)"

            header=$(head -n 1 "$filepath" | tr -d '\r')

            if echo "$header" | grep -qE '^random_seed,milestone_percent,time_interval,cumulative_time,food_distribution,algorithm_mode,num_robots,total_food$'; then
                cp "$filepath" "$output_file"
                echo "  -> Input already in details format; copied directly."
            else
                python3 scripts/convert_results_format.py \
                    --input_file "$filepath" \
                    --output_file "$output_file" \
                    --food_distribution "$dist_code"

                if [ $? -ne 0 ]; then
                    echo "  -> Conversion failed for $filename; skipping summary."
                    continue
                fi
            fi

            python3 scripts/generate_summary_from_details.py \
                --input_file "$output_file" \
                --output_file "$summary_file"
            
            count=$((count + 1))
        fi
    done
}

# Iterate through all experiment folders
for exp_folder in "$EXPERIMENTS_DIR"/*/; do
    if [ ! -d "$exp_folder" ]; then
        continue
    fi

    # Strip trailing slash
    exp_folder="${exp_folder%/}"
    folder_name=$(basename "$exp_folder")

    # Extract configuration and arena size using regex
    # Expected format: config_name_arena_10_10
    if [[ "$folder_name" =~ (.*)_arena_([0-9]+)_([0-9]+)$ ]]; then
        config_name="${BASH_REMATCH[1]}"
        width="${BASH_REMATCH[2]}"
        height="${BASH_REMATCH[3]}"
        
        # Determine the unified analysis folder name
        analysis_folder="resource_collection_analysis_${config_name}"
        
        # Determine the arena subfolder (e.g., 10x10)
        arena_subfolder="${width}x${height}"
        
        # Construct the output path: analysis_folder/arenas/10x10/DISTRIBUTION/
        base_arena_output="$analysis_folder/arenas/$arena_subfolder"
        
        echo "Processing $folder_name -> $base_arena_output"

        # 1. Random Distribution
        process_files "$exp_folder" "$base_arena_output/random_distribution" "random.csv" 0

        # 2. Powerlaw (Semi-Clustered) Distribution
        process_files "$exp_folder" "$base_arena_output/powerlaw_distribution" "semi_cluster.csv" 1

        # 3. Clustered Distribution
        process_files "$exp_folder" "$base_arena_output/cluster_distribution" "clustered.csv" 2

    else
        echo "Skipping folder with unexpected name format: $folder_name"
    fi
done

echo "Processing complete."
