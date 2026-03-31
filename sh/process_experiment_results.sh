#!/bin/bash

# Script to process experiment results from experiments/ and populate resource collection analysis folders

EXPERIMENTS_DIR="experiments"

if [ ! -d "$EXPERIMENTS_DIR" ]; then
    echo "Error: Directory '$EXPERIMENTS_DIR' not found."
    exit 1
fi

echo "Processing results from $EXPERIMENTS_DIR..."

# Function to handle file processing
process_files() {
    local input_dir=$1
    local output_base_dir=$2
    local suffix_pattern=$3
    local dist_folder=$4
    local dist_code=$5
    
    # Search for files matching the pattern
    # We look for files starting with Cluster_CPFA_ and ending with the distribution suffix
    # Example: Cluster_CPFA_80res_14x14_random.csv
    
    count=0
    for filepath in "$input_dir"/Cluster_CPFA_*_"$suffix_pattern"; do
        if [ ! -f "$filepath" ]; then
            continue
        fi
        
        filename=$(basename "$filepath")
        
        # Extract resource count using regex
        # Look for digits followed by 'res'
        if [[ "$filename" =~ ([0-9]+)res ]]; then
            res_count="${BASH_REMATCH[1]}"
            
            output_dir="${output_base_dir}/${dist_folder}/${res_count}_resources/new"
            output_file="${output_dir}/resource_collection_analysis_details.csv"
            summary_file="${output_dir}/resource_collection_analysis_summary.csv"
            
            # Create output directory if it doesn't exist
            mkdir -p "$output_dir"
            
            echo "Converting $filename..."
            echo "  -> Target: $dist_folder ($res_count resources)"

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
        else
            echo "Warning: Could not parse resource count from $filename"
        fi
    done
    
    if [ $count -eq 0 ]; then
        # echo "No files found for pattern *_${suffix_pattern} in $input_dir"
        :
    fi
}

for exp_folder in "$EXPERIMENTS_DIR"/*/; do
    if [ ! -d "$exp_folder" ]; then
        continue
    fi

    # Strip trailing slash
    exp_folder="${exp_folder%/}"
    folder_name=$(basename "$exp_folder")

    # Construct the corresponding analysis folder name
    # Matching the logic in compare_algos.py: resource_collection_analysis + folder_name
    analysis_folder="resource_collection_all/resource_collection_analysis_${folder_name}"

    echo "--------------------------------------------------"
    echo "Processing Experiment: $folder_name"
    echo "Output Folder: $analysis_folder"

    # 1. Random Distribution
    # Pattern: *_random.csv -> code 0 -> random_distribution
    process_files "$exp_folder" "$analysis_folder" "random.csv" "random_distribution" 0

    # 2. Powerlaw (Semi-Clustered) Distribution
    # Pattern: *_semi_cluster.csv -> code 1 -> powerlaw_distribution
    process_files "$exp_folder" "$analysis_folder" "semi_cluster.csv" "powerlaw_distribution" 1

    # 3. Clustered Distribution
    # Pattern: *_clustered.csv -> code 2 -> cluster_distribution
    process_files "$exp_folder" "$analysis_folder" "clustered.csv" "cluster_distribution" 2

done

echo "Processing complete."
