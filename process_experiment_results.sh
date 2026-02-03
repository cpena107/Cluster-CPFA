#!/bin/bash

# Script to process experiment results and populate resource collection analysis folders
# Usage: ./process_experiment_results.sh <input_folder> <number_of_sites>

INPUT_DIR=$1
sites=$2  # Number of sites is fixed at 14 for this analysis
secs=$3  # Number of seconds for this analysis

if [ -z "$INPUT_DIR" ]; then
    echo "Usage: $0 <input_folder>"
    exit 1
fi

if [ ! -d "$INPUT_DIR" ]; then
    echo "Error: Directory '$INPUT_DIR' not found."
    exit 1
fi

echo "Processing results from $INPUT_DIR..."

# Function to handle file processing
process_files() {
    local suffix_pattern=$1
    local dist_folder=$2
    local dist_code=$3
    
    # Search for files matching the pattern
    # We look for files starting with Cluster_CPFA_ and ending with the distribution suffix
    # Example: Cluster_CPFA_80res_14x14_random.csv
    
    count=0
    for filepath in "$INPUT_DIR"/Cluster_CPFA_*_"$suffix_pattern"; do
        if [ ! -f "$filepath" ]; then
            continue
        fi
        
        filename=$(basename "$filepath")
        
        # Extract resource count using regex
        # Look for digits followed by 'res'
        if [[ "$filename" =~ ([0-9]+)res ]]; then
            res_count="${BASH_REMATCH[1]}"
            
            output_dir="resource_collection_analysis_${sites}_sites_${secs}_seconds/${dist_folder}/${res_count}_resources/new"
            output_file="${output_dir}/resource_collection_analysis_details.csv"
            
            # Create output directory if it doesn't exist
            mkdir -p "$output_dir"
            
            echo "Converting $filename..."
            echo "  -> Target: $dist_folder ($res_count resources)"
            
            python3 scripts/convert_results_format.py \
                --input_file "$filepath" \
                --output_file "$output_file" \
                --food_distribution "$dist_code"
            
            count=$((count + 1))
        else
            echo "Warning: Could not parse resource count from $filename"
        fi
    done
    
    if [ $count -eq 0 ]; then
        echo "No files found for pattern *_${suffix_pattern} in $INPUT_DIR"
    fi
}

# 1. Random Distribution
# Pattern: *_random.csv -> code 0 -> random_distribution
process_files "random.csv" "random_distribution" 0

# 2. Powerlaw (Semi-Clustered) Distribution
# Pattern: *_semi_cluster.csv -> code 1 -> powerlaw_distribution
process_files "semi_cluster.csv" "powerlaw_distribution" 1

# 3. Clustered Distribution
# Pattern: *_clustered.csv -> code 2 -> cluster_distribution
process_files "clustered.csv" "cluster_distribution" 2

echo "Processing complete."
