#!/bin/bash

# Script to extract the 48_resources data and place it into a 'new' subfolder within each arena folder
# Usage: ./extract_48_resources.sh

# The script iterates over the "resource_collection_analysis_tol_*" folders created by process_experiment_arenas.sh

for analysis_dir in resource_collection_arenas/resource_collection_analysis_tol_*; do
    if [ ! -d "$analysis_dir" ]; then
        continue
    fi
    
    echo "Processing $analysis_dir..."
    
    # Iterate through arenas (e.g., 8x8, 10x10, etc.)
    for arena_dir in "$analysis_dir"/arenas/*x*; do
        if [ ! -d "$arena_dir" ]; then
            continue
        fi
        
        arena_size=$(basename "$arena_dir")
        # echo "  Arena: $arena_size"
        
        # We need to find the CSVs in the distribution subfolders (random, cluster, powerlaw)
        # Specifically for 48_resources.
        
        # Target directory: resource_collection_analysis_tol_.../arenas/NxN/new/
        target_dir="$arena_dir/new"
        cp -r resource_collection_analysis/* "$analysis_dir/."
        
        # We only want results from random_distribution
        dist_dir="$arena_dir/random_distribution"
        
        if [ ! -d "$dist_dir" ]; then
            continue
        fi
        
        # Look for the 48_resources folder inside
        source_res_dir="$dist_dir/48_resources"
        
        if [ -d "$source_res_dir" ]; then
            # Find the CSV file
            csv_file="$source_res_dir/resource_collection_analysis_details.csv"
            
            if [ -f "$csv_file" ]; then
                echo "    Copying 48_resources from random_distribution to $target_dir"
                cp "$csv_file" "$target_dir/resource_collection_analysis_details.csv"
            fi
        else
            # echo "    No 48_resources in random_distribution"
            :
        fi
    done
done

echo "Extraction complete."
