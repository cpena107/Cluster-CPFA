#!/bin/bash

# Script to run resource_analysis.py in all distribution folders across all analysis directories

# Get the current directory where the script is running
BASE_DIR=$(pwd)

# Find all directories matching resource_collection_arenas/*
# We use sort to process them in a predictable order
for analysis_dir in $(ls -d resource_collection_arenas/* | sort); do
    if [ -d "$analysis_dir" ]; then
        echo "=================================================="
        echo "Processing Analysis Directory: $analysis_dir"
        echo "=================================================="
        
        target_dir="$analysis_dir/arenas"
        
        if [ -d "$target_dir" ]; then
            script_path="$target_dir/plot_arenas_boxplot.py"
            
            if [ -f "$script_path" ]; then
                echo "  -> Running analysis in: $target_dir"
                
                # Navigate to the directory to ensure relative paths work correctly
                cd "$target_dir" || continue
                
                # Run the python script
                if python3 plot_arenas_boxplot.py; then
                    # create a directory in the base dir to (if it doesn't exist) to store the resulting .png, .pdf, .svg files
                    mkdir -p "$BASE_DIR/analysis_arena_plots_${analysis_dir#resource_collection_analysis_}"
                    # Move the generated plot files to the base directory under analysis_plots_#_sites
                    mv *resource_analysis_plot.* "$BASE_DIR/analysis_arena_plots_${analysis_dir#resource_collection_analysis_}/"
                    echo "     [SUCCESS]"
                else
                    echo "     [FAILURE] Script returned non-zero exit code"
                fi
                
                # Return to base directory
                cd "$BASE_DIR" || exit 1
            else
                echo "  -> [SKIP] plot_arenas_boxplot.py not found in $target_dir"
            fi
        else
            echo "  -> [SKIP] Directory $target_dir does not exist"
        fi
    fi
done

echo "All analyses complete."
