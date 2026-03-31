#!/bin/bash

# Script to run resource_analysis.py in all distribution folders across all analysis directories

# Get the current directory where the script is running
BASE_DIR=$(pwd)

# Find all directories matching resource_collection_analysis_*
# We use sort to process them in a predictable order
for analysis_dir in $(ls -d resource_collection_all/resource_collection_analysis* | sort); do
    if [ -d "$analysis_dir" ]; then
        echo "=================================================="
        echo "Processing Analysis Directory: $analysis_dir"
        echo "=================================================="
        # Skip arena size not 14x14
        if [[ "$analysis_dir" != *"14_14"* ]]; then
            echo "  -> [SKIP] Not a 14x14 arena analysis directory"
            continue
        fi
        distributions=("cluster_distribution" "powerlaw_distribution" "random_distribution")
        
        for dist in "${distributions[@]}"; do
            target_dir="$analysis_dir/$dist"
            
            if [ -d "$target_dir" ]; then
                script_path="$target_dir/resource_analysis.py"
                
                if [ -f "$script_path" ]; then
                    echo "  -> Running analysis in: $target_dir"
                    
                    # Navigate to the directory to ensure relative paths work correctly
                    cd "$target_dir" || continue
                    
                    # Run the python script
                    if python3 resource_analysis.py; then
                        # create a directory in the base dir to (if it doesn't exist) to store the resulting .png, .pdf, .svg files
                        mkdir -p "$BASE_DIR/analysis_plots_${analysis_dir#resource_collection_analysis_}"
                        # Move the generated plot files to the base directory under analysis_plots_#_sites
                        mv *resource_analysis_plot.* "$BASE_DIR/analysis_plots_${analysis_dir#resource_collection_analysis_}/"
                        echo "     [SUCCESS] moved generated plots to $BASE_DIR/analysis_plots_${analysis_dir#resource_collection_analysis_}/"
                    else
                        echo "     [FAILURE] Script returned non-zero exit code"
                    fi

                    percent_script_path="$target_dir/plot_48_resources_summary.py"

                    if [ -f "$percent_script_path" ]; then
                        echo "  -> Running summary plot in: $target_dir"
                        if python3 plot_48_resources_summary.py; then
                            mkdir -p "$BASE_DIR/analysis_plots_${analysis_dir#resource_collection_analysis_}"
                            mv *distribution_48.* "$BASE_DIR/analysis_plots_${analysis_dir#resource_collection_analysis_}/"
                            echo "     [SUCCESS] moved generated summary plots to $BASE_DIR/analysis_plots_${analysis_dir#resource_collection_analysis_}/"
                        else
                            echo "     [FAILURE] Summary script returned non-zero exit code"
                        fi
                    else
                        echo "  -> [SKIP] plot_48_resources_summary.py not found in $target_dir"
                    fi
                    
                    # Return to base directory
                    cd "$BASE_DIR" || exit 1
                else
                    echo "  -> [SKIP] resource_analysis.py not found in $target_dir"
                fi
            else
                echo "  -> [SKIP] Directory $target_dir does not exist"
            fi
        done
        echo ""
    fi
done

echo "All analyses complete."
