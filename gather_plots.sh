#!/bin/bash

# Create the target directory
mkdir -p analysis_plots

# List of site counts
sites=(25 50 75 100 125 150 175 200 225 250 275 300 500)
seconds=(5 10 15 20)

for s in "${sites[@]}"; do
    for sec in "${seconds[@]}"; do
        src_dir="analysis_plots_${s}_sites_${sec}_seconds"
        if [ -d "$src_dir" ]; then
            echo "Processing $src_dir..."
            cp "$src_dir/cluster_resource_analysis_plot.svg" "analysis_plots/${s}_sites_${sec}_seconds_cluster.svg"
            cp "$src_dir/powerlaw_resource_analysis_plot.svg" "analysis_plots/${s}_sites_${sec}_seconds_powerlaw.svg"
            cp "$src_dir/random_resource_analysis_plot.svg" "analysis_plots/${s}_sites_${sec}_seconds_random.svg"
        else
            echo "Warning: Directory $src_dir not found"
        fi
    done
done

echo "Plots gathered in 'analysis_plots'"
