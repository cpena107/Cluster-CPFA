#!/bin/bash

# Create the target directory
mkdir -p analysis_plots

for s in analysis_plots_resource_collection_analysis*; do
        if [ -d "$s" ]; then
            src_dir="$s"
             # Extract seconds from the directory name using regex
            echo "Processing $src_dir..."
            cp "${src_dir}/cluster_resource_analysis_plot.svg" "analysis_plots/${src_dir}_cluster.svg"
            cp "${src_dir}/powerlaw_resource_analysis_plot.svg" "analysis_plots/${src_dir}_powerlaw.svg"
            cp "${src_dir}/random_resource_analysis_plot.svg" "analysis_plots/${src_dir}_random.svg"
        else
            echo "Warning: Directory $s not found"
        fi
    done

echo "Plots gathered in 'analysis_plots'"
