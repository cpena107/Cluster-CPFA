#!/bin/bash

# Script to run multiple ARGoS simulations and collect milestone results
# Usage: ./run_simulations.sh <xml_config_file> <output_file> [num_runs]

# Check if required arguments are provided
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <xml_config_file> <output_file> [num_runs]"
    echo "Example: $0 experiments/CPFA_ClusterMap_cluster_resources.xml results.txt 10"
    exit 1
fi

XML_FILE=$1
OUTPUT_FILE=$2
NUM_RUNS=${3:-10}  # Default to 10 runs if not specified

# Check if XML file exists
if [ ! -f "$XML_FILE" ]; then
    echo "Error: XML file '$XML_FILE' not found!"
    exit 1
fi

# Create output file with requested header
echo "random_seed,milestone_percent,time_interval,cumulative_time,food_distribution,algorithm_mode,num_robots,total_food" > "$OUTPUT_FILE"
echo "Starting $NUM_RUNS simulations using $XML_FILE"
echo "Results will be saved to $OUTPUT_FILE"
echo "----------------------------------------"

# Run simulations
for i in $(seq 1 $NUM_RUNS); do
    echo "Running simulation $i of $NUM_RUNS..."
    
    # Run ARGoS and capture output
    OUTPUT=$(argos3 -c "$XML_FILE" 2>&1)
    STATUS=$?

    if [ "$STATUS" -ne 0 ]; then
        echo "  Run $i failed (exit=$STATUS). Skipping."
        echo "$OUTPUT" | tail -5
        continue
    fi

    # Extract lines in format:
    # random_seed,milestone_percent,time_interval,cumulative_time,food_distribution,algorithm_mode,num_robots,total_food
    RUN_ROWS=$(echo "$OUTPUT" | awk -F',' '
        NF == 8 {
            for(i=1;i<=8;i++) {
                gsub(/^[ \t]+|[ \t]+$/, "", $i)
            }
            if($1 ~ /^[0-9]+$/ &&
               $2 ~ /^-?[0-9]+(\.[0-9]+)?$/ &&
               $3 ~ /^-?[0-9]+(\.[0-9]+)?$/ &&
               $4 ~ /^-?[0-9]+(\.[0-9]+)?$/ &&
               $5 ~ /^-?[0-9]+$/ &&
               $6 ~ /^-?[0-9]+$/ &&
               $7 ~ /^-?[0-9]+$/ &&
               $8 ~ /^-?[0-9]+$/) {
                print $1 "," $2 "," $3 "," $4 "," $5 "," $6 "," $7 "," $8
            }
        }
    ')

    if [ -z "$RUN_ROWS" ]; then
        echo "  Run $i: no milestone rows found in output."
        continue
    fi

    # Append all milestone rows from this run
    echo "$RUN_ROWS" >> "$OUTPUT_FILE"
    echo "  Run $i: recorded $(echo "$RUN_ROWS" | wc -l) milestone rows"
done

echo "----------------------------------------"
echo "All simulations complete!"
echo "Results saved to $OUTPUT_FILE"

echo ""
echo "Preview:"
echo "----------------------------------------"
head -n 12 "$OUTPUT_FILE"
