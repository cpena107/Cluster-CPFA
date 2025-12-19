#!/bin/bash

# Script to run multiple ARGoS simulations and collect results
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

# Create output file with header
echo "Run,FinalTime,ResourcesCollected" > "$OUTPUT_FILE"
echo "Starting $NUM_RUNS simulations using $XML_FILE"
echo "Results will be saved to $OUTPUT_FILE"
echo "----------------------------------------"

# Run simulations
for i in $(seq 1 $NUM_RUNS); do
    echo "Running simulation $i of $NUM_RUNS..."
    
    # Run ARGoS and capture output
    OUTPUT=$(argos3 -c "$XML_FILE" 2>&1)
    
    # Get the last line of output that starts with a number (contains time, resources)
    LAST_LINE=$(echo "$OUTPUT" | grep -E "^[0-9]" | tail -1)
    
    # Extract time and resources from the last line (format: "time, resources")
    FINAL_TIME=$(echo "$LAST_LINE" | awk -F',' '{print $1}' | tr -d ' ')
    RESOURCES=$(echo "$LAST_LINE" | awk -F',' '{print $2}' | tr -d ' ')
    
    # Save to output file
    echo "$i,$FINAL_TIME,$RESOURCES" >> "$OUTPUT_FILE"
    echo "  Run $i: Time=$FINAL_TIME, Resources=$RESOURCES"
done

echo "----------------------------------------"
echo "All simulations complete!"
echo "Results saved to $OUTPUT_FILE"

# Display summary statistics
echo ""
echo "Summary Statistics:"
echo "----------------------------------------"
awk -F',' 'NR>1 {sum_time+=$2; sum_res+=$3; count++} 
    END {
        if(count>0) {
            print "Average Time: " sum_time/count
            print "Average Resources: " sum_res/count
            print "Total Runs: " count
        }
    }' "$OUTPUT_FILE"
