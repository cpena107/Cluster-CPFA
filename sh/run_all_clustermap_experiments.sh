#!/bin/bash

# Script to run all CPFA_ClusterMap experiments with proper output file naming
# Output files follow the pattern: Cluster_CPFA_{n}res_{distribution}.csv
# Each CSV row is a milestone record from ARGoS output:
# random_seed,milestone_percent,time_interval,cumulative_time,food_distribution,algorithm_mode,num_robots,total_food

NUM_RUNS=${1:-30}  # Default to 30 runs if not specified
OUTPUT_DIR=${2:-"."}  # Default to current directory if not specified
VISITED_TOLERANCE=${3:-0.5}  # Default visited tolerance
RECORDING_FREQ=${4:-2}  # Default recording frequency
MAX_VISITED=${5:-50}  # Default max visited locations
MAX_RADIUS=${6:-0.75}  # Default max cluster radius
ARENA_X=${7:-14.0}  # Default arena size X
ARENA_Y=${8:-14.0}  # Default arena size Y
# Create output directory if it doesn't exist
if [ ! -d "$OUTPUT_DIR" ]; then
    echo "Creating output directory: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
fi

echo "========================================"
echo "Running all CPFA_ClusterMap experiments"
echo "Number of runs per experiment: $NUM_RUNS"
echo "Output directory: $OUTPUT_DIR"
echo "========================================"
echo ""

# Generation is now handled by the calling python script
# if [ -f "scripts/generate_experiments.py" ]; then
#     python3 scripts/generate_experiments.py \
#       --visited-tolerance ${VISITED_TOLERANCE} \
#       --recording-freq ${RECORDING_FREQ} \
#       --max-visited ${MAX_VISITED} \
#       --max-radius ${MAX_RADIUS} \
#       --arena-x ${ARENA_X} \
#       --arena-y ${ARENA_Y} \
#       --output-dir ${OUTPUT_DIR}
# elif [ -f "generate_experiments.py" ]; then
#     python3 generate_experiments.py \
#       --visited-tolerance ${VISITED_TOLERANCE} \
#       --recording-freq ${RECORDING_FREQ} \
#       --max-visited ${MAX_VISITED} \
#       --max-radius ${MAX_RADIUS} \
#       --arena-x ${ARENA_X} \
#       --arena-y ${ARENA_Y} \
#       --output-dir ${OUTPUT_DIR}
# else
#     echo "Error: generate_experiments.py not found!"
#     exit 1
# fi

# Array to store experiment files
EXPERIMENTS=(
    "${OUTPUT_DIR}/CPFA_ClusterMap_clustered_16.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_clustered_32.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_clustered_48.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_clustered_64.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_clustered_80.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_random_16.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_random_32.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_random_48.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_random_64.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_random_80.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_semi_cluster_16.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_semi_cluster_32.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_semi_cluster_48.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_semi_cluster_64.xml"
    "${OUTPUT_DIR}/CPFA_ClusterMap_semi_cluster_80.xml"
)

# Function to extract the last number from filename
extract_number() {
    local filename="$1"
    # Extract the last number before .xml (e.g., 16, 32, 48, 64, 80)
    echo "$filename" | grep -oE '[0-9]+' | tail -1
}

# Function to run a single experiment
run_experiment() {
    local XML_FILE="$1"
    local NUM_RUNS="$2"
    local OUTPUT_DIR="$3"
    local NUM
    local TYPE
    local OUTPUT_FILE
    local OUTPUT
    local STATUS
    local RUN_ROWS
    
    # Check if file exists
    if [ ! -f "$XML_FILE" ]; then
        echo "Warning: File '$XML_FILE' not found, skipping..."
        return
    fi
    
    # Extract the number from the filename
    NUM=$(extract_number "$XML_FILE")
    TYPE=$(echo "$XML_FILE" | grep -oE 'clustered|random|semi_cluster')
    
    # Generate output filename
    OUTPUT_FILE="${OUTPUT_DIR}/Cluster_CPFA_${NUM}res_${TYPE}.csv"
    
    echo "----------------------------------------"
    echo "Processing: $XML_FILE"
    echo "Output file: $OUTPUT_FILE"
    echo "----------------------------------------"
    
    # Create output file with milestone header
    echo "random_seed,milestone_percent,time_interval,cumulative_time,food_distribution,algorithm_mode,num_robots,total_food" > "$OUTPUT_FILE"
    
    # Run simulations
    for i in $(seq 1 $NUM_RUNS); do
        echo "  Running simulation $i of $NUM_RUNS for $XML_FILE..."
        
        # Run ARGoS and capture output
        OUTPUT=$(argos3 -c "$XML_FILE" 2>&1)
        STATUS=$?

        if [ "$STATUS" -ne 0 ]; then
            echo "    Run $i failed (exit=$STATUS), skipping."
            echo "$OUTPUT" | tail -5
            continue
        fi

        # Extract lines with exactly 8 CSV fields:
        # random_seed,milestone_percent,time_interval,cumulative_time,food_distribution,algorithm_mode,num_robots,total_food
        RUN_ROWS=$(echo "$OUTPUT" | awk -F',' '
            NF == 8 {
                for(j=1;j<=8;j++) {
                    gsub(/^[ \t]+|[ \t]+$/, "", $j)
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
            echo "    Run $i: no milestone rows found."
            continue
        fi

        echo "$RUN_ROWS" >> "$OUTPUT_FILE"
        echo "    Run $i: recorded $(echo "$RUN_ROWS" | wc -l) milestone rows"
    done
    
    # Display summary statistics for this experiment
    echo ""
    echo "  Summary for $OUTPUT_FILE:"
    awk -F',' 'NR>1 {
            count++
            seeds[$1]=1
            if($2+0 == 100) completed++
        }
        END {
            if(count>0) {
                seed_count=0
                for(k in seeds) seed_count++
                print "    Total milestone rows: " count
                print "    Unique seeds recorded: " seed_count
                print "    100% milestones recorded: " completed
            }
        }' "$OUTPUT_FILE"
    echo ""
}

# Export function and variables for parallel execution
export -f run_experiment
export -f extract_number

# Run all experiments in parallel
for XML_FILE in "${EXPERIMENTS[@]}"; do
    run_experiment "$XML_FILE" "$NUM_RUNS" "$OUTPUT_DIR" &
done

# Wait for all background jobs to complete
wait

echo "========================================"
echo "All experiments complete!"
echo "========================================"
echo ""
echo "Output files generated:"