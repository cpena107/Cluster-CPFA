#!/bin/bash

NUM_RUNS=30
VISITED_TOLERANCE=0.5
#for VISITED_TOLERANCE in 0.25 0.5 0.75 1.0 1.25 1.5 1.75 2.0; do
    for RECORDING_FREQ in 1 2 3 4 5; do
        for MAX_VISITED in 25 50 75 100; do
            for MAX_RADIUS in 0.5 0.75 1.0 1.25 1.5 1.75 2.0; do
                for ARENA_X in 8 10 12 14; do
                        OUTPUT_DIR="experiments/tol_${VISITED_TOLERANCE}m_freq_${RECORDING_FREQ}s_visited_${MAX_VISITED}_radius_${MAX_RADIUS}m_arena_${ARENA_X}_${ARENA_X}"
                        
                        mkdir -p "$OUTPUT_DIR"

                        ./run_all_clustermap_experiments.sh \
                            "$NUM_RUNS" \
                            "$OUTPUT_DIR" \
                            "$VISITED_TOLERANCE" \
                            "$RECORDING_FREQ" \
                            "$MAX_VISITED" \
                            "$MAX_RADIUS" \
                            "$ARENA_X" \
                            "$ARENA_X"
                        cat $OUTPUT_DIR
                done
            done
        done
    done
done