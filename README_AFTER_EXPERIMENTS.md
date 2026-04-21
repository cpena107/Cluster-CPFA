# Collect all experiment data from `experiments\`

```
python scripts/collect_experiment_results.py
```

* Generates `all_experiment_results.csv`

# Copy all experiment results in GPFA format into `resource_collection_all/`

```
python scripts/compare_algos.py
```

* Copies `resource_collections_analysis\` to `resource_collection_all\resource_collection_analysis_{config}` where {config} is the experiment configuration.

# Needs fixing

Run
```
./sh/process_experiment_results.sh
```
Then again
```
python scripts/compare_algos.py
```


# Run all analyses from `resource_collection_all/`

```
./sh/run_all_analyses.sh
```

* Runs all python scripts to generate the plots for 14x14 arena sizes

# Run Mann-Whitney U statistical comparisons

```
python3 scripts/compare_groups_stats.py \
	--analysis-dir resource_collection_all/resource_collection_analysis_tol_0.75m_freq_1.0s_visited_75_radius_0.75m_arena_14_14 \
	--resources 16 32 48 64 80 \
	--milestones 25 50 75 100 \
	--pairs new:baseline new:algorithm
```

* Writes `stat_tests_mannwhitney.csv` in the selected analysis directory.
* Writes `stat_tests_mannwhitney_skipped.csv` when a comparison has missing/insufficient samples.

# Run Welch t-test statistical comparisons

```
python3 scripts/compare_groups_ttest.py \
	--analysis-dir resource_collection_all/resource_collection_analysis_tol_0.75m_freq_1.0s_visited_75_radius_0.75m_arena_14_14 \
	--resources 16 32 48 64 80 \
	--milestones 25 50 75 100 \
	--pairs new:baseline new:algorithm
```

* Writes `stat_tests_ttest.csv` in the selected analysis directory.
* Writes `stat_tests_ttest_skipped.csv` when a comparison has missing/insufficient samples.

