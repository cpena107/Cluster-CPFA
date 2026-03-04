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


# Run all analyses from `resource_collection_all/`

```
./sh/run_all_analyses.sh
```

* Runs all python scripts to generate the plots for 14x14 arena sizes

