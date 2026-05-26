import pandas as pd
import glob
import os

# Define the pattern for 100 sites
patterns = {
    'random': 'Cluster_GPFA_100sites_*res_14x14_random.csv',
    'clustered': 'Cluster_GPFA_100sites_*res_14x14_clustered.csv',
    'semi-clustered': 'Cluster_GPFA_100sites_*res_14x14_semi_cluster.csv'
}

resource_counts = [16, 32, 48, 64, 80]

results = {}

for dist_name, pattern_template in patterns.items():
    results[dist_name] = {}
    for res in resource_counts:
        # Construct specific filename pattern
        # Note: filenames use 'semi_cluster' (underscore) based on attachments
        # pattern_template uses * so we can just match loosely or construct exact name
        # Attachment list shows: Cluster_GPFA_100sites_16res_14x14_random.csv
        
        if dist_name == 'semi-clustered':
            filename = f"Cluster_GPFA_100sites_{res}res_14x14_semi_cluster.csv"
        else:
            filename = f"Cluster_GPFA_100sites_{res}res_14x14_{dist_name}.csv"
            
        if not os.path.exists(filename):
            # Try searching just in case
            files = glob.glob(f"*{res}res*{dist_name}*.csv")
            if files:
                filename = files[0]
            else:
                print(f"File not found for {dist_name} {res}")
                continue
                
        try:
            df = pd.read_csv(filename)
            # Use FinalTime column
            times = df['FinalTime']
            
            # Calculate stats
            stats = {
                'min': times.min(),
                'q1': times.quantile(0.25),
                'median': times.median(),
                'q3': times.quantile(0.75),
                'max': times.max()
            }
            results[dist_name][res] = stats
            
        except Exception as e:
            print(f"Error processing {filename}: {e}")

# Output results in a format easy to parse or copy
for dist in ['random', 'clustered', 'semi-clustered']:
    print(f"--- {dist} ---")
    for res in resource_counts:
        s = results[dist].get(res)
        if s:
            print(f"Resources {res}: lower whisker={s['min']}, lower quartile={s['q1']}, median={s['median']}, upper quartile={s['q3']}, upper whisker={s['max']}")
