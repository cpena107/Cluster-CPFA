import os
import glob
import re
import pandas as pd

def _find_resource_counts(root_dir):
    """Discover all resource-count subfolders like '16_resources' and return sorted ints."""
    counts = []
    try:
        for name in os.listdir(root_dir):
            m = re.match(r"^(\d+)_resources$", name)
            if m:
                if int(m.group(1)) not in [8, 24, 40]:  # Exclude these counts
                    counts.append(int(m.group(1)))
    except FileNotFoundError:
        return []
    return sorted(counts)

def _pick_csv(files):
    """Prefer a '*details*.csv' file; otherwise return the first CSV found."""
    if not files:
        return None
    for f in files:
        if "details" in os.path.basename(f).lower():
            return f
    return files[0]

def calculate_means(root_dir):
    print(f"Analyzing {root_dir}")
    types = {'baseline': 'CPFA', 'algorithm': 'GCFA', 'new': 'CCPFA'}

    resources = _find_resource_counts(root_dir)
    if not resources:
        print("  (no resource folders found)\n" + "-" * 30)
        return

    results = []

    for res in resources:
        res_dir = os.path.join(root_dir, f"{res}_resources")
        if not os.path.isdir(res_dir):
            continue

        row = {'Resources': res}
        for type_dir, label in types.items():
            path = os.path.join(res_dir, type_dir)
            if not os.path.isdir(path):
                continue
            files = glob.glob(os.path.join(path, "*.csv"))
            target = _pick_csv(files)
            if not target:
                continue

            try:
                df = pd.read_csv(target)
                # Prefer milestone-filtered cumulative_time when available
                if 'milestone_percent' in df.columns and 'cumulative_time' in df.columns:
                    completed = df[df['milestone_percent'] == 100]
                    if not completed.empty:
                        row[label] = completed['cumulative_time'].mean()
                elif 'FinalTime' in df.columns:
                    row[label] = df['FinalTime'].mean()
            except Exception:
                # Skip unreadable/malformed files
                pass
        results.append(row)

    df_res = pd.DataFrame(results)
    print(df_res)
    print("-" * 30)

if __name__ == "__main__":
    # calculate means for each distribution type for 25, 50, 75, 100, 125, 150, 175,
    # 200, 225, 250, 275, 300, 500 sites
    for sites in [25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 500]:
        for secs in [5, 10, 15, 20]:
            base = f"resource_collection_analysis_{sites}_sites_{secs}_seconds"
            calculate_means(os.path.join(base, "cluster_distribution"))
            calculate_means(os.path.join(base, "random_distribution"))
            calculate_means(os.path.join(base, "powerlaw_distribution"))