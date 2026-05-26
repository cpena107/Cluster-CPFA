import os
import re
import glob
import pandas as pd

SITES = [25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 500]
SECS = [5, 10, 15, 20]
DISTRIBUTIONS = [
    ("cluster_distribution", "Clustered"),
    ("powerlaw_distribution", "Powerlaw"),
    ("random_distribution", "Random"),
]
ALG_MAP = {"baseline": "CPFA", "algorithm": "GCFA", "new": "CCPFA"}
EXCLUDE_COUNTS = {8, 24, 40}


def find_resource_counts(root_dir):
    counts = []
    try:
        for name in os.listdir(root_dir):
            m = re.match(r"^(\d+)_resources$", name)
            if m:
                val = int(m.group(1))
                if val not in EXCLUDE_COUNTS:
                    counts.append(val)
    except FileNotFoundError:
        return []
    return sorted(counts)


def pick_csv(files):
    if not files:
        return None
    for f in files:
        if "details" in os.path.basename(f).lower():
            return f
    return files[0]


def compute_means_for(root_dir, res_count, secs):
    res_dir = os.path.join(root_dir, f"{res_count}_resources")
    if not os.path.isdir(res_dir):
        return {}
    row = {}
    for sub, label in ALG_MAP.items():
        path = os.path.join(res_dir, sub)
        if not os.path.isdir(path):
            continue
        files = glob.glob(os.path.join(path, "*.csv"))
        target = pick_csv(files)
        if not target:
            continue
        try:
            df = pd.read_csv(target)
            if "milestone_percent" in df.columns and "cumulative_time" in df.columns:
                completed = df[df["milestone_percent"] == 100]
                if not completed.empty:
                    row[label] = float(completed["cumulative_time"].mean())
            elif "FinalTime" in df.columns:
                row[label] = float(df["FinalTime"].mean())
        except Exception:
            pass
    return row


def fmt(x, digits=2):
    return ("-" if x is None else f"{x:.{digits}f}")


def bold_min(values_map):
    # values_map: {label: value or None}
    present = {k: v for k, v in values_map.items() if v is not None}
    if not present:
        return {k: fmt(values_map.get(k)) for k in values_map}
    min_label = min(present, key=lambda k: present[k])
    out = {}
    for k in values_map:
        val = values_map.get(k)
        text = fmt(val)
        if k == min_label and val is not None:
            out[k] = f"\\textbf{{{text}}}"
        else:
            out[k] = text
    return out


def percent_improvement(cpfa, ccpfa):
    if cpfa is None or ccpfa is None:
        return "-"
    if cpfa == 0:
        return "-"
    pct = (cpfa - ccpfa) / cpfa * 100.0
    return f"{pct:.1f}\\%"


def generate_tables():
    parts = []
    parts.append("% Auto-generated tables: algorithm comparison with % improvement from CPFA to CCPFA\n")
    for sites in SITES:
        for secs in SECS:
            base = f"resource_collection_analysis_{sites}_sites_{secs}_seconds"
            for dist_dir, dist_label in DISTRIBUTIONS:
                root_dir = os.path.join(base, dist_dir)
                counts = find_resource_counts(root_dir)
                if not counts:
                    continue
            # Table header
            parts.append("\\begin{table}[htbp]")
            parts.append("  \\centering")
            parts.append(f"  \\caption{{Mean collection times (seconds) — {dist_label} — {sites} Sites, {secs} Seconds Sample Time}}")
            parts.append("  \\begin{tabular}{|l|r|r|r|r|}")
            parts.append("    \\hline")
            parts.append("    Resources & CPFA & GCFA & CCPFA & CCPFA vs CPFA \\\\ ")
            parts.append("    \\hline")
            # Rows
            for res in counts:
                vals = compute_means_for(root_dir, res, secs)
                cpfa = vals.get("CPFA")
                gcfa = vals.get("GCFA")
                ccpfa = vals.get("CCPFA")
                styled = bold_min({"CPFA": cpfa, "GCFA": gcfa, "CCPFA": ccpfa})
                imp = percent_improvement(cpfa, ccpfa)
                parts.append(
                    f"    {res} & {styled['CPFA']} & {styled['GCFA']} & {styled['CCPFA']} & {imp} \\\\"
                )
            parts.append("    \\hline")
            parts.append("  \\end{tabular}")
            parts.append("\\end{table}\n")
    return "\n".join(parts)


if __name__ == "__main__":
    output = generate_tables()
    with open("algorithm_comparison_tables.tex", "w") as f:
        f.write(output)
    print("Wrote algorithm_comparison_tables.tex")
