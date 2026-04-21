import os
import pandas as pd
from scipy.stats import ttest_ind

MILESTONES = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0]

def load_milestone_times(group: str, milestone: float, distribution: str) -> pd.Series:
    """Return cumulative_time at a given milestone percent for each seed in baseline or new."""
    BASE_DIR = os.path.join(
        os.path.dirname(__file__),
        "..",
        "resource_collection_all",
        "resource_collection_analysis_tol_0.75m_freq_1.0s_visited_75_radius_1.25m_arena_14_14",
        distribution,
        "48_resources"
    )

    csv_path = os.path.join(
        BASE_DIR,
        group,
        "resource_collection_analysis_details.csv",
    )
    df = pd.read_csv(csv_path)
    # The float values might not match exactly due to floating point precision, 
    # but 10.0 etc are usually exact enough or we can use round() comparison
    completed = df[df["milestone_percent"].round(1) == milestone]["cumulative_time"]
    return completed.reset_index(drop=True)

def main(baseline: str = "baseline", algorithm: str = "algorithm", new: str = "new", distribution: str = "cluster_distribution"):
    results_baseline_new = []
    results_algorithm_new = []

    for milestone in MILESTONES:
        try:
            baseline_data = load_milestone_times(baseline, milestone, distribution)
            new_data = load_milestone_times(new, milestone, distribution)
            algorithm_data = load_milestone_times(algorithm, milestone, distribution)
        except FileNotFoundError as e:
            print(f"File not found: {e.filename}")
            continue
            
        if len(baseline_data) == 0 or len(new_data) == 0:
            print(f"Not enough data for milestone {milestone}%")
            continue

        t_stat, p_val = ttest_ind(baseline_data, new_data, equal_var=False)
        tstat_alg, p_val_alg = ttest_ind(algorithm_data, new_data, equal_var=False)
        results_baseline_new.append(
            {
                "milestone_percent": f"{int(milestone)}\\%",
                "baseline_mean": round(baseline_data.mean(), 2),
                "algorithm_mean": round(algorithm_data.mean(), 2),
                "new_mean": round(new_data.mean(), 2),
                #"t_statistic": round(t_stat, 4),
                "p_value_baseline_new": p_val,
                "p_value_algorithm_new": p_val_alg,
                #"significant (p<0.05)": p_val < 0.05,
            }
        )
        
    if not results_baseline_new:
        print("No results to display.")
        return

    df_results_baseline_new = pd.DataFrame(results_baseline_new)
    pd.set_option("display.float_format", "{:.4f}".format)
    pd.set_option("display.max_columns", None)
    pd.set_option("display.width", 140)
    #print(df_results_baseline_new.to_string(index=False))

    out_path = os.path.join(os.path.dirname(__file__), "ttest_milestones_results.csv")
    df_results_baseline_new.to_csv(out_path, index=False)
    #print(f"\nResults saved to {out_path}")


    algorithm_label = "GPFA"
    baseline_label = "CPFA" 
    new_label = "CCPFA"
    if distribution == "random_distribution":
        distribution_label = "Random"
    elif distribution == "cluster_distribution":
        distribution_label = "Cluster"
    elif distribution == "powerlaw_distribution":
        distribution_label = "Powerlaw"
    # LaTeX output
    latex_df = df_results_baseline_new.copy()
    latex_df["p_value_baseline_new"] = latex_df["p_value_baseline_new"].apply(lambda x: f"{x:.4f}" if x >= 0.05 else f"\\textbf{{{x:.4f}}}" if x >= 0.0001 else "\\textbf{<0.0001}")
    latex_df["p_value_algorithm_new"] = latex_df["p_value_algorithm_new"].apply(lambda x: f"{x:.4f}" if x >= 0.05 else f"\\textbf{{{x:.4f}}}" if x >= 0.0001 else "\\textbf{<0.0001}") 
    #latex_df["significant (p<0.05)"] = latex_df["significant (p<0.05)"].apply(lambda x: "Yes" if x else "No")
    latex_df.columns = ["Milestone", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{baseline_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{algorithm_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{new_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}{{@{{}}c@{{}}}} p-value \\\\ ({baseline_label} vs {new_label}) \\end{{tabular}}", f"\\begin{{tabular}}{{@{{}}c@{{}}}} p-value \\\\ ({algorithm_label} vs {new_label}) \\end{{tabular}}"]
    latex_str = latex_df.to_latex(
        index=False,
        column_format="|l|c|c|c|c|c|l|",
        escape=False,
        caption=f"Experimnet II: {baseline_label} and {algorithm_label} vs {new_label} Algorithm by Milestone ({distribution_label})",
        label=f"tab:ttest_milestones_results_{baseline}_{algorithm}_{new}_{distribution}",
    )
    #print("\n% ==== LaTeX Table ====")
    print(latex_str)

if __name__ == "__main__":
    main(distribution="cluster_distribution")
    main(distribution="powerlaw_distribution")
    main(distribution="random_distribution")

