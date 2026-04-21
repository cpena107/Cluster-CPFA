import os
import pandas as pd
from scipy.stats import ttest_ind

BASE_DIR = os.path.join(
    os.path.dirname(__file__),
    "..",
    "resource_collection_all",
    "resource_collection_analysis_tol_0.75m_freq_2.0s_visited_50_radius_0.75m_arena_14_14",
)

DISTRIBUTIONS = ["cluster_distribution", "powerlaw_distribution", "random_distribution"]
RESOURCE_COUNTS = [8, 16, 24, 32, 40, 48, 64, 80]


def load_completion_times(distribution: str, n_resources: int, group: str) -> pd.Series:
    """Return cumulative_time at milestone 100% for each seed in baseline or new."""
    csv_path = os.path.join(
        BASE_DIR,
        distribution,
        f"{n_resources}_resources",
        group,
        "resource_collection_analysis_details.csv",
    )
    df = pd.read_csv(csv_path)
    completed = df[df["milestone_percent"] == 100.0]["cumulative_time"]
    return completed.reset_index(drop=True)


def main(baseline: str = "baseline", new: str = "new"):
    results = []

    for dist in DISTRIBUTIONS:
        dist_label = dist.replace("_distribution", "")
        for n in RESOURCE_COUNTS:
            try:
                baseline_results = load_completion_times(dist, n, baseline)
                new_results = load_completion_times(dist, n, new)
            except FileNotFoundError:
                continue

            t_stat, p_val = ttest_ind(baseline_results, new_results, equal_var=False)
            results.append(
                {
                    "distribution": dist_label.capitalize(),
                    "resources": n,
                    "baseline_mean": round(baseline_results.mean(), 2),
                    "new_mean": round(new_results.mean(), 2),
                    #"t_statistic": round(t_stat, 4),
                    "p_value": p_val,
                    "significant (p<0.05)": p_val < 0.05,
                }
            )

    if baseline == "baseline":
        baseline_label = "CPFA"
    else:
        baseline_label = "GPFA"
    new_label = "CCPFA"
    
    df_results = pd.DataFrame(results)
    pd.set_option("display.float_format", "{:.4f}".format)
    pd.set_option("display.max_columns", None)
    pd.set_option("display.width", 140)
    #print(df_results.to_string(index=False))

    out_path = os.path.join(os.path.dirname(__file__), "ttest_results.csv")
    df_results.to_csv(out_path, index=False)
    #print(f"\nResults saved to {out_path}")

    # LaTeX output
    latex_df = df_results.copy()
    latex_df["p_value"] = latex_df["p_value"].apply(lambda x: f"{x:.4f}" if x >= 0.0001 else "<0.0001")
    latex_df["significant (p<0.05)"] = latex_df["significant (p<0.05)"].apply(lambda x: "Yes" if x else "No")
    latex_df.columns = ["Distribution", "Resources", f"{baseline_label} Mean (s)", f"{new_label} Mean (s)", "p-value", "Significant"]
    latex_str = latex_df.to_latex(
        index=False,
        column_format="|l|l|r|r||r|l|",
        escape=False,
        caption=f"Experiment I: {baseline_label} vs {new_label} Algorithm (cumulative completion time in seconds)",
        label=f"tab:ttest_results_{baseline}_{new}",
    )
    #print("\n% ==== LaTeX Table ====")
    print(latex_str)


if __name__ == "__main__":
    main(baseline="baseline", new="new")
    main(baseline="algorithm", new="new")
