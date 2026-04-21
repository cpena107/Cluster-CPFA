import os
import pandas as pd
from scipy.stats import ttest_ind

BASE_DIR = os.path.join(
    os.path.dirname(__file__),
    "..",
    "resource_collection_arenas",
    "resource_collection_analysis_tol_0.75m_freq_1.0s_visited_75_radius_1.25m",
    "arenas"
)

ARENAS = ["8x8", "10x10", "12x12", "14x14", "16x16"]


def load_completion_times(arena: str, group: str) -> pd.Series:
    """Return cumulative_time at milestone 100% for each seed in baseline or new."""
    csv_path = os.path.join(
        BASE_DIR,
        arena,
        group,
        "resource_collection_analysis_details.csv",
    )
    df = pd.read_csv(csv_path)
    completed = df[df["milestone_percent"] == 100.0]["cumulative_time"]
    return completed.reset_index(drop=True)


def main(baseline: str = "algorithm", new: str = "new"):
    results = []

    for arena in ARENAS:
        try:
            baseline_results = load_completion_times(arena, baseline)
            new_results = load_completion_times(arena, new)
        except FileNotFoundError as e:
            print(f"File not found: {e.filename}")
            continue

        if len(baseline_results) == 0 or len(new_results) == 0:
            print(f"Not enough data for arena {arena}")
            continue

        t_stat, p_val = ttest_ind(baseline_results, new_results, equal_var=False)
        results.append(
            {
                "arena_size": arena,
                "baseline_mean": round(baseline_results.mean(), 2),
                "new_mean": round(new_results.mean(), 2),
                #"t_statistic": round(t_stat, 4),
                "p_value": p_val,
                #"significant (p<0.05)": p_val < 0.05,
            }
        )

    if not results:
        print("No results to display.")
        return

    if baseline == "algorithm":
        baseline_label = "GPFA"
    else:
        baseline_label = "CPFA"
    new_label = "CCPFA"

    df_results = pd.DataFrame(results)
    pd.set_option("display.float_format", "{:.6f}".format)
    pd.set_option("display.max_columns", None)
    pd.set_option("display.width", 140)
    #print(df_results.to_string(index=False))

    out_path = os.path.join(os.path.dirname(__file__), "ttest_arenas_results.csv")
    df_results.to_csv(out_path, index=False)
    #print(f"\nResults saved to {out_path}")

    # LaTeX output
    latex_df = df_results.copy()
    latex_df["p_value"] = latex_df["p_value"].apply(lambda x: f"{x:.4f}" if x >= 0.05 else f"\\textbf{{{x:.4f}}}" if x >= 0.0001 else "\\textbf{<0.0001}")
    #latex_df["significant (p<0.05)"] = latex_df["significant (p<0.05)"].apply(lambda x: "Yes" if x else "No")
    latex_df.columns = ["Arena Size", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{baseline_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{new_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}{{@{{}}c@{{}}}} p-value \\\\ ({baseline_label} vs {new_label}) \\end{{tabular}}"]
    latex_str = latex_df.to_latex(
        index=False,
        column_format="|l|c|c|c|c|l|",
        escape=False,
        caption=f"Experiment III: {baseline_label} vs {new_label} Algorithm on Different Arena Sizes (cumulative completion time in seconds)",
        label=f"tab:ttest_arenas_results_{baseline}_{new}",
    )
    #print("\n% ==== LaTeX Table ====")
    print(latex_str)


if __name__ == "__main__":
    main(baseline="baseline", new="new")
    main(baseline="algorithm", new="new")

