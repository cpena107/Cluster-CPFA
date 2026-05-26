import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import ttest_ind

BASE_DIR = os.path.join(
    os.path.dirname(__file__),
    "..",
    "resource_collection_all",
    "resource_collection_analysis_tol_0.75m_freq_2s_visited_75_radius_0.75m_arena_14_14",
)

DISTRIBUTIONS = ["cluster_distribution", "powerlaw_distribution", "random_distribution"]
RESOURCE_COUNTS = [16, 32, 48, 64, 80]

BASELINE_LABEL  = "CPFA"
ALGORITHM_LABEL = "GCFA"
NEW_LABEL       = "ARPC"

PALETTE = {
    BASELINE_LABEL:  '#4C72B0',
    ALGORITHM_LABEL: '#C0392B',
    NEW_LABEL:       '#27AE60',
}


def sig_label(p: float) -> str:
    """Return significance star notation based on p-value."""
    if p < 0.001:
        return '***'
    elif p < 0.01:
        return '**'
    elif p < 0.05:
        return '*'
    return '-'


def draw_bracket(ax, x1, x2, y_top, label, color='black', lw=0.9, fontsize=9):
    """Draw a significance bracket between x1 and x2 at height y_top."""
    y_range = ax.get_ylim()[1] - ax.get_ylim()[0]
    tick_h = y_range * 0.012
    ax.plot([x1, x1, x2, x2],
            [y_top, y_top + tick_h, y_top + tick_h, y_top],
            lw=lw, c=color, clip_on=False)
    ax.text((x1 + x2) / 2, y_top + tick_h * 1.15, label,
            ha='center', va='bottom', color=color, fontsize=fontsize,
            clip_on=False)


def load_all_data(distribution: str,
                  baseline: str = "baseline",
                  algorithm: str = "algorithm",
                  new: str = "new") -> pd.DataFrame:
    """Load 100% milestone completion times for all three groups and resource counts."""
    frames = []
    for group, label in [(baseline, BASELINE_LABEL),
                         (algorithm, ALGORITHM_LABEL),
                         (new, NEW_LABEL)]:
        for n in RESOURCE_COUNTS:
            csv_path = os.path.join(
                BASE_DIR, distribution,
                f"{n}_resources", group,
                "resource_collection_analysis_details.csv",
            )
            if not os.path.exists(csv_path):
                continue
            df = pd.read_csv(csv_path)
            df = df[df['milestone_percent'] == 100.0].copy()
            df['Num Resources'] = str(n)
            df['Experiment Type'] = label
            frames.append(df[['cumulative_time', 'Num Resources', 'Experiment Type']])
    return pd.concat(frames, ignore_index=True) if frames else pd.DataFrame()


def plot_distribution(distribution: str,
                      distribution_label: str,
                      baseline: str = "baseline",
                      algorithm: str = "algorithm",
                      new: str = "new") -> None:
    """Create a box-plot with significance brackets for a single distribution."""
    data = load_all_data(distribution, baseline, algorithm, new)
    if data.empty:
        print(f"No data for {distribution}, skipping plot.")
        return

    sns.set_theme(style='whitegrid')
    sns.set_context('paper', font_scale=2.2)
    plt.rcParams.update({
        'font.size': 16,
        'axes.titlesize': 24,
        'axes.labelsize': 18,
        'xtick.labelsize': 16,
        'ytick.labelsize': 16,
        'legend.fontsize': 16,
    })

    fig, ax = plt.subplots(figsize=(14, 7))

    hue_order = [BASELINE_LABEL, ALGORITHM_LABEL, NEW_LABEL]
    # x-order: resource counts present in data as strings
    x_order = [str(n) for n in RESOURCE_COUNTS if str(n) in data['Num Resources'].values]

    sns.boxplot(
        data=data,
        x='Num Resources',
        y='cumulative_time',
        hue='Experiment Type',
        palette=PALETTE,
        hue_order=hue_order,
        order=x_order,
        width=0.45,
        fliersize=3,
        linewidth=1.1,
        dodge=True,
        showfliers=True,
        ax=ax,
    )

    ax.set_ylabel('Time (seconds)', fontsize=18)
    ax.set_xlabel('Number of Resources', fontsize=18)

    for artist in ax.artists:
        artist.set_edgecolor('black')
        artist.set_linewidth(0.8)

    # ---- significance brackets ----
    box_width = 0.45 / 3          # matches width / n_groups
    offsets = np.array([-box_width, 0.0, box_width])  # CPFA, GCFA, ARPC

    y_max_global = data['cumulative_time'].max()
    ax.set_ylim(top=y_max_global * 1.28)

    for tick_idx, n_str in enumerate(x_order):
        sub = data[data['Num Resources'] == n_str]
        cpfa_vals  = sub[sub['Experiment Type'] == BASELINE_LABEL]['cumulative_time'].values
        gcfa_vals  = sub[sub['Experiment Type'] == ALGORITHM_LABEL]['cumulative_time'].values
        ARPC_vals = sub[sub['Experiment Type'] == NEW_LABEL]['cumulative_time'].values

        if len(cpfa_vals) == 0 or len(gcfa_vals) == 0 or len(ARPC_vals) == 0:
            continue

        _, p_cpfa = ttest_ind(cpfa_vals,  ARPC_vals, equal_var=False)
        _, p_gcfa = ttest_ind(gcfa_vals,  ARPC_vals, equal_var=False)

        y_base = sub['cumulative_time'].max()
        gap    = (ax.get_ylim()[1] - ax.get_ylim()[0]) * 0.055

        x_cpfa  = tick_idx + offsets[0]
        x_gcfa  = tick_idx + offsets[1]
        x_ARPC = tick_idx + offsets[2]

        # Lower bracket: GCFA vs ARPC
        draw_bracket(ax, x_gcfa, x_ARPC, y_base + gap * 0.6,
                     sig_label(p_gcfa), fontsize=8)
        # Upper bracket: CPFA vs ARPC
        draw_bracket(ax, x_cpfa, x_ARPC, y_base + gap * 1.7,
                     sig_label(p_cpfa), fontsize=8)

    # ---- legend (inside plot, top-left) ----
    ax.legend(loc='upper left', borderaxespad=0.5, title='')

    plt.tight_layout()

    out_dir = os.path.dirname(__file__)
    out_prefix = os.path.join(out_dir, f"ttest_boxplot_{distribution}")
    fig.savefig(f"{out_prefix}.png", dpi=300, bbox_inches='tight')
    fig.savefig(f"{out_prefix}.pdf", bbox_inches='tight')
    fig.savefig(f"{out_prefix}.svg", bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {out_prefix}.{{png,pdf,svg}}")


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


def main(baseline: str = "baseline", algorithm: str = "algorithm", new: str = "new"):
    results = []

    for dist in DISTRIBUTIONS:
        dist_label = dist.replace("_distribution", "")
        for n in RESOURCE_COUNTS:
            try:
                baseline_results = load_completion_times(dist, n, baseline)
                algorithm_results = load_completion_times(dist, n, algorithm)
                new_results = load_completion_times(dist, n, new)
            except FileNotFoundError:
                continue

            t_stat, p_val = ttest_ind(baseline_results, new_results, equal_var=False)
            t_stat_alg, p_val_alg = ttest_ind(algorithm_results, new_results, equal_var=False)
            results.append(
                {
                    "distribution": dist_label.capitalize(),
                    "resources": n,
                    "baseline_mean": round(baseline_results.mean(), 2),
                    "algorithm_mean": round(algorithm_results.mean(), 2),
                    "new_mean": round(new_results.mean(), 2),
                    #"t_statistic": round(t_stat, 4),
                    "p_value": p_val,
                    "p_value_algorithm": p_val_alg,
                    #"significant (p<0.05)": p_val < 0.05,
                }
            )


    baseline_label = "CPFA"
    algorithm_label = "GPFA"
    new_label = "ARPC"
    
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
    latex_df["p_value"] = latex_df["p_value"].apply(lambda x: f"{x:.4f}" if x >= 0.05 else f"\\textbf{{{x:.4f}}}" if x >= 0.0001 else "\\textbf{<0.0001}")
    latex_df["p_value_algorithm"] = latex_df["p_value_algorithm"].apply(lambda x: f"{x:.4f}" if x >= 0.05 else f"\\textbf{{{x:.4f}}}" if x >= 0.0001 else "\\textbf{<0.0001}") 
    latex_df["baseline_mean"] = latex_df["baseline_mean"].apply(lambda x: f"{x:.2f}")
    latex_df["algorithm_mean"] = latex_df["algorithm_mean"].apply(lambda x: f"{x:.2f}")
    latex_df["new_mean"] = latex_df["new_mean"].apply(lambda x: f"{x:.2f}")
    #latex_df["significant (p<0.05)"] = latex_df["significant (p<0.05)"].apply(lambda x: "Yes" if x else "No")
    latex_df.columns = ["Distribution", "Resources", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{baseline_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{algorithm_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}[c]{{@{{}}c@{{}}}}{new_label} \\\\ Means \\end{{tabular}}", f"\\begin{{tabular}}{{@{{}}c@{{}}}} p-value \\\\ ({baseline_label} vs {new_label}) \\end{{tabular}}", f"\\begin{{tabular}}{{@{{}}c@{{}}}} p-value \\\\ ({algorithm_label} vs {new_label}) \\end{{tabular}}"]
    latex_str = latex_df.to_latex(
        index=False,
        column_format="|l|c|c|c|c|c|l|",
        escape=False,
        caption=f"Experiment I: {baseline_label}, {algorithm_label} vs {new_label} Algorithm (cumulative completion time in seconds)",
        label=f"tab:ttest_results_{baseline}_{algorithm}_{new}",
    )
    #print("\n% ==== LaTeX Table ====")
    print(latex_str)

    # Generate one box-plot per distribution
    for dist in DISTRIBUTIONS:
        dist_label = dist.replace("_distribution", "").capitalize()
        plot_distribution(dist, dist_label, baseline, algorithm, new)


if __name__ == "__main__":
    main(baseline="baseline", algorithm="algorithm", new="new")
