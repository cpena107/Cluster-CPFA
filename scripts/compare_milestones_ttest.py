import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import ttest_ind

MILESTONES = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0]

BASELINE_LABEL  = "CPFA"
ALGORITHM_LABEL = "GCFA"
NEW_LABEL       = "ARPC"

PALETTE = {
    BASELINE_LABEL:  '#4C72B0',
    ALGORITHM_LABEL: '#C0392B',
    NEW_LABEL:       '#27AE60',
}


def sig_label(p: float) -> str:
    """Significance star notation."""
    if p < 0.001:
        return '***'
    elif p < 0.01:
        return '**'
    elif p < 0.05:
        return '*'
    return '-'


def draw_bracket(ax, x1, x2, y_top, label, color='black', lw=0.9, fontsize=9):
    """Draw a bracket between x1 and x2 at y_top with a significance label."""
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
    """Load details CSVs for all three groups and return a combined DataFrame."""
    base_dir = os.path.join(
        os.path.dirname(__file__), "..",
        "resource_collection_all_4_22",
        "resource_collection_analysis_tol_0.75m_freq_2s_visited_75_radius_0.75m_arena_14_14",
        distribution, "48_resources",
    )
    frames = []
    for group, label in [(baseline, BASELINE_LABEL),
                         (algorithm, ALGORITHM_LABEL),
                         (new, NEW_LABEL)]:
        csv_path = os.path.join(base_dir, group, "resource_collection_analysis_details.csv")
        df = pd.read_csv(csv_path)
        df = df[df['milestone_percent'].round(1).isin([float(m) for m in range(10, 101, 10)])].copy()
        df['milestone_percent'] = df['milestone_percent'].round(0).astype(int)
        df['Experiment Type'] = label
        frames.append(df[['milestone_percent', 'cumulative_time', 'Experiment Type']])
    return pd.concat(frames, ignore_index=True)


def plot_distribution(distribution: str,
                      distribution_label: str,
                      baseline: str = "baseline",
                      algorithm: str = "algorithm",
                      new: str = "new") -> None:
    """Create a box-plot with significance brackets for a single distribution."""
    data = load_all_data(distribution, baseline, algorithm, new)

    sns.set_theme(style='whitegrid')
    sns.set_context('paper', font_scale=2.2)
    plt.rcParams.update({
        'font.size': 18,
        'axes.titlesize': 16,
        'axes.labelsize': 14,
        'xtick.labelsize': 20,
        'ytick.labelsize': 20,
        'legend.fontsize': 14,
    })

    fig, ax = plt.subplots(figsize=(14, 7))

    hue_order = [BASELINE_LABEL, ALGORITHM_LABEL, NEW_LABEL]
    sns.boxplot(
        data=data,
        x='milestone_percent',
        y='cumulative_time',
        hue='Experiment Type',
        palette=PALETTE,
        hue_order=hue_order,
        width=0.45,
        fliersize=3,
        linewidth=1.1,
        dodge=True,
        showfliers=True,
        ax=ax,
    )

    ax.set_ylabel('Time (seconds)', fontsize=24)
    ax.set_xlabel('Percent of Resources Collected', fontsize=24)

    for artist in ax.artists:
        artist.set_edgecolor('black')
        artist.set_linewidth(0.8)

    # ---- significance brackets ----
    milestones = sorted(data['milestone_percent'].unique())
    box_width = 0.45 / 3          # matches width/n_groups
    offsets = np.array([-box_width, 0.0, box_width])  # CPFA, GCFA, ARPC

    y_max_global = data['cumulative_time'].max()
    ax.set_ylim(top=y_max_global * 1.28)

    for tick_idx, ms in enumerate(milestones):
        sub = data[data['milestone_percent'] == ms]
        cpfa_vals  = sub[sub['Experiment Type'] == BASELINE_LABEL]['cumulative_time'].values
        gcfa_vals  = sub[sub['Experiment Type'] == ALGORITHM_LABEL]['cumulative_time'].values
        ARPC_vals = sub[sub['Experiment Type'] == NEW_LABEL]['cumulative_time'].values

        _, p_cpfa  = ttest_ind(cpfa_vals,  ARPC_vals, equal_var=False)
        _, p_gcfa  = ttest_ind(gcfa_vals,  ARPC_vals, equal_var=False)

        y_base = sub['cumulative_time'].max()
        gap    = (ax.get_ylim()[1] - ax.get_ylim()[0]) * 0.055

        x_cpfa  = tick_idx + offsets[0]
        x_gcfa  = tick_idx + offsets[1]
        x_ARPC = tick_idx + offsets[2]

        # Lower bracket: GCFA vs ARPC
        draw_bracket(ax, x_gcfa, x_ARPC, y_base + gap * 0.6,
                     sig_label(p_gcfa), fontsize=14)
        # Upper bracket: CPFA vs ARPC
        draw_bracket(ax, x_cpfa, x_ARPC, y_base + gap * 1.7,
                     sig_label(p_cpfa), fontsize=14)

    # ---- two-legend layout ----
    ax.legend(loc='upper left', borderaxespad=0.5, title='')

    plt.tight_layout()

    out_dir = os.path.dirname(__file__)
    out_prefix = os.path.join(out_dir, f"ttest_boxplot_milestones_{distribution}")
    fig.savefig(f"{out_prefix}.png", dpi=300, bbox_inches='tight')
    fig.savefig(f"{out_prefix}.pdf", bbox_inches='tight')
    fig.savefig(f"{out_prefix}.svg", bbox_inches='tight')
    plt.close(fig)
    print(f"Saved {out_prefix}.{{png,pdf,svg}}")


def load_milestone_times(group: str, milestone: float, distribution: str) -> pd.Series:
    """Return cumulative_time at a given milestone percent for each seed in baseline or new."""
    BASE_DIR = os.path.join(
        os.path.dirname(__file__),
        "..",
        "resource_collection_all_4_22",
        "resource_collection_analysis_tol_0.75m_freq_2s_visited_75_radius_0.5m_arena_14_14",
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
    new_label = "ARPC"
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
    latex_df["baseline_mean"] = latex_df["baseline_mean"].apply(lambda x: f"{x:.2f}")
    latex_df["algorithm_mean"] = latex_df["algorithm_mean"].apply(lambda x: f"{x:.2f}")
    latex_df["new_mean"] = latex_df["new_mean"].apply(lambda x: f"{x:.2f}")
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

    # Generate box-plot with significance annotations for this distribution
    plot_distribution(distribution, distribution_label, baseline, algorithm, new)


if __name__ == "__main__":
    main(distribution="cluster_distribution")
    main(distribution="powerlaw_distribution")
    main(distribution="random_distribution")

