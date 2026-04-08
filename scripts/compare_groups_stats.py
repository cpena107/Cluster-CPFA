#!/usr/bin/env python3

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, List, Tuple

import pandas as pd
from scipy.stats import mannwhitneyu


DEFAULT_DISTRIBUTIONS = [
    "random_distribution",
    "powerlaw_distribution",
    "cluster_distribution",
]


@dataclass(frozen=True)
class ComparisonSpec:
    left: str
    right: str

    @property
    def label(self) -> str:
        return f"{self.left}_vs_{self.right}"


def parse_pairs(raw_pairs: Iterable[str]) -> List[ComparisonSpec]:
    pairs: List[ComparisonSpec] = []
    for raw in raw_pairs:
        if ":" not in raw:
            raise ValueError(f"Invalid pair '{raw}'. Expected format left:right")
        left, right = [item.strip() for item in raw.split(":", 1)]
        if not left or not right:
            raise ValueError(f"Invalid pair '{raw}'. Both group names are required")
        pairs.append(ComparisonSpec(left=left, right=right))
    if not pairs:
        raise ValueError("At least one comparison pair must be provided")
    return pairs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run Mann-Whitney U tests on resource collection details CSVs across "
            "distribution/resource/milestone slices."
        )
    )
    parser.add_argument(
        "--analysis-dir",
        required=True,
        help="Path to resource_collection_analysis_* directory",
    )
    parser.add_argument(
        "--resources",
        type=int,
        nargs="+",
        default=[16, 32, 48, 64, 80],
        help="Resource counts to include (default: 16 32 48 64 80)",
    )
    parser.add_argument(
        "--milestones",
        type=float,
        nargs="+",
        default=[25, 50, 75, 100],
        help="Milestone percentages to include (default: 25 50 75 100)",
    )
    parser.add_argument(
        "--pairs",
        nargs="+",
        default=["new:baseline", "new:algorithm"],
        help="Comparison pairs in format left:right (default: new:baseline new:algorithm)",
    )
    parser.add_argument(
        "--alternative",
        choices=["two-sided", "less", "greater"],
        default="two-sided",
        help="Alternative hypothesis for Mann-Whitney U (default: two-sided)",
    )
    parser.add_argument(
        "--output",
        default="stat_tests_mannwhitney.csv",
        help="Output CSV filename (written inside --analysis-dir unless absolute)",
    )
    return parser.parse_args()


def load_details_csv(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required_columns = {"milestone_percent", "cumulative_time"}
    missing = required_columns.difference(df.columns)
    if missing:
        raise ValueError(f"Missing required columns {sorted(missing)} in {csv_path}")

    df = df[["milestone_percent", "cumulative_time"]].copy()
    df["milestone_percent"] = pd.to_numeric(df["milestone_percent"], errors="coerce")
    df["cumulative_time"] = pd.to_numeric(df["cumulative_time"], errors="coerce")
    df = df.dropna(subset=["milestone_percent", "cumulative_time"])
    return df


def sample_for_group(
    analysis_dir: Path,
    distribution: str,
    resource_count: int,
    group_name: str,
    milestone: float,
) -> List[float]:
    csv_path = (
        analysis_dir
        / distribution
        / f"{resource_count}_resources"
        / group_name
        / "resource_collection_analysis_details.csv"
    )
    if not csv_path.exists():
        return []

    df = load_details_csv(csv_path)
    subset = df[df["milestone_percent"] == milestone]
    return subset["cumulative_time"].tolist()


def rank_biserial_from_u(u_stat: float, n_left: int, n_right: int) -> float:
    if n_left == 0 or n_right == 0:
        return float("nan")
    return (2.0 * u_stat) / (n_left * n_right) - 1.0


def adjust_pvalues_holm(p_values: List[float]) -> List[float]:
    m = len(p_values)
    if m == 0:
        return []

    indexed = sorted(enumerate(p_values), key=lambda item: item[1])
    adjusted_sorted = [0.0] * m
    running_max = 0.0

    for rank, (_, p_val) in enumerate(indexed):
        factor = m - rank
        candidate = min(1.0, factor * p_val)
        running_max = max(running_max, candidate)
        adjusted_sorted[rank] = running_max

    adjusted = [0.0] * m
    for rank, (original_index, _) in enumerate(indexed):
        adjusted[original_index] = adjusted_sorted[rank]
    return adjusted


def adjust_pvalues_fdr_bh(p_values: List[float]) -> List[float]:
    m = len(p_values)
    if m == 0:
        return []

    indexed = sorted(enumerate(p_values), key=lambda item: item[1])
    adjusted_sorted = [0.0] * m
    running_min = math.inf

    for rank in range(m - 1, -1, -1):
        _, p_val = indexed[rank]
        q_val = ((rank + 1) / m)
        candidate = min(1.0, p_val / q_val)
        running_min = min(running_min, candidate)
        adjusted_sorted[rank] = running_min

    adjusted = [0.0] * m
    for rank, (original_index, _) in enumerate(indexed):
        adjusted[original_index] = adjusted_sorted[rank]
    return adjusted


def add_adjusted_pvalues(results: List[dict]) -> None:
    if not results:
        return
    p_values = [float(row["p_value"]) for row in results]
    holm = adjust_pvalues_holm(p_values)
    fdr = adjust_pvalues_fdr_bh(p_values)
    for row, p_holm, p_fdr in zip(results, holm, fdr):
        row["p_value_holm"] = p_holm
        row["p_value_fdr_bh"] = p_fdr


def run_tests(
    analysis_dir: Path,
    distributions: List[str],
    resources: List[int],
    milestones: List[float],
    pairs: List[ComparisonSpec],
    alternative: str,
) -> Tuple[List[dict], List[dict]]:
    results: List[dict] = []
    skipped: List[dict] = []

    for distribution in distributions:
        for resource_count in resources:
            for milestone in milestones:
                for pair in pairs:
                    left_sample = sample_for_group(
                        analysis_dir,
                        distribution,
                        resource_count,
                        pair.left,
                        milestone,
                    )
                    right_sample = sample_for_group(
                        analysis_dir,
                        distribution,
                        resource_count,
                        pair.right,
                        milestone,
                    )

                    if len(left_sample) < 2 or len(right_sample) < 2:
                        skipped.append(
                            {
                                "distribution": distribution,
                                "resource_count": resource_count,
                                "milestone_percent": milestone,
                                "comparison": pair.label,
                                "reason": "insufficient_samples",
                                "n_left": len(left_sample),
                                "n_right": len(right_sample),
                            }
                        )
                        continue

                    stat = mannwhitneyu(left_sample, right_sample, alternative=alternative)
                    u_value = float(stat.statistic)
                    p_value = float(stat.pvalue)

                    n_left = len(left_sample)
                    n_right = len(right_sample)

                    results.append(
                        {
                            "distribution": distribution,
                            "resource_count": resource_count,
                            "milestone_percent": milestone,
                            "comparison": pair.label,
                            "left_group": pair.left,
                            "right_group": pair.right,
                            "n_left": n_left,
                            "n_right": n_right,
                            "median_left": float(pd.Series(left_sample).median()),
                            "median_right": float(pd.Series(right_sample).median()),
                            "mean_left": float(pd.Series(left_sample).mean()),
                            "mean_right": float(pd.Series(right_sample).mean()),
                            "u_statistic": u_value,
                            "p_value": p_value,
                            "rank_biserial": rank_biserial_from_u(u_value, n_left, n_right),
                            "alternative": alternative,
                        }
                    )

    return results, skipped


def write_csv(rows: List[dict], output_path: Path, fieldnames: List[str]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def resolve_output_path(analysis_dir: Path, output_arg: str) -> Path:
    output_path = Path(output_arg)
    if output_path.is_absolute():
        return output_path
    return analysis_dir / output_path


def main() -> None:
    args = parse_args()
    analysis_dir = Path(args.analysis_dir)
    if not analysis_dir.exists() or not analysis_dir.is_dir():
        raise SystemExit(f"Analysis directory does not exist: {analysis_dir}")

    try:
        pairs = parse_pairs(args.pairs)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    results, skipped = run_tests(
        analysis_dir=analysis_dir,
        distributions=DEFAULT_DISTRIBUTIONS,
        resources=sorted(set(args.resources)),
        milestones=sorted(set(args.milestones)),
        pairs=pairs,
        alternative=args.alternative,
    )
    add_adjusted_pvalues(results)

    output_path = resolve_output_path(analysis_dir, args.output)
    fieldnames = [
        "distribution",
        "resource_count",
        "milestone_percent",
        "comparison",
        "left_group",
        "right_group",
        "n_left",
        "n_right",
        "median_left",
        "median_right",
        "mean_left",
        "mean_right",
        "u_statistic",
        "p_value",
        "p_value_holm",
        "p_value_fdr_bh",
        "rank_biserial",
        "alternative",
    ]
    write_csv(results, output_path, fieldnames)

    if skipped:
        skipped_path = output_path.with_name(output_path.stem + "_skipped.csv")
        write_csv(
            skipped,
            skipped_path,
            [
                "distribution",
                "resource_count",
                "milestone_percent",
                "comparison",
                "reason",
                "n_left",
                "n_right",
            ],
        )
        print(f"Skipped comparisons written to: {skipped_path}")

    print(f"Wrote {len(results)} test rows to: {output_path}")
    if results:
        df = pd.DataFrame(results)
        top = df.sort_values("p_value").head(10)
        print("\nTop 10 smallest p-values:")
        print(
            top[
                [
                    "distribution",
                    "resource_count",
                    "milestone_percent",
                    "comparison",
                    "p_value",
                    "rank_biserial",
                ]
            ].to_string(index=False)
        )


if __name__ == "__main__":
    main()