import argparse
import os
import sys

import pandas as pd


REQUIRED_COLUMNS = [
    "random_seed",
    "milestone_percent",
    "time_interval",
    "cumulative_time",
    "food_distribution",
    "algorithm_mode",
    "num_robots",
    "total_food",
]


def generate_summary(input_file: str, output_file: str) -> None:
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file not found: {input_file}")

    df = pd.read_csv(input_file)

    print(df.groupby("milestone_percent", as_index=False)["time_interval"].agg(
        num_simulations="count",
        mean_time="mean",
        std_time="std",
        median_time="median",
        min_time="min",
        max_time="max",
    ).sort_values("milestone_percent"))

    missing = [column for column in REQUIRED_COLUMNS if column not in df.columns]
    if missing:
        raise ValueError(
            "Input details file is missing required columns: " + ", ".join(missing)
        )

    summary = (
        df.groupby("milestone_percent", as_index=False)["time_interval"]
        .agg(
            num_simulations="count",
            mean_time="mean",
            std_time="std",
            median_time="median",
            min_time="min",
            max_time="max",
        )
        .sort_values("milestone_percent")
    )

    summary = summary[
        [
            "milestone_percent",
            "num_simulations",
            "mean_time",
            "std_time",
            "median_time",
            "min_time",
            "max_time",
        ]
    ]

    for column in ["mean_time", "std_time", "median_time", "min_time", "max_time"]:
        summary[column] = summary[column].round(3)

    output_dir = os.path.dirname(output_file)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    summary.to_csv(output_file, index=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Generate resource_collection_analysis_summary.csv from "
            "resource_collection_analysis_details.csv"
        )
    )
    parser.add_argument("--input_file", required=True, help="Path to details CSV")
    parser.add_argument(
        "--output_file",
        default=None,
        help=(
            "Path to summary CSV. Defaults to "
            "<input_dir>/resource_collection_analysis_summary.csv"
        ),
    )

    args = parser.parse_args()

    output_file = args.output_file
    if output_file is None:
        output_file = os.path.join(
            os.path.dirname(args.input_file),
            "resource_collection_analysis_summary.csv",
        )

    try:
        generate_summary(args.input_file, output_file)
        print(f"Summary generated: {output_file}")
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)
