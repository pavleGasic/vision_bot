#!/usr/bin/env python3
"""
analyze_benchmark.py

Usage:
    python3 scripts/analyze_benchmark.py --input /tmp/visionbot_benchmark --output scripts/results

Expects CSV files named {model_name}_{timestamp}.csv in --input directory.
Generates plots and a summary table in --output directory.
"""

import argparse
import os
import re
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

# ── helpers ──────────────────────────────────────────────────────────────────

MODEL_COLORS = {
    "yolov8n":  "#4C9BE8",
    "yolov8s":  "#2E6DB5",
    "yolov11n": "#E8874C",
    "yolov11s": "#B55A2E",
}

def color(model):
    return MODEL_COLORS.get(model, "#888888")


def load_csvs(input_dir: str) -> pd.DataFrame:
    frames = []
    pattern = re.compile(r"^(.+?)_\d+\.csv$")
    for fname in os.listdir(input_dir):
        if not fname.endswith(".csv"):
            continue
        m = pattern.match(fname)
        if not m:
            continue
        run_model = m.group(1)
        path = os.path.join(input_dir, fname)
        df = pd.read_csv(path)
        # run index: count how many files we've seen for this model so far
        df["run"] = sum(1 for f in frames if f["model_name"].iloc[0] == run_model)
        frames.append(df)
    if not frames:
        raise FileNotFoundError(f"No CSV files found in {input_dir}")
    return pd.concat(frames, ignore_index=True)


def per_run_metrics(df: pd.DataFrame) -> pd.DataFrame:
    """Compute recall, precision, avg_latency per model per run."""
    records = []
    for (model, run), g in df.groupby(["model_name", "run"]):
        visits        = len(g)
        detected      = (g["detected"] == True).sum()  # noqa: E712
        recall        = detected / visits if visits > 0 else 0.0

        # FP: any visit where fp_class is non-empty and detected=False
        # (model reported wrong class, never reported right class)
        fp_visits     = ((g["fp_class"].notna()) & (g["fp_class"] != "") & (g["detected"] == False)).sum()  # noqa: E712
        tp_visits     = detected
        precision     = tp_visits / (tp_visits + fp_visits) if (tp_visits + fp_visits) > 0 else 1.0

        avg_latency   = g["inference_ms_avg"].mean()

        records.append({
            "model":     model,
            "run":       run,
            "recall":    recall,
            "precision": precision,
            "latency_ms": avg_latency,
            "visits":    visits,
            "detected":  int(detected),
        })
    return pd.DataFrame(records)


def aggregate_runs(run_metrics: pd.DataFrame) -> pd.DataFrame:
    """Mean ± std across runs per model."""
    agg = run_metrics.groupby("model").agg(
        recall_mean=("recall",     "mean"),
        recall_std= ("recall",     "std"),
        precision_mean=("precision","mean"),
        precision_std= ("precision","std"),
        latency_mean=  ("latency_ms","mean"),
        latency_std=   ("latency_ms","std"),
        n_runs=        ("run",      "count"),
    ).reset_index()
    agg["recall_std"]    = agg["recall_std"].fillna(0)
    agg["precision_std"] = agg["precision_std"].fillna(0)
    agg["latency_std"]   = agg["latency_std"].fillna(0)
    return agg


def per_object_recall(df: pd.DataFrame) -> pd.DataFrame:
    """Recall per (model, object) averaged across runs."""
    records = []
    for (model, obj), g in df.groupby(["model_name", "gt_object_id"]):
        visits   = len(g)
        detected = (g["detected"] == True).sum()  # noqa: E712
        records.append({
            "model":   model,
            "object":  obj,
            "recall":  detected / visits if visits > 0 else 0.0,
            "visits":  visits,
        })
    return pd.DataFrame(records)


# ── plots ─────────────────────────────────────────────────────────────────────

def plot_accuracy_vs_latency(agg: pd.DataFrame, out_dir: str):
    fig, ax = plt.subplots(figsize=(7, 5))

    for _, row in agg.iterrows():
        c = color(row["model"])
        ax.errorbar(
            row["latency_mean"], row["recall_mean"],
            xerr=row["latency_std"], yerr=row["recall_std"],
            fmt="o", color=c, markersize=10, capsize=4, linewidth=1.5,
            label=row["model"],
        )
        ax.annotate(
            row["model"],
            (row["latency_mean"], row["recall_mean"]),
            textcoords="offset points", xytext=(8, 4), fontsize=9,
        )

    ax.set_xlabel("Avg inference latency (ms)", fontsize=11)
    ax.set_ylabel("Recall", fontsize=11)
    ax.set_title("Accuracy vs Latency — YOLO model comparison (RPi5)", fontsize=12)
    ax.set_ylim(0, 1.05)
    ax.grid(True, linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "accuracy_vs_latency.png"), dpi=150)
    plt.close(fig)
    print("  Saved: accuracy_vs_latency.png")


def plot_per_object_heatmap(obj_recall: pd.DataFrame, out_dir: str):
    models  = sorted(obj_recall["model"].unique())
    objects = sorted(obj_recall["object"].unique())

    matrix = np.zeros((len(objects), len(models)))
    for i, obj in enumerate(objects):
        for j, model in enumerate(models):
            row = obj_recall[(obj_recall["object"] == obj) & (obj_recall["model"] == model)]
            matrix[i, j] = row["recall"].values[0] if len(row) > 0 else np.nan

    fig, ax = plt.subplots(figsize=(len(models) * 1.6 + 1, len(objects) * 0.55 + 1.5))
    im = ax.imshow(matrix, vmin=0, vmax=1, cmap="RdYlGn", aspect="auto")

    ax.set_xticks(range(len(models)));   ax.set_xticklabels(models, fontsize=10)
    ax.set_yticks(range(len(objects))); ax.set_yticklabels(objects, fontsize=9)
    ax.set_title("Per-object recall by model", fontsize=12)

    for i in range(len(objects)):
        for j in range(len(models)):
            val = matrix[i, j]
            if not np.isnan(val):
                ax.text(j, i, f"{val:.2f}", ha="center", va="center",
                        fontsize=9, color="black" if 0.3 < val < 0.8 else "white")

    fig.colorbar(im, ax=ax, label="Recall")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "per_object_recall_heatmap.png"), dpi=150)
    plt.close(fig)
    print("  Saved: per_object_recall_heatmap.png")


def plot_latency_boxplot(df: pd.DataFrame, out_dir: str):
    models = sorted(df["model_name"].unique())
    data   = [df[df["model_name"] == m]["inference_ms_avg"].dropna().values for m in models]

    fig, ax = plt.subplots(figsize=(len(models) * 1.4 + 1, 5))
    bp = ax.boxplot(data, patch_artist=True, medianprops={"color": "black", "linewidth": 2})

    for patch, model in zip(bp["boxes"], models):
        patch.set_facecolor(color(model))
        patch.set_alpha(0.8)

    ax.set_xticks(range(1, len(models) + 1))
    ax.set_xticklabels(models, fontsize=10)
    ax.set_ylabel("Inference latency (ms)", fontsize=11)
    ax.set_title("Inference latency distribution per model", fontsize=12)
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "latency_boxplot.png"), dpi=150)
    plt.close(fig)
    print("  Saved: latency_boxplot.png")


def plot_fp_analysis(df: pd.DataFrame, out_dir: str):
    """Bar chart: how many visits had a false positive per model."""
    models = sorted(df["model_name"].unique())
    fp_counts = []
    for m in models:
        g = df[df["model_name"] == m]
        fp = ((g["fp_class"].notna()) & (g["fp_class"] != "")).sum()
        fp_counts.append(fp)

    fig, ax = plt.subplots(figsize=(len(models) * 1.4 + 1, 4))
    bars = ax.bar(models, fp_counts, color=[color(m) for m in models], alpha=0.85)
    ax.bar_label(bars, padding=3, fontsize=10)
    ax.set_ylabel("Visits with FP detection", fontsize=11)
    ax.set_title("False positive visits per model", fontsize=12)
    ax.grid(True, axis="y", linestyle="--", alpha=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fp_analysis.png"), dpi=150)
    plt.close(fig)
    print("  Saved: fp_analysis.png")


def save_summary_table(agg: pd.DataFrame, out_dir: str):
    path = os.path.join(out_dir, "summary.csv")
    agg.to_csv(path, index=False, float_format="%.4f")
    print("  Saved: summary.csv")

    print("\n── Summary ──────────────────────────────────────────────────────")
    print(f"{'Model':<12} {'Recall':>8} {'±':>4} {'Precision':>10} {'±':>4} {'Latency ms':>12} {'±':>6}  Runs")
    print("─" * 70)
    for _, r in agg.iterrows():
        print(f"{r['model']:<12} {r['recall_mean']:>8.3f} {r['recall_std']:>4.3f} "
              f"{r['precision_mean']:>10.3f} {r['precision_std']:>4.3f} "
              f"{r['latency_mean']:>12.1f} {r['latency_std']:>6.1f}  {int(r['n_runs'])}")


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input",  required=True, help="Directory with CSV benchmark results")
    parser.add_argument("--output", required=True, help="Directory to write plots and summary")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    print(f"Loading CSVs from: {args.input}")
    df = load_csvs(args.input)
    print(f"  Loaded {len(df)} rows from {df['model_name'].nunique()} model(s), "
          f"{df.groupby(['model_name','run']).ngroups} run(s) total\n")

    run_metrics = per_run_metrics(df)
    agg         = aggregate_runs(run_metrics)
    obj_recall  = per_object_recall(df)

    print("Generating plots...")
    plot_accuracy_vs_latency(agg, args.output)
    plot_per_object_heatmap(obj_recall, args.output)
    plot_latency_boxplot(df, args.output)
    plot_fp_analysis(df, args.output)
    save_summary_table(agg, args.output)

    print(f"\nDone. Results in: {args.output}")


if __name__ == "__main__":
    main()
