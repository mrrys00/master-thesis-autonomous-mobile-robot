
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union
import json
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def _safe_get(d: dict, keys: Iterable[str], default=None):
    for k in keys:
        if isinstance(d, dict) and k in d:
            return d[k]
    return default

def _best_method_name(ex: dict) -> str:
    return _safe_get(
        ex,
        ["method","planner","strategy","algorithm","name","variant"],
        default="unknown"
    )

def _ensure_list(x):
    if x is None:
        return []
    if isinstance(x, list):
        return x
    return [x]

def _coverage_from_occupancy_grid(grid: np.ndarray) -> Tuple[float,int,int]:
    total = grid.size
    known = np.count_nonzero(grid != -1)
    cov = known / total if total > 0 else 0.0
    return cov, int(known), int(total)

def _coverage_series_from_maps(maps: List[Any]) -> Tuple[List[float], List[int], List[int]]:
    coverages, known_counts, totals = [], [], []
    for m in maps:
        arr = np.array(m)
        cov, known, total = _coverage_from_occupancy_grid(arr)
        coverages.append(cov)
        known_counts.append(known)
        totals.append(total)
    return coverages, known_counts, totals

def extract_timeseries_from_json(
    path: str,
    field_map: Optional[Dict[str,str]] = None,
    default_dt: float = 1.0
) -> pd.DataFrame:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    runs = None
    if isinstance(data, list):
        runs = data
    elif isinstance(data, dict):
        candidate_keys = [
            "all_experiments_data", "experiments", "runs", "episodes", "results"
        ]
        if field_map and "runs_key" in field_map:
            candidate_keys = [field_map["runs_key"]] + candidate_keys
        for k in candidate_keys:
            if k in data and isinstance(data[k], list):
                runs = data[k]
                break
        if runs is None:
            runs = [data]
    else:
        raise ValueError("Unsupported JSON structure: expected list or dict at top-level.")

    rows = []
    for ridx, ex in enumerate(runs):
        method = _best_method_name(ex)
        run_id = _safe_get(ex, ["run_id","id","episode","name"], default=f"run_{ridx}")

        times = _safe_get(ex, ["timestamps","time","t","time_series_t"])
        preds = _safe_get(ex, ["pred_time_remaining","pred_remaining","pred_tr","y_pred","est_remaining"])
        actuals = _safe_get(ex, ["actual_time_remaining","actual_remaining","actual_tr","y_true","gt_remaining"])
        maps = _safe_get(ex, ["maps","occupancy_maps","map_series"])

        if field_map:
            times = ex.get(field_map.get("times_key","timestamps"), times)
            preds = ex.get(field_map.get("pred_key","pred_time_remaining"), preds)
            actuals = ex.get(field_map.get("actual_key","actual_time_remaining"), actuals)
            maps = ex.get(field_map.get("maps_key","maps"), maps)

        times = _ensure_list(times)
        preds = _ensure_list(preds)
        actuals = _ensure_list(actuals)
        maps = _ensure_list(maps)

        coverages, knowns, totals = ([], [], [])
        if len(maps) > 0:
            coverages, knowns, totals = _coverage_series_from_maps(maps)

        lengths = [len(x) for x in [times, preds, actuals, coverages] if len(x) > 0]
        series_len = max(lengths) if lengths else 0

        if not times or len(times) == 0:
            times = [i * default_dt for i in range(series_len)]
        elif len(times) < series_len:
            if len(times) > 0:
                start = times[-1] + default_dt
            else:
                start = 0.0
            extra = [start + i * default_dt for i in range(series_len - len(times))]
            times = list(times) + extra

        def _pad_list(L, n, padval=np.nan):
            L = list(L)
            if len(L) >= n:
                return L[:n]
            return L + [padval] * (n - len(L))

        preds   = _pad_list(preds, series_len, np.nan)
        actuals = _pad_list(actuals, series_len, np.nan)
        if coverages:
            coverages = _pad_list(coverages, series_len, np.nan)
            knowns    = _pad_list(knowns, series_len, np.nan)
            totals    = _pad_list(totals, series_len, np.nan)
        else:
            coverages = [np.nan] * series_len
            knowns    = [np.nan] * series_len
            totals    = [np.nan] * series_len

        if all([isinstance(a, float) and np.isnan(a) for a in actuals]) and len(times) > 0:
            max_t = max(times)
            actuals = [max_t - t for t in times]

        for i in range(series_len):
            rows.append({
                "run_id": run_id,
                "method": method,
                "t": float(times[i]) if times else np.nan,
                "pred_time_remaining": float(preds[i]) if not pd.isna(preds[i]) else np.nan,
                "actual_time_remaining": float(actuals[i]) if not pd.isna(actuals[i]) else np.nan,
                "coverage": float(coverages[i]) if not pd.isna(coverages[i]) else np.nan,
                "known_cells": int(knowns[i]) if not pd.isna(knowns[i]) else np.nan,
                "total_cells": int(totals[i]) if not pd.isna(totals[i]) else np.nan,
            })

    df = pd.DataFrame(rows)
    if not df.empty:
        df["t"] = pd.to_numeric(df["t"], errors="coerce")
        for col in ["pred_time_remaining","actual_time_remaining","coverage"]:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df

def compute_eta_time(df: pd.DataFrame, clamp: bool = True) -> pd.DataFrame:
    out = df.dropna(subset=["pred_time_remaining","actual_time_remaining"]).copy()
    out["abs_error"] = (out["pred_time_remaining"] - out["actual_time_remaining"]).abs()
    rel_err = out["abs_error"] / out["actual_time_remaining"].replace({0: np.nan})
    out["eta_percent"] = 100.0 * (1.0 - rel_err)
    if clamp:
        out["eta_percent"] = out["eta_percent"].clip(lower=0.0, upper=100.0)
    return out

def summarize_eta_by_method(eta_df: pd.DataFrame, threshold: float = 0.60) -> pd.DataFrame:
    # Guarantee required fields
    if "abs_error" not in eta_df.columns or "eta_percent" not in eta_df.columns:
        eta_df = compute_eta_time(eta_df)

    g = eta_df.groupby("method", dropna=False)

    s_count = g["eta_percent"].count().rename("count_rows")
    s_mean  = g["eta_percent"].mean().rename("mean_eta")
    s_med   = g["eta_percent"].median().rename("median_eta")
    s_std   = g["eta_percent"].std().rename("std_eta")

    # Use Series.groupby to ensure a Series result; avoid DataFrame.apply
    s_pct = g["eta_percent"].apply(lambda s: (s >= 100.0*threshold).mean() * 100.0)
    s_pct.name = "pct_ge_thresh"

    s_mae  = g["abs_error"].mean().rename("mae")
    s_rmse = g["abs_error"].apply(lambda s: float(np.sqrt(np.mean(np.square(s)))))
    s_rmse.name = "rmse"

    valid = eta_df[eta_df["actual_time_remaining"] != 0]
    s_mape = (valid["abs_error"] / valid["actual_time_remaining"].abs()) \
                .groupby(valid["method"], dropna=False) \
                .mean() \
                .mul(100.0)
    s_mape.name = "mape"

    summary = pd.concat([s_count, s_mean, s_med, s_std, s_pct, s_mae, s_rmse, s_mape], axis=1)

    if "count_rows" in summary.columns:
        try:
            summary["count_rows"] = summary["count_rows"].astype("Int64")
        except Exception:
            pass

    return summary.reset_index().sort_values("mean_eta", ascending=False)

def summarize_runs(df: pd.DataFrame) -> pd.DataFrame:
    def last_non_na(series):
        return series.dropna().iloc[-1] if series.dropna().size > 0 else np.nan

    g = df.sort_values(["run_id","t"]).groupby(["run_id","method"], as_index=False)
    total_time = g["t"].max().rename(columns={"t":"total_time"})
    final_cov  = g.agg(final_coverage=("coverage", last_non_na),
                       final_known_cells=("known_cells", last_non_na),
                       total_cells=("total_cells", last_non_na))
    out = pd.merge(total_time, final_cov, on=["run_id","method"], how="left")
    return out


def plot_eta_boxplot_by_method(eta_df: pd.DataFrame, figsize: Tuple[int,int]=(8,5), min_points: int = 1):
    """
    Boxplot of eta_percent grouped by method.
    Robust to empty groups and NaN method names; guarantees labels and data lengths match.
    """
    df = eta_df.dropna(subset=["eta_percent"]).copy()
    if df.empty:
        print("No data to plot.")
        return

    grouped = list(df.groupby("method", dropna=False))
    data, labels = [], []
    for method, g in grouped:
        vals = g["eta_percent"].values
        if vals.size >= min_points:
            label = "unknown" if (pd.isna(method)) else str(method)
            data.append(vals)
            labels.append(label)

    if not data:
        print("No groups with sufficient data to plot.")
        return

    plt.figure(figsize=figsize)
    plt.boxplot(data, labels=labels, showmeans=True)
    plt.title("Eta (time-remaining accuracy) by Method")
    plt.ylabel("eta_percent")
    plt.xlabel("method")
    plt.grid(True, axis="y")
    plt.xticks(rotation=15)
    plt.tight_layout()
    plt.show()
def plot_pred_vs_actual_scatter(eta_df: pd.DataFrame, figsize: Tuple[int,int]=(6,6)):
    plt.figure(figsize=figsize)
    plt.scatter(eta_df["actual_time_remaining"], eta_df["pred_time_remaining"], s=8, alpha=0.6)
    lim = np.nanmax([eta_df["actual_time_remaining"].max(), eta_df["pred_time_remaining"].max()])
    lim = float(lim) if not np.isnan(lim) else 1.0
    xs = np.linspace(0, lim, 100)
    plt.plot(xs, xs, linewidth=1)
    plt.xlabel("Actual time remaining")
    plt.ylabel("Predicted time remaining")
    plt.title("Predicted vs Actual Time Remaining (all methods)")
    plt.grid(True)
    plt.show()

def plot_error_histogram(eta_df: pd.DataFrame, bins: int = 50, figsize: Tuple[int,int]=(8,5)):
    plt.figure(figsize=figsize)
    plt.hist(eta_df["abs_error"].dropna().values, bins=bins)
    plt.xlabel("|pred - actual|")
    plt.ylabel("count")
    plt.title("Absolute Error Distribution")
    plt.grid(True, axis="y")
    plt.show()

def plot_calibration_curve(eta_df: pd.DataFrame, num_bins: int = 20, figsize: Tuple[int,int]=(7,5)):
    df = eta_df.dropna(subset=["pred_time_remaining","actual_time_remaining"]).copy()
    if df.empty:
        print("No data to plot.")
        return
    df["bin"] = pd.qcut(df["pred_time_remaining"], q=min(num_bins, df.shape[0]), duplicates="drop")
    by = df.groupby("bin", observed=True).agg(
        pred_mean=("pred_time_remaining","mean"),
        actual_mean=("actual_time_remaining","mean"),
        count=("pred_time_remaining","count")
    ).reset_index(drop=True)

    plt.figure(figsize=figsize)
    plt.plot(by["pred_mean"], by["actual_mean"], marker="o")
    lim = float(np.nanmax([by["pred_mean"].max(), by["actual_mean"].max()]))
    xs = np.linspace(0, lim, 100)
    plt.plot(xs, xs, linewidth=1)
    plt.xlabel("Binned mean predicted time remaining")
    plt.ylabel("Binned mean actual time remaining")
    plt.title("Calibration Curve (Predicted vs Actual)")
    plt.grid(True)
    plt.show()

def plot_eta_by_coverage_bin(eta_df: pd.DataFrame, num_bins: int = 10, figsize: Tuple[int,int]=(8,5)):
    df = eta_df.dropna(subset=["coverage","eta_percent"]).copy()
    if df.empty:
        print("No data to plot.")
        return
    df["cov_bin"] = pd.cut(df["coverage"], bins=num_bins)
    by = df.groupby("cov_bin", observed=True)["eta_percent"].mean().reset_index()
    centers = by["cov_bin"].apply(lambda b: (b.left + b.right)/2 if pd.notna(b) else np.nan).astype(float)

    plt.figure(figsize=figsize)
    plt.plot(centers, by["eta_percent"], marker="o")
    plt.xlabel("Coverage (bin centers)")
    plt.ylabel("Mean eta_percent")
    plt.title("Eta vs Coverage")
    plt.grid(True)
    plt.show()

def plot_runtime_cdf(df: pd.DataFrame, figsize: Tuple[int,int]=(8,5)):
    plt.figure(figsize=figsize)
    for method, g in df.groupby("method"):
        vals = np.sort(g["total_time"].dropna().values)
        if vals.size == 0:
            continue
        y = np.arange(1, vals.size + 1) / vals.size
        plt.step(vals, y, where="post", label=str(method))
    plt.xlabel("Total mapping time")
    plt.ylabel("CDF")
    plt.title("Total Mapping Time CDF by Method")
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_coverage_over_time_for_run(df: pd.DataFrame, run_id: str, figsize: Tuple[int,int]=(8,5)):
    g = df[df["run_id"] == run_id].sort_values("t")
    if g.empty:
        print(f"No data for run_id={run_id}")
        return
    plt.figure(figsize=figsize)
    plt.plot(g["t"], g["coverage"])
    plt.xlabel("time [s] (assumed)")
    plt.ylabel("coverage")
    plt.title(f"Coverage over Time (run_id={run_id})")
    plt.grid(True)
    plt.show()

def hypothesis_check_report(
    path: str,
    field_map: Optional[Dict[str,str]] = None,
    threshold: float = 0.60
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    ts = extract_timeseries_from_json(path, field_map=field_map)
    eta_df = compute_eta_time(ts)
    summary = summarize_eta_by_method(eta_df, threshold=threshold)
    return eta_df, summary
