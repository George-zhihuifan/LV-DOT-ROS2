#!/usr/bin/env python3
import argparse
import csv
import math
from statistics import median


def percentile(values, p):
    if not values:
        return 0.0
    xs = sorted(values)
    if len(xs) == 1:
        return float(xs[0])
    pos = (len(xs) - 1) * p
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return float(xs[lo])
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def to_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs-csv", required=True)
    parser.add_argument("--out-txt", required=True)
    parser.add_argument("--out-md", required=True)
    args = parser.parse_args()

    rows = []
    with open(args.runs_csv, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    total_runs = len(rows)
    preflight_fail_runs = 0
    launch_fail_runs = 0
    runtime_crash_runs = 0
    completed_runs = 0
    pass_runs = 0
    fail_runs = 0

    completed_hits = []
    for row in rows:
        status = (row.get("status", "") or "").strip()
        validation = (row.get("validation", "") or "").strip().upper()
        hit_count = to_int(row.get("hit_count", "0"))

        if status == "preflight_fail":
            preflight_fail_runs += 1
        elif status == "launch_fail":
            launch_fail_runs += 1
        elif status == "runtime_crash":
            runtime_crash_runs += 1
        elif status == "completed":
            completed_runs += 1
            completed_hits.append(hit_count)
            if validation == "PASS":
                pass_runs += 1
            else:
                fail_runs += 1
        else:
            # Unknown status is treated as non-completed failure for safety.
            fail_runs += 1

    # preflight_fail is excluded from crash-rate denominator
    crash_numerator = launch_fail_runs + runtime_crash_runs
    crash_denominator = total_runs - preflight_fail_runs
    crash_rate = (crash_numerator / crash_denominator) if crash_denominator > 0 else 0.0

    if completed_hits:
        mean_hit = sum(completed_hits) / len(completed_hits)
        median_hit = float(median(completed_hits))
        p25_hit = percentile(completed_hits, 0.25)
        p75_hit = percentile(completed_hits, 0.75)
    else:
        mean_hit = 0.0
        median_hit = 0.0
        p25_hit = 0.0
        p75_hit = 0.0

    lines = [
        f"total_runs={total_runs}",
        f"status_preflight_fail={preflight_fail_runs}",
        f"status_launch_fail={launch_fail_runs}",
        f"status_runtime_crash={runtime_crash_runs}",
        f"status_completed={completed_runs}",
        f"completed_pass_runs={pass_runs}",
        f"completed_fail_runs={fail_runs}",
        f"crash_rate={crash_rate:.6f}",
        f"hit_count_mean={mean_hit:.6f}",
        f"hit_count_median={median_hit:.6f}",
        f"hit_count_p25={p25_hit:.6f}",
        f"hit_count_p75={p75_hit:.6f}",
        f"runs_csv={args.runs_csv}",
    ]
    with open(args.out_txt, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    md = []
    md.append("# A5-6 Summary (Status Split)")
    md.append("")
    md.append(f"- total_runs: {total_runs}")
    md.append(f"- status_preflight_fail: {preflight_fail_runs}")
    md.append(f"- status_launch_fail: {launch_fail_runs}")
    md.append(f"- status_runtime_crash: {runtime_crash_runs}")
    md.append(f"- status_completed: {completed_runs}")
    md.append(f"- completed_pass_runs: {pass_runs}")
    md.append(f"- completed_fail_runs: {fail_runs}")
    md.append(f"- crash_rate (exclude preflight_fail): {crash_rate:.6f}")
    md.append(f"- hit_count_mean (completed only): {mean_hit:.6f}")
    md.append(f"- hit_count_median (completed only): {median_hit:.6f}")
    md.append(f"- hit_count_p25 (completed only): {p25_hit:.6f}")
    md.append(f"- hit_count_p75 (completed only): {p75_hit:.6f}")
    md.append("")
    md.append("## Runs")
    md.append("")
    md.append("| run_id | artifact_dir | status | hit_count | validation | crash | log_path |")
    md.append("|---:|---|---|---:|---|---|---|")
    for row in rows:
        md.append(
            f"| {row.get('run_id','')} | {row.get('artifact_dir','')} | {row.get('status','')} | "
            f"{row.get('hit_count','')} | {row.get('validation','')} | {row.get('crash','')} | {row.get('log_path','')} |"
        )

    with open(args.out_md, "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")


if __name__ == "__main__":
    main()
