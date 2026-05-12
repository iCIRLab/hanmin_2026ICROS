#!/usr/bin/env python3
"""Create a compact baseline-vs-CM CSV summary from controller logs."""

import argparse
import csv
import math
import os
import sys


def read_rows(path):
    if not path or not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def values(rows, key):
    out = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            out.append(value)
    return out


def mean(items):
    return sum(items) / len(items) if items else float("nan")


def last(items):
    return items[-1] if items else float("nan")


def summarize(label, controller_csv):
    rows = read_rows(controller_csv)
    posture = values(rows, "posture_error")
    xy = values(rows, "xy_error")
    yaw = values(rows, "target_yaw_error")
    base_sat = values(rows, "base_saturated")
    arm_sat = values(rows, "arm_saturated")
    object_xy = values(rows, "object_drift_xy")
    object_z = values(rows, "object_drift_z")
    object_yaw = values(rows, "object_drift_yaw")
    return {
        "mode": label,
        "controller_csv": controller_csv,
        "samples": len(rows),
        "avg_posture_error": mean(posture),
        "max_posture_error": max(posture) if posture else float("nan"),
        "final_posture_error": last(posture),
        "avg_follower_base_error": mean(xy),
        "final_follower_base_error": last(xy),
        "avg_yaw_alignment_error": mean([abs(x) for x in yaw]),
        "final_yaw_alignment_error": abs(last(yaw)) if yaw else float("nan"),
        "base_saturation_ratio": mean(base_sat),
        "arm_saturation_ratio": mean(arm_sat),
        "final_object_drift_xy": last(object_xy),
        "final_object_drift_z": last(object_z),
        "final_object_drift_yaw": last(object_yaw),
    }


def seed_costs(seed_comparison_csv):
    rows = read_rows(seed_comparison_csv)
    out = {"baseline": float("nan"), "cm_seed": float("nan")}
    for row in rows:
        try:
            cost = float(row.get("total_cost", row.get("cost", "nan")))
        except ValueError:
            continue
        seed_type = row.get("seed_type", "")
        if "baseline" in seed_type:
            out["baseline"] = cost
        elif "cm" in seed_type:
            out["cm_seed"] = cost
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-controller", required=True)
    parser.add_argument("--cm-controller", required=True)
    parser.add_argument("--seed-comparison", default="/tmp/mm_verify_logs/figure2_seed_comparison.csv")
    parser.add_argument("--output", default="/tmp/mm_verify_logs/baseline_vs_cm_summary.csv")
    args = parser.parse_args()

    costs = seed_costs(args.seed_comparison)
    rows = [
        summarize("baseline", args.baseline_controller),
        summarize("cm_seed", args.cm_controller),
    ]
    rows[0]["seed_cost"] = costs["baseline"]
    rows[1]["seed_cost"] = costs["cm_seed"]
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print("wrote", args.output)
    for row in rows:
        print(
            "%s samples=%d avg_posture=%.4f final_posture=%.4f avg_base=%.4f sat(base/arm)=%.3f/%.3f drift_xy=%.4f"
            % (
                row["mode"],
                row["samples"],
                row["avg_posture_error"],
                row["final_posture_error"],
                row["avg_follower_base_error"],
                row["base_saturation_ratio"],
                row["arm_saturation_ratio"],
                row["final_object_drift_xy"],
            )
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
