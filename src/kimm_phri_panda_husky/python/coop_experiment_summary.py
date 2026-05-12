#!/usr/bin/env python3
"""Summarize cooperative transport CSV logs for paper/demo iteration."""

import argparse
import csv
import math
import os


def read_rows(path):
    if not path or not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def floats(rows, key):
    out = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            out.append(value)
    return out


def stat_line(name, values):
    if not values:
        return "%s: n=0" % name
    mean = sum(values) / len(values)
    return "%s: n=%d mean=%.6f min=%.6f max=%.6f" % (
        name,
        len(values),
        mean,
        min(values),
        max(values),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="")
    parser.add_argument("--reference", default="")
    parser.add_argument("--cm", default="")
    args = parser.parse_args()

    scene = read_rows(args.scene)
    reference = read_rows(args.reference)
    cm = read_rows(args.cm)

    print("scene_csv:", args.scene or "none")
    print(stat_line("object_drift_xy", floats(scene, "object_drift_xy")))
    print(stat_line("object_drift_z", floats(scene, "object_drift_z")))
    print(stat_line("object_drift_yaw", floats(scene, "object_drift_yaw")))
    print(stat_line("left_drift_xy", floats(scene, "left_drift_xy")))
    print(stat_line("right_drift_xy", floats(scene, "right_drift_xy")))

    print("reference_csv:", args.reference or "none")
    print(stat_line("metric_mu_proxy", floats(reference, "metric_mu_proxy")))
    print(stat_line("metric_F_proxy", floats(reference, "metric_F_proxy")))
    print(stat_line("metric_B_proxy", floats(reference, "metric_B_proxy")))
    print(stat_line("metric_cost_proxy", floats(reference, "metric_cost_proxy")))

    print("cm_csv:", args.cm or "none")
    print(stat_line("mu", floats(cm, "mu")))
    print(stat_line("F", floats(cm, "F")))
    print(stat_line("B", floats(cm, "B")))
    print(stat_line("cost", floats(cm, "cost")))


if __name__ == "__main__":
    main()
