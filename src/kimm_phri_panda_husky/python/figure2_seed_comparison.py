#!/usr/bin/env python3
"""
Generate Figure 2 seed-comparison artifacts for the ICROS2026 draft.

This script does not modify the MuJoCo scene. It builds a reproducible,
paper-facing schematic from the verified dual-scene keyframe. The CM result
is an environment-adapted CM-like query: it samples feasible follower base and
arm seed candidates, evaluates Zhang-style mu(q) x F(q) x B(q) proxies, and
selects the best candidate. It is not a full offline IK/FK Capability Map yet.
"""

import argparse
import csv
import math
import os
import re
import sys
import xml.etree.ElementTree as ET


HOME_POSTURE = [0.0, 0.0, 0.0, -math.pi / 2.0, 0.0, math.pi / 2.0, -math.pi / 4.0]
PANDA_LIMITS = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
]


def yaw_from_quat(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def norm(values):
    return math.sqrt(sum(v * v for v in values))


def read_keyframe_qpos(scene_xml):
    root = ET.parse(scene_xml).getroot()
    key = root.find("./keyframe/key[@name='coop_start']")
    if key is None:
        raise RuntimeError("coop_start keyframe not found in %s" % scene_xml)
    qpos = [float(x) for x in re.split(r"\s+", key.attrib["qpos"].strip()) if x]
    if len(qpos) < 47:
        raise RuntimeError("unexpected coop_start qpos length: %d" % len(qpos))
    return qpos


def latest_controller_seed(controller_csv):
    if not controller_csv or not os.path.exists(controller_csv):
        return None
    with open(controller_csv) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    row = rows[-1]
    required = ["base_x", "base_y", "base_yaw"] + ["q%d" % i for i in range(1, 8)]
    if any(name not in row for name in required):
        return None
    return {
        "base": [float(row["base_x"]), float(row["base_y"]), float(row["base_yaw"])],
        "q_arm": [float(row["q%d" % i]) for i in range(1, 8)],
        "source": controller_csv,
    }


def latest_cm_table_seed(cm_samples_csv):
    if not cm_samples_csv or not os.path.exists(cm_samples_csv):
        return None
    with open(cm_samples_csv) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None

    def cost(row):
        try:
            return float(row.get("total_cost", row.get("cost", "-inf")))
        except ValueError:
            return float("-inf")

    row = max(rows, key=cost)
    required = ["follower_base_x", "follower_base_y", "follower_base_yaw"] + [
        "q%d" % i for i in range(1, 8)
    ]
    if any(name not in row for name in required):
        return None
    numeric_metrics = {}
    for name in [
        "reach",
        "reach_proxy",
        "mu_proxy",
        "F_proxy",
        "B_proxy",
        "environment_proxy",
        "object_facing_yaw",
        "object_facing_yaw_error",
        "total_cost",
        "posture_error_from_h",
        "max_joint_error",
        "target_ee_error",
        "manipulability",
    ]:
        if name in row:
            try:
                numeric_metrics[name] = float(row[name])
            except ValueError:
                pass

    return {
        "seed_type": "cm_like_query_seed",
        "seed_source": "offline_cm_samples_csv",
        "base": [
            float(row["follower_base_x"]),
            float(row["follower_base_y"]),
            float(row["follower_base_yaw"]),
        ],
        "q_arm": [float(row["q%d" % i]) for i in range(1, 8)],
        "candidate_rank": int(float(row.get("candidate_rank", 1))),
        "candidate_count": int(float(row.get("candidate_count", len(rows)))),
        "candidate_alpha": float(row.get("candidate_alpha", 0.0)),
        "candidate_dx": float(row.get("candidate_dx", 0.0)),
        "candidate_dy": float(row.get("candidate_dy", 0.0)),
        "candidate_dyaw": float(row.get("candidate_dyaw", 0.0)),
        "cm_table_metrics": numeric_metrics,
    }


def posture_error(q_arm):
    return norm([q_arm[i] - HOME_POSTURE[i] for i in range(7)])


def max_joint_error(q_arm):
    return max(abs(q_arm[i] - HOME_POSTURE[i]) for i in range(7))


def joint_limit_proxy(q_arm):
    margins = []
    for q, (lo, hi) in zip(q_arm, PANDA_LIMITS):
        half = 0.5 * (hi - lo)
        center = 0.5 * (hi + lo)
        margins.append(max(0.0, 1.0 - abs(q - center) / max(half, 1e-9)))
    return sum(margins) / len(margins)


def within_joint_limits(q_arm):
    for q, (lo, hi) in zip(q_arm, PANDA_LIMITS):
        if q < lo or q > hi:
            return False
    return True


def posture_proxy(q_arm):
    # 2.0 rad is a practical normalization for this current initial posture.
    return max(0.0, 1.0 - posture_error(q_arm) / 2.0)


def reach_proxy(base, object_pose):
    follower_grasp = object_grasp_point(object_pose, side="follower")
    reach = math.hypot(follower_grasp[0] - base[0], follower_grasp[1] - base[1])
    preferred = 0.98
    max_reach = 1.35
    return reach, max(0.0, 1.0 - abs(reach - preferred) / max_reach)


def object_facing_yaw(base, object_pose, side="follower"):
    grasp = object_grasp_point(object_pose, side=side)
    return math.atan2(grasp[1] - base[1], grasp[0] - base[0])


def formation_proxy(base, object_pose):
    follower_grasp = object_grasp_point(object_pose, side="follower")
    lateral_err = abs(math.hypot(base[0] - follower_grasp[0], base[1] - follower_grasp[1]) - 0.98)
    yaw_err = abs(wrap_angle(base[2] - object_facing_yaw(base, object_pose, side="follower")))
    return math.exp(-(lateral_err + 0.25 * yaw_err))


def environment_proxy(base, object_pose):
    # Current Figure 2 focuses on the verified straight corridor around the
    # object. This proxy is a light-weight replacement for the paper's B(q)
    # environmental relation until full scene collision checking is connected.
    corridor_y_min = -2.50
    corridor_y_max = 2.50
    clearance = min(base[1] - corridor_y_min, corridor_y_max - base[1])
    clearance_score = max(0.0, min(1.0, clearance / 0.75))
    facing_yaw = object_facing_yaw(base, object_pose, side="follower")
    heading_score = max(0.0, 1.0 - abs(wrap_angle(base[2] - facing_yaw)) / math.pi)
    return 0.75 * clearance_score + 0.25 * heading_score


def object_grasp_point(object_pose, side):
    x, y, _, yaw = object_pose
    offset_y = 0.85 if side == "leader" else -0.85
    return [
        x - math.sin(yaw) * offset_y,
        y + math.cos(yaw) * offset_y,
    ]


def score_seed(seed, object_pose):
    reach, reach_score = reach_proxy(seed["base"], object_pose)
    mu = 0.65 * posture_proxy(seed["q_arm"]) + 0.35 * reach_score
    f_value = formation_proxy(seed["base"], object_pose)
    env_value = environment_proxy(seed["base"], object_pose)
    b_value = joint_limit_proxy(seed["q_arm"]) * env_value
    cost = mu * f_value * b_value
    return {
        "reach": reach,
        "reach_proxy": reach_score,
        "mu_proxy": mu,
        "F_proxy": f_value,
        "B_proxy": b_value,
        "environment_proxy": env_value,
        "object_facing_yaw": object_facing_yaw(seed["base"], object_pose, side="follower"),
        "object_facing_yaw_error": abs(
            wrap_angle(seed["base"][2] - object_facing_yaw(seed["base"], object_pose, side="follower"))
        ),
        "total_cost": cost,
        "posture_error_from_h": posture_error(seed["q_arm"]),
        "max_joint_error": max_joint_error(seed["q_arm"]),
    }


def interpolate_pose(q_start, q_goal, alpha):
    return [q_start[i] + alpha * (q_goal[i] - q_start[i]) for i in range(len(q_start))]


def generate_cm_like_candidates(baseline, object_pose, controller_seed):
    # Algorithm shape adapted from Zhang et al.:
    # 1) Build a discrete candidate set around the target relation.
    # 2) Reject infeasible candidates.
    # 3) Evaluate c(q) = mu(q) x F(q) x B(q).
    # 4) Use the best-cost candidate as the seed.
    candidates = []
    alphas = [0.0, 0.20, 0.40, 0.55, 0.70]
    base_dx = [-0.10, 0.0, 0.10]
    base_dy = [-0.08, 0.0, 0.08]
    yaw_offsets = [-0.10, 0.0, 0.10]

    for alpha in alphas:
        q_arm = interpolate_pose(baseline["q_arm"], HOME_POSTURE, alpha)
        if not within_joint_limits(q_arm):
            continue
        for dx in base_dx:
            for dy in base_dy:
                for dyaw in yaw_offsets:
                    base_xy = [baseline["base"][0] + dx, baseline["base"][1] + dy]
                    facing_yaw = object_facing_yaw([base_xy[0], base_xy[1], 0.0], object_pose, side="follower")
                    seed = {
                        "seed_type": "cm_like_query_seed",
                        "seed_source": "discrete_mu_F_B_candidate_query",
                        "base": [
                            base_xy[0],
                            base_xy[1],
                            wrap_angle(facing_yaw + dyaw),
                        ],
                        "q_arm": q_arm,
                    }
                    reach, _ = reach_proxy(seed["base"], object_pose)
                    if reach < 0.35 or reach > 1.35:
                        continue
                    metrics = score_seed(seed, object_pose)
                    seed.update(metrics)
                    seed["candidate_alpha"] = alpha
                    seed["candidate_dx"] = dx
                    seed["candidate_dy"] = dy
                    seed["candidate_dyaw"] = dyaw
                    candidates.append(seed)

    if controller_seed is not None and within_joint_limits(controller_seed["q_arm"]):
        seed = {
            "seed_type": "cm_like_query_seed",
            "seed_source": "validated_home_posture_controller_output_candidate",
            "base": controller_seed["base"],
            "q_arm": controller_seed["q_arm"],
            "candidate_alpha": -1.0,
            "candidate_dx": 0.0,
            "candidate_dy": 0.0,
            "candidate_dyaw": 0.0,
        }
        reach, _ = reach_proxy(seed["base"], object_pose)
        if 0.35 <= reach <= 1.35:
            seed.update(score_seed(seed, object_pose))
            candidates.append(seed)

    if not candidates:
        raise RuntimeError("no feasible CM-like seed candidate found")

    candidates.sort(key=lambda item: item["total_cost"], reverse=True)
    for rank, seed in enumerate(candidates, 1):
        seed["candidate_rank"] = rank
        seed["candidate_count"] = len(candidates)
    return candidates


def make_seed_rows(qpos, controller_seed, cm_table_seed=None):
    baseline = {
        "seed_type": "baseline_keyframe",
        "seed_source": "coop_start_keyframe",
        "base": [qpos[20], qpos[21], yaw_from_quat(qpos[23], qpos[24], qpos[25], qpos[26])],
        "q_arm": qpos[31:38],
    }
    object_pose = [
        qpos[40],
        qpos[41],
        qpos[42],
        yaw_from_quat(qpos[43], qpos[44], qpos[45], qpos[46]),
    ]

    cm_candidates = generate_cm_like_candidates(baseline, object_pose, controller_seed)
    cm_like = cm_table_seed if cm_table_seed is not None else cm_candidates[0]
    if cm_table_seed is not None:
        cm_candidates = [cm_table_seed] + [
            c for c in cm_candidates if c.get("candidate_rank", 0) != cm_table_seed.get("candidate_rank", -1)
        ]

    rows = []
    for seed in (baseline, cm_like):
        metrics = score_seed(seed, object_pose)
        if seed.get("cm_table_metrics"):
            metrics.update(seed["cm_table_metrics"])
        row = {
            "seed_type": seed["seed_type"],
            "seed_source": seed["seed_source"],
            "candidate_rank": seed.get("candidate_rank", 0),
            "candidate_count": seed.get("candidate_count", len(cm_candidates)),
            "candidate_alpha": seed.get("candidate_alpha", 0.0),
            "candidate_dx": seed.get("candidate_dx", 0.0),
            "candidate_dy": seed.get("candidate_dy", 0.0),
            "candidate_dyaw": seed.get("candidate_dyaw", 0.0),
            "follower_base_x": seed["base"][0],
            "follower_base_y": seed["base"][1],
            "follower_base_yaw": seed["base"][2],
            "object_x": object_pose[0],
            "object_y": object_pose[1],
            "object_z": object_pose[2],
            "object_yaw": object_pose[3],
        }
        for i, value in enumerate(seed["q_arm"], 1):
            row["q%d" % i] = value
        row.update(metrics)
        rows.append(row)
    return rows, cm_candidates


def candidate_rows(candidates, object_pose):
    rows = []
    for seed in candidates:
        row = {
            "candidate_rank": seed["candidate_rank"],
            "candidate_count": seed["candidate_count"],
            "seed_source": seed["seed_source"],
            "candidate_alpha": seed["candidate_alpha"],
            "candidate_dx": seed["candidate_dx"],
            "candidate_dy": seed["candidate_dy"],
            "candidate_dyaw": seed["candidate_dyaw"],
            "follower_base_x": seed["base"][0],
            "follower_base_y": seed["base"][1],
            "follower_base_yaw": seed["base"][2],
            "object_x": object_pose[0],
            "object_y": object_pose[1],
            "object_z": object_pose[2],
            "object_yaw": object_pose[3],
        }
        for i, value in enumerate(seed["q_arm"], 1):
            row["q%d" % i] = value
        metrics = score_seed(seed, object_pose)
        if seed.get("cm_table_metrics"):
            metrics.update(seed["cm_table_metrics"])
        row.update(metrics)
        rows.append(row)
    return rows


def write_csv(rows, output):
    os.makedirs(os.path.dirname(output), exist_ok=True)
    fieldnames = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with open(output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def arm_polyline(base, q_arm):
    # Schematic arm, not exact FK. It makes posture differences visible with a
    # fixed camera and preserves the seed values in the CSV for quantitative use.
    lengths = [0.22, 0.22, 0.20, 0.18, 0.15, 0.12, 0.10]
    angle = base[2] + 0.55
    x = base[0] + 0.18 * math.cos(base[2])
    y = base[1] + 0.18 * math.sin(base[2])
    xs = [x]
    ys = [y]
    for i, (q, length) in enumerate(zip(q_arm, lengths)):
        angle += 0.35 * q + (0.10 if i % 2 == 0 else -0.06)
        x += length * math.cos(angle)
        y += length * math.sin(angle)
        xs.append(x)
        ys.append(y)
    return xs, ys


def draw_seed(ax, row, title):
    base = [float(row["follower_base_x"]), float(row["follower_base_y"]), float(row["follower_base_yaw"])]
    obj = [float(row["object_x"]), float(row["object_y"]), float(row["object_z"]), float(row["object_yaw"])]
    q_arm = [float(row["q%d" % i]) for i in range(1, 8)]

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-7.45, -5.65)
    ax.set_ylim(-1.65, 1.10)
    ax.set_title(title, fontsize=12)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, color="#dddddd", linewidth=0.6)

    # Straight corridor context.
    ax.plot([-7.45, -5.65], [0.90, 0.90], color="#8a8a8a", linewidth=3)
    ax.plot([-7.45, -5.65], [-1.55, -1.55], color="#8a8a8a", linewidth=3)

    # Transport object rectangle, drawn at yaw.
    obj_len = 1.60
    obj_w = 1.70
    corners = [
        (-obj_len / 2, -obj_w / 2),
        (obj_len / 2, -obj_w / 2),
        (obj_len / 2, obj_w / 2),
        (-obj_len / 2, obj_w / 2),
        (-obj_len / 2, -obj_w / 2),
    ]
    c = math.cos(obj[3])
    s = math.sin(obj[3])
    xs = [obj[0] + c * x - s * y for x, y in corners]
    ys = [obj[1] + s * x + c * y for x, y in corners]
    ax.fill(xs, ys, color="#77aadd", alpha=0.35, edgecolor="#225ea8", linewidth=2)
    ax.text(obj[0], obj[1], "object", ha="center", va="center", fontsize=9, color="#174a7c")

    leader_grasp = object_grasp_point(obj, "leader")
    follower_grasp = object_grasp_point(obj, "follower")
    ax.scatter([leader_grasp[0]], [leader_grasp[1]], s=45, color="#2ca25f", zorder=5)
    ax.scatter([follower_grasp[0]], [follower_grasp[1]], s=55, color="#238b45", zorder=5)
    ax.text(leader_grasp[0], leader_grasp[1] + 0.06, "leader grasp", ha="center", fontsize=8)
    ax.text(follower_grasp[0], follower_grasp[1] - 0.13, "follower grasp", ha="center", fontsize=8)

    # Follower base.
    ax.scatter([base[0]], [base[1]], s=100, color="#fdae61", edgecolor="#7f3b08", zorder=6)
    ax.arrow(
        base[0],
        base[1],
        0.32 * math.cos(base[2]),
        0.32 * math.sin(base[2]),
        width=0.015,
        head_width=0.08,
        head_length=0.08,
        color="#7f3b08",
        zorder=6,
    )
    ax.text(base[0], base[1] - 0.17, "follower base", ha="center", fontsize=8)

    ax.plot([base[0], follower_grasp[0]], [base[1], follower_grasp[1]], "--", color="#525252", linewidth=1.1)
    arm_x, arm_y = arm_polyline(base, q_arm)
    ax.plot(arm_x, arm_y, "-o", color="#54278f", markersize=3.5, linewidth=2.0, label="follower arm seed")

    metric = (
        "posture error: %.3f rad\n"
        "mu,F,B: %.3f, %.3f, %.3f\n"
        "cost: %.3f"
        % (
            float(row["posture_error_from_h"]),
            float(row["mu_proxy"]),
            float(row["F_proxy"]),
            float(row["B_proxy"]),
            float(row["total_cost"]),
        )
    )
    ax.text(
        0.98,
        0.03,
        metric,
        transform=ax.transAxes,
        va="bottom",
        ha="right",
        fontsize=9,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "#bbbbbb", "alpha": 0.92},
    )


def write_figures(rows, output_dir, prefix="figure2"):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(output_dir, exist_ok=True)
    names = {
        "baseline_keyframe": ("Baseline seed", "%s_baseline_seed.png" % prefix),
        "cm_like_query_seed": ("CM-like seed", "%s_cm_seed.png" % prefix),
    }
    for row in rows:
        fig, ax = plt.subplots(figsize=(6.0, 5.0), dpi=180)
        title, filename = names[row["seed_type"]]
        draw_seed(ax, row, title)
        fig.tight_layout()
        fig.savefig(os.path.join(output_dir, filename))
        plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.2), dpi=180)
    draw_seed(axes[0], rows[0], "Baseline seed")
    draw_seed(axes[1], rows[1], "CM-like seed")
    fig.tight_layout(w_pad=2.4)
    combined = os.path.join(output_dir, "%s_seed_comparison.png" % prefix)
    fig.savefig(combined)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene-xml",
        default="/home/ryoo/mmcm_ws/src/kimm_robots_description/husky_description/husky_coop/coop_transport_scene.xml",
    )
    parser.add_argument(
        "--controller-csv",
        default="/tmp/mm_verify_logs/base_arm_home_goal_final_controller.csv",
    )
    parser.add_argument(
        "--output-csv",
        default="/tmp/mm_verify_logs/figure2_seed_comparison.csv",
    )
    parser.add_argument(
        "--candidate-csv",
        default="/tmp/mm_verify_logs/figure2_cm_candidates.csv",
    )
    parser.add_argument(
        "--cm-samples",
        default="/tmp/mm_verify_logs/offline_cm_samples.csv",
    )
    parser.add_argument("--output-dir", default="/tmp/mm_verify_logs")
    parser.add_argument("--figure-prefix", default="figure2")
    args = parser.parse_args()

    qpos = read_keyframe_qpos(args.scene_xml)
    rows, candidates = make_seed_rows(
        qpos,
        latest_controller_seed(args.controller_csv),
        latest_cm_table_seed(args.cm_samples),
    )
    object_pose = [
        qpos[40],
        qpos[41],
        qpos[42],
        yaw_from_quat(qpos[43], qpos[44], qpos[45], qpos[46]),
    ]
    write_csv(rows, args.output_csv)
    write_csv(candidate_rows(candidates, object_pose), args.candidate_csv)
    write_figures(rows, args.output_dir, args.figure_prefix)

    print("wrote", args.output_csv)
    print("wrote", args.candidate_csv)
    for filename in [
        "%s_baseline_seed.png" % args.figure_prefix,
        "%s_cm_seed.png" % args.figure_prefix,
        "%s_seed_comparison.png" % args.figure_prefix,
    ]:
        path = os.path.join(args.output_dir, filename)
        print("wrote", path, os.path.getsize(path), "bytes")
    for row in rows:
        print(
            "%s posture=%.4f mu=%.4f F=%.4f B=%.4f cost=%.4f"
            % (
                row["seed_type"],
                row["posture_error_from_h"],
                row["mu_proxy"],
                row["F_proxy"],
                row["B_proxy"],
                row["total_cost"],
            )
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
