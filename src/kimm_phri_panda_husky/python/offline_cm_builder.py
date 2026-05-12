#!/usr/bin/env python3
"""Build an offline CM-compatible seed table for the dual coop scene.

When Pinocchio is available this script evaluates local Panda arm candidates
with FK, frame Jacobians, and Yoshikawa manipulability. If Pinocchio cannot be
imported, it falls back to the light-weight proxy table used by the current
Figure 2 and controller scripts.
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

from figure2_seed_comparison import (
    PANDA_LIMITS,
    generate_cm_like_candidates,
    joint_limit_proxy,
    latest_controller_seed,
    object_grasp_point,
    read_keyframe_qpos,
    score_seed,
    wrap_angle,
    yaw_from_quat,
)


OPENROBOTS_SITE = "/opt/openrobots/lib/python3.8/site-packages"
PANDA_URDF_R = (
    "/home/ryoo/mmcm_ws/src/kimm_robots_description/"
    "franka_panda_description/robots/panda_arm_hand_r.urdf"
)
ARM_MOUNT_XYZ = (0.35, 0.0, 0.25)
HOME_POSTURE = [0.0, 0.0, 0.0, -math.pi / 2.0, 0.0, math.pi / 2.0, -math.pi / 4.0]


def ensure_openrobots_python_path():
    if os.path.isdir(OPENROBOTS_SITE) and OPENROBOTS_SITE not in sys.path:
        sys.path.insert(0, OPENROBOTS_SITE)


def import_pinocchio():
    ensure_openrobots_python_path()
    import pinocchio as pin

    return pin


def pinocchio_status():
    try:
        pin = import_pinocchio()
        return "available:%s" % pin.__version__
    except Exception as exc:
        return "unavailable:%s" % type(exc).__name__


def scene_seed_context(scene_xml, controller_csv):
    qpos = read_keyframe_qpos(scene_xml)
    baseline = {
        "seed_type": "baseline_keyframe",
        "seed_source": "coop_start_keyframe",
        "base": [qpos[20], qpos[21], yaw_from_quat(qpos[23], qpos[24], qpos[25], qpos[26])],
        "base_root_z": qpos[22],
        "q_arm": qpos[31:38],
    }
    object_pose = [
        qpos[40],
        qpos[41],
        qpos[42],
        yaw_from_quat(qpos[43], qpos[44], qpos[45], qpos[46]),
    ]
    controller_seed = latest_controller_seed(controller_csv)
    return baseline, object_pose, controller_seed


def add_mujoco_grasp_frame(pin, model):
    link7_id = model.getFrameId("panda_link7")
    link7_frame = model.frames[link7_id]
    placement = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.1654]))
    return model.addFrame(
        pin.Frame(
            "mujoco_ee_grasp_frame",
            link7_frame.parentJoint,
            link7_id,
            placement,
            pin.FrameType.OP_FRAME,
        )
    )


def load_pinocchio_panda(urdf_path):
    pin = import_pinocchio()
    model = pin.buildModelFromUrdf(urdf_path)
    frame_id = add_mujoco_grasp_frame(pin, model)
    data = model.createData()
    return pin, model, data, frame_id


def yoshikawa_manipulability(jacobian):
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    return float(np.prod(np.maximum(singular_values, 0.0)))


def rotate_yaw(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def target_in_arm_base(seed, object_pose, base_root_z):
    grasp_xy = object_grasp_point(object_pose, side="follower")
    grasp_world = np.array([grasp_xy[0], grasp_xy[1], object_pose[2]])
    mount_x, mount_y = rotate_yaw(ARM_MOUNT_XYZ[0], ARM_MOUNT_XYZ[1], seed["base"][2])
    arm_origin = np.array(
        [
            seed["base"][0] + mount_x,
            seed["base"][1] + mount_y,
            base_root_z + ARM_MOUNT_XYZ[2],
        ]
    )
    dx = grasp_world - arm_origin
    c = math.cos(-seed["base"][2])
    s = math.sin(-seed["base"][2])
    return np.array([c * dx[0] - s * dx[1], s * dx[0] + c * dx[1], dx[2]])


def joint_limit_margin(q_arm):
    return joint_limit_proxy(q_arm)


def pinocchio_candidate_metrics(pin, model, data, frame_id, q_arm, target_local):
    q = np.array(q_arm, dtype=float)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    placement = data.oMf[frame_id]
    ee = np.array(placement.translation).reshape(3)
    jacobian = pin.computeFrameJacobian(
        model,
        data,
        q,
        frame_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )
    return {
        "ee": ee,
        "target_error": float(np.linalg.norm(ee - target_local)),
        "manipulability": yoshikawa_manipulability(jacobian),
    }


def normalize_pinocchio_rows(rows):
    max_mu = max(float(row["manipulability"]) for row in rows) if rows else 1.0
    max_mu = max(max_mu, 1e-9)
    for row in rows:
        seed = {
            "base": [
                row["follower_base_x"],
                row["follower_base_y"],
                row["follower_base_yaw"],
            ],
            "q_arm": [row["q%d" % i] for i in range(1, 8)],
        }
        base_metrics = score_seed(seed, [row["object_x"], row["object_y"], row["object_z"], row["object_yaw"]])
        mu_norm = max(0.0, min(1.0, float(row["manipulability"]) / max_mu))
        target_score = math.exp(-float(row["target_ee_error"]) / 0.35)
        f_value = base_metrics["F_proxy"]
        b_value = joint_limit_margin(seed["q_arm"]) * base_metrics["environment_proxy"] * target_score
        row.update(base_metrics)
        row["mu_proxy"] = mu_norm
        row["B_proxy"] = b_value
        row["total_cost"] = mu_norm * f_value * b_value
    rows.sort(key=lambda item: item["total_cost"], reverse=True)
    for rank, row in enumerate(rows, 1):
        row["candidate_rank"] = rank
        row["candidate_count"] = len(rows)
    return rows


def build_pinocchio_cm(baseline, object_pose, controller_seed, urdf_path=PANDA_URDF_R):
    pin, model, data, frame_id = load_pinocchio_panda(urdf_path)
    candidates = generate_cm_like_candidates(baseline, object_pose, controller_seed)
    status = "available:%s" % pin.__version__
    rows = []
    for candidate in candidates:
        target_local = target_in_arm_base(candidate, object_pose, baseline["base_root_z"])
        metrics = pinocchio_candidate_metrics(
            pin, model, data, frame_id, candidate["q_arm"], target_local
        )
        row = {
            "seed_type": "cm_seed",
            "seed_source": "pinocchio_fk_jacobian_local_cm",
            "candidate_rank": 0,
            "candidate_count": len(candidates),
            "candidate_alpha": candidate.get("candidate_alpha", 0.0),
            "candidate_dx": candidate.get("candidate_dx", 0.0),
            "candidate_dy": candidate.get("candidate_dy", 0.0),
            "candidate_dyaw": candidate.get("candidate_dyaw", 0.0),
            "pinocchio_status": status,
            "ik_status": "not_used_q_sampling",
            "fk_status": "success",
            "follower_base_x": candidate["base"][0],
            "follower_base_y": candidate["base"][1],
            "follower_base_yaw": candidate["base"][2],
            "object_x": object_pose[0],
            "object_y": object_pose[1],
            "object_z": object_pose[2],
            "object_yaw": object_pose[3],
            "ee_x": float(metrics["ee"][0]),
            "ee_y": float(metrics["ee"][1]),
            "ee_z": float(metrics["ee"][2]),
            "target_ee_x": float(target_local[0]),
            "target_ee_y": float(target_local[1]),
            "target_ee_z": float(target_local[2]),
            "target_ee_error": metrics["target_error"],
            "manipulability": metrics["manipulability"],
        }
        for i, value in enumerate(candidate["q_arm"], 1):
            row["q%d" % i] = value
        rows.append(row)
    return normalize_pinocchio_rows(rows)


def build_fallback_local_cm(baseline, object_pose, controller_seed):
    candidates = generate_cm_like_candidates(baseline, object_pose, controller_seed)
    status = pinocchio_status()
    target = object_grasp_point(object_pose, side="follower")

    rows = []
    for candidate in candidates:
        reach = candidate.get("reach", 0.0)
        ee_x = candidate["base"][0] + reach * 0.65
        ee_y = candidate["base"][1] + (target[1] - candidate["base"][1]) * 0.65
        ee_z = object_pose[2]
        row = {
            "seed_type": "cm_seed",
            "seed_source": candidate["seed_source"],
            "candidate_rank": candidate["candidate_rank"],
            "candidate_count": candidate["candidate_count"],
            "candidate_alpha": candidate["candidate_alpha"],
            "candidate_dx": candidate["candidate_dx"],
            "candidate_dy": candidate["candidate_dy"],
            "candidate_dyaw": candidate["candidate_dyaw"],
            "pinocchio_status": status,
            "ik_status": "fallback_proxy_no_ik" if status.startswith("unavailable") else "not_run",
            "fk_status": "fallback_proxy_no_fk" if status.startswith("unavailable") else "not_run",
            "follower_base_x": candidate["base"][0],
            "follower_base_y": candidate["base"][1],
            "follower_base_yaw": candidate["base"][2],
            "object_x": object_pose[0],
            "object_y": object_pose[1],
            "object_z": object_pose[2],
            "object_yaw": object_pose[3],
            "ee_x": ee_x,
            "ee_y": ee_y,
            "ee_z": ee_z,
            "target_ee_x": target[0],
            "target_ee_y": target[1],
            "target_ee_z": object_pose[2],
            "target_ee_error": ((ee_x - target[0]) ** 2 + (ee_y - target[1]) ** 2) ** 0.5,
            "manipulability": candidate.get("mu_proxy", 0.0),
        }
        for i, value in enumerate(candidate["q_arm"], 1):
            row["q%d" % i] = value
        row.update(score_seed(candidate, object_pose))
        rows.append(row)
    return rows


def write_cm_samples(rows, output):
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def build_rows(scene_xml, controller_csv, mode="auto", urdf_path=PANDA_URDF_R):
    baseline, object_pose, controller_seed = scene_seed_context(scene_xml, controller_csv)
    if mode in ("auto", "pinocchio"):
        try:
            return build_pinocchio_cm(baseline, object_pose, controller_seed, urdf_path)
        except Exception as exc:
            if mode == "pinocchio":
                raise
            print("pinocchio_cm_unavailable", repr(exc))
    return build_fallback_local_cm(baseline, object_pose, controller_seed)


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
    parser.add_argument("--output", default="/tmp/mm_verify_logs/offline_cm_samples.csv")
    parser.add_argument("--mode", choices=["auto", "pinocchio", "fallback"], default="auto")
    parser.add_argument("--prefer-pinocchio", action="store_true")
    parser.add_argument("--urdf", default=PANDA_URDF_R)
    args = parser.parse_args()

    mode = "pinocchio" if args.prefer_pinocchio else args.mode
    rows = build_rows(args.scene_xml, args.controller_csv, mode, args.urdf)
    write_cm_samples(rows, args.output)

    best = rows[0]
    status = best.get("pinocchio_status", pinocchio_status())
    print("wrote", args.output)
    print("pinocchio_status", status)
    print(
        "best rank=%s posture=%.4f mu=%.4f F=%.4f B=%.4f cost=%.4f target_err=%.4f manip=%.6f"
        % (
            best["candidate_rank"],
            best["posture_error_from_h"],
            best["mu_proxy"],
            best["F_proxy"],
            best["B_proxy"],
            best["total_cost"],
            best["target_ee_error"],
            best["manipulability"],
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
