#!/usr/bin/env python3
"""Smoke-test Panda FK/Jacobian/manipulability through Pinocchio."""

import argparse
import os
import sys

import numpy as np


OPENROBOTS_SITE = "/opt/openrobots/lib/python3.8/site-packages"
if os.path.isdir(OPENROBOTS_SITE) and OPENROBOTS_SITE not in sys.path:
    sys.path.insert(0, OPENROBOTS_SITE)


HOME_POSTURE = np.array([0.0, 0.0, 0.0, -np.pi / 2.0, 0.0, np.pi / 2.0, -np.pi / 4.0])


def add_mujoco_grasp_frame(pin, model, link7_name="panda_link7"):
    link7_id = model.getFrameId(link7_name)
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


def yoshikawa_manipulability(jacobian):
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    return float(np.prod(np.maximum(singular_values, 0.0)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        default="/home/ryoo/mmcm_ws/src/kimm_robots_description/franka_panda_description/robots/panda_arm_hand_r.urdf",
    )
    parser.add_argument("--output", default="/tmp/mm_verify_logs/pinocchio_panda_smoke_test.txt")
    parser.add_argument("--frame", default="mujoco_ee_grasp_frame")
    args = parser.parse_args()

    try:
        import pinocchio as pin
    except Exception as exc:
        raise SystemExit("pinocchio import failed: %s: %s" % (type(exc).__name__, exc))

    model = pin.buildModelFromUrdf(args.urdf)
    grasp_frame_id = add_mujoco_grasp_frame(pin, model)
    data = model.createData()
    q = HOME_POSTURE.copy()

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    placement = data.oMf[grasp_frame_id]
    jacobian = pin.computeFrameJacobian(
        model,
        data,
        q,
        grasp_frame_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
    )
    manipulability = yoshikawa_manipulability(jacobian)

    joint_names = [model.names[i] for i in range(model.njoints) if "panda_joint" in model.names[i]]
    frame_names = [
        frame.name
        for frame in model.frames
        if frame.name in ("panda_link7", "panda_link8", "panda_hand", "mujoco_ee_grasp_frame")
    ]

    lines = [
        "pinocchio_version: %s" % pin.__version__,
        "pinocchio_file: %s" % pin.__file__,
        "urdf: %s" % args.urdf,
        "model_nq: %d" % model.nq,
        "model_nv: %d" % model.nv,
        "joint_names: %s" % ", ".join(joint_names),
        "frame_candidates: %s" % ", ".join(frame_names),
        "selected_frame: mujoco_ee_grasp_frame",
        "selected_frame_id: %d" % grasp_frame_id,
        "home_q: %s" % " ".join("%.9f" % value for value in q),
        "ee_translation_xyz: %.9f %.9f %.9f" % tuple(placement.translation.tolist()),
        "jacobian_shape: %dx%d" % jacobian.shape,
        "yoshikawa_manipulability: %.12f" % manipulability,
    ]

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print("wrote", args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
