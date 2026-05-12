#!/usr/bin/env python3
"""Run short baseline-vs-CM cooperative transport verification experiments."""

import argparse
import os
import signal
import subprocess
import sys


def run(cmd, timeout, env=None):
    print("RUN", " ".join(cmd))
    process = subprocess.Popen(cmd, preexec_fn=os.setsid, env=env)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print("TIMEOUT terminating", " ".join(cmd))
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        try:
            return process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            try:
                return process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait()
                return 124


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="straight")
    parser.add_argument("--duration", default="4")
    parser.add_argument("--speed", default="0.01")
    parser.add_argument("--rate", default="10")
    parser.add_argument("--command-rate", default="25")
    parser.add_argument("--sim-start-delay", default="1.0")
    parser.add_argument("--output-dir", default="/tmp/mm_verify_logs")
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--cm-builder-mode", choices=["auto", "pinocchio", "fallback"], default="auto")
    parser.add_argument("--cm-output", default="")
    parser.add_argument("--seed-comparison-output", default="")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    cm_table = args.cm_output or os.path.join(args.output_dir, "offline_cm_samples.csv")
    figure_csv = args.seed_comparison_output or os.path.join(args.output_dir, "figure2_seed_comparison.csv")
    base_env = os.environ.copy()
    openrobots_site = "/opt/openrobots/lib/python3.8/site-packages"
    if os.path.isdir(openrobots_site):
        base_env["PYTHONPATH"] = "%s:%s" % (openrobots_site, base_env.get("PYTHONPATH", ""))
    base_env["LD_LIBRARY_PATH"] = "/opt/openrobots/lib:%s" % base_env.get("LD_LIBRARY_PATH", "")
    base_env["PATH"] = "/opt/openrobots/bin:%s" % base_env.get("PATH", "")
    base_env["CMAKE_PREFIX_PATH"] = "/opt/openrobots:%s" % base_env.get("CMAKE_PREFIX_PATH", "")

    run(
        [
            "python3",
            "src/kimm_phri_panda_husky/python/offline_cm_builder.py",
            "--mode",
            args.cm_builder_mode,
            "--output",
            cm_table,
        ],
        20,
        env=base_env,
    )
    run(
        [
            "python3",
            "src/kimm_phri_panda_husky/python/figure2_seed_comparison.py",
            "--cm-samples",
            cm_table,
            "--output-csv",
            figure_csv,
            "--candidate-csv",
            os.path.join(args.output_dir, "figure2_cm_candidates.csv"),
            "--output-dir",
            args.output_dir,
        ],
        20,
        env=base_env,
    )

    runs = [
        ("baseline", "home"),
        ("cm_seed", "cm"),
    ]
    controller_logs = {}
    for run_index, (label, seed_mode) in enumerate(runs):
        env = os.environ.copy()
        env.update(base_env)
        env["ROS_MASTER_URI"] = "http://localhost:%d" % (11411 + run_index)
        controller = os.path.join(args.output_dir, "%s_controller.csv" % label)
        controller_logs[label] = controller
        cmd = [
            "roslaunch",
            "kimm_phri_panda_husky",
            "coop_transport_base_arm_home_verify.launch",
            "scenario:=%s" % args.scenario,
            "duration:=%s" % args.duration,
            "rate:=%s" % args.rate,
            "command_rate:=%s" % args.command_rate,
            "speed:=%s" % args.speed,
            "sim_start_delay:=%s" % args.sim_start_delay,
            "seed_mode:=%s" % seed_mode,
            "cm_seed_table_path:=%s" % cm_table,
            "scene_log_output:=%s" % os.path.join(args.output_dir, "%s_scene.csv" % label),
            "reference_log_output:=%s" % os.path.join(args.output_dir, "%s_reference.csv" % label),
            "controller_log_output:=%s" % controller,
        ]
        run(cmd, args.timeout, env=env)

    summary = os.path.join(args.output_dir, "baseline_vs_cm_summary.csv")
    result = run(
        [
            "python3",
            "src/kimm_phri_panda_husky/python/baseline_vs_cm_summary.py",
            "--baseline-controller",
            controller_logs["baseline"],
            "--cm-controller",
            controller_logs["cm_seed"],
            "--seed-comparison",
            figure_csv,
            "--output",
            summary,
        ],
        20,
    )
    print("summary", summary)
    return int(result)


if __name__ == "__main__":
    sys.exit(main())
