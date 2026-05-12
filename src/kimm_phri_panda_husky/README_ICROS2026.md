# ICROS2026 Cooperative Transport Workspace Notes

This document records the current ICROS2026 simulation path in `mmcm_ws`.
The original single-robot pHRI/HQP launch files are intentionally preserved.

## Current Status

- Dual Husky-Panda MuJoCo scene exists in `husky_coop/coop_transport_scene.xml`.
- The scene loads two prefixed robots: `l_husky_with_panda_hand.xml` and `r_husky_with_panda_hand.xml`.
- `transport_object` is connected to both EE grasp frames through active weld constraints.
- The current grasp is fixed-weld based. It is visually/contact aligned with the gripper region, but it is not yet a pure contact-force grasp.
- Pinocchio 3.4.0 is available through `/opt/openrobots` and is used for Panda FK/Jacobian/manipulability in the local CM builder.
- The current CM path is FK/Jacobian q-sampling local CM, not yet IK-based inverse reachability CM.

## Official Experiment Path

Set the Pinocchio environment:

```bash
cd /home/ryoo/mmcm_ws
source devel/setup.bash
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

Build the Pinocchio local CM table:

```bash
python3 src/kimm_phri_panda_husky/python/offline_cm_builder.py \
  --mode pinocchio \
  --output /tmp/mm_verify_logs/pinocchio_offline_cm_samples.csv
```

Generate Figure 2 seed comparison:

```bash
python3 src/kimm_phri_panda_husky/python/figure2_seed_comparison.py \
  --cm-samples /tmp/mm_verify_logs/pinocchio_offline_cm_samples.csv \
  --output-csv /tmp/mm_verify_logs/pinocchio_figure2_seed_comparison.csv \
  --candidate-csv /tmp/mm_verify_logs/pinocchio_figure2_cm_candidates.csv \
  --output-dir /tmp/mm_verify_logs \
  --figure-prefix pinocchio_figure2
```

Run the short baseline-vs-CM verification:

```bash
python3 src/kimm_phri_panda_husky/python/run_baseline_vs_cm_experiment.py \
  --scenario straight \
  --duration 3 \
  --speed 0.01 \
  --rate 10 \
  --command-rate 25 \
  --sim-start-delay 1.0 \
  --cm-builder-mode pinocchio \
  --output-dir /tmp/mm_verify_logs/pinocchio_bvc \
  --timeout 45
```

## Runtime Nodes

- `leader_motion_player.py`: prescribed leader base/EE/object references.
- `follower_coop_planner.py`: follower base/EE targets and cooperative proxy metrics.
- `follower_base_arm_home_controller.py`: current monolithic 26-actuator verification controller.
- `coop_scene_state_logger.py`: scene stability CSV logger.
- `coop_reference_logger.py`: reference/metric CSV logger.
- `coop_sim_run_once.py`: one-shot MuJoCo sim start helper.

## CM and Figure Tools

- `pinocchio_panda_smoke_test.py`: Panda URDF FK/Jacobian/manipulability smoke test.
- `offline_cm_builder.py`: Pinocchio local CM table generation with fallback proxy path.
- `figure2_seed_comparison.py`: baseline-vs-CM seed figure and CSV generation.

## Experiment Tools

- `run_baseline_vs_cm_experiment.py`: automated baseline and CM seed comparison.
- `baseline_vs_cm_summary.py`: controller-log summary CSV writer.

## Legacy Verification Files

These are useful for earlier verification stages but are not the preferred final experiment path:

- `follower_base_only_controller.py`
- `dual_scene_hold_controller.py`
- `dual_scene_base_command_bridge.py`
- `coop_transport_hold_verify.launch`
- `coop_transport_follower_base_verify.launch`
- `coop_transport_baseline_demo.launch`
- `coop_transport_experiment.launch`
- `cm_seed_logger.py`

Do not delete the original `ns1_*` launch/python files or the C++ controller sources; they preserve the single-robot controller and ctrl_mode reference.

## Recommended Dual Controller Architecture

The preferred structure is Option A:

```text
leader_controller  -> /leader/joint_command   (13 actuator JointSet)
follower_controller -> /follower/joint_command (13 actuator JointSet)
dual_joint_set_merger -> /coop_scene/mujoco_ros/mujoco_ros_interface/joint_set
```

Only `dual_joint_set_merger.py` should publish the final 26-actuator MuJoCo command when separated controllers are active. This avoids command overwrite on the shared `joint_set` topic.

Actuator slices:

```text
0..3   leader wheels
4..10  leader Panda arm
11..12 leader fingers
13..16 follower wheels
17..23 follower Panda arm
24..25 follower fingers
```

## Current Limitations

- The object grasp is fixed-weld based, not pure finger contact grasp.
- Finger joints have slide joints and actuators, but the current baseline controller does not actively close them.
- The current local CM evaluates q candidates with FK/Jacobian; it is not yet Zhang-style IK inverse reachability CM.
- EE tracking, lifting, low-ceiling avoidance, and full HQP/WBC dual control are not complete.

## Next Recommended Steps

1. Verify the thin plate object remains stable in the fixed-weld scene.
2. Add separated leader/follower base-only controllers that publish 13-actuator commands.
3. Use `dual_joint_set_merger.py` as the single final `joint_set` publisher.
4. Add lifting test by removing or disabling the object support and commanding a small EE/object z rise.
5. Replace q-sampling CM with IK-based inverse reachability CM.
