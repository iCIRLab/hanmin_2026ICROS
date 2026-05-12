# Simulation Commands

Run from:

```bash
cd /home/ryoo/mmcm_ws
source devel/setup.bash
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
```

## Base Movement

```bash
roslaunch kimm_phri_panda_husky coop_transport_dual_base_only_verify.launch \
  scenario:=straight duration:=45.0 loop:=false \
  speed:=0.20 yaw_rate:=0.0 \
  rate:=10 command_rate:=25 \
  force_limit:=18.0 k_forward:=14.0 k_turn:=20.0 \
  max_xy_error:=10.0 \
  scene_log_output:=/tmp/mm_verify_logs/watch_to_narrow_scene.csv \
  reference_log_output:=/tmp/mm_verify_logs/watch_to_narrow_reference.csv \
  leader_controller_log_output:=/tmp/mm_verify_logs/watch_to_narrow_leader.csv \
  follower_controller_log_output:=/tmp/mm_verify_logs/watch_to_narrow_follower.csv
```

## Fixed-Weld Arm Lift

```bash
roslaunch kimm_phri_panda_husky coop_transport_grasp_lift_verify.launch \
  duration:=300 \
  rate:=10 \
  command_rate:=100 \
  speed:=0.0 \
  yaw_rate:=0.0 \
  force_limit:=0.0 \
  lift_start_delay:=15.0 \
  lift_duration:=6.0 \
  object_lift_height:=0.08 \
  object_lift_kp:=120.0 \
  object_lift_kd:=10.0 \
  object_lift_force_max:=45.0 \
  ee_z_force:=24.0 \
  ramp_gravity_with_lift:=true \
  scene_log_output:=/tmp/mm_verify_logs/hold_lift_scene.csv \
  leader_controller_log_output:=/tmp/mm_verify_logs/hold_lift_leader.csv \
  follower_controller_log_output:=/tmp/mm_verify_logs/hold_lift_follower.csv
```

## Pinocchio Smoke Test

```bash
python3 src/kimm_phri_panda_husky/python/pinocchio_panda_smoke_test.py \
  --urdf /home/ryoo/mmcm_ws/src/kimm_robots_description/franka_panda_description/robots/panda_arm_hand_l.urdf \
  --output /tmp/mm_verify_logs/pinocchio_left_smoke.txt

python3 src/kimm_phri_panda_husky/python/pinocchio_panda_smoke_test.py \
  --urdf /home/ryoo/mmcm_ws/src/kimm_robots_description/franka_panda_description/robots/panda_arm_hand_r.urdf \
  --output /tmp/mm_verify_logs/pinocchio_right_smoke.txt
```

## Figure 2 Seed Comparison

```bash
python3 src/kimm_phri_panda_husky/python/offline_cm_builder.py \
  --output /tmp/mm_verify_logs/pinocchio_offline_cm_samples.csv \
  --mode auto

python3 src/kimm_phri_panda_husky/python/figure2_seed_comparison.py \
  --cm-samples /tmp/mm_verify_logs/pinocchio_offline_cm_samples.csv
```

