# Cooperative Transportation of Differential-Drive Mobile Manipulators Using Capability Map-Based Initial Posture Selection

**English** · [한국어](README.ko.md)

**If the initial arm posture is chosen with a Capability Map before transport begins, two differential-drive mobile manipulators can carry an object together without collapsing a good configuration.**

2026 ICROS undergraduate research project · Department of Mechatronics Engineering, Chungnam National University · [iCIR Lab](https://sites.google.com/view/icir-lab/)

A ROS 1 / MuJoCo simulation in which two Husky–Panda mobile manipulators cooperatively transport a single object. The arm posture selected by a **Capability Map (CM)** before transport is compared against a generic home posture under identical transport conditions, and each robot's Whole-Body HQP command is merged into a single 26-axis MuJoCo command.

---

## Poster and Full Scenario Video

| Material | Contents |
|---|---|
| [Undergraduate research poster (PDF)](assets/poster/학부연구생포스터_유한민.pdf) | Research background, Whole-Body HQP, CM seed selection and evaluation |
| [Full scenario video](https://www.youtube.com/watch?v=ukqPhyuMfp4) | Cooperative transport with the home baseline vs. the CM-applied posture |

[![Poster preview](assets/poster/poster_preview.jpg)](assets/poster/학부연구생포스터_유한민.pdf)

[![Full scenario video preview](assets/video/video_preview.jpg)](https://www.youtube.com/watch?v=ukqPhyuMfp4)

---

## System Overview

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                          /coop_phri_simul                               │
│        weld attach · object target · lift readiness · TF/metrics        │
│        CM runtime (q_sym) · 13+13 → 26-axis command merge               │
│                    ▲                            │                       │
│      state split   │                            │  26-axis joint cmd    │
│                    │                            ▼                       │
│   ┌────────────────┴─────────────────┐   /coop_scene/mujoco_ros         │
│   │                                  │   (integrated MuJoCo state)      │
│   ▼                                  ▼                                  │
│ /leader_hqp_controller      /follower_hqp_controller                    │
│   13-axis command             13-axis command                           │
│   (Whole-Body HQP)            (Whole-Body HQP)                          │
│           ▲                          ▲                                  │
│           └────────── posture reference q_sym ──────────┐               │
│                                                         │               │
│                       config/holding_cm_seed_lateral_nullspace.yaml     │
│                                (offline CM seed)                        │
└─────────────────────────────────────────────────────────────────────────┘
```

### Key Features

- **Whole-Body HQP control** — a 7-DoF Panda arm on a differential-drive (nonholonomic) Husky base, solved as one hierarchical QP per robot.
- **Leader–follower cooperative transport** — both end-effectors are rigidly coupled to the object, and the formation is maintained by driving the bases rather than by deforming the arms.
- **Offline Capability Map seed** — the transport starting posture is selected in advance from manipulability, grasp formation, and base feasibility.
- **A/B comparison under identical conditions** — the CM seed and the home posture are compared with the same object, weld stiffness, sensors, 0.10 m lift, and mode 235 path conditions.
- **Single integrated simulation** — one MuJoCo state is split per robot, and the two 13-axis commands are merged into a 26-axis command at about 500 Hz.

---

## Layout

```text
src/kimm_phri_panda_husky/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── cm_transport.launch        # CM scenario (also included by the baseline launch)
│   └── home_transport.launch      # home baseline, redefines only the CM-related args
├── config/
│   └── holding_cm_seed_lateral_nullspace.yaml   # initial state + selected offline CM seed
├── include/kimm_phri_panda_husky/
│   └── panda_husky_hqp.h
└── src/
    ├── coop_husky_hqp_node.cpp    # state split, reference/Fext input, per-robot 13-axis command
    ├── coop_phri_simul.cpp        # weld test, object target, key input, CM runtime, command merge
    └── panda_husky_hqp.cpp        # Whole-Body HQP for posture and cooperative transport
```

| File | Role |
|---|---|
| `launch/cm_transport.launch` | Default CM scenario that brings up MuJoCo and the leader/follower HQP nodes |
| `launch/home_transport.launch` | Home baseline scenario that reuses the CM launch |
| `config/holding_cm_seed_lateral_nullspace.yaml` | Initial state, selected offline CM seed, and its evaluation scores |
| `src/coop_phri_simul.cpp` | Weld test, object target, lift readiness, key input, and 26-axis command merge |
| `src/coop_husky_hqp_node.cpp` | Integrated state split, reference/Fext input, and per-robot 13-axis command publishing |
| `src/panda_husky_hqp.cpp` | Whole-Body HQP control for posture and cooperative transport |

Four nodes are launched:

```text
/coop_scene/mujoco_ros
/coop_phri_simul
/leader_hqp_controller
/follower_hqp_controller
```

---

## 1. Runtime Environment

Simulation only. No real robot hardware is required, and none is covered by this repository.

| Item | Version / Install |
|---|---|
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| Eigen3 | `sudo apt install libeigen3-dev` |
| yaml-cpp | `sudo apt install libyaml-cpp-dev` |
| Pinocchio | installed under `/opt/openrobots` ([official guide](https://stack-of-tasks.github.io/pinocchio/download.html)) |

This repository provides only the core scenario package, so the following packages must already be present in the same catkin workspace:

| Package | Provides |
|---|---|
| `kimm_hqp_controller` | Hierarchical QP whole-body controller |
| `mujoco_ros`, `mujoco_ros_msgs` | MuJoCo–ROS bridge and messages |
| `husky_description` | Robot models and the cooperative transport scenes |

`husky_description` must contain the following files:

```text
husky_coop/coop_transport_scene_straight_clean_generated.xml       # CM scene
husky_coop/coop_transport_scene_straight_clean_home_baseline.xml   # home baseline scene
husky_single/husky_panda_hand.urdf
```

---

## 2. Build

```bash
cd <catkin_workspace>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Before running, check that both launch files resolve:

```bash
roslaunch kimm_phri_panda_husky cm_transport.launch   --nodes
roslaunch kimm_phri_panda_husky home_transport.launch --nodes
```

Install rules for the library, headers, `coop_husky_hqp_node`, `coop_phri_simul`, and the launch/config directories are included, so an install space works as well.

---

## 3. Comparison Scenarios

Both scenarios follow the same three-step key sequence: **attach → lift → transport**.
The simulation stays paused until `c`, so the weld is formed from a clean grasp-ready posture.

### 3.1 With the CM initial posture

`selected/q_sym` from `holding_cm_seed_lateral_nullspace.yaml` is applied as the posture reference for both arms in mode 238.

```bash
roslaunch kimm_phri_panda_husky cm_transport.launch
```

| Key | Action |
|:---:|---|
| `c` | Simulation-only weld attach |
| `y` | Apply the CM posture and lift 0.10 m (mode 238) |
| `0` | Start cooperative transport (mode 235) |

### 3.2 Home baseline without the CM

No CM seed is published; the generic home posture is held through the same lift and transport path. `home_transport.launch` includes `cm_transport.launch` and redefines only the arguments needed for the baseline.

```bash
roslaunch kimm_phri_panda_husky home_transport.launch
```

| Key | Action |
|:---:|---|
| `c` | Simulation-only weld attach |
| `t` | Hold the home posture and lift 0.10 m (mode 211) |
| `0` | Start cooperative transport (mode 235) |

> The two scenarios share the same object size, weld stiffness, sensors, 0.10 m lift, and mode 235 path conditions. However, the CM `q_sym` and the original home posture reach different final gripper-center heights, so each scene uses object and support heights matched to its own posture.

---

## 4. Control Mode

| Mode | Owner | Meaning |
|---:|---|---|
| `1` | HQP | Home / posture hold |
| `21` | orchestration | Simulation weld attach test |
| `200` | HQP | Track external EE/base/posture references |
| `211` | orchestration | Home baseline lift without the CM |
| `238` | orchestration | Lift with the CM posture applied |
| `235` | HQP | Final cooperative transport |

Lift readiness for mode 238 opens only within 1 cm of the 10 cm lift target. If the grasp metric is lost for more than 0.5 s or leaves the tolerance again, readiness is reset to false, blocking a new entry into mode 235.

---

## 5. Capability Map Seed

Initial posture candidates are evaluated together on arm manipulability $\mu(q)$, grasp formation $F(q)$, and base feasibility $B(q)$ as a weighted geometric mean:

$$
S_{CM}=\mu(q)^{0.40}\times F(q)^{0.35}\times B(q)^{0.25}
$$

The highest-scoring candidate is stored as `selected/q_sym` in `config/holding_cm_seed_lateral_nullspace.yaml` and applied once, at the start of transport.

**The CM here is an offline seed, not an online planner.** It is not re-queried during transport; the posture selected in advance is maintained as far as possible. Obstacle-avoidance direction and yaw alignment are handled by the mode 235 base trajectory and the HQP mobile task.

---

## 6. Configuration

### 6.1 Launch arguments

| Argument | Default | Description |
|---|---|---|
| `model_file` | `husky_coop/coop_transport_scene_straight_clean_generated.xml` | MuJoCo scene |
| `cm_seed_file` | `config/holding_cm_seed_lateral_nullspace.yaml` | Offline CM seed |
| `apply_cm_posture` | `true` | Publish `q_sym` as the posture reference |
| `apply_cm_base` | `false` | Apply the CM base pose (unused in the current scenarios) |
| `publish_lift_home_posture` | `false` | Publish the home posture during lift (baseline path) |
| `initial_ctrl_mode` | `200` | Control mode entered at startup |
| `hqp_rate` | `500.0` | HQP solve and command publish rate [Hz] |
| `start_keyboard` | `true` | Start the keyboard interface |
| `attach_request_delay` | `1.0` | Delay before the attach request [s] |

### 6.2 Weld attach conditions

The weld attach behind the `c` key is a grasp for MuJoCo simulation testing.

The green `l_ee_grasp_site` / `r_ee_grasp_site` are the actual grasp references in simulation. Rather than treating the base-height and frame differences between the controller FK and the MuJoCo model as estimates, the real site poses published by the scene's `framepos` / `framequat` sensors are used for EE target correction and diagnostics. The weld `relpose` is likewise generated from a non-penetrating pose between those sites and the grasp frames at both ends of the object.

At the initial weld, each base is placed 5 mm further outward than the nominal seed, giving 10 mm of total lateral clearance. Penetration is flagged only when the EE, link7, or elbow lies inside the actual object box volume, and it must be zero at attach. During mode 235 transport, penetration diagnostics are logged but are not used as an entry-failure condition.

---

## 7. Simulation Verification

Confirmed for both scenarios (`c → y → 0` and `c → t → 0`):

- Zero EE / link7 / elbow penetration at attach
- Lift of approximately `+0.10 m`
- Mode 235 phases 0–5 completed, 13-axis commands published on both sides at about 500 Hz, no NaN/Inf

---

## 8. Current Scope and Limitations

- **Simulation only.** The `c` key weld attach is a MuJoCo test path and does not replace a real gripper command; real-robot bring-up is not included in this repository.
- **The CM is an offline initial seed.** Local CM re-query during transport is not included.
- **External force (Fext) compensation was not used.** The code and the `fext_*` launch arguments exist, but both comparison scenarios leave them disabled at their `false` defaults, and the behavior is unverified.
- **The two scenes do not share the same world heights.** Because the CM `q_sym` and the original home posture reach different real gripper heights, each scene uses its own object and support heights. Recomputing a CM seed that also matches world heights changes the control conditions themselves, so it is left as future work.

---

## 9. Troubleshooting

**The object slips off the support and the weld is rejected**
- The simulation is intentionally paused until `c`. Starting the simulation before attaching widens the gap and the weld is rejected.

**`catkin_make` cannot find `kimm_hqp_controller` or Pinocchio**
- Check that `kimm_hqp_controller` is in the same workspace and that Pinocchio is installed under `/opt/openrobots`.

**MuJoCo fails to load the scene**
- Check that the `model_file` path resolves inside `husky_description` (`husky_coop/…_generated.xml` for CM, `…_home_baseline.xml` for the baseline).

**Nodes do not start**
- First confirm that the node graph resolves with `--nodes`, then check with `rosnode list` that all four nodes are present.

---

## Citation

```bibtex
@inproceedings{ryoo2026capability,
  title     = {Capability Map 기반 초기 자세 선택을 활용한
               차동구동 모바일 매니퓰레이터의 협업 운반},
  author    = {유한민 and 박진성},
  booktitle = {제어로봇시스템학회 학술대회 논문집 (ICROS 2026)},
  year      = {2026}
}
```

## Authors

- **Hanmin Ryoo** — Department of Mechatronics Engineering, Chungnam National University
- **Jinseong Park** — Department of Mechatronics Engineering, Chungnam National University / iCIR Lab

## License

MIT License. See [LICENSE](LICENSE) for details.
