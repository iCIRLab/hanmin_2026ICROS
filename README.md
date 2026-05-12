# hanmin_2026ICROS

ICROS2026 cooperative transportation simulation progress repository.

This repository records the current MuJoCo simulation implementation for two Husky-Panda mobile manipulators transporting a fixed-weld plate object.

## Current Scope

- Dual Husky-Panda MuJoCo scene
- Fixed-weld rigid plate grasp setup
- Leader/follower separated command architecture
- Base-only movement verification
- Arm lift verification foundation
- Panda arm URDF / Pinocchio smoke tests
- Preliminary baseline seed vs CM-like seed comparison tools

## Main Directories

```text
docs/meetings/2026-05-12/
  Weekly meeting PDF, run commands, code evidence, and demo videos

src/kimm_robots_description/
  MuJoCo / URDF robot and scene descriptions

src/kimm_phri_panda_husky/
  ROS launch files, controllers, reference generators, and CM tools
```

## Meeting Record

See:

```text
docs/meetings/2026-05-12/README.md
```

## Notes

This is a progress record, not a final cooperative transportation controller.

Current grasp is based on MuJoCo fixed weld constraints, not contact-force grasp.
The current CM implementation is a local/CM-like seed comparison pipeline and is not yet a full Zhang-style IK-based inverse Capability Map.

