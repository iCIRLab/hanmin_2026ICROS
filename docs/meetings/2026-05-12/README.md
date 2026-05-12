# 2026-05-12 Weekly Meeting

## Summary

This folder records the implementation status presented in the 2026-05-12 weekly meeting.

## Main Progress

1. Dual Husky-Panda MuJoCo scene
2. Fixed-weld plate grasp setup
3. Leader/follower separated command architecture
4. Base-only movement verification
5. Arm lift verification foundation
6. Preliminary CM-like seed comparison

## Presentation

- `20260512_weekly_meeting.pdf`

## Media

| File | Description |
|---|---|
| `media/base_movement.mp4` | Base-only movement with the fixed-weld plate |
| `media/fixed_weld_grasp.mp4` | Fixed-weld grasp setup |
| `media/arm_lift.mp4` | Arm lift verification |

## Figures

| File | Description |
|---|---|
| `media/figures/command_architecture.png` | Leader/follower command architecture |
| `media/figures/figure2_seed_comparison.png` | Baseline seed vs CM-like seed comparison |
| `media/figures/base_x_over_time.png` | Base movement log plot |
| `media/figures/final_lift_recheck_object_z.png` | Object z lift verification plot |
| `media/figures/pinocchio_validation_summary.png` | Pinocchio validation summary |

## Supporting Documents

| File | Description |
|---|---|
| `commands.md` | Reproducible simulation commands |
| `code_evidence.md` | Code-level evidence for the dual robot command architecture |
| `final_weekly_ppt_content.md` | Final slide draft |
| `revised_slides_4_to_8.md` | Revised technical slide notes |

## Current Limitations

- The current grasp is fixed-weld based, not contact-force grasp.
- The base-only movement verifies the command pipeline, not a completed cooperative transportation controller.
- The lift experiment is a verification foundation and still requires grasp-frame and EE tracking improvements.
- The CM-like seed comparison is preliminary and should be replaced with IK-based inverse CM.
