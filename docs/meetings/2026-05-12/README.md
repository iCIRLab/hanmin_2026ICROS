# 2026-05-12 주간 면담 기록

이번 폴더는 2026년 5월 12일 면담에서 사용한 최종 발표 자료와 관련 영상, 실행 명령, 코드 근거를 정리한 기록입니다.

## 발표 핵심

- 지난주: MuJoCo에서 두 Husky-Panda를 동시에 불러오는 dual robot scene 구성
- 이번주: 두 로봇을 실제로 움직이기 위한 command 구조와 검증 실험 진행
- 다음주: grasp frame 정렬, lift 안정화, lift 후 base 이동, IK 기반 CM으로 확장

## 포함 파일

| 파일 | 설명 |
|---|---|
| `20260512_weekly_meeting.pdf` | 최종 면담 발표 PDF |
| `commands.md` | 실행 명령 정리 |
| `code_evidence.md` | 기존 KIMM 코드와 현재 구현의 차이를 코드 근거로 정리 |

## 영상

| 파일 | 설명 |
|---|---|
| `media/base_movement.mp4` | base-only movement 검증 |
| `media/fixed_weld_grasp.mp4` | fixed-weld grasp 상태 확인 |
| `media/arm_lift.mp4` | arm lift 검증 |

## 이미지

| 파일 | 설명 |
|---|---|
| `media/figures/command_architecture.png` | leader/follower command 분리 구조 |
| `media/figures/figure2_seed_comparison.png` | baseline seed와 CM-like seed 비교 |
| `media/figures/base_x_over_time.png` | base movement 로그 |
| `media/figures/final_lift_recheck_object_z.png` | object z lift 로그 |
| `media/figures/pinocchio_validation_summary.png` | Pinocchio 검증 요약 |
| `media/figures/mujoco_actual_object_closeup.png` | MuJoCo object close-up |

## 현재 결과 해석

- base movement는 두 로봇 command가 분리되어 들어가고 plate가 함께 이동하는지 확인한 단계입니다.
- arm lift는 fixed-weld grasp 상태에서 object height를 변화시킬 수 있는지 확인한 단계입니다.
- Pinocchio 검증은 Panda arm-only URDF에 대한 FK/Jacobian/manipulability 계산 경로 확인입니다.
- CM-like seed 비교는 완성된 CM 결과가 아니라, 향후 IK 기반 inverse CM으로 확장하기 위한 예비 비교입니다.

