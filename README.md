

https://github.com/user-attachments/assets/559327f4-ee5c-4952-bcbe-b52dec5e30c0







https://github.com/user-attachments/assets/c5752b20-a5c3-4817-a8b1-691c5b017ce5



# hanmin_2026ICROS

ICROS2026 논문 구현을 위한 협동운반 시뮬레이션 진행 기록입니다.

두 대의 Husky-Panda mobile manipulator가 MuJoCo 환경에서 하나의 plate object를 함께 운반하는 구조를 구현하고 있습니다. 현재 단계는 최종 제어기 완성이 아니라, dual robot scene, fixed-weld grasp, leader/follower command 구조, base movement, arm lift 검증 기반을 정리한 것입니다.

## 현재 구현한 내용

- MuJoCo에서 두 대의 Husky-Panda를 동시에 로드
- `l_`, `r_` prefix를 이용해 joint/body/actuator 이름 충돌 방지
- 얇은 plate object를 두 end-effector에 fixed weld로 연결
- leader/follower controller를 분리
- `dual_joint_set_merger.py`로 13 + 13 actuator command를 26 actuator command로 병합
- base-only movement 검증
- fixed-weld grasp 기반 arm lift 검증
- Panda arm URDF를 이용한 Pinocchio FK/Jacobian/manipulability 계산 확인
- baseline seed와 CM-like seed 비교용 예비 pipeline 구성

## 영상

| 파일 | 내용 |
|---|---|
| [`base_movement.mp4`](docs/meetings/2026-05-12/media/base_movement.mp4) | 두 mobile base가 움직이고 fixed-weld plate가 함께 이동하는 장면 |
| [`fixed_weld_grasp.mp4`](docs/meetings/2026-05-12/media/fixed_weld_grasp.mp4) | plate가 두 end-effector에 fixed weld로 연결된 장면 |
| [`arm_lift.mp4`](docs/meetings/2026-05-12/media/arm_lift.mp4) | fixed-weld grasp 상태에서 arm command로 plate 높이를 올리는 검증 |


## 주요 코드 위치

```text
src/kimm_robots_description/husky_description/husky_coop/
  coop_transport_scene.xml
  coop_transport_scene_lift_support_on_generated.xml

src/kimm_phri_panda_husky/launch/
  coop_transport_scene.launch
  coop_transport_dual_base_only_verify.launch
  coop_transport_grasp_lift_verify.launch

src/kimm_phri_panda_husky/python/
  leader_motion_player.py
  follower_coop_planner.py
  leader_base_only_controller.py
  follower_base_only_separated_controller.py
  dual_joint_set_merger.py
  pinocchio_panda_smoke_test.py
  offline_cm_builder.py
  figure2_seed_comparison.py
```
  
## 현재 한계

- 현재 grasp는 실제 contact force grasp가 아니라 fixed weld 기반입니다.
- base movement는 command pipeline 검증 단계이며, 완성된 cooperative transportation controller는 아닙니다.
- arm lift는 가능성 검증 단계이며, 안정적인 end-effector tracking은 다음 단계입니다.
- 현재 CM은 완성된 Zhang-style inverse Capability Map이 아니라, baseline seed와 CM-like seed 비교용 예비 구조입니다.

## 다음 단계

1. end-effector grasp frame과 object grasp frame 재정렬
2. support-on 상태에서 lift 안정화
3. lift 후 base 이동 실험
4. base movement와 arm lift 동시 수행
5. follower arm end-effector tracking 추가
6. IK 기반 inverse Capability Map 구현
7. narrowing corridor 주행 시나리오 검증

