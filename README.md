# Capability Map 기반 초기 자세 Seed 선택을 통한 차동구동 모바일 매니퓰레이터 협업 운반 제어

2026 ICROS 학부연구생 연구 프로젝트 · 충남대학교 메카트로닉스공학과 iCIR Lab

두 대의 Husky-Panda 모바일 매니퓰레이터가 하나의 물체를 안정적으로 협업 운반하기 위한 시뮬레이션 프로젝트입니다. 운반 시작 전 Capability Map(CM)을 이용해 로봇 팔의 초기 자세 seed를 선택하고, Whole-Body Hierarchical Quadratic Programming(HQP) 제어를 통해 물체 자세, 로봇 간 formation, 팔 조작성과 차동구동 제약을 함께 고려합니다.

## 발표 자료와 시연 영상

| 자료 | 내용 |
|---|---|
| [학부연구생 포스터 PDF](assets/poster/학부연구생포스터_유한민.pdf) | 연구 배경, 협업 운반 모델, Whole-Body HQP, CM 기반 seed 선택 및 평가 결과 |
| [전체 시나리오 영상](assets/video/2026ICROS_학부연구생_유한민.mp4) | CM 미적용·적용 시나리오와 운반 궤적 비교를 포함한 통합 영상 |

[![학부연구생 포스터 미리보기](assets/poster/poster_preview.jpg)](assets/poster/학부연구생포스터_유한민.pdf)

[![전체 시나리오 영상 미리보기](assets/video/video_preview.jpg)](assets/video/2026ICROS_학부연구생_유한민.mp4)

미리보기를 클릭하면 각각 포스터 PDF와 전체 시나리오 영상이 열립니다.

## 연구 배경 및 목표

협업 운반에서는 다음 조건을 동시에 만족해야 합니다.

- 운반 물체의 자세와 안정성 유지
- 두 로봇 사이의 grasp formation 유지
- 로봇 팔의 조작성과 관절 여유 확보
- 차동구동 모바일 베이스의 비홀로노믹 제약 만족
- 장애물 회피 과정에서 실행 가능한 베이스·팔 자세 생성

부적절한 초기 자세는 운반 중 로봇 팔을 특이점이나 관절 한계에 가깝게 만들 수 있습니다. 본 연구는 운반 시작 전에 여러 자세 후보를 평가하고, 협업 운반에 유리한 초기 자세를 선택하는 CM 기반 방법을 다룹니다.

## 시스템 구성

- **플랫폼:** 2 × Husky 차동구동 모바일 베이스
- **매니퓰레이터:** 2 × Franka Emika Panda
- **시뮬레이터:** MuJoCo + ROS
- **제어 구조:** leader/follower cooperative transport
- **운반 물체:** 두 end-effector에 연결된 plate object

각 로봇의 목표 end-effector 자세는 목표 물체 자세와 물체 기준 grasp transform으로부터 계산됩니다. Whole-Body HQP 제어기는 이 목표와 차동구동 제약, 관절 및 토크 한계를 계층적으로 처리합니다.

## Capability Map 기반 Seed 선택

초기 자세 후보는 다음 세 항목을 함께 평가합니다.

| 평가 항목 | 의미 |
|---|---|
| Manipulability $\mu(q)$ | 목표 이동 방향에 대한 end-effector 조작성과 관절 여유 |
| Grasp Formation $F(q)$ | 두 end-effector와 물체 사이의 grasp 관계 유지 가능성 |
| Base Feasibility $B(q)$ | 베이스 자세, 장애물 여유, 회전 및 전진 가능성 |

포스터에서 사용한 종합 점수는 다음과 같습니다.

$$
S_{CM}=\mu(q)^{0.40}\times F(q)^{0.35}\times B(q)^{0.25}
$$

가장 높은 점수의 자세를 Whole-Body HQP 제어기의 posture reference로 사용합니다.

## 전체 시나리오

1. **CM 미적용:** 기본 자세에서 운반을 시작하며, 측방 회피 중 팔의 조작성과 관절 여유가 감소하는 상황을 확인합니다.
2. **CM 적용:** 회피 방향과 물체–로봇 관계를 고려해 선택된 자세에서 운반을 시작합니다.
3. **협업 운반:** leader가 물체와 이동 기준을 생성하고 follower가 formation을 유지하며 추종합니다.
4. **결과 비교:** leader/follower의 base 및 end-effector 궤적으로 CM 적용 전후의 실행 가능성과 운반 안정성을 비교합니다.

## 평가 결과

포스터의 feasibility 평가에서는 회피 방향에 맞는 `q1` posture seed를 선택했을 때 y 방향 조작성과 관절 여유가 증가했습니다. 특히 $|dy|=0.50\,\mathrm{m}$ 조건에서 양방향 lateral avoidance의 feasible 영역이 baseline 자세보다 확장되는 것을 확인했습니다.

## 현재 구현한 내용

- `l_`, `r_` prefix를 적용한 dual Husky-Panda MuJoCo scene
- 두 end-effector와 plate object 사이의 fixed-weld grasp
- leader/follower 명령 구조와 13 + 13 actuator command 병합
- base movement와 arm lift 검증 경로
- Panda URDF 기반 Pinocchio FK/Jacobian/manipulability 계산
- baseline seed와 CM seed 비교용 offline sampling 및 자동 실험 도구
- scene/reference/controller 상태 기록과 결과 요약

권장 명령 흐름은 다음과 같습니다.

```text
leader controller   ──> /leader/joint_command   ──┐
                                                   ├─> dual_joint_set_merger ──> MuJoCo joint_set
follower controller ──> /follower/joint_command ──┘
```

분리된 제어기를 사용할 때는 `dual_joint_set_merger.py`만 최종 26-actuator `joint_set`을 발행하도록 구성해 명령 충돌을 방지합니다.

## 주요 코드

```text
src/kimm_robots_description/husky_description/husky_coop/
  coop_transport_scene.xml                 # dual robot 협업 운반 scene

src/kimm_phri_panda_husky/launch/
  coop_transport_scene.launch              # 기본 MuJoCo scene
  coop_transport_dual_base_only_verify.launch
  coop_transport_grasp_lift_verify.launch

src/kimm_phri_panda_husky/python/
  leader_motion_player.py                  # leader 이동·물체 reference
  follower_coop_planner.py                 # follower 목표와 협업 지표
  dual_joint_set_merger.py                 # 13 + 13 actuator 명령 병합
  offline_cm_builder.py                    # Pinocchio 기반 local CM 생성
  figure2_seed_comparison.py               # baseline/CM seed 비교
  run_baseline_vs_cm_experiment.py         # 비교 실험 자동 실행
  baseline_vs_cm_summary.py                # 결과 요약
```

## 실행 방법

아래 명령은 catkin workspace의 루트에서 실행합니다. Pinocchio가 `/opt/openrobots`에 설치된 환경을 기준으로 합니다.

```bash
source devel/setup.bash
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PATH=/opt/openrobots/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

### 1. Offline CM 생성

```bash
python3 src/kimm_phri_panda_husky/python/offline_cm_builder.py \
  --mode pinocchio \
  --output /tmp/mm_verify_logs/pinocchio_offline_cm_samples.csv
```


## 현재 범위와 한계

- 현재 물체 파지는 순수 접촉력 기반 grasp가 아닌 fixed-weld 방식입니다.
- follower arm의 정밀 end-effector tracking과 완전한 dual Whole-Body HQP 제어는 계속 개발 중입니다.
- 운반 중 online local CM 재탐색은 현재 범위에 포함되지 않습니다.

## 향후 연구

1. 접촉력 기반 grasp 및 물체 안정성 평가
2. follower end-effector tracking과 dual Whole-Body HQP 통합
3. IK 기반 inverse Capability Map 구축
4. 협소 통로와 low-ceiling 환경에서의 회피 시나리오 확장
5. 운반 중 online CM 재탐색 및 seed 갱신

## Authors

- 유한민 — 충남대학교 메카트로닉스공학과
- 박진성 — 충남대학교 메카트로닉스공학과 / iCIR Lab
