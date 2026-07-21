# Capability Map 기반 초기 자세 Seed 선택을 통한 차동구동 모바일 매니퓰레이터 협업 운반 제어

2026 ICROS 학부연구생 연구 프로젝트 · 충남대학교 메카트로닉스공학과 iCIR Lab

두 대의 Husky-Panda 모바일 매니퓰레이터가 하나의 물체를 협업 운반하는 ROS1/MuJoCo 시뮬레이션입니다. 운반 시작 전 Capability Map(CM)으로 선택한 팔 자세와 일반 home 자세를 같은 운반 조건에서 비교하며, 각 로봇의 Whole-Body HQP 명령을 하나의 26축 MuJoCo command로 병합합니다.

## 발표 자료와 전체 시나리오

| 자료 | 내용 |
|---|---|
| [학부연구생 포스터 PDF](assets/poster/학부연구생포스터_유한민.pdf) | 연구 배경, Whole-Body HQP, CM seed 선정 및 평가 |
| [전체 시나리오 영상](assets/video/2026ICROS_학부연구생_유한민.mp4) | Home baseline과 CM 적용 협업 운반 비교 |

[![포스터 미리보기](assets/poster/poster_preview.jpg)](assets/poster/학부연구생포스터_유한민.pdf)

[![전체 시나리오 영상 미리보기](assets/video/video_preview.jpg)](assets/video/2026ICROS_학부연구생_유한민.mp4)

## 저장소 범위

이 저장소에는 두 비교 시나리오를 구현하는 C++ 패키지만 포함합니다. 이전 Python 검증 코드, 사용하지 않는 launch, RViz/RQT 설정, robot mesh와 전체 description 패키지는 포함하지 않습니다.

```text
src/kimm_phri_panda_husky/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── cm_transport.launch
│   └── home_transport.launch
├── config/
│   └── holding_cm_seed_lateral_nullspace.yaml
├── include/kimm_phri_panda_husky/
│   └── panda_husky_hqp.h
└── src/
    ├── coop_husky_hqp_node.cpp
    ├── coop_phri_simul.cpp
    └── panda_husky_hqp.cpp
```

## 비교 시나리오

### 1. CM 초기 자세 적용

`holding_cm_seed_lateral_nullspace.yaml`의 `selected/q_sym`을 mode 238에서 양팔 posture reference로 적용합니다.

```bash
roslaunch kimm_phri_panda_husky cm_transport.launch
```

키 입력 순서:

```text
c  simulation-only weld attach
y  CM posture 적용 및 0.10 m lift (mode 238)
0  협업 운반 시작 (mode 235)
```

### 2. CM 미적용 Home Baseline

CM seed를 발행하지 않고 일반 home posture를 유지한 상태에서 같은 lift와 운반 경로를 수행합니다. `home_transport.launch`는 `cm_transport.launch`를 포함하고 baseline에 필요한 인자만 재정의합니다.

```bash
roslaunch kimm_phri_panda_husky home_transport.launch
```

키 입력 순서:

```text
c  simulation-only weld attach
t  home posture 유지 및 0.10 m lift (mode 211)
0  협업 운반 시작 (mode 235)
```

## 실행 구조

`cm_transport.launch`가 다음 네 노드를 구성합니다.

```text
/coop_scene/mujoco_ros
          │ integrated state
          ▼
/coop_phri_simul ── object target, attach/lift, CM runtime, command merge
          ▲                                  │
          │                                  ▼
/leader_hqp_controller    13축 command ──┐
                                         ├── 26축 MuJoCo joint command
/follower_hqp_controller  13축 command ──┘
```

| 파일 | 역할 |
|---|---|
| `launch/cm_transport.launch` | MuJoCo와 leader/follower HQP 노드를 실행하는 기본 CM 시나리오 |
| `launch/home_transport.launch` | CM launch를 재사용하는 home baseline 시나리오 |
| `config/holding_cm_seed_lateral_nullspace.yaml` | 초기 상태, 선택된 offline CM seed와 평가 지표 |
| `src/coop_phri_simul.cpp` | weld 시험, object target, lift readiness, 키 입력과 26축 command 병합 |
| `src/coop_husky_hqp_node.cpp` | 통합 상태 분할, reference/Fext 입력과 로봇별 13축 명령 발행 |
| `src/panda_husky_hqp.cpp` | posture 및 협업 운반 Whole-Body HQP 제어 |

## Capability Map Seed

초기 자세 후보는 팔 조작성 $\mu(q)$, grasp formation $F(q)$, base feasibility $B(q)$를 함께 평가합니다.

$$
S_{CM}=\mu(q)^{0.40}\times F(q)^{0.35}\times B(q)^{0.25}
$$

현재 시나리오는 운반 중 CM을 반복 탐색하는 online planner가 아닙니다. 사전에 선택한 `q_sym`을 운반 시작 자세로 적용하고, 이후 장애물 회피 방향과 yaw 정렬은 mode 235의 base trajectory 및 HQP mobile task가 담당합니다.

## Control Mode

| Mode | 담당 | 의미 |
|---:|---|---|
| 21 | orchestration | simulation weld attach 시험 |
| 200 | HQP | 외부 EE/base/posture reference 추종 |
| 211 | orchestration | CM 미적용 home baseline lift |
| 238 | orchestration | CM posture 적용 lift |
| 235 | HQP | 최종 협업 운반 |

## 빌드 및 실행 환경

이 저장소는 핵심 시나리오 패키지만 제공하므로 다음 패키지가 같은 catkin workspace에 준비되어 있어야 합니다.

- ROS1 Noetic, Eigen3, `yaml-cpp`
- `mujoco_ros`, `mujoco_ros_msgs`
- `kimm_hqp_controller`와 `/opt/openrobots`의 Pinocchio 환경
- `husky_description`
  - `husky_coop/coop_transport_scene_straight_clean_generated.xml`
  - `husky_coop/coop_transport_scene_straight_clean_home_baseline.xml`
  - `husky_single/husky_panda_hand.urdf`

```bash
cd <catkin_workspace>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

roslaunch kimm_phri_panda_husky cm_transport.launch --nodes
roslaunch kimm_phri_panda_husky home_transport.launch --nodes
```

## 시뮬레이션 검증 결과

- Home `c → t → 0`: 0.10 m lift 후 mode 235 phase 0–5 완료
- CM `c → y → 0`: CM posture 적용 및 0.10 m lift 후 mode 235 phase 0–5 완료
- 양쪽 13축 command 약 500 Hz 발행 및 NaN/Inf 없음
- 초기 EE/link7/elbow의 물체 내부 관통 없음
- MuJoCo 종료 시 clean shutdown 확인

## 현재 범위와 한계

- `c` 키의 weld attach는 MuJoCo 시뮬레이션 시험용이며 실제 gripper command를 대체하지 않습니다.
- 현재 CM은 offline 초기 seed이며 운반 중 local CM 재탐색은 포함하지 않습니다.
- robot model, mesh와 MuJoCo scene은 `husky_description` 외부 의존 패키지에서 관리합니다.
- 실제 F/T sensor의 힘·토크 방향과 보상 gain은 실제 하드웨어에서 추가 검증해야 합니다.

## Authors

- 유한민 — 충남대학교 메카트로닉스공학과
- 박진성 — 충남대학교 메카트로닉스공학과 / iCIR Lab

## License

MIT License. 자세한 내용은 [LICENSE](LICENSE)를 참고하십시오.
