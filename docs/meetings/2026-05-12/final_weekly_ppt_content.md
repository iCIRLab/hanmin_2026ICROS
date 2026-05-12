# 이번주 면담 발표자료 최종안

발표 주제: ICROS2026 협동운반 시뮬레이션 구현 진행 상황

---

## Slide 1. 연구 목표와 현재 단계

### 제목
Capability Map 기반 협동운반 시뮬레이션 구현

### 핵심 내용
- 최종 목표: 협동운반 중 mobile manipulator가 optimal posture를 유지
- 시스템: 두 대의 Husky-Panda mobile manipulator + rigid plate object
- 현재 구현 단계: masterless가 아니라 prescribed leader + cooperative follower 구조
- 이번 목표: 두 로봇을 MuJoCo에 띄우는 단계를 넘어, 실제 제어 가능한 구조로 확장

### 발표 멘트
> 최종적으로는 협동운반 중 follower가 Capability Map을 이용해 좋은 자세를 유지하도록 하는 것이 목표입니다. 현재는 완전한 masterless 구조가 아니라, leader는 미리 정의된 trajectory를 재생하고 follower가 이를 따라가는 중간 단계로 구현하고 있습니다.

### 이미지
- Dual Husky-Panda + plate scene 전체 화면

---

## Slide 2. 저번주 진행 내용

### 제목
저번주: MuJoCo dual robot scene 구성

### 핵심 내용
- MuJoCo XML에서 두 Husky-Panda를 동시에 로드
- `l_`, `r_` prefix가 붙은 robot XML을 include하여 이름 충돌 방지
- plate object와 corridor/map 환경 추가
- 교수님 피드백: “두 로봇을 불러오는 것은 잘 되었고, 이제 제어 관점에서 어떻게 분리해 움직일지 확인 필요”

### 코드 근거
```xml
<!-- coop_transport_scene.xml -->
<include file="../robots/l_husky_with_panda_hand.xml"/>
<include file="../robots/r_husky_with_panda_hand.xml"/>
```

### 발표 멘트
> 저번주에는 MuJoCo XML에서 두 모바일 매니퓰레이터를 동시에 불러오는 환경을 구성했습니다. 핵심은 left/right robot XML에 prefix를 붙여 joint, body, actuator 이름 충돌을 피한 것입니다. 다만 이 단계는 화면에 두 로봇을 띄운 것이고, 실제 제어 관점의 분리는 아직 다음 과제였습니다.

### 이미지
- 전체 scene screenshot
- XML include 구조 간단 다이어그램

---

## Slide 3. 기존 KIMM 코드 분석

### 제목
기존 KIMM double robot 구조 분석

### 핵심 내용
- 기존 KIMM 코드는 두 대짜리 URDF 하나를 쓰는 방식이 아님
- MuJoCo에서는 `l_`, `r_` robot XML 두 개를 include
- Controller/Pinocchio에서는 single Husky-Panda URDF를 두 번 로드
- 하나의 C++ controller가 left/right torque를 26 actuator command로 직접 합침

### 코드 근거
```cpp
// husky_double_launch/src/main.cpp
robot_[0] = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, true, false);
robot_[1] = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, true, false);
```

```cpp
setJointCommand(state_[0].torque_, state_[1].torque_);
joint_command_pub_.publish(joint_command_msg_);
```

### 발표 멘트
> 기존 KIMM double 코드를 확인해보니, 제어 쪽에서는 single Husky-Panda URDF를 두 번 로드해서 `robot_[0]`, `robot_[1]`로 다루고 있었습니다. 즉 두 대짜리 하나의 URDF가 아니라, 같은 mobile manipulator model을 두 instance로 쓰는 구조입니다. 이 방식은 현재 구현에서도 중요한 참고 기준이 됩니다.

### 이미지
- 기존 KIMM 구조: `single URDF x2 -> one controller -> 26 actuator`

---

## Slide 4. 이번주 구현 1: command 구조 분리

### 제목
두 로봇을 움직이기 위한 separated command architecture

### 핵심 내용
- Dual scene actuator 구조: leader 13개 + follower 13개 = 총 26개
- 두 controller가 같은 `/joint_set`에 publish하면 command overwrite 가능
- 따라서 leader/follower controller를 분리하고, merger가 최종 26 actuator command 생성

### 구현 구조
```text
leader_base_only_controller
  -> /leader/joint_command       13 actuator

follower_base_only_separated_controller
  -> /follower/joint_command     13 actuator

dual_joint_set_merger
  -> /coop_scene/.../joint_set   26 actuator
```

### 코드 근거
```python
# dual_joint_set_merger.py
torque = fit_command(leader + follower, self.output_size)
msg.torque = torque
self.pub.publish(msg)
```

### 발표 멘트
> 이번주 핵심 구현은 두 로봇의 command를 분리한 것입니다. 기존처럼 한 controller가 모든 것을 직접 제어할 수도 있지만, 제 논문에서는 leader와 follower 역할이 다르기 때문에 controller를 분리하는 편이 더 적합합니다. 그래서 leader와 follower가 각각 13 actuator command를 만들고, merger가 26 actuator command로 합쳐 MuJoCo에 전달하도록 구성했습니다.

### 이미지
- command architecture diagram

---

## Slide 5. 이번주 구현 2: base-only movement 검증

### 제목
Base-only movement verification

### 핵심 내용
- 이번 단계에서는 arm trajectory tracking보다 base movement를 먼저 검증
- leader base: prescribed `/leader/base_ref` 추종
- follower base: `/follower/base_target`을 따라 yaw 정렬 후 forward assist
- arm은 자세 유지 또는 약한 hold 상태
- fixed-weld plate는 base 이동에 따라 함께 움직임

### 실행 명령
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

### 발표 멘트
> 첫 번째 검증은 base-only movement입니다. leader와 follower base가 각각 target을 받고, yaw를 먼저 정렬한 뒤 forward command를 주는 방식입니다. 이 실험을 통해 두 로봇의 command pipeline이 실제로 MuJoCo에서 동작하고, fixed-weld plate가 base 이동에 따라 함께 움직이는 것을 확인했습니다. 다만 아직 좁은 통로를 안정적으로 통과하는 완성된 협동운반은 아닙니다.

### 이미지
- base trajectory plot
- MuJoCo movement screenshot 또는 영상 캡처

---

## Slide 6. 이번주 구현 3: grasp/lift 검증 기반

### 제목
Fixed-weld grasp와 lift 검증

### 핵심 내용
- 현재 grasp는 contact grasp가 아니라 fixed weld 기반
- plate가 두 EE grasp frame에 고정되어 있음
- support 위에서 시작한 뒤 object z target을 높이는 lift 검증 수행
- 목표: 물체가 급락하지 않고 z 방향으로 상승 가능한지 확인
- 아직 안정적인 lift controller 완성 단계는 아님

### 실행 명령
```bash
roslaunch kimm_phri_panda_husky coop_transport_grasp_lift_verify.launch \
  duration:=4 \
  rate:=10 \
  command_rate:=100 \
  object_lift_height:=0.1 \
  scene_log_output:=/tmp/mm_verify_logs/lift_scene.csv \
  leader_controller_log_output:=/tmp/mm_verify_logs/lift_leader.csv \
  follower_controller_log_output:=/tmp/mm_verify_logs/lift_follower.csv
```

### 발표 멘트
> 두 번째 검증은 lift 가능성 확인입니다. 아직 실제 gripper contact force로 잡는 단계는 아니고, fixed weld로 물체와 EE를 연결한 상태입니다. object z error를 기반으로 arm command를 생성해서 plate가 위로 움직일 수 있는지 확인했습니다. 현재는 lift 가능성 검증 단계이고, 다음에는 grasp frame 정렬과 EE tracking을 더 개선해야 합니다.

### 강조하지 않을 내용
- support block 위치 조정 같은 세부 XML 수정은 본문에서 길게 설명하지 않음
- 질문이 나오면 “초기 조건 안정화를 위해 support/plate 높이를 재정렬했다”고 답변

### 이미지
- gripper/object close-up
- object z lift plot

---

## Slide 7. URDF / Pinocchio / CM seed 현재 상태

### 제목
모델 기반 계산 경로와 CM seed 비교 기반

### 핵심 내용
- Pinocchio 검증은 full mobile manipulator WBC가 아님
- Panda arm-only URDF에 대해 FK/Jacobian/manipulability 계산 경로 확인
- 향후 IK 기반 inverse CM으로 확장 예정
- 현재 CM은 완성된 Zhang-style CM이 아니라 seed comparison pipeline

### Pinocchio 실행 명령
```bash
python3 src/kimm_phri_panda_husky/python/pinocchio_panda_smoke_test.py \
  --urdf /home/ryoo/mmcm_ws/src/kimm_robots_description/franka_panda_description/robots/panda_arm_hand_l.urdf \
  --output /tmp/mm_verify_logs/pinocchio_left_smoke.txt
```

```bash
python3 src/kimm_phri_panda_husky/python/pinocchio_panda_smoke_test.py \
  --urdf /home/ryoo/mmcm_ws/src/kimm_robots_description/franka_panda_description/robots/panda_arm_hand_r.urdf \
  --output /tmp/mm_verify_logs/pinocchio_right_smoke.txt
```

### CM 설명
- 현재 의미: baseline seed vs local/CM-like seed 비교 구조
- 계산 항목: posture error, manipulability proxy, formation proxy, cost proxy
- 아직 부족한 부분: IK 기반 inverse reachability CM

### 발표 멘트
> 이번주에는 Pinocchio를 이용해 Panda arm-only URDF의 FK, Jacobian, manipulability 계산 경로를 확인했습니다. 이것이 바로 완성된 whole-body controller를 의미하는 것은 아니고, 향후 CM seed 계산과 IK 기반 inverse CM을 구현하기 위한 준비 단계입니다. 현재 Figure 2는 완성된 CM 결과라기보다 baseline seed와 local CM-like seed를 비교하는 예비 pipeline입니다.

### 이미지
- Figure 2 seed comparison
- Pinocchio validation summary

---

## Slide 8. 현재 한계와 다음주 계획

### 제목
현재 한계 및 다음 단계

### 현재 한계
- base-only movement는 확인했지만, full cooperative transport는 아직 아님
- arm EE tracking은 아직 제한적
- grasp는 contact grasp가 아니라 fixed weld 기반
- lift는 가능성 검증 단계이며 안정적인 lifting controller는 아님
- CM은 아직 Zhang-style IK inverse CM이 아님
- narrowing corridor 통과는 장거리 route/controller 안정화가 더 필요

### 다음주 계획
1. EE/object grasp frame 재정렬
2. support-on 상태에서 lift 안정화
3. lift 후 base 이동 실험
4. leader/follower arm posture hold 강화
5. IK 기반 inverse CM 구현
6. baseline vs CM seed 비교 실험 고도화
7. narrowing route scenario 추가 및 통과 검증

### 발표 멘트
> 정리하면, 저번주는 두 로봇을 MuJoCo에 불러오는 단계였고, 이번주는 이를 실제 제어 가능한 구조로 확장하는 단계였습니다. 특히 13+13 actuator command를 분리하고 merger로 합치는 구조를 구현해 base movement를 확인했습니다. 다음주는 grasp/lift 초기 조건을 더 안정화하고, lift 후 base 이동 및 IK 기반 CM으로 넘어가는 것이 목표입니다.

### 이미지
- confirmed vs remaining summary
- next-step roadmap

---

# 발표 전체 흐름 요약

## 1분 요약

저번주에는 MuJoCo XML에서 두 Husky-Panda를 동시에 불러오는 환경을 만들었다.  
이번주에는 기존 KIMM double robot 코드를 분석하여 single URDF를 두 번 로드하고 26 actuator command로 합치는 구조를 확인했다.  
이를 바탕으로 현재 연구 목표에 맞게 leader/follower controller를 분리하고, `dual_joint_set_merger`로 13+13 command를 합치는 구조를 구현했다.  
Base-only 실험에서는 두 모바일 베이스가 reference를 따라 움직이고 fixed-weld plate가 함께 이동하는 것을 확인했다.  
Lift 실험은 아직 완성 단계는 아니지만, fixed-weld grasp 상태에서 object z 방향 상승 가능성을 검증하는 기반을 만들었다.  
다음주는 grasp frame 정렬, lift 안정화, lift 후 base 이동, IK 기반 inverse CM 구현을 진행할 계획이다.

---

# 발표에 사용할 이미지 목록

| 용도 | 추천 파일 |
|---|---|
| 전체 scene | `/tmp/mm_presentation_assets/current_week/images/mujoco_actual_object_closeup.png` 또는 직접 viewer 캡처 |
| command 구조 | `/tmp/mm_presentation_assets/current_week/images/command_architecture.png` |
| base movement plot | `/tmp/mm_presentation_assets/current_week/plots/base_x_over_time.png` |
| base tracking/saturation | `/tmp/mm_presentation_assets/current_week/plots/base_tracking_and_saturation.png` |
| lift 결과 | `/tmp/mm_presentation_assets/current_week/plots/final_lift_recheck_object_z.png` |
| Figure 2 seed 비교 | `/tmp/mm_presentation_assets/current_week/images/figure2_seed_comparison.png` |
| 현재/남은 과제 | `/tmp/mm_presentation_assets/current_week/images/confirmed_vs_remaining_summary.png` |

---

# 교수님 질문 대비 답변

## Q1. 기존 KIMM 코드가 더 나은 것 아닌가?

기존 KIMM 코드는 두 로봇을 한 controller 안에서 동시에 다루는 완성도 높은 참고 구조입니다.  
하지만 제 논문에서는 leader와 follower 역할이 다르기 때문에, controller를 분리하고 command merger로 합치는 구조가 더 적합합니다.  
기존 코드는 actuator index, RobotWrapper 두 개 운용, gravity compensation 참고용으로 활용하고 있습니다.

## Q2. 두 대짜리 URDF를 만든 것인가?

아닙니다. 기존 KIMM도 두 대짜리 URDF 하나를 쓰지 않고, single Husky-Panda URDF를 두 번 로드합니다.  
MuJoCo scene에서는 left/right prefix가 붙은 XML 두 개를 include합니다.  
현재 구현도 이 원칙을 따르고 있습니다.

## Q3. 지금 물체를 실제 gripper가 잡고 있는가?

현재는 contact force grasp가 아니라 fixed weld grasp입니다.  
즉, 물리적으로 gripper contact로 잡는 단계는 아니고, EE grasp frame과 object grasp frame을 fixed constraint로 연결한 상태입니다.  
논문/demo 초기 단계에서는 cooperative transport constraint를 안정화하기 위해 fixed weld를 먼저 사용하고 있습니다.

## Q4. CM은 완성되었는가?

아직 완성된 Zhang-style inverse CM은 아닙니다.  
현재는 baseline seed와 local/CM-like seed를 비교하는 pipeline입니다.  
다음 단계에서 Pinocchio IK를 이용해 object grasp target에 대해 가능한 q와 base pose를 찾는 inverse CM으로 확장할 예정입니다.

---

# 최종 발표 메시지

저번주: 두 로봇을 MuJoCo에 로드하는 환경 구성  
이번주: 두 로봇을 제어 가능한 구조로 분리하고 base movement/lift 가능성 검증  
다음주: grasp/lift 안정화와 IK 기반 CM으로 확장

