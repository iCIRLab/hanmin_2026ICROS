# 수정 반영본: Slide 4~8

## Slide 4. 두 로봇 command를 어떻게 분리해서 보냈는가

### 제목
Leader/Follower 분리 제어와 command merger

### 핵심 내용
- MuJoCo dual scene의 actuator는 총 26개이다.
  - leader robot: 13개
  - follower robot: 13개
- MuJoCo의 최종 command topic은 하나이다.
  - `/coop_scene/mujoco_ros/mujoco_ros_interface/joint_set`
- 따라서 leader controller와 follower controller가 같은 topic에 따로 publish하면 마지막에 들어온 command가 앞 command를 덮어쓸 수 있다.
- 이를 막기 위해 각 controller는 13개 command만 만들고, `dual_joint_set_merger`가 하나의 26개 command로 합친다.

### 기존 KIMM 코드와 차이

기존 KIMM double controller는 하나의 C++ node 안에서 두 로봇 command를 직접 합쳤다.

```cpp
setJointCommand(state_[0].torque_, state_[1].torque_);
joint_command_pub_.publish(joint_command_msg_);
```

현재 구현은 controller를 분리하고, merger가 같은 역할을 담당한다.

```python
# dual_joint_set_merger.py
torque = fit_command(leader + follower, self.output_size)
msg.torque = torque
self.pub.publish(msg)
```

### 왜 다르게 했는가
- 기존 KIMM 구조: 두 로봇을 하나의 controller 내부에서 함께 제어
- 현재 연구 구조: leader와 follower의 역할이 다름
  - leader: prescribed trajectory 재생
  - follower: leader/object 관계를 보고 협력 target 생성
- 그래서 controller를 분리하는 것이 이후 CM seed, follower planner, cooperative control을 붙이기 쉽다.

### 발표 멘트
> 기존 KIMM 코드는 하나의 C++ controller 안에서 두 로봇의 torque를 합쳐서 26 actuator command를 보냈습니다. 저는 leader와 follower의 역할이 다르기 때문에 controller를 분리했습니다. 다만 MuJoCo는 최종적으로 하나의 `joint_set` topic에서 26개 actuator command를 받아야 하므로, `dual_joint_set_merger`가 leader 13개와 follower 13개 command를 합쳐서 최종 command를 보냅니다. 즉 merger는 새로운 제어기가 아니라 command arbitration layer입니다.

### 그림
```text
leader controller      follower controller
  13 actuator cmd        13 actuator cmd
        |                      |
        v                      v
 /leader/joint_command   /follower/joint_command
        \                      /
         \                    /
          v                  v
          dual_joint_set_merger
                 |
                 v
 /coop_scene/.../joint_set  = 26 actuator cmd
```

---

## Slide 5. Base-only movement는 실제로 어떻게 움직였는가

### 제목
Base-only movement verification

### 핵심 내용
- 이번 단계에서는 arm trajectory tracking보다 mobile base movement를 먼저 검증했다.
- 두 base 모두 differential-drive 방식으로 움직인다.
  - 목표 방향으로 yaw 정렬
  - yaw error가 작아지면 forward command
- leader는 `/leader/base_ref`를 추종한다.
- follower는 `/follower/base_target`을 추종한다.
- plate는 fixed weld로 EE에 연결되어 있어 base 이동에 따라 함께 움직인다.

### 이 단계에서의 leader/follower 의미
- 현재 leader/follower는 완성된 협동운반 제어 개념이 아니라, reference 역할 분리이다.
- leader는 미리 정의된 base trajectory를 따라가는 motion source이다.
- follower는 leader/object 관계에서 생성된 target을 따라간다.
- 아직 force-based cooperative transport나 full WBC 단계는 아니다.

### base command 생성 방식
```text
target pose - actual pose
        |
        v
position error, yaw error 계산
        |
        v
yaw align 먼저 수행
        |
        v
forward + turn command
        |
        v
left/right wheel torque command
```

### 발표 멘트
> 이 실험은 두 로봇이 협동운반을 완성했다는 의미보다는, 두 모바일 베이스에 command가 분리되어 들어가고 실제로 움직이는지를 확인한 것입니다. leader는 prescribed base reference를 따라가고, follower는 follower target을 따라갑니다. 두 controller 모두 yaw를 먼저 맞춘 뒤 전진하는 differential-drive 방식으로 wheel command를 생성합니다. plate는 fixed weld로 연결되어 있기 때문에 base가 움직이면 함께 이동합니다.

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

### 발표에서 조심할 표현
- “완성된 leader-follower cooperative transport”라고 말하지 않기
- “base command pipeline과 fixed-weld object 동반 이동을 확인했다”라고 말하기

---

## Slide 6. 로봇팔 lift는 어떻게 구현했는가

### 제목
Fixed-weld grasp 기반 arm lift verification

### 핵심 내용
- 현재 grasp는 contact grasp가 아니라 fixed weld grasp이다.
- object와 EE grasp frame이 constraint로 연결되어 있다.
- lift 실험에서는 object의 z target을 올리고, 두 arm이 이를 따라 upward command를 만든다.
- 목적은 contact grasp 완성이 아니라, arm command로 object height를 변화시킬 수 있는지 확인하는 것이다.

### arm command 구성
```text
arm command =
  posture hold torque
  + Pinocchio gravity compensation
  + object z error 기반 upward force
  + Jacobian 기반 lift 방향 보정
```

### 코드 관점
- `leader_base_only_controller.py` 안에서 wheel command와 arm command를 함께 만든다.
- base command는 `command[0..3]`
- arm command는 `command[4..10]`
- finger command는 `command[11..12]`

```python
command[0] = command[2] = base_cmd["left"]
command[1] = command[3] = base_cmd["right"]
command[4:11] = arm_cmd
command[11] = finger
command[12] = finger
```

### base-only와 lift를 동시에 할 수 있는가?
- 구조적으로는 가능하다.
  - 같은 13 actuator command 안에 wheel torque와 arm torque가 같이 들어가기 때문
- 하지만 현재는 검증을 분리했다.
  - Step 1: base-only movement 확인
  - Step 2: stationary lift 가능성 확인
  - Step 3: lift 후 base 이동
  - Step 4: lift와 이동 동시 수행
- 지금 바로 동시에 하면 object drift, weld constraint, arm saturation 원인을 분리하기 어렵다.

### 발표 멘트
> lift 실험에서는 실제 gripper contact force로 물체를 잡는 것이 아니라, fixed weld로 연결된 object의 높이를 arm command로 변화시킬 수 있는지 확인했습니다. arm command는 자세 유지 torque에 Pinocchio gravity compensation과 object z error 기반 upward force를 더하는 방식입니다. 구조적으로는 base 이동과 arm lift를 동시에 줄 수 있지만, 현재는 원인 분석을 위해 base movement와 lift를 따로 검증했습니다.

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

### 발표에서 조심할 표현
- “gripper가 실제 contact로 잡았다”라고 말하지 않기
- “fixed-weld grasp에서 lift 가능성을 확인하는 단계”라고 말하기

---

## Slide 7. Pinocchio와 CM seed는 현재 무엇을 했는가

### 제목
Pinocchio 기반 모델 계산과 CM seed 비교 준비

### Pinocchio를 사용한 이유
- controller와 CM 계산에는 robot model 기반 계산이 필요하다.
- 이번주에는 full WBC가 아니라 Panda arm-only 모델에 대해 다음 계산 경로를 확인했다.
  - URDF load
  - joint order 확인
  - FK 계산
  - Jacobian 계산
  - manipulability 계산
  - gravity compensation에 사용할 모델 경로 확인

### 발표 멘트: Pinocchio
> Pinocchio는 제가 trajectory를 직접 만드는 도구라기보다, 로봇팔의 FK, Jacobian, manipulability, gravity compensation을 계산하기 위한 모델 기반 계산 도구입니다. 이번주에는 Panda arm-only URDF가 Pinocchio에서 정상적으로 로드되고, 향후 arm control과 CM 계산에 필요한 Jacobian/manipulability 계산이 가능한지 확인했습니다.

### CM seed 비교가 의미하는 것
- 현재 CM은 완성된 Zhang-style inverse CM이 아니다.
- 현재 구현은 baseline seed와 local/CM-like seed를 비교하는 예비 pipeline이다.
- baseline seed:
  - 기존 home posture 또는 keyframe 초기 자세
- CM-like seed:
  - 여러 candidate posture 중 cost proxy가 더 좋은 후보
- 비교 항목:
  - posture error
  - manipulability proxy
  - formation proxy
  - joint-limit/bound proxy
  - total cost proxy

### 왜 CM-like seed가 더 좋은가
- baseline은 단순 고정 자세이다.
- CM-like seed는 candidate 중에서 manipulability가 더 높고, joint limit/posture/formation proxy가 더 좋은 자세를 선택한다.
- 따라서 Figure 2는 “CM을 쓰면 초기 follower posture를 더 유리한 후보로 선택할 수 있다”는 예비 결과이다.
- 단, 아직 object grasp target에 대해 IK를 푼 inverse CM은 아니므로 과장하면 안 된다.

### 발표 멘트: CM
> Figure 2는 완성된 CM 실험 결과라기보다, baseline seed와 local CM-like seed를 비교하는 예비 그림입니다. baseline은 home posture에 가까운 고정 seed이고, CM-like seed는 candidate 중 manipulability, posture error, formation proxy를 기준으로 더 좋은 cost를 갖는 자세를 선택한 것입니다. 다음 단계에서는 이 구조를 IK 기반 inverse CM으로 바꿔서 object grasp target을 만족하는 base/arm seed를 선택하도록 확장할 계획입니다.

### 실행 명령
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

---

## Slide 8. 현재 한계와 다음주 계획

### 제목
현재 한계 및 다음 단계

### 현재까지 확인한 것
- 두 Husky-Panda를 MuJoCo scene에 로드
- leader/follower 13+13 command 분리 구조 구현
- merger를 통해 26 actuator command publish
- base-only movement에서 두 base가 움직이고 plate가 함께 이동하는 것 확인
- fixed-weld grasp 기반 lift 가능성 검증 시작
- Pinocchio로 Panda arm FK/Jacobian/manipulability 계산 경로 확인
- baseline seed vs CM-like seed 비교 pipeline 구성

### 현재 한계
- base-only movement는 가능하지만 full cooperative transport는 아님
- leader/follower는 아직 역할 분리 reference 수준이며, force/object-aware WBC는 아님
- arm lift는 가능성 검증 단계이고 안정적인 EE tracking controller는 아님
- grasp는 contact grasp가 아니라 fixed weld grasp
- CM은 아직 IK 기반 inverse CM이 아님
- narrowing corridor 통과는 아직 안정적으로 검증되지 않음

### 다음주 계획
1. EE grasp frame과 object grasp frame 재정렬
2. support-on 상태에서 lift 안정화
3. lift 후 base 이동 실험
4. base movement와 arm lift를 동시에 주는 실험으로 확장
5. follower arm EE tracking 추가
6. IK 기반 inverse CM 구현
7. narrowing corridor trajectory 추가 및 통과 검증

### 발표 멘트
> 이번주는 두 로봇을 단순히 띄운 상태에서 벗어나, 실제 command를 분리하고 MuJoCo에 넣는 구조를 만든 것이 핵심입니다. 다만 아직 완성된 협동운반은 아니고, base 이동과 arm lift를 분리해서 검증하는 단계입니다. 다음주는 grasp frame 정렬과 lift 안정화를 먼저 한 뒤, lift 후 base 이동, 그리고 IK 기반 CM으로 확장하는 순서로 진행하려고 합니다.

---

## 교수님 질문 대비: fixed weld grasp

### 질문
지금 물체를 실제 그리퍼가 잡고 있는가?

### 답변
현재는 실제 contact force grasp가 아니라 fixed weld grasp입니다.
즉, gripper finger와 물체 사이의 마찰/contact force로 잡는 것이 아니라,
EE grasp frame과 object grasp frame을 MuJoCo fixed constraint로 연결한 상태입니다.

이 방식을 먼저 쓰는 이유는 grasp 자체보다 협동운반과 자세 유지 알고리즘을 먼저 검증하기 위해서입니다.
다만 논문/demo에서 실제 grasp처럼 보이도록 object grasp tab, gripper pose, EE frame 정렬은 더 개선해야 합니다.

