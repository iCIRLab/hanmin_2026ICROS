# KIMM 기존 코드 vs 현재 구현: 코드 근거 비교

## 발표에서 말할 핵심

기존 KIMM 코드는 두 로봇을 하나의 C++ controller 안에서 동시에 다루는 구조이고,
현재 구현은 leader/follower controller를 분리한 뒤 merger로 합치는 구조이다.

즉, 기존 KIMM 코드가 "무조건 더 낫다"가 아니라,
기존 코드는 actuator index와 두 RobotWrapper 운용 방식의 참고 구현이고,
현재 구조는 논문 목표인 prescribed leader + cooperative follower에 더 맞게 분리한 구조이다.

## 1. 두 로봇을 MuJoCo에 불러오는 방식

| 항목 | 코드 근거 | 발표 설명 |
|---|---|---|
| 기존 KIMM double scene | `double_husky_panda_hand.xml:10-11`<br>`<include file="../robots/l_husky_with_panda_hand.xml"/>`<br>`<include file="../robots/r_husky_with_panda_hand.xml"/>` | MuJoCo에는 left/right prefix가 붙은 robot XML 두 개를 include한다. |
| 현재 coop scene | `coop_transport_scene.xml:5-6`<br>`<include file="../robots/l_husky_with_panda_hand.xml"/>`<br>`<include file="../robots/r_husky_with_panda_hand.xml"/>` | 현재 구현도 같은 방식으로 두 Husky-Panda를 불러온다. 두 로봇 로드 방식 자체는 기존 KIMM 방식과 일관된다. |

## 2. 기존 KIMM controller는 single URDF를 두 번 로드

| 항목 | 코드 근거 | 발표 설명 |
|---|---|---|
| launch에서 single URDF 지정 | `local_viewer.launch:8-10`<br>`model_file=.../double_husky_panda_hand.xml`<br>`robot_urdf=/husky_double/husky_panda_hand_free.urdf` | 시뮬레이션 모델은 double XML이지만, controller/Pinocchio 쪽 URDF는 single Husky-Panda URDF이다. |
| 같은 URDF로 robot_[0], robot_[1] 생성 | `main.cpp:102`<br>`robot_[0] = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, true, false);`<br><br>`main.cpp:109`<br>`robot_[1] = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, true, false);` | 두 대짜리 URDF 하나를 만든 것이 아니라, single mobile manipulator model을 두 번 생성해서 left/right로 다룬다. |
| URDF가 single mobile manipulator인 근거 | `husky_panda_hand_free.urdf:55,63,71`<br>`v_x_joint`, `v_y_joint`, `v_t_joint`<br><br>`husky_panda_hand_free.urdf:337-493`<br>`panda_joint1` ~ `panda_joint7` | URDF 내부에는 mobile base를 위한 가상 x/y/yaw joint와 Panda 7 joints가 있다. dual robot 전체 URDF가 아니라 single robot model이다. |

## 3. 기존 KIMM은 한 controller가 26 actuator를 직접 채움

| 항목 | 코드 근거 | 발표 설명 |
|---|---|---|
| 최종 `/joint_set` publisher가 C++ controller 안에 있음 | `main.cpp:84`<br>`joint_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("/mujoco_ros_interface/joint_set", 5);` | 기존 구조는 하나의 controller가 최종 MuJoCo command topic에 직접 publish한다. |
| left/right torque를 한 메시지로 합침 | `main.cpp:839-840`<br>`setJointCommand(state_[0].torque_, state_[1].torque_);`<br>`joint_command_pub_.publish(joint_command_msg_);` | left/right robot torque를 내부에서 합쳐 26 actuator command로 보낸다. |
| left wheel/arm index | `main.cpp:875-881`<br>`torque[0..3] = left wheel`<br>`torque[i+4] = left arm` | left robot command slice는 0~12 영역이다. |
| right wheel/arm index | `main.cpp:885-890`<br>`torque[13..16] = right wheel`<br>`torque[i+17] = right arm` | right robot command slice는 13~25 영역이다. |

## 4. 현재 구현은 controller를 분리하고 merger가 26 actuator를 생성

| 항목 | 코드 근거 | 발표 설명 |
|---|---|---|
| leader controller는 13 actuator command publish | `leader_base_only_controller.py:58-60`<br>`base_ref_topic = /leader/base_ref`<br>`command_topic = /leader/joint_command` | leader는 prescribed reference를 보고 자기 13 actuator 명령만 만든다. |
| follower controller는 topic/index를 follower로 override | `follower_base_only_separated_controller.py:11-14`<br>`base_ref_topic = /follower/base_target`<br>`command_topic = /follower/joint_command` | follower는 follower target을 보고 자기 13 actuator 명령만 만든다. |
| follower qpos/qvel slice 분리 | `follower_base_only_separated_controller.py:15-22`<br>`qpos_offset=20`, `qvel_offset=19`<br>`arm_qpos_offset=31`, `arm_qvel_offset=29` | joint_states에서 follower robot state를 별도 slice로 읽는다. |
| merger 입력 topic | `dual_joint_set_merger.py:34-37`<br>`/leader/joint_command`<br>`/follower/joint_command`<br>`/coop_scene/.../joint_set` | leader/follower command를 중간 topic으로 분리하고 최종 publish는 merger 하나가 담당한다. |
| 13+13 -> 26 merge | `dual_joint_set_merger.py:105-115`<br>`torque = fit_command(leader + follower, self.output_size)`<br>`msg.torque = torque` | command overwrite를 피하기 위해 두 13-element command를 하나의 26-element JointSet으로 합친다. |
| launch에서 세 노드 실행 | `coop_transport_dual_base_only_verify.launch:41-78`<br>`leader_base_only_controller`<br>`follower_base_only_separated_controller`<br>`dual_joint_set_merger` | 실제 실행 구조도 leader, follower, merger 3개 노드로 분리되어 있다. |

## 발표용 한 문장

기존 KIMM double controller는 single Husky-Panda URDF를 두 번 로드하고, 하나의 C++ controller가 left/right torque를 26 actuator command로 직접 합치는 구조였다. 현재 구현은 같은 13+13 actuator 구조를 유지하되, 논문 목표에 맞게 leader/follower controller를 분리하고 `dual_joint_set_merger`가 최종 26 actuator command를 생성하도록 바꾸었다.

## 현재 구현이 기존 KIMM 코드에서 참고해야 할 부분

| 참고할 부분 | 이유 |
|---|---|
| `robot_[0]`, `robot_[1]`처럼 같은 URDF를 두 번 로드하는 방식 | dual robot control에서도 model을 두 개의 independent robot instance로 다루는 것이 자연스럽다. |
| `setJointCommand()`의 actuator index mapping | left 0~12, right 13~25 mapping 검증 기준으로 사용할 수 있다. |
| `computeAllTerms()`, `nonLinearEffects()` 사용 | arm gravity/coriolis compensation 개선 시 참고 가능하다. |
| `TaskSE3Equality`, `TaskMobileEquality`, `TaskJointPosture` | 향후 full WBC/HQP로 넘어갈 때 참고 가능하다. |

## 현재 구현을 유지해야 하는 이유

| 현재 구조 | 유지 이유 |
|---|---|
| leader/follower controller 분리 | 논문 설정이 prescribed leader + cooperative follower이기 때문이다. |
| `/leader/joint_command`, `/follower/joint_command` 중간 topic | 두 controller가 같은 `/joint_set`을 덮어쓰는 문제를 방지한다. |
| `dual_joint_set_merger.py` | 최종 MuJoCo command publisher를 하나로 제한한다. |
| Pinocchio arm-only 검증 경로 | CM seed, manipulability, 향후 IK-CM 구현에 바로 연결된다. |

