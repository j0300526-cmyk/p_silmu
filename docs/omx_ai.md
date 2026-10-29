# OMX-AI 대상으로 바뀐 부분

실기 목표는 ROBOTIS OMX-AI의 follower인 OMX-F다.
현재 개발 PC는 Ubuntu 22.04이며 기존 요청의 ROS 2 Humble을 유지한다.
시뮬레이션의 기본 모델을 OMX-F로 바꾸었지만 실기 장치에 명령을 보내는 backend는 추가하지 않았다.

## 모델과 환경의 구분

| 항목 | OMX-F / OMX-AI follower | 기존 OpenManipulator-X |
| --- | --- | --- |
| 팔 구동 관절 | joint1~joint5 (5축) | joint1~joint4 (4축) |
| 그리퍼 구동 관절 | gripper_joint_1, 회전식 rad | gripper_left_joint, 직선식 m |
| 그리퍼 종속 관절 | gripper_joint_2, multiplier=-1 | gripper_right_joint, multiplier=1 |
| 입력 기준 프레임 | link0 | link1 |
| 본 프로젝트 옵션 | robot_model:=omx_f (기본) | robot_model:=open_manipulator_x |
| 공식 실기 소프트웨어 | ROS 2 Jazzy | Humble 공식 브랜치 존재 |

공식 OMX 문서는 5축+그리퍼 사양과 Jazzy 기반 소프트웨어를 설명한다.
또한 leader-follower AI teleoperation 모드와 MoveIt을 사용하는 Robot Control 모드를 구분한다.
따라서 기존 4축 IK나 그리퍼 수치를 복사하거나 공식 AI 모드에 이 프로젝트의 액션 경로를
그대로 연결하면 안 된다.

자료:

- [OMX 하드웨어](https://docs.robotis.com/docs/systems/omx/specifications/hardware/)
- [OMX 소프트웨어](https://docs.robotis.com/docs/systems/omx/specifications/software/)
- [공식 leader-follower 모드](https://docs.robotis.com/docs/systems/omx/quick_start_guide/operation_guide/teleoperation/)
- [시뮬레이션에 사용한 공식 기구 소스](https://github.com/ROBOTIS-GIT/open_manipulator/tree/f60b8c4237f57bd832c0214ee2bb3f516b7402fb/open_manipulator_description/urdf/omx_f)

## Humble 시뮬레이션 포팅

이 프로젝트는 공식 소스의 외형·질량·관성·관절 기하와 8개 메시를 가져왔다.
공식 Jazzy launch/하드웨어 플러그인을 Humble에 섞어 실행하지 않는다.
자체 world 고정, ros2_control, Gazebo Classic/fake 플러그인과 MoveIt 설정을 조립한다.
이는 이 프로젝트의 Humble 포팅이며 ROBOTIS가 공식 지원하는 Humble OMX-F 배포판이라는 의미가 아니다.

메시는 scripts/fetch_omx_f_assets.py가 고정 커밋에서 받고 Git blob 해시를 검증한다.
공식 URDF의 넓은 ±2π 범위는 제품 문서의 관절 범위를 기준으로 시뮬레이션에서 제한했다.
실기에서는 실제 firmware 설정, 영점, 방향, joint range, 속도·전류 한계를 다시 확인해야 한다.
공식 원본의 effort 수치도 실기 토크 보정을 의미하지 않는다.

기구학은 5개 관절과 TCP의 작은 비대칭 오프셋까지 계산한다.
QP도 관절 수를 읽어 4변수 또는 5변수를 처리한다.
기본 입력은 여전히 끝단 위치 3축과 그리퍼다. 손목 자세와 5개 관절을 독립적으로 조종하는 UI는 아니다.

## 실행

~~~bash
# 설치 및 빌드 후, OMX-F를 기본으로 웹캠과 함께 실행
bash scripts/run_hand_control.sh

# 명시적으로 모델 지정
ros2 launch omx_bringup bringup.launch.py robot_model:=omx_f mode:=servo hand:=true

# 기존 4축 모델 비교
ros2 launch omx_bringup bringup.launch.py robot_model:=open_manipulator_x hand:=true
~~~

한 손을 보여주고 Space로 활성화한 뒤 잠시 고정한다.
현재 끝단 위치에서 손목 이동을 따라가며 같은 손의 pinch로 그리퍼를 조작한다.
기존 양손 클러치 방식은 hand_scheme:=two_hand로 선택한다.

## 검증 상태와 실제 실기 전환

총 55개 테스트(ROS 독립 50개, ROS 메시지 기반 콜백 5개)가 통과했다. 이 중 OMX-F 검사는 300개 자세의 독립 행렬 FK/IK,
5열 Jacobian 수치 미분, 5축 QP 폐루프, 그리퍼 단위와 mimic, 메시 해시를 포함한다.
두 로봇 × 두 backend 모델의 Xacro 확장도 확인했다.
ROS Humble·Gazebo·MoveIt을 설치하고 OMX-F의 두 backend × 세 모드 ROS 통합 검사가 통과했다.
팔 이동·그리퍼·입력 유실 후 위치 유지를 JointState로 확인했다. Gazebo GUI와 RViz도 실행했다.
내장 웹캠 부팅 문제를 수정한 뒤 실제 640×480 영상, MediaPipe 손 추적과
Gazebo 관절 이동을 확인했다. 변경 전/후 인터페이스별 관측 범위는 [검증 기록](verification.md)에 있다.

물리적 OMX-F로 전환할 때는 공식 Jazzy 환경을 별도로 구성하고 모델/TF와 그리퍼 인터페이스를
검증한 뒤 입력 노드와 제어 어댑터를 연결해야 한다. 본 프로젝트의 backend는 현재
gazebo_classic/fake만 허용하며, USB 연결만으로 실제 장비의 토크를 켜지 않는다.
