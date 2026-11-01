# p_silmu

ROS 2 Humble에서 **노트북 웹캠 → 한 손 인식 → MoveIt 2 Servo → Gazebo 로봇 제어**를 실행하는 프로젝트입니다.
기본 모델은 OMX-AI의 **OMX-F follower(팔 5축 + 그리퍼)**이며, 기존 OpenManipulator-X 4축 모델도 선택할 수 있습니다.

이 저장소는 **Ubuntu 22.04용 Humble 시뮬레이션 포팅**입니다. 물리 OMX-AI의 DYNAMIXEL을 구동하는 backend는 포함하지 않습니다.
공식 OMX 실기 소프트웨어는 [ROBOTIS OMX 문서](https://docs.robotis.com/docs/systems/omx/specifications/software/)를 기준으로 별도로 구성해야 합니다.

## 실행 환경

| 항목 | 기준 |
|---|---|
| OS / Python | Ubuntu 22.04 amd64 / Python 3.10 |
| ROS / 계획·제어 | ROS 2 Humble, MoveIt 2, MoveIt Servo, ros2_control |
| 시뮬레이션 | Gazebo Classic 11 또는 fake hardware |
| 손 인식 | OpenCV, MediaPipe 0.10.14, 로컬 V4L2 웹캠 |
| 기본 조작 | 한 손, 상대 위치 매핑, 1€ 필터, pinch 그리퍼 |

Gazebo Classic은 지원 종료된 기존 시뮬레이터입니다. 이 프로젝트는 요청된 Humble 환경과의 호환성을 위해 사용합니다.
Modern Gazebo나 공식 Jazzy용 하드웨어 플러그인을 Humble에 혼합하지 않습니다.

## 설치 및 실행

새 PC의 터미널에서 실행합니다. 시스템 패키지 설치만 관리자 인증이 필요합니다.

```bash
git clone https://github.com/j0300526-cmyk/p_silmu.git
cd p_silmu

sudo bash scripts/install_system.sh
rosdep update --rosdistro humble
bash scripts/bootstrap.sh --vision

bash scripts/run_hand_control.sh
```

`bootstrap.sh`는 공식 description 소스를 고정 커밋으로 가져오고, OMX-F 메시 8개를 내려받아 해시를 확인한 뒤 빌드합니다.
빌드·가상환경·vendor checkout·다운로드한 메시를 Git에 넣지 않으므로 처음 설치할 때 네트워크가 필요합니다.
[고정 의존성](dependencies.repos), [메시 출처](src/omx_project/omx_description/config/omx_f_assets.json), [Python 버전](requirements-vision.txt)을 참조하세요.

카메라 없이 먼저 실행하거나 나중에 카메라를 연결할 수도 있습니다.

```bash
# 터미널 1: Gazebo + MoveIt Servo + RViz
bash scripts/run_simulation.sh

# 터미널 2: 실행 중인 시뮬레이션에 카메라 연결
bash scripts/run_camera_input.sh --wait
```

두 터미널은 같은 ROS_DOMAIN_ID를 사용해야 합니다. 전체 hand launch와 분리된 camera input을 동시에 중복 실행하지 않습니다.
카메라 진단만 하려면 `.venv/bin/python scripts/check_camera.py`를 실행하세요.

## 한 손 사용법

1. 카메라 영상 창을 클릭하고 왼손·오른손 중 **한 손만** 보여줍니다.
2. **Space**를 누르고 잠시 손을 고정해 현재 로봇 위치에서 기준점을 잡습니다.
3. 손목을 움직여 팔을 조작합니다. **같은 손의 엄지·검지를 붙이면 그리퍼 닫기**, 벌리면 열기입니다.
4. **Space를 다시 누르면 정지**합니다. 손을 재배치한 뒤 다시 시작할 수 있습니다.

| 입력 | 동작 |
|---|---|
| 손목 좌우 / 상하 | 로봇 y축 / z축 목표 이동 |
| 손바닥의 영상 내 크기 변화 | x축 목표 이동. 절대 깊이 측정은 아님 |
| 같은 손의 엄지·검지 pinch | 그리퍼 닫기 / 열기 |
| P | 정밀 모드, 입력 배율 0.3 |
| Enter | planned 모드에서 현재 목표 확정 |
| Esc 또는 실행 터미널 Ctrl+C | 종료 |

선택한 손이나 최신 관절 피드백을 잃으면 정지하고 재활성화를 요구합니다.
다른 손이 들어와도 자동으로 제어를 넘기지 않습니다. auto에서 처음부터 양손이 보이면 오른손을 선택합니다.
창에 표시되는 그리퍼 상태는 마지막 명령이며 파지 성공이나 실제 힘을 뜻하지 않습니다.
자세한 동작과 검사는 [한 손 조작](docs/one_hand.md)을 참조하세요.

## 기존 양손 방식의 단점

이 프로젝트의 `two_hand`는 **왼손으로 팔 이동과 pinch 클러치를 유지하고, 오른손으로 그리퍼를 조작**하는 방식입니다.
한 대의 로봇 팔을 조작하는 인터페이스이며, 두 로봇 팔을 동시에 다루는 양팔 원격조작과는 평가 대상이 다릅니다.

| 항목 | 현재 양손 인터페이스에서 생기는 부담 |
|---|---|
| 동시 조작 | 왼손의 위치·클러치를 유지하면서 오른손으로 개폐해야 하므로 두 역할을 함께 익혀야 합니다. |
| 지속적인 pinch | 팔을 움직이는 동안 왼손 엄지·검지를 계속 붙여야 합니다. 손가락 유지 부담이 있고, 의도치 않게 풀면 제어가 일시 정지합니다. |
| 카메라 화각·가림 | 팔과 그리퍼를 함께 조작하려면 두 손이 모두 보여야 합니다. 좁은 화각에서 손이 겹치거나 화면을 벗어나면 인식이 불안정해질 수 있습니다. |
| 추적 유실 | 왼손을 놓치면 제어가 비활성화돼 재활성화가 필요합니다. 오른손을 놓치면 그리퍼 입력이 끊겨 timeout 후 진행 중 그리퍼 액션이 취소될 수 있습니다. |
| 키보드 조작 | 양손이 사용 중이므로 Space·P를 누르려고 손을 옮길 때 클러치나 손 추적을 놓칠 수 있습니다. |
| 조작 대비 제어 범위 | 입력은 끝단 위치 3축과 그리퍼로, 현재 한 손 방식과 같습니다. 두 손을 사용해도 손목 자세 등 추가 제어 축이 생기지는 않습니다. |

따라서 기본값을 **한 손의 손목 이동 → 팔, 같은 손의 pinch → 그리퍼, Space → 시작·정지**로 변경했습니다.
기존 양손 방식은 클러치 조작을 선호하는 사용자와 비교 실험을 위해 옵션으로 유지합니다.
이 표는 현재 구현의 조작 요구와 사용 중 제기된 불편을 정리한 것입니다.
양손 대비 한 손의 피로도·처리 속도·작업 성공률 우위는 정량 사용자 실험으로 검증하지 않았습니다.

## 제어 모드와 모델

| 모드 | 명령 경로 | 용도 |
|---|---|---|
| `servo` — 기본 | 제약 미분 IK QP → JointJog → MoveIt Servo → 컨트롤러 | 연속 손 추종 |
| `planned` | URDF 기반 IK → MoveGroup → OMPL 계획·실행 | Enter로 확정한 목표 실행 |
| `direct_ik` | URDF 기반 IK → FollowJointTrajectory | 직접 IK 비교 |

```bash
# MoveIt 경로 계획 후 실행
bash scripts/run_hand_control.sh mode:=planned

# 기존 양손 클러치 방식
bash scripts/run_hand_control.sh hand_scheme:=two_hand

# 왼손만 사용
bash scripts/run_hand_control.sh control_hand:=left

# 기존 OpenManipulator-X 모델 또는 Gazebo 없는 fake backend
bash scripts/run_hand_control.sh robot_model:=open_manipulator_x
ros2 launch omx_bringup bringup.launch.py backend:=fake hand:=false
```

두 번째 예제의 양손 방식에서는 왼손 pinch가 클러치, 오른손 pinch가 그리퍼입니다.
`ros2 launch`를 직접 실행할 때는 먼저 ROS와 workspace 환경을 source해야 합니다.

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
```

자주 쓰는 launch 옵션:

| 옵션 | 기본값 | 선택 |
|---|---|---|
| robot_model | omx_f | omx_f / open_manipulator_x |
| backend | gazebo_classic | gazebo_classic / fake |
| mode | servo | servo / planned / direct_ik |
| hand_scheme | one_hand | one_hand / two_hand |
| control_hand | auto | auto / left / right |
| speed_profile | auto | auto / official / conservative |
| hand | false | 카메라 노드 실행. run_hand_control.sh는 true로 설정 |
| camera_device | auto | /dev/video* 탐색 또는 특정 장치 경로 |
| rviz / gui / show_camera | true | 시각화 창 표시 |

## 속도 기준

OMX-F의 기본 auto는 [공식 MoveIt 설정](https://github.com/ROBOTIS-GIT/open_manipulator/blob/f60b8c4237f57bd832c0214ee2bb3f516b7402fb/open_manipulator_moveit_config/config/omx_f/joint_limits.yaml)의
속도·가속도 한계 5.0과 기본 배율 0.1을 따릅니다.

- 유효 관절 상한: **0.5 rad/s, 0.5 rad/s²**
- 프로젝트의 웹캠 목표 속도 상한: **0.15 m/s**, 추종 주기 **50Hz**
- 이전 설정 비교: `speed_profile:=conservative`

웹캠 입력 설정은 프로젝트의 튜닝 값입니다. 실제 끝단 속도는 자세·관절 제약·충돌 감속에 따라 더 낮아집니다.
공식 소프트웨어 설정 정렬이며 실물의 부하·전압·모터 응답을 실측 보정한 것은 아닙니다.
측정 결과와 원본 출처는 [공식 속도 기준](docs/official_speed.md)에 있습니다.

## 프로젝트 구조

```text
src/omx_project/
├── omx_description/      # OMX-F / 기존 X 모델과 메시 출처
├── omx_control/          # Gazebo / fake 컨트롤러
├── omx_simulation/       # 공통 작업대·장애물 설정
├── omx_moveit_config/    # SRDF, OMPL, Servo, 속도 한계, RViz
├── omx_bringup/          # 모델 생성, 준비 확인, scene 등록, launch
├── omx_hand_input/       # 카메라·손 인식·한 손 인터페이스
└── omx_teleop/           # IK, QP, 입력 gate, 액션 관리
scripts/                 # 설치·빌드·실행·진단·검증
tests/                  # 단위 및 콜백 검사
docs/                   # 설계, 논문 적용, 한계, 검증 절차
artifacts/verification/ # 공개 가능한 실행 검증 요약
```

로봇 모델과 controller_manager는 launch가 한 번씩 생성합니다.
Gazebo 월드와 MoveIt PlanningScene은 같은 YAML에서 작업대·장애물을 생성합니다.
준비된 관절 피드백·컨트롤러·TF·clock과 scene 적용을 확인한 다음 teleop을 시작합니다.

주요 설정:

- [손 입력](src/omx_project/omx_hand_input/config/hand_control.yaml)
- [브리지 기본 설정](src/omx_project/omx_teleop/config/teleop.yaml)
- [OMX-F 속도 오버레이](src/omx_project/omx_teleop/config/omx_f_official.yaml), [손 입력 오버레이](src/omx_project/omx_hand_input/config/omx_f_official.yaml)
- [공통 작업 환경](src/omx_project/omx_simulation/config/workcell_objects.yaml)
- [Servo 충돌·정지 설정](src/omx_project/omx_moveit_config/config/servo.yaml)

## 검증

Ubuntu 22.04 / Humble에서 다음을 수행했습니다.

- **55개 검사 통과**: ROS 독립 50개 + ROS 메시지를 사용하는 격리된 카메라 콜백 검사 5개
- **fake / Gazebo × 세 제어 모드 통합 검사 통과**: 팔 이동, 그리퍼, scene 등록, 입력 유실 후 위치 유지
- 두 모델 × 두 backend의 실제 Xacro 확장과 메시 확인
- 실제 내장 웹캠의 640×480 프레임·MediaPipe 실행 및 손 입력에 따른 Gazebo 관절 이동 관측
- 공식 속도 프로파일의 Gazebo 시험: 실제 위치 차분 최고 속도 **0.5001 rad/s**

카메라 관측의 변경 전/후 인터페이스와 수동 검증 범위는 [검증 기록](docs/verification.md)을 참조하세요.
영상·사용자별 실행 로그는 공개 저장소에 포함하지 않습니다.

```bash
# ROS / venv / workspace를 source한 터미널에서
python -m unittest discover -s tests -v
python scripts/check_model.py

# 다른 프로젝트 시뮬레이션을 종료한 뒤 실제 ROS 통합 검사
python scripts/smoke_ros.py --backend gazebo_classic --mode servo
python scripts/smoke_ros.py --backend gazebo_classic --mode planned --speed-check
```

ROS 환경을 source하지 않은 경우 ROS 메시지 콜백 검사 5개는 skip됩니다.
GitHub Actions에는 두 backend × 세 모드 행렬 검사를 구성했습니다. 실행 결과는 Actions 탭에서 확인합니다.

## 문서와 적용 범위

| 문서 | 내용 |
|---|---|
| [한 손 조작](docs/one_hand.md) | 손 선택, 재활성화, 그리퍼 매핑 |
| [다음 작업 메모](docs/next_session.md) | 카메라 구성 확정 후 3D 손목·회전 제어 확장 |
| [OMX-AI 모델과 실기 전환](docs/omx_ai.md) | 5축 OMX-F와 기존 4축 X 구분, Jazzy 전환 경계 |
| [논문 아이디어](docs/research.md) | 1€ 필터, 상대 매핑, AnyTeleop, 제약 QP, 합성 비교 실험 |
| [공식 속도 기준](docs/official_speed.md) | 고정한 원본 설정, 단위, 실제 Gazebo 측정 |
| [검증 기록](docs/verification.md) | 수행한 검사와 미검증 항목 |
| [Acer 내장 카메라](docs/laptop_camera.md) | 특정 ANV16S-41 모델의 카메라 부팅 문제 복구 |
| [초기 설계 기록](docs/architecture.md) | 구현 전 설계 이력. 현재 사용법은 이 README 기준 |

Servo의 충돌 감속은 장애물을 돌아가는 경로 계획과 다릅니다.
`direct_ik`와 그리퍼 직접 액션에는 MoveIt 경로 계획·사전 충돌 검사가 적용되지 않습니다.
소프트웨어 정지는 실제 장비의 비상정지 장치를 대신하지 않습니다.
실제 물체 파지용 Gazebo attachment, 물체 인식·자동 정렬, 물리 장비 backend는 구현 범위에 포함하지 않습니다.

VS Code 확장 추천과 workspace 설정은 [.vscode](.vscode)에 있습니다.
설정은 workspace 상대 경로를 사용하며 새 터미널에서 ROS·venv·overlay를 불러옵니다.

## 라이선스

[Apache-2.0](LICENSE). ROBOTIS에서 가져온 모델·설정·fixture의 출처와 변경 사항은
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)에 기록했습니다.
논문 전체 구현이나 원 논문의 성능 재현을 주장하지 않습니다.
