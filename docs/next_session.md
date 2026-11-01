# 다음 작업: 카메라 구성에 맞춘 손목 제어 확장

기록일: 2026-11-01 (Asia/Seoul)

## 사용자와 정한 방향

카메라 종류와 설치 방식이 정해진 다음, 그 입력에 맞춰 3D 손목 위치와
회전 제어를 검토한다. 지금은 기존 웹캠의 손목 위치 + 같은 손 pinch 제어를
유지하고 안정화한다. 카메라 구성이 아직 정해지지 않았으므로 회전 제어부터
구현하거나 새 장비를 전제로 변경하지 않는다.

다음 대화에서 “카메라 방식에 맞춰 손목 제어 이어서 하자”라고 하면
이 문서를 먼저 읽고 아래 순서로 이어서 진행한다.

## 현재 구현과 최근 수정

- 환경: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic, OMX-F 팔 5축 + 그리퍼.
- 기본 입력: 한 손의 손목 좌우·상하 이동, 손바닥의 영상 내 크기 변화로 앞뒤 추정.
- 손 특징은 손목(랜드마크 0)과 중지 MCP(9), 그리퍼 pinch는 엄지 끝(4)과 검지 끝(8)을 사용한다.
- 앞뒤 입력은 실제 거리 측정이 아니다. 손바닥 기울기나 가림도 크기 변화에 영향을 줄 수 있다.
- 현재 끝단 위치만 제어한다. 손목 회전각은 로봇 제어에 반영하지 않는다.
- 상대 위치 매핑, 1€ 필터, 시작 시 기준점 보정, 손/피드백 유실 시 정지 및 명시적 재활성화를 사용한다.
- 2026-11-01의 동작 중단은 Gazebo 일시정지로 관절 피드백이 끊긴 상태에서 확인했다.
  Gazebo 창의 Space와 카메라 창의 Space는 역할이 다르다.
- 카메라 창 오른쪽 위 START/STOP 버튼을 추가했다.
  `NO JOINT FEEDBACK`, `HAND LOST`, `HAND TOO SMALL OR INVALID`를 구분해 표시한다.
  Gazebo Play로 재개해도 자동으로 팔을 활성화하지 않는다.
- 관련 수정 커밋: `e53db74` (GitHub main 반영).

## 다음에 먼저 확인할 사항

1. 카메라 모델, 일반 RGB / RGB-D / 다중 카메라 여부, 설치 위치와 방향을 확인한다.
2. 실제 입력 해상도·프레임률, 손 전체가 보이는 범위, 가림과 지연을 측정한다.
3. RGB만 있으면 현재 상대 위치 방식과 3D 추정 방식을 비교한다.
   RGB-D이면 깊이와 RGB 정렬, 내참수, 손목 깊이의 유효성을 먼저 확인한다.
4. 좌표축 방향·단위·기준점과 카메라→로봇 좌표 변환을 정한다.
5. OMX-F 5축의 도달 범위를 확인하고, 위치와 회전 중 어떤 성분을 우선할지 정한다.
   위치 3축 + 회전 3축을 모두 독립 추종할 수 있다고 가정하지 않는다.
6. 그 뒤에 회전 입력·매핑·IK/QP를 확장한다. 기존 위치 제어와 비교할 수 있도록 옵션으로 둔다.
7. 손 유실·깊이 이상·Gazebo 일시정지 후 정지와 재활성화, 팔·그리퍼의 실제 반응을 검증한다.

## 논문과 현재 구현의 차이

- [AnyTeleop, RSS 2023](https://arxiv.org/html/2307.04577v1):
  카메라에서 손목의 3D 위치와 회전을 추정해 팔에 전달하고, 손가락은 로봇 손에 매핑한다.
  RGB만 사용하는 손목 위치 추정과 RGB-D 방식이 모두 있으며, RGB 위치 추정의 오차 한계를 설명한다.
- [OPEN TEACH](https://arxiv.org/html/2403.07870v1):
  Quest 3의 손 추적으로 손목 위치와 손바닥 좌표계 방향을 얻어 끝단 위치·회전에 매핑한다.
  VR 기반 입력이므로 현재 단일 웹캠 입력과 동일한 조건이 아니다.
- 현재 프로젝트는 위 논문의 아이디어 일부를 적용한 구현이다. 전체 시스템 재현이나 동일 성능을 주장하지 않는다.
  상세 출처와 적용 경계는 [research.md](research.md)를 참조한다.

## 이어서 볼 코드와 실행 방법

- 손 인식·창·시작/정지: `src/omx_project/omx_hand_input/omx_hand_input/hand_control_node.py`
- 손 특징·필터·상대 매핑: `src/omx_project/omx_teleop/omx_teleop/input_processing.py`
- 로봇 명령 경로: `src/omx_project/omx_teleop/omx_teleop/robot_bridge_node.py`
- 위치 제약 QP: `src/omx_project/omx_teleop/omx_teleop/differential_ik.py`
- 사용법과 입력 설정: [one_hand.md](one_hand.md),
  `src/omx_project/omx_hand_input/config/hand_control.yaml`

```bash
# 저장소 루트에서, 기존 실행이 없는지 확인한 다음 실행
bash scripts/run_hand_control.sh

# 검사 시 프로젝트 환경을 불러온다.
source scripts/runtime_environment.sh
python -m unittest discover -s tests -v

# 아래 통합 검사는 기존 시뮬레이션을 종료한 뒤 실행한다.
python scripts/smoke_ros.py --backend gazebo_classic --mode servo
```

최근 확인: 56개 단위/콜백 검사 통과. 실제 Gazebo 통합 검사에서 팔 이동·그리퍼·
입력 유실 후 위치 유지 통과. 수정한 카메라 노드에서 Gazebo 일시정지 감지,
피드백 복구 표시, 자동 재활성화 방지를 확인했다. 웹캠에서 손 검출도 관측했다.
사용자의 수정 후 연속 조작 성공 여부와 새 카메라/회전 제어는 아직 확인하지 않았다.
