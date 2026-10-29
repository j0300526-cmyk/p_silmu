# 검증 기록

검증일: **2026-10-30 (Asia/Seoul)**. Ubuntu 22.04 amd64, Python 3.10, ROS 2 Humble, Gazebo Classic 11에서 수행했다.
현재 저장소의 공개 요약은 [summary.json](../artifacts/verification/summary.json)에 있다.
이 문서는 물리 OMX-AI 장비의 성능 인증이나 원 논문 재현 결과를 뜻하지 않는다.

## 자동 검사

| 검사 | 결과 |
|---|---|
| ROS 독립 단위 검사 | 50개 통과 |
| ROS 메시지 기반 격리 카메라 콜백 검사 | 5개 통과 |
| 합계 | **55개 통과** |
| OMX-F / 기존 X × fake / Gazebo Xacro 확장 | 링크·플러그인·메시 확인 |
| fake / Gazebo × planned / servo / direct_ik | **6개 통합 검사 통과** |

단위 검사는 독립적인 URDF 행렬 FK/IK, Jacobian 수치 미분, QP 최적성 및 관절 명령 제약,
필터·상대 매핑·액션 취소·손 선택·손/피드백 유실 정지 등을 포함한다.
카메라 콜백 검사에서는 합성 관측과 메모리 내 발행기를 사용한다. 실제 카메라를 열거나
ROS 제어 토픽에 명령을 발행하지 않는다. ROS 환경이 없으면 이 5개는 skip된다.

통합 검사는 실제 ROS launch와 controller action을 사용한다. Gazebo/fake의 JointState 피드백으로
팔 이동·그리퍼 동작·입력 heartbeat 유실 후 위치 유지를 확인하고 MoveIt scene 등록도 검사한다.
두 backend 모두 동일한 테스트를 실행하며 CI 설정은 [.github/workflows/ros-humble.yml](../.github/workflows/ros-humble.yml)에 있다.
GitHub hosted CI의 현재 결과는 저장소 Actions 탭을 기준으로 확인한다.

## 실제 웹캠 관측

내장 ACER HD User Facing (0408:4035)의 실제 640×480 프레임과 MediaPipe 실행을 확인했다.
Acer ANV16S-41의 부팅 문제는 [카메라 복구 문서](laptop_camera.md)의 대상 모델 전용 옵션으로 해결했다.

- **변경 전 양손 인터페이스**: 20초간 실제 손 목표 47개와 JointState 1996개를 받았으며,
  joint1 이동 범위 약 0.079 rad를 관측했다. 손 유실 후 비활성 상태도 확인했다.
  [관측 요약](../artifacts/verification/webcam_motion.json)
- **현재 한 손 인터페이스**: 실제 카메라 노드의 one_hand / Right 선택 / tracking=true와
  브리지 ready=true를 관측했다. [실행 스냅샷](../artifacts/verification/one_hand_runtime.json)
- 한 손만으로 팔 이동과 그리퍼 개폐를 함께 수행하는 **정량 수동 시험은 완료하지 않았다**.
  해당 매핑·손 교대 방지·재활성화·그리퍼 명령은 격리 콜백 검사에서 확인했다.

실행 스냅샷의 tracking=true는 그 시각의 상태이며 현재 실행 상태나 모든 제스처의 성공을 보장하지 않는다.
원본 영상, 사람의 손 랜드마크, 호스트별 실행 로그는 공개 저장소에 포함하지 않았다.

## 공식 속도 프로파일

공식 OMX-F MoveIt 기본값인 관절 속도·가속도 한계 5.0 × 기본 배율 0.1을 적용했다.
Gazebo에서 yaw 목표 0.7 rad(약 40°)를 계획·실행한 결과:

| 항목 | 측정 |
|---|---|
| 목표 최고 속도 | 0.5000 rad/s |
| 실제 위치 차분 최고 속도 | **0.5001 rad/s** |
| 목표 최고 가속도 | 약 0.5017 rad/s² |
| 실행 후 yaw 이동량 | 약 0.7075 rad |

[측정 요약](../artifacts/verification/official_speed_motion.json)과
[공식 설정·단위·한계](official_speed.md)를 함께 참조한다.
위치는 최소 80ms 구간의 차분으로 속도를 계산했다. 컨트롤러가 보고한 velocity 값은 위치 변화와
차이가 있어 실제 위치 차분을 교차 확인했다. 목표 가속도는 리샘플링 허용 오차를 포함해 검사했다.
실물의 부하·전압·펌웨어 프로파일·마찰·추종 지연까지 보정한 결과는 아니다.

## 재현 명령

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source install/setup.bash
python -m unittest discover -s tests -v
python scripts/check_model.py
```

다른 프로젝트 시뮬레이션을 종료한 뒤 실행한다.

```bash
for backend in fake gazebo_classic; do
  for mode in planned servo direct_ik; do
    python scripts/smoke_ros.py --backend "$backend" --mode "$mode"
  done
done
python scripts/smoke_ros.py --backend gazebo_classic --mode planned --speed-check
```

새 실행의 원본 로그와 진단 결과는 /tmp/omx_smoke_*.log 또는 artifacts/install/에 생성되며 Git에서 제외한다.
공개 요약은 검증일에 수집한 기록이며 자동으로 최신 실행 결과로 바뀌지 않는다.

## 남은 수동 확인

- Space 비활성 상태에서 손을 움직여도 로봇이 움직이지 않는지 확인한다.
- 한 손을 고정한 뒤 활성화할 때 현재 로봇 위치에서 시작하는지 확인한다.
- 손목 이동과 pinch를 함께 수행하며 팔 목표의 흔들림·가림·오분류 빈도를 측정한다.
- 손·카메라·피드백 유실 후 정지하고 명시적 재활성화를 요구하는지 확인한다.
- P 정밀 전환과 Space 정지/재배치/재활성화 시 큰 목표 변화가 없는지 확인한다.
- 기존 two_hand에서는 클러치 해제 후 정지와 재결합 기준점 재설정을 확인한다.
- planned는 유효한 경로를 실행하고 실패한 요청은 움직임으로 이어지지 않는지 확인한다.
- Servo는 장애물·자가 충돌 근처에서 감속/정지하는지 확인한다. 자동 우회 경로 생성과 구분한다.
- direct_ik 및 그리퍼 직접 액션에는 MoveIt 사전 충돌 검사가 없음을 고려한다.

물리적 OMX-AI 연결, 실제 물체 파지와 Gazebo attachment 연동은 구현 범위 밖이다.
