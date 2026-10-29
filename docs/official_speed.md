# OMX-F 공식 기본 속도 기준

사용자가 선택한 기준은 실물 이동 시간의 실측 보정이 아니라 공식 OMX-F 기본 설정이다.
모델 메시와 같은 ROBOTIS 커밋 f60b8c4237f57bd832c0214ee2bb3f516b7402fb를 기준으로 고정했다.

- [공식 MoveIt joint_limits.yaml](https://github.com/ROBOTIS-GIT/open_manipulator/blob/f60b8c4237f57bd832c0214ee2bb3f516b7402fb/open_manipulator_moveit_config/config/omx_f/joint_limits.yaml)
- [공식 하드웨어 설정](https://github.com/ROBOTIS-GIT/open_manipulator/blob/f60b8c4237f57bd832c0214ee2bb3f516b7402fb/open_manipulator_description/ros2_control/omx_f.ros2_control.xacro)
- [XL430 Drive Mode 및 Profile 문서](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#drive-mode10)
- [XL330 Profile 문서](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#profile-velocity112)

공식 MoveIt 파일을 변경 없이 설치했다. SHA256과 원본 URL은
omx_moveit_config/config/omx_f/speed_reference.json에 기록했다.

| 항목 | 이전 보수 설정 | 공식 기본 기준 |
| --- | --- | --- |
| MoveIt 관절 속도 한계 | 0.6 rad/s | 5.0 rad/s |
| MoveIt 관절 가속도 한계 | 0.8 rad/s² | 5.0 rad/s² |
| 기본 속도·가속도 배율 | 0.2 | 0.1 |
| Servo/직접 IK 관절 속도 상한 | 0.4 rad/s | 0.5 rad/s (=5×0.1) |
| Servo/직접 IK 관절 가속도 상한 | 0.8 rad/s² | 0.5 rad/s² (=5×0.1) |
| 브리지 제어 주기 | 20 Hz | 50 Hz |
| 웹캠 목표·추종 Cartesian 속도 상한 | 0.06 m/s | 0.15 m/s |
| 추종 비례 이득 | 2.0 | 8.0 |
| 목표 선행 거리 상한 | 0.04 m | 0.06 m |
| 1€ 최소 cutoff | 1 Hz | 2 Hz |

0.15 m/s, 제어 주기, 추종 이득과 필터는 이 프로젝트의 입력 튜닝 값이다.
공식 하드웨어의 Cartesian 속도 사양으로 주장하지 않는다. 팔 자세·관절 상한·특이점·충돌 감속에 따라
실제 끝단 속도는 이보다 낮다. 공식 기본 배율 0.1은 초보자를 위한 낮은 배율이다.
URDF의 원래 4.8 rad/s 한계도 유지하며 이 프로파일의 유효 0.5 rad/s보다 크다.

MoveIt 계획과 Servo 모두 동일한 유효 관절 한계를 사용한다. 직접 IK는 작은 구간마다
정지 속도로 연결하므로 한계가 같아도 전체 소요 시간이 같지는 않다.
그리퍼 직접 액션은 공식과 같은 GripperActionController를 사용하지만 MoveIt 계획 배율을 거치지 않는다.

공식 하드웨어는 Drive Mode=4, Profile Velocity=50, Profile Acceleration=25다.
Drive Mode의 bit 2가 켜진 시간 기반 프로파일이므로 각각 전체 시간 50ms, 가속 시간 25ms이며,
50×0.229rpm으로 계산하면 잘못된 값이 된다. Gazebo Classic 플러그인은 이 펌웨어 프로파일과
실제 전압·부하·마찰·서보 응답을 재현하지 않는다. 이 작업은 공식 소프트웨어 한계 정렬이며
실물의 최대 속도 또는 실측 동특성과의 일치를 보장하는 보정은 아니다.

~~~bash
# OMX-F는 기본 auto에서 공식 기준을 선택한다.
bash scripts/run_hand_control.sh

# 이전 보수 설정으로 비교
bash scripts/run_hand_control.sh speed_profile:=conservative

# 명시적 공식 기준
bash scripts/run_hand_control.sh speed_profile:=official
~~~

기존 OpenManipulator-X의 auto는 이전 설정을 유지한다. official은 OMX-F에만 허용한다.
분리 실행한 run_camera_input.sh도 실행 중인 브리지의 speed_profile을 읽어 같은 입력 설정을 적용한다.

실제 ROS 검사는 다음 명령으로 양의 yaw 약 40°를 계획·실행하고 컨트롤러의 목표 속도·가속도와
JointState 위치 변화에서 계산한 속도를 저장한다. 공식 기준 속도 상한 0.5 rad/s,
목표 가속도 0.5 rad/s²를 검사하며 리샘플링·Gazebo 추종 오차에 대한 허용 오차를 구분한다.

~~~bash
# 다른 프로젝트 시뮬레이션을 종료한 상태에서 실행
python scripts/smoke_ros.py --backend gazebo_classic --mode planned --speed-check
~~~

측정 결과: 목표 최고 속도 0.5000 rad/s, 실제 위치 변화에서 계산한 최고 속도 0.5001 rad/s,
목표 최고 가속도 약 0.5017 rad/s², 목표 yaw 0.7 rad 대비 실행 후 이동 약 0.7075 rad였다.
Gazebo의 보고된 velocity 값만으로는 위치 변화와 일치하지 않아 실제 위치 차분을 교차 확인했다.
관측은 artifacts/install/official_speed_motion.json, 로그는 official_speed_check.log에 남는다.
물리 장비와 더 정확히 맞추려면 동일 궤적의 실물 JointState·명령 시각·하중·모터 프로파일을 수집해
Gazebo의 위치 추종 응답과 비교해야 한다.

공개 저장소의 [속도 측정 요약](../artifacts/verification/official_speed_motion.json)과 [실행 설정 스냅샷](../artifacts/verification/one_hand_runtime.json)을 함께 볼 수 있다. artifacts/install의 원본 로그는 로컬 생성 파일로 Git에서 제외한다.
