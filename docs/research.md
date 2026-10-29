# 논문 아이디어 적용과 검증

공개된 학회 원문·저자 자료를 확인해 OMX-F 및 기존 OpenManipulator-X에 적용했다.
목적은 손 입력 품질과 제어 제약을 개선하고 이를 재현 가능하게 비교하는 것이다.
논문 전체 재현이나 SOTA 달성을 주장하지 않는다. 웹캠 입력과 운동학 모듈에 맞춘 구현이다.
2026-10-30부터 기본 조작은 한 손의 손목으로 팔을, 같은 손의 pinch로 그리퍼를 조작한다.
아래 왼손 클러치는 hand_scheme:=two_hand에서 사용하는 기존 비교 방식이다.

## 출처와 적용 경계

| 자료 | 확인한 아이디어 | 이 프로젝트의 구현 | 그대로 구현하지 않은 부분 |
| --- | --- | --- | --- |
| Qin et al., **AnyTeleop**, RSS 2023 | 손목 움직임과 손가락 동작을 각각 팔과 로봇 손에 대응 | 한 손의 손목/손바닥으로 위치, 같은 손의 pinch로 그리퍼 제어 | 전체 6D 손목 추정, 다지 손 retargeting, 원 논문 모션 생성과 사용자 실험 |
| Casiez, Roussel, Vogel, **1€ Filter**, CHI 2012 | 움직임 속도에 따라 저역통과 필터 대역 조절 | 정규화한 손 특징에 1€ 필터, 실제 샘플 간격 사용, 유실 시 초기화 | 논문의 모든 실험 조건과 성능 재현 |
| Iyer et al., **OPEN TEACH**, CoRL 2024, PMLR 270 (2025) | 손 기반 로봇 매핑, 일시정지 후 재배치, 정밀 작업용 해상도 조절 | 상대 위치 매핑, 왼손 pinch 클러치, P 정밀 모드 | Quest 3의 3D 추적·VR UI, 논문의 로봇 및 정책 학습 |
| Rauscher, Kimmel, Hirche, **Constrained Robot Control Using Control Barrier Functions**, IROS 2016 | 명목 제어와 제약을 QP로 결합 | 위치 Jacobian 기반 감쇠 미분 IK, 관절 여유·명령 속도·가속도 제약을 동시에 풂 | 원 논문의 동역학/토크 제어 및 전체 안전성 증명 |

공식 출처:

- [AnyTeleop 저자 프로젝트와 RSS 논문](https://yzqin.github.io/anyteleop/)
- [1€ 저자 페이지와 CHI 논문 링크](https://gery.casiez.net/1euro/)
- [OPEN TEACH 정식 proceedings](https://proceedings.mlr.press/v270/iyer25a.html)
- [OPEN TEACH 원문](https://raw.githubusercontent.com/mlresearch/v270/main/assets/iyer25a/iyer25a.pdf)
- [IROS 논문 저자 소속 대학 기록·초록·DOI](https://portal.fis.tum.de/de/publications/constrained-robot-control-using-control-barrier-functions/)
- [IROS 저자 원고](https://mediatum.ub.tum.de/doc/1320219/855622.pdf)

OPEN TEACH는 2024년 학회에서 발표되었고 정식 PMLR 권호의 연도는 2025년이다.
출판사/기관 원문 링크의 일부는 접근 제한이 있어 저자 자료 및 공개 원문을 함께 확인했다.

## 1. 적응형 필터와 입력 유효성

파일: src/omx_project/omx_teleop/omx_teleop/input_processing.py

1€ 필터는 원시 신호의 차분을 먼저 필터링하고, 필터링된 속도 크기에 비례해 cutoff를 높인다.
느린 동작에서는 떨림을 줄이고 빠른 변화에서는 지연을 줄이려는 구조다.
기본 보수 설정은 min_cutoff=1 Hz, beta=0.8, derivative_cutoff=1 Hz다.
OMX-F 공식 속도 프로파일에서는 입력 지연을 줄이기 위해 min_cutoff=2 Hz를 사용한다.
beta의 적절한 값은 입력 단위에 의존하므로 이 구현은 픽셀 대신 정규화된 특징을 사용한다.

입력은 다음 세 성분이다.

~~~text
[ log(손목-중지 MCP 거리 / 영상 너비), 손목 x / 영상 너비, -손목 y / 영상 높이 ]
~~~

첫 성분은 손의 영상 내 크기 변화다. 실제 깊이나 카메라 외부 파라미터를 추정한 것이 아니다.
손 크기, 손바닥 기울기, 가림에 따른 오차가 남는다.
좌표의 유한성·화면 주변 범위·최소 손바닥 크기를 검사하며, 이 검사를 신뢰도 추정 모델로 부르지 않는다.

moving_average 옵션으로 기존 5샘플 이동평균과 비교할 수 있다.
이전 시각보다 이르거나 NaN인 입력은 거부하고 0.35초 이상 끊긴 필터는 초기화한다.

## 2. 상대 이동, 클러치, 정밀 모드

기존 rate 입력은 손이 화면 중심을 벗어나 있으면 목표 위치를 계속 누적한다.
새 기본값 relative는 클러치를 잡은 시점의 손 특징과 실제 끝단 위치를 기준점으로 삼는다.
손이 정지하면 목표도 정지한다.

~~~text
desired_target = robot_anchor + resolution * gains * (hand_feature - hand_anchor)
~~~

- Space: 활성화/정지. 재활성화 시 5개의 안정적인 관측으로 현재 로봇 위치에서 기준점을 잡는다.
- one_hand: 선택한 손의 pinch는 그리퍼다. 별도의 pinch 클러치는 없다.
- two_hand: 왼손 pinch가 클러치다. 0.08초 dwell과 두 개의 문턱값을 사용한다.
  풀면 비활성 heartbeat로 실행 취소/정지를 요청하고 다시 잡으면 현재 위치에서 기준점을 잡는다.
- P: 정밀 모드. 이동 배율과 목표 최대 속도가 0.3배가 된다. 전환 시 기준점을 다시 잡는다.
- 손/관절 피드백 유실: 키보드 활성화도 해제한다. 재검출만으로 동작하지 않는다.

목표는 작업 공간, 목표 이동 속도, 현재 끝단과 목표 사이 최대 거리 0.04 m로 제한한다.
현재 관절 상태가 불연속적으로 바뀌어 속도·거리 제약을 함께 유지할 수 없으면 재보정한다.
손 입력은 여전히 기존 해석 IK로 도달 가능한지 확인해 도달 불가능한 목표 누적을 막는다.
이 검사는 충돌 검사와 다르다.

## 3. 위치 Jacobian과 제약 QP

파일: src/omx_project/omx_teleop/omx_teleop/differential_ik.py

4축 팔의 위치만 추종하므로 3×4 Jacobian을 사용한다.
기존처럼 여러 pitch를 순회해 매 프레임 다른 해를 택하는 대신, 실제 관절 상태 주변에서
작은 관절 속도를 계산한다. Jacobian은 URDF 기하로 계산하며 수치 미분과 대조했다.

이 프로젝트의 목적 함수는 다음과 같다. 학습된 정책이나 논문의 동역학 제어식을 복제한 것이 아니다.

~~~text
minimize over v:
  0.5 ||J(q)v - capped_cartesian_velocity||²
  + 0.5 λ² ||v||²
  + 0.5 μ ||v - v_previous||²
  + 0.5 ν ||v - posture_velocity||²
~~~

- Cartesian 속도는 목표 위치 오차에 비례하며 크기를 제한한다.
- λ는 3D 위치 Jacobian의 최소 특이값에 따라 0.01~0.06으로 조절한다.
- μ=0.0002는 이전 명령과의 차이에 대한 정규화다. 독립적인 효과는 ablation으로 확인한다.
- 자세 선호 가중치 ν의 기본값은 0이다. 손이 정지한 상태에서 관절이 중립 자세를 향해 움직이지 않게 한다.

각 관절에 관절 여유 범위 q_min+m ≤ q ≤ q_max-m를 두고, 운동학적 명령 제약을 만든다.

~~~text
-α(q - q_min - m) ≤ v ≤ α(q_max - m - q)
(q_min + m - q)/dt ≤ v ≤ (q_max - m - q)/dt
-v_max ≤ v ≤ v_max
v_previous - a_max*dt ≤ v ≤ v_previous + a_max*dt
~~~

첫 줄은 CBF 아이디어를 속도 제어에 적용한 본 프로젝트의 재구성이고,
둘째 줄은 이상적인 Euler 한 스텝의 위치 범위를 제한한다.
모든 제약은 같은 QP에서 처리하며 먼저 풀고 나중에 각각 clip하는 방식과 구분한다.

4변수 box QP는 최대 81개 active set을 조사한다. 양의 정부호 Hessian을 검사하고
KKT 조건을 만족하는 해를 찾는다. NumPy·SciPy·외부 QP 라이브러리를 새로 요구하지 않는다.
시간이 0.1초를 초과하거나 관절·가속도 조건이 충돌하면 명령을 정지하고 재활성화를 요구한다.
제약을 조용히 완화하거나 기존 IK로 우회하지 않는다.

### 보장하는 것과 확인이 필요한 것

수학적 제약은 입력 상태와 이전 **송신 명령**에 대한 운동학 계산에 적용된다.
로봇 동역학, 관측 지연, 모터 추종 오차, MoveIt Servo의 후단 필터/감속까지 포함한
전체 시스템의 불변성·실제 가속도 한계를 증명하지 않는다.
관절 여유 m=0.08 rad는 Servo 설정과 맞췄다. 충돌 형상은 MoveIt Servo/PlanningScene이 처리한다.
이 QP 자체에는 장애물 거리나 전체 링크 충돌 제약이 없다.

노드 status의 control_diagnostics에서 feasible, sigma_min, damping, tracking_error_m,
active_bounds, solve_ms, reason을 확인할 수 있다. input_age_ms는 마지막 수신 입력의
캡처→수신 시각 차이며 실제 로봇 응답까지 포함한 지연은 아니다.
카메라 메시지에는 영상 read 직후 시각을 넣어 MediaPipe 계산 시간을 오래된 입력 검사에 반영한다.

## 4. 재현 가능한 비교 실험

~~~bash
python3 -m unittest discover -s tests -v
python3 scripts/benchmark_research.py --seeds 5
~~~

결과:

- artifacts/research_benchmark.json: 조건, 시드, 평균·표준편차, 명령 한계, 계산 시간.
- artifacts/research_benchmark.md: 요약 표.
- tests/test_research.py: 필터, clutch 재결합, 목표 속도/거리, 100개 QP의 KKT 조건,
  Jacobian 미분 대조, 폐루프 관절 여유·속도·가속도 제약, 제약 충돌 시 거부.

비교는 동일한 모델과 목표에서 수행한다.

| 방법 | 정의 |
| --- | --- |
| analytic | 기존 해석 IK + 관절 비례 속도 + 속도/가속도 제한 |
| dls_clip | 동일한 감쇠 미분 IK 목적 함수의 비제약 해 + 속도/가속도 후처리 |
| qp_no_temporal | 제약 QP, μ=0 |
| cbf_qp | 제약 QP, μ=0.0002 |

일반 추종 실험과 관절 여유를 요구하는 실험을 분리했다.
후자는 물리적 관절 범위 안이지만 설정한 0.08 rad 여유 밖에 목표를 둔다.
따라서 여유 위반을 줄이면 목표 추종 오차가 늘 수 있다. 이 절충을 숨기지 않고 보고한다.

최초 5개 시드 합성 실험에서 필터 정지 구간 표준편차는 이동평균 0.01089,
1€ 0.00764였다. 단위 계단 90% 도달 시간은 각각 121.8 ms, 105.3 ms였다.
관절 여유 실험에서는 baseline의 평균 295/300 스텝 여유 침범이 QP에서 0/300이었다.
대신 위치 RMS 오차는 약 2.1 mm에서 9.9 mm로 늘었다.
이는 물리적 관절 한계를 넘긴 횟수나 실제 카메라/로봇 성능을 의미하지 않는다.

현재 합성 실험에서 일반 추종의 RMS 차이와 temporal 항의 독립 효과는 작다.
큰 성능 향상이나 논문보다 우수한 결과라고 결론내리지 않는다.
운동학 모델은 이상적인 Euler 적분이고 장애물·모터·카메라·사람을 포함하지 않는다.
실제 검증에서는 사용자별 성공률/완료 시간, 손 떨림, 제어 지연 p95,
최소 충돌 거리, 추종 오차와 정지 지연을 별도로 측정해야 한다.

## 5. 적용 및 baseline 실행

기본 실행은 새 입력 방식과 QP를 사용한다.

~~~bash
ros2 launch omx_bringup bringup.launch.py mode:=servo hand:=true
~~~

기존 계열 동작과 비교하려면:

~~~bash
ros2 launch omx_bringup bringup.launch.py mode:=servo hand:=true \
  servo_solver:=analytic mapping_mode:=rate input_filter:=moving_average clutch:=false
~~~

각 기능은 독립적으로 바꿀 수 있어 같은 사용자/목표에서 ablation을 진행할 수 있다.
카메라·ROS 실행은 현재 작업 환경에서 검증하지 못했다. CI/ROS smoke 및
docs/verification.md의 실험을 마친 뒤 실제 동작에 대한 결론을 내려야 한다.
