# DAMVI VESC · 조향 · UST-10LX · IMU 모사

현재 SSH 서버의 **기존 Isaac Sim GUI**에 `live_gui.py`를 연결한다. 기존 네 바퀴 articulation과 실제 지면 접촉을 유지하고 앞바퀴에 조향 knuckle 두 개를 추가한다. PhysX Vehicle로 교체하지 않았다. 별도 UI는 생성하지 않는다.

## 제어와 출력

`/commands/motor/speed` (`std_msgs/msg/Float64`, signed ERPM) → 펌웨어 방식 목표 램프/PID → 전류 → 모터/구동계 토크 → PhysX 바퀴 회전 → `/sensors/core` (`vesc_msgs/msg/VescStateStamped`). `state.speed`와 시뮬레이션 timestamp만 모사하며 다른 VESC 필드는 기본값이다. `/commands/motor/brake`는 감속 전류 명령이다. 마지막으로 받은 명령 모드가 적용되고, 입력이 0.5 simulated seconds 동안 없으면 속도 목표 0으로 전기제동한다.

ERPM은 네 바퀴 회전의 구동축 대응값에서 계산한다. 차체 이동속도·명령속도를 telemetry로 복사하지 않는다. 한쪽 바퀴가 헛돌면 디프 입력에 기여하는 회전도 출력에 포함된다. 정지 시 마지막 0 근처 표본 이후 발행을 생략할 수 있다. `/clock`은 계속 발행한다.

`/cmd_vel` (`geometry_msgs/msg/Twist`)도 받는다. 이 프로젝트에서는 `linear.x`가 목표 m/s이고 **`angular.z`는 nominal 조향각(rad)**이다. 일반적인 yaw-rate 명령 의미와 다르며 기존 teleop adapter의 의미를 따른다. 최신 속도 명령(`/cmd_vel` 또는 VESC ERPM 토픽)이 우선한다. 조향은 `/cmd_vel`에서만 받는다.

## 설정 파일

- `config/vesc_sim.yaml`: VESC PID·모터·구동계·ROS 및 아래 파일 경로.
- `config/steering.yaml`: 비선형 조향 보간표·조향 한계·actuator/관절 설정.
- `config/lidar.yaml`: UST-10LX 사양·프레임 ID·ROS 설정.
- `config/imu.yaml`: IMU 주기·토픽·중력·노이즈·필터·공분산.
- `config/tf.yaml`: 센서 간 장착 위치·회전과 `/tf_static` 발행 설정.
- `config/runtime.yaml`: 화면 갱신·물리 실행·진단 기록 설정.

모든 파일을 Play할 때 다시 읽는다. VESC YAML의 `steering_config`, `lidar_config`, `imu_config`, `tf_config`, `runtime_config` 경로는 같은 config 폴더를 기준으로 해석한다. 센서별 YAML에는 프레임 ID를 두며, 장착 위치·회전은 공통 TF 파일에만 둔다. runtime의 worker 수·broadphase 변경은 정지 상태에서 adapter를 재설치해야 적용된다.

## 비선형 조향 추정

실차 MCAP 다섯 개에서 `/imu/data` orientation의 yaw 변화율을 계산했다. cmd_vel은 기록되어 있지 않아 `/drive.drive.steering_angle`을 proxy로 사용하되, 실제 서보 명령 `0.5 + 0.52 * input`과 0.04 rad 이내에서 일치하는 표본만 채택했다. `/sensors/imu/raw`는 기록에서 0 값이므로 사용하지 않았다.

VESC ERPM을 1538로 환산해 속도 0.6–2.5 m/s, 완만한 종가속 `|dv/dt| <= 0.5 m/s²`를 선택했다. 횡가속 추정 1.5 m/s² 이하, IMU gyro와 yaw 미분의 일치, PF 위치 변화 속도와의 일치도 추가 검사했다. PF 속도는 슬립 일관성 확인용이며 ground truth는 아니다. 선택된 데이터는 2734개/54.68초다.

지정된 `/home/shin/Desktop/raceline/Raceline-Optimization/params/f110_rgb.ini`의 휠베이스 0.26 m를 사용해 `delta_equiv = atan(0.26 * yaw_rate / wheel_equivalent_speed)`를 역산했다. 약 0.07초의 명령→IMU 지연을 정렬에 사용했다. 이 지연에는 차체·센서 반응이 섞이므로 actuator에 추가 지연으로 중복 적용하지 않았다. actuator 시정수 0.07초는 INI 값이다.

독립 검토에서 실제 드라이버 echo `/sensors/servo_position_command`를 추가로 확인했다. 서보 입력은 0.15–0.85로 제한되며, 명령과 제한 후 echo의 일치율은 기록별 99.983–100%다. 따라서 nominal 입력은 **±0.673076923 rad(±38.56°)에서 포화**한다. 기본 모델은 이 관측된 비선형 제한과 중앙 구간의 단순 보정식이다.

```text
effective_input = clip(cmd_vel.angular.z, -0.673076923, +0.673076923)
equivalent_angle = 0.014131440 + 0.418474846 * effective_input
```

기존 C99 단조 PCHIP 보간기에 두 knot로 이 식을 정확하게 표현한다. 여러 knot의 곡선도 같은 YAML/API로 사용할 수 있다. 물리 각도 상한은 INI의 ±0.301 rad다. C 보간은 독립 SciPy reference 401점과 일치하며, 권장식의 별도 2001점 검사도 통과했다.

20초 외부 시간블록 5-fold, 내부 모델 선택과 0.5초 embargo를 적용한 검증에서 기본 모델 RMSE는 1.303°, 기록 전체를 제외한 검증은 2.120°다. 실제 서보 포화에 내부 PCHIP를 더한 후보는 각각 1.305°/2.055°이며, 복잡한 내부 곡선의 개선은 불확실성 범위 안이다. 기존 2초 holdout은 정규화 선택에도 재사용했으므로 그 수치와 새 검증을 개선폭으로 비교하지 않는다. 현재 데이터로 기계 링크의 내부 곡률이나 유격까지 확정할 수 없으며, 결과는 IMU와 휠 속도로 역산한 **등가 조향각**이다.

최종 [독립 검토](calibration/steering_review/review.md), [상세 보고서](calibration/steering_review/report.json), [서보 포화 그래프](calibration/steering_review/servo_clipping.png)를 참고한다. 대안 곡선은 `calibration/steering_review/alternative_curved_steering.yaml`에 보존했다. 재현 도구는 `tools/review_steering_model.py`이며, 앞선 추출/탐색 도구와 산출물도 남겼다. SciPy/matplotlib는 오프라인 분석에만 필요하다.

앞바퀴에 Z축 조향 관절을 만들고 기존 Y축 바퀴 회전 관절을 knuckle 아래로 연결한다. 실제 관절 각도를 기록하며 시각 메시를 직접 회전시켜 가짜 각도를 만들지 않는다. front carrier 질량을 knuckle과 나누므로 총질량은 늘지 않는다. 기존 USD 휠베이스 0.25 m, 앞 윤거 0.20 m는 유지한다. 실차 0.26 m와 4% 차이가 있어 완전한 주행 궤적 일치까지 의미하지는 않는다. 좌우 바퀴 각도는 기존 USD geometry에 맞는 Ackermann 관계를 따른다.

## HOKUYO UST-10LX

`/scan` (`sensor_msgs/msg/LaserScan`)은 현재 GUI의 PhysX 충돌 장면을 실제 raycast한다. 270°, 0.25°, 1081개, 40 Hz이며 rolling scan과 최초 빔 simulation timestamp를 사용한다. `/tf_static`으로 base_link→laser 장착 변환을 발행한다. 제조사 최소 0.06 m와 bag의 최대 30 m 설정을 구분하며, 30 m는 geometric cutoff이고 모든 재질에 대한 보증 거리가 아니다.

렌더링 전용 CAD는 감지하지 않고 충돌 형상을 감지한다. 반사율·유리·강한 외광·광학 잡음·intensity는 모사하지 않는다. 상세 사양·출처·제약은 [lidar_notes.md](lidar_notes.md)에 있다.

## IMU와 공통 TF

`/imu/data` (`sensor_msgs/msg/Imu`, frame `imu`)는 기본 **1000 Hz simulation time**이다. 사용자 지정 물리는 **200 Hz**이며, 완료된 두 물리 상태 사이의 자세·속도를 보간해 1 ms 간격의 IMU를 발행한다. 5 ms 물리 구간마다 최대 다섯 개를 묶어서 보내므로 중간 출력에 최대 약 4 ms 지연이 있고, 독립적인 1 kHz 물리 측정은 아니다. 실제 COM 속도의 변화율, 중력, 센서 레버암의 접선·구심가속도와 장착 회전을 반영한다. `sampling_mode: physics_state`는 실제 물리 표본만 쓰며 이 모드에서는 물리보다 빠른 센서 주기를 거부한다.

기본은 이상 센서이며 노이즈·바이어스·랜덤워크·드롭률·미분 필터를 `imu.yaml`에서 켤 수 있다. 접촉 solver의 작은 속도 진동도 미분되어 가속도 피크가 생길 수 있다. 실차 IMU의 내부 fusion·온도·잡음 스펙트럼을 보정한 모델은 아니다. 자세는 시뮬레이션 world 기준이며 map 정렬을 추가하지 않는다. 상세 내용은 [imu_notes.md](imu_notes.md)에 있다.

사용자가 지정한 [red_damvi TF 퍼블리셔](https://github.com/rcv-formula/f1_stack_for_damvi/blob/red_damvi/f1tenth_stack/f1tenth_stack/tf_publisher.py)의 활성 변환을 `tf.yaml`에 옮겼다. base_link→laser는 `[0.27, 0, 0.16] m`, base_link→imu는 `[0.27, 0, 0.06] m`, 두 회전은 identity다. 이 사용자 지정값이 bag의 과거 장착값보다 우선한다. 센서가 계산에 쓰는 변환과 중앙 `/tf_static` 퍼블리셔가 같은 트리를 사용한다. 주석 처리된 카메라 변환은 활성화하지 않았다.

## 선택된 실차 설정

사용자가 선택한 윈도우 바탕화면 `RCV-formula-motor-settings_default.xml`에서 가져왔다. 원본은 수정하지 않았다. SHA256와 원본 경로는 [설정 YAML](config/vesc_sim.yaml)에 기록했다.

- `speed_to_erpm_gain: 1538.0`: 사용자 지정, YAML의 이 항목을 변경한다.
- Kp 0.009, Ki 0, Kd 0.000015, D filter 0. 펌웨어 식에서 filter 0이면 D 상태가 갱신되지 않는다.
- 최소 목표 800 ERPM: 1538 기준 약 0.52 m/s 미만 목표는 속도 PID 대신 저속 전기제동 요청이 된다.
- 목표 램프 50000 ERPM/s: 가속·감속·역전 모두 적용한다.
- 속도 PID 5000 Hz, 물리 200 Hz. 물리 표본 사이 PID는 동일한 측정값으로 25회 진행한다.
- 모터 전류 62 A, 제동 62 A, 배터리 구동 250 A / 회생 200 A 제한.

펌웨어 출처와 모사 범위는 [firmware_notes.md](firmware_notes.md)에 있다. XML에는 정확한 실차 firmware build가 없으므로 6.05 호환 속도 루프를 기준으로 했다. 0821 bag의 명령/출력은 전진에서 양수이고 기록 당시 명령 환산계수는 약 4000.66이다. 이번 1538 설정과 구분한다. bag 읽기 도구는 `tools/inspect_bag.py`이며 bag을 네트워크에 재생하지 않는다.

## 구동계 및 전기제동 모델의 범위

구동계는 한 모터, 앞뒤 점성 저항형 디프, 센터 디프 없는 **유한 강성·감쇠의 연결축**으로 근사한다. 앞뒤 축은 독립 모터가 아니며 연결축 반력 토크가 작용한다. 이상적인 무한강성 기어 구속은 아니므로 과도 상태에 앞뒤 평균 회전차가 생길 수 있다. 디프 저항·센터 강성·내부 Coulomb/점성 마찰은 YAML 설정이며 실차 보정이 남아 있다.

전류-토크 상수는 XML의 극쌍 수·자속에서 계산한다. 회생 전력, 상 저항 손실, 전류 응답 지연과 저속 단락제동을 평균 모델로 구현했다. 기계 브레이크는 없다. PWM/FOC 전류 루프·전기적 observer/PLL·자기 포화·BMS·열·실제 배터리 충전상태는 모사하지 않는다. 배터리 전압은 설정된 상수다. 실제 정지 유지 동작의 완전한 펌웨어 재현은 아니며, 단락제동 근사는 회전이 0일 때 토크도 0이다.

현재 USD 타이어 지름은 55.88 mm, XML 기입값은 83 mm이다. USD 형상은 유지하고 **1538과 실제 USD 반지름에 맞는 유효 감속비**를 사용한다. XML 감속비를 중복 곱하지 않는다. 극수·기어비·타이어 치수를 실제 차량과 일치시키는 후속 보정이 필요하다.

기존 씬에는 바퀴 관절 마찰이 FL 200, 나머지 1000으로 저장돼 있었다. 실행 중 session layer에서 이를 0으로 하고 별도 마찰 모델을 적용한다. 원본 USD는 저장하지 않는다. 뒤차축 중앙에 `/World/RC_Car/Chassis/base_link`를 둔다. ground-truth odometry나 움직이는 map/odom→base_link TF는 발행하지 않는다. 센서 장착용 base_link→laser 정적 TF는 발행한다.

## 기존 GUI에서 사용

서버 경로: `/home/shin/Desktop/RC_Car_Isaac/vesc_sim`.

설치된 GUI의 localhost Python Server를 통해 다음을 실행한다. 새 앱이나 새 stage를 만들지 않는다.

```bash
python3 /home/shin/Desktop/RC_Car_Isaac/isaac_live_client.py \
  /home/shin/Desktop/RC_Car_Isaac/vesc_sim/tools/install_gui.py
```

별도 Isaac UI는 생성하지 않는다. 기존 Play/Stop/Pause를 사용하고 명령은 ROS 토픽으로만 입력한다. 시작 시 속도 목표는 0이다.

`config/vesc_sim.yaml`을 **Play할 때마다 다시 읽는다**. PID·전류·램프·ERPM 계수·전기제동/구동계 계수와 ROS 설정을 새로 적용한다. 설정이 잘못되면 토크를 해제하고 일시정지하며 오류를 상태 파일에 남긴다. 차량 prim 경로 변경은 adapter 재설치가 필요하다.

ROS 속도 명령 예: 1.2 m/s는 1845.6 ERPM이다. 연속 명령을 유지해야 timeout으로 정지하지 않는다. 기존 `ackermann_to_vesc`를 사용할 경우 그 노드의 `speed_to_erpm_gain`도 같은 값으로 지정한다.

GUI의 기존 한 프레임 반복 재생 설정은 물리 시간이 누적되도록 해제한다. 실행 상태는 서버 `outputs/live_gui/status.json`에 저장한다. trace는 메모리에 보관하고, 전체 JSON을 반복 직렬화하는 비용을 줄이기 위해 기본 자동 저장을 끈다. GUI 검증 도구가 필요한 시점에 `tools/flush_gui_diagnostics.py`로 snapshot을 요청한다.

현재 물리는 200 Hz, 화면 목표는 30 Hz로 분리했다. `realtime_wall_clock: true`로 실제 경과 시간을 따라 진행하며, Isaac의 Play 동작이 수동 고정 스텝으로 되돌리지 않도록 설정한다. `min_frame_rate: 20`은 200/30의 6~7 물리 스텝을 허용한다. 이전 값 30에서는 프레임당 여섯 스텝으로 제한돼 시간이 약 10% 부족했다. 이 설정은 하드 실시간 보장이 아니며 `status.json`의 `realtime_factor`로 실제 진행률을 확인한다.

LiDAR는 동일 PhysX 장면에서 정수 hit ID를 반환하는 `physics_core` 경로를 사용한다. 기존 API와 1087개 광선을 비교해 거리 차이가 0이었고, 조회 시간 중앙값은 12.06 ms→5.38 ms였다. 자세한 전후 측정은 `validation/performance/`에 있다. GUI 복구/새 실행 시 `~/isaac_sim_6_2/python.sh tools/start_gui.py`로 같은 차량·맵과 controller를 함께 열 수 있다.

## 빌드와 검사

핵심 계산은 C99이고 Isaac API와 ROS I/O는 Python이다.

```bash
cmake -S . -B build
cmake --build build
python3 -m pytest tests -q
```

ROS 메시지는 기존 `damvi_ws`의 `vesc_msgs`를 변경 없이 복사했다. 서버의 ASCII 경로 `/home/shin/damvi_vesc_ros_ws`에 ROS 2 Jazzy로 빌드했다. 별도 standalone runner는 초기 물리 검증용으로 남겨 두었으며, 현재 작업은 기존 GUI에서 진행한다.

```bash
./run.sh --stage /home/shin/Desktop/RC_Car_Isaac/rc_car_vehicle_only.usd \
  --flat-ground --scenario forward_reverse --duration 12
```

검증 결과는 `validation/`을 참고한다. 제어기·구동계·조향·TF·센서 검사 총 67개를 통과했다. 현재 200 Hz 물리에서 전후진 선회·정지·조향 포화·IMU 각속도·LiDAR 주기와 Play 설정 재읽기를 확인했다. 실제 조향 관절 sweep 오차는 최대 약 0.75°였고, 동시에 LaserScan 371개를 받았다. IMU 출력 간격은 1 ms, 정지 평균 가속도 z는 약 9.80998 m/s²다. 최종 10.00117초 동안 시뮬레이션은 10.005초 진행되어 평균 1.00038배속이었다. 과거 1 kHz 물리 검증도 별도 보존한다. 실차 bag과 동특성을 정량적으로 일치시킨 상태는 아니다.

속도 루프는 VESC 코드를 참고한 GPL-3.0-or-later 구현이다. 복사된 `vesc_msgs`에는 원래 BSD 라이선스와 package metadata를 유지했다.
