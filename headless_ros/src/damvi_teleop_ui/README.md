# DAMVI 시뮬레이터 주행 모드 UI (ROS 2)

정지 / 수동주행 / 자율주행 버튼, 수동 속도 입력, 현재 출력 명령 숫자와 최근 10초 그래프를 제공한다. 외부 Python 그래프 라이브러리 없이 PyQt5로 실행한다. 시뮬레이터 또는 VESC 변환 노드에 연결할 명령 선택기이며, 차량 물리 시뮬레이터 자체는 포함하지 않는다.

ROS 2 `ament_python` 패키지이며 패키지 이름은 `damvi_teleop_ui`다. UI는 흰 배경·검정 글자·흑백 그래프를 사용하고, 선택 모드는 굵은 테두리와 회색 배경으로 표시한다.

## 원본 인터페이스 확인

참조: [damvi_ws](https://github.com/rcv-formula/damvi_ws/tree/a137421618c8c305606f41071d69d0c53a1aae25), 여기에 고정된 [f1_stack_for_damvi](https://github.com/rcv-formula/f1_stack_for_damvi/tree/deb995496a7dcb4c52fce34b4f8fc681db6360ec).

원본 경로:

```text
/cmd_vel (Twist) → drive_publisher → /drive (AckermannDriveStamped)
                                                    ↓ ackermann_mux ← /teleop (RF/joy)
                                         /ackermann_drive (AckermannDriveStamped)
                                                    ↓ ackermann_to_vesc
                                  /commands/motor/speed + /commands/servo/position
```

이 패키지는 `drive_publisher + ackermann_mux + RF/joy`의 명령 선택 역할을 대신한다. 명령 구독은 1개, 명령 발행은 1개다. 기본 입력 `/cmd_vel`, 기본 최종 출력 `/ackermann_drive`로 원본 이름과 형식을 유지한다. 원본의 최종 출력은 `cmd_vel`이나 `Twist`가 아니다.

| 항목 | 원본 및 이 패키지의 의미 |
|---|---|
| `Twist.linear.x` | 속도, m/s |
| `Twist.angular.z` | 조향각, rad (양수=좌, 음수=우) |
| `AckermannDriveStamped.drive.speed` | 입력의 `linear.x` |
| `AckermannDriveStamped.drive.steering_angle` | 입력의 `angular.z` |

`angular.z`를 yaw rate(rad/s)로 해석하는 시뮬레이터에는 별도 bicycle model 변환이 필요하다. 원본 `drive_publisher.py`와 동일하게 조향각으로 처리한다.

**원본 연결 주의점:** `damvi/launch/bringup_launch.py`는 mux 출력을 `ackermann_drive`로 remap하지만 `vesc_ackermann/src/ackermann_to_vesc.cpp`는 `ackermann_cmd`를 구독한다. VESC 변환 노드에 `remappings=[('ackermann_cmd', '/ackermann_drive')]`를 추가해야 연결된다.

## 빌드 및 실행

ROS 2 Humble / Ubuntu 22.04 기준. 아래 예시는 현재 작업 디렉터리가 이 패키지의 상위 폴더일 때 사용한다. 기존 ROS workspace에서는 이 패키지 폴더를 `src/`에 넣어 같은 방식으로 빌드할 수 있다.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths damvi_teleop_ui --ignore-src -r -y
colcon build --packages-select damvi_teleop_ui --symlink-install
source install/setup.bash
ros2 launch damvi_teleop_ui teleop_ui.launch.py
```

기본 launch는 UI만 실행하며 실차 bringup이나 하드웨어 드라이버를 시작하지 않는다. GUI 실행에는 데스크톱 DISPLAY가 필요하다.

## 조작

- 시작 상태는 **정지**, 속도·조향을 모두 0으로 50 Hz 발행한다.
- **수동주행:** UI에 포커스를 두고 ↑=전진 +2 m/s, ↓=후진 −2 m/s. UI의 속도 입력에서 절댓값을 즉시 변경한다(기본 2, 범위 0–20 m/s). 이 범위는 UI 편집 범위이며 차량 성능 제한을 뜻하지 않는다.
- ←=최대 좌향 +0.6730769 rad, →=최대 우향 −0.6730769 rad. 전후진과 조향은 동시에 입력 가능하다. 조향만 눌러도 정차 상태에서 조향이 출력된다.
- 키를 놓으면 해당 축은 0이다. 반대 키를 동시에 누르면 해당 축은 0이다. 수동 중 앱 포커스를 잃으면 정지 모드로 전환한다. 속도 입력란의 방향키는 숫자 편집에만 쓰며, 모드 버튼을 다시 누르면 운전 포커스가 돌아온다.
- **자율주행:** 해당 모드에 진입한 뒤 새로 받은 입력만 전달한다. 기본 출력에서는 원본처럼 속도와 조향을 Ackermann 형식으로 옮기며, Twist 출력에서는 여섯 필드를 모두 그대로 복사한다. 수동 속도·조향 제한으로 자율 입력을 자르지 않는다.
- 기본적으로 자율 입력이 0.5초 끊기면 0을 출력한다. `auto_timeout: 0.0`이면 시간 제한 없이 마지막 입력을 유지한다. 비유한 값(NaN/Inf)은 0 처리한다.
- **정지:** 입력 및 키 상태와 관계없이 0을 계속 출력한다. 모드 변경 시 이전 키와 자율 입력을 지운다. 창을 정상 종료할 때도 0을 한 번 보낸다.

최대 조향은 원본 `vesc.yaml`의 `servo_min=0.15`, `servo_max=0.85`, `offset=0.50`, `gain=-0.52`에서 `(0.50-0.15)/0.52 ≈ 0.6730769 rad = 38.56°`로 산출했다. 원본 joy 스케일 ±0.8은 이 서보 범위를 넘는다. 이는 설정상 최대값이며 실제 차량/시뮬레이터의 기계적 한계가 다르면 `max_steering_angle`을 수정한다.

숫자 및 그래프는 **현재 출력 명령**이다. 실제 속도와 바퀴 조향 센서 측정값을 표시하지 않는다. GUI나 프로세스의 강제 종료·멈춤에서는 마지막 정지 메시지 전달을 보장할 수 없으므로 시뮬레이터 수신부는 명령 타임아웃을 구현해야 한다.

## 최종 출력도 Twist가 필요한 시뮬레이터

원본에는 서로 다른 입력/출력 Twist 토픽 쌍이 없으므로, 이 경우 새 최종 토픽 `/cmd_vel_out`을 지정한다. 입력과 출력을 같은 이름으로 설정하면 시작 시 거부한다.

```bash
ros2 run damvi_teleop_ui teleop_ui --ros-args \
  -p output_type:=twist -p output_topic:=/cmd_vel_out
```

```text
자율 제어기 /cmd_vel (Twist) → UI → /cmd_vel_out (Twist) → 시뮬레이터
```

이 경로에서도 `angular.z`는 조향각(rad)이다. 토픽 이름은 `input_topic`, `output_topic` 파라미터 또는 ROS remap으로 변경할 수 있다.

## 기존 노드 연결

이 UI가 최종 토픽을 발행할 때 기존 `ackermann_mux`를 함께 실행하면 같은 최종 토픽에 두 발행자가 생긴다. 시뮬레이션용 launch에서는 기존 `joy`, `joy_teleop`, RF 노드, `drive_publisher`, `ackermann_mux` 대신 이 UI를 실행하고, 시뮬레이터가 최종 토픽을 구독하게 한다. VESC 경로를 재사용할 때는 아래처럼 변환 노드의 입력을 연결한다.

```python
Node(
    package='vesc_ackermann', executable='ackermann_to_vesc_node',
    name='ackermann_to_vesc_node', parameters=[vesc_config],
    remappings=[('ackermann_cmd', '/ackermann_drive')],
)
```

## 확인

별도 터미널에서(동일 ROS 환경 source):

```bash
ros2 topic echo /ackermann_drive
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.2}, angular: {z: 0.2}}'
```

정지에서 0, 수동에서 키 입력값, 자율에서 1.2 m/s·0.2 rad가 나와야 한다. 입력 publisher를 중단하면 0.5초 뒤 0이 된다.

```bash
cd damvi_teleop_ui
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m pytest -q
```

테스트는 모드 전환, 동시 키, 키 해제, 속도 편집, 포커스 상실, 자율 입력 전달과 시간 초과, 정상 종료를 실제 ROS 2 publisher/subscriber와 Qt 이벤트로 확인한다. 추가로 Twist 출력의 모든 필드 보존을 확인한다.
