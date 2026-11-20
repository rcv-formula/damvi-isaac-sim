# DAMVI Isaac Sim

DAMVI RC 차량과 트랙을 위한 NVIDIA Isaac Sim/PhysX 시뮬레이터입니다. 기존 실행 환경은 Isaac Sim 6.2와 ROS 2 Jazzy입니다.

- [`rc_car_scene.usd`](rc_car_scene.usd)에는 차량과 기본 `0821` 트랙이 들어 있습니다. Isaac Sim에서 직접 열 수 있습니다. 차량 전용 장면과 교체용 트랙은 [맵 안내](maps/README.txt)를 참고하세요.
- [`vesc_sim/`](vesc_sim/)은 VESC 구동·제동, 조향, 타이어, LiDAR, IMU와 ROS 브리지를 포함합니다. [설정과 실행법](vesc_sim/README.md)에 CMake 빌드와 실행 예제가 있습니다.
- [`headless_ros/`](headless_ros/)는 ROS 주행·위치추정 패키지, 검증 도구, 점유 격자 맵과 레이스라인을 담고 있습니다. 트랙의 변환 결과는 [`occupancy_to_usd/generated/`](headless_ros/project/occupancy_to_usd/generated/)에도 있습니다.

## 실행 환경

`vesc_sim/tools/start_gui.py`에 `--headless`를 추가하면 같은 차량 어댑터를 화면 없이 실행합니다. `--dataset`으로 기존 트랙을 선택할 수 있습니다. 주행 어댑터를 실행하기 전에 `cmake -S vesc_sim -B vesc_sim/build`와 `cmake --build vesc_sim/build`로 네이티브 라이브러리를 빌드합니다.

ROS 연동 및 랩 검증에는 ROS 설치와 워크스페이스 빌드가 필요합니다. `vesc_msgs` 소스는 `vesc_sim/ros_ws/src/`에 있습니다. 기존 호스트에서 쓰던 경로가 `vesc_sim/tools/install_gui.py`, `vesc_sim/live_gui.py`, `headless_ros/server_ros.sh` 및 검증 도구에 들어 있으므로 다른 환경에서는 Isaac 설치·체크아웃·ROS 오버레이 경로를 맞춰야 합니다. `headless_ros/server_ros.sh`는 `DAMVI_SERVER_ROS_WS`로 ROS 워크스페이스 경로를 받습니다.
