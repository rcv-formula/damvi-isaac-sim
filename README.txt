RC Car - Fusion 360 / Isaac Sim

완성 장면: rc_car_scene.usd
Fusion 문서: platform-nuc-one-plate v6
실제 CAD 메시를 포함합니다. 이전 흰색 임시 차체는 이 장면에 없습니다.

원격 Ubuntu: /home/shin/Desktop/RC_Car_Isaac/rc_car_scene.usd
Windows 바탕화면: RC_Car_Isaac 폴더 / RC_Car_Isaac.zip
Isaac Sim에서 rc_car_scene.usd를 열고 Play를 누르면 물리 시뮬레이션이 시작됩니다.
파일을 열 때 OBJ/MTL이나 인터넷 에셋을 추가로 불러올 필요가 없습니다.

가져온 형상
- Fusion에서 OBJ(밀리미터)로 내보낸 차체, 프레임, 장비 및 센서 모사체
- 실제 크기 보존: 길이 533.512 mm,
  최대 폭 270.000 mm, 높이 284.121 mm
- 시각 메시 105개, 면 191,712개
- 전체 CAD를 하나의 강체로 묶었습니다. 부품 간 완전한 기계 조립은 요구하지 않습니다.
- 충돌은 CAD 그룹별 상자 15개를 합친 근사 형상이며,
  차대뿐 아니라 높은 뒤 프레임과 상단 장비도 포함합니다.
- 모델 좌표: +X 전방, +Y 왼쪽, +Z 위쪽. 초기 차체 최저 지상고 12 mm.

INI 및 요청값 반영
- 전체 질량 3.5 kg (차체, 바퀴, 연결부 모두 포함)
- 명목 무게중심 높이 70 mm, 초기 저장 자세의 전체 무게중심 (0, 0, 0.07) m
- 무게중심에서 앞/뒤 차축까지 150 / 100 mm: 축간거리 250 mm
- 앞/뒤 윤거 200 / 230 mm (바퀴 중심 사이)
- 타이어 외경 55.88 mm (2.2 inch), 폭 40 mm, 원기둥 4개
- 바퀴는 Isaac Sim용으로 새로 만들었으며 Fusion 파일에서 가져온 것이 아닙니다.
- 타이어 외형은 원주 256분할 메시이며 옆면에 부드러운 법선을 적용했습니다.
  바퀴당 정점 514개, 렌더링 삼각형 1024개입니다.
- 타이어 충돌은 같은 외경/폭의 해석적 원기둥입니다. 접촉 계산에는 다각형 근사를 사용하지 않습니다.
- 전체 수직축 관성 0.04712 kg*m^2
- 중력 9.81 m/s^2, 바닥과 타이어 마찰계수 0.9, 반발계수 0
- 서스펜션 상하 이동 한계 +/-1.5 mm, 강성 20000 N/m, 감쇠 200 N*s/m
- 바퀴마다 상하 프리즘 관절과 자유 회전축이 있습니다. 고정 조인트가 아닙니다.
- 물리 계산 480 Hz, 화면에 보이는 바닥 20 x 20 m, 충돌 바닥은 Z=0 평면

원본 값 간 차이와 추정치
INI의 일반 차량 길이/폭 400/220 mm와 실제 CAD의 외곽 크기가 다릅니다.
CAD를 강제로 축소하지 않았으며 바퀴 배치는 더 구체적인 축간거리/윤거를 따릅니다.
폭 40 mm 타이어를 포함한 바퀴 바깥 폭은 앞 240 mm, 뒤 270 mm입니다.
CAD의 270 mm 최대 폭은 앞 범퍼를 포함합니다.
개별 무게가 없어 차체 3.1 kg, 바퀴 각각 0.075 kg, 연결부 각각 0.025 kg으로
배분했습니다. 차체의 질량중심과 관성을 보정해 합산 값이 INI와 일치하게 했습니다.
가로/세로 회전 관성과 서스펜션 값은 추정치이며 실차 측정으로 보정할 수 있습니다.
타이어는 강체이며 고무 자체의 변형은 모델링하지 않습니다. 연결부의 미세한 탄성은 유지됩니다.
Physics > Approximate Cylinders With Convex Meshes 설정을 끈 상태를 USD에 저장했습니다.
/physics/collisionApproximateCylinders = false, PhysicsScene GPU Dynamics = false입니다.
설치된 Isaac Sim AnalyticCylinderDemo와 동일한 native Cylinder 충돌 방식을 사용합니다.
바닥 충돌은 native Plane입니다. 현재 설치 버전에서 작은 원기둥과 큰 상자 바닥의
접촉이 불안정해지는 현상을 재현해, 안정적인 평면 접촉으로 변경했습니다.

실제 검증 결과
Isaac Sim 6.0.1-rc.7의 일반 Play로 10.004초 실행했습니다.
정적 설정 및 실제 물리 안정성 검사를 통과했습니다.
- 측정 전체 질량: 3.499999918 kg
- 초기 전체 무게중심 높이: 70.000000 mm
- 안착 후 무게중심 높이: 69.8970 ~ 69.8970 mm
- 관측된 최대 서스펜션 움직임: 0.1437 mm
- 자세, 바닥 접촉, 차체 지상고, 관절 연결, 정지 후 이동/속도 검사 통과
자세한 실측값과 검사 기준은 validation_final.json에 있습니다.
추가로 실제 차량에 초기 속도 0.3 m/s와 일치하는 바퀴 각속도를 한 번 부여해
2.000초 동안 자유 구름을 검증했습니다.
- 전진 거리: 599.996 mm
- 초기 0.25초를 제외한 바퀴 높이의 최대 변동 폭: 0.000101 mm
- 네 바퀴 각각 3회전 이상, 바닥 접촉/미끄러짐/서스펜션/자세 검사 통과
시험 후 저장된 초기 자세로 복원했습니다. 시험용 속도나 구동기는 최종 파일에 남기지 않았습니다.
이 검증은 평지 정지/안착 및 자유 구름 시험입니다. 동력 주행/조향 성능 시험은 포함하지 않습니다.

현재 범위
ROS, 실제 센서, 추진/조향 제어는 설정하지 않았습니다.
CAD의 센서 외형은 포함되지만 센서 데이터를 생성하지 않습니다.
INI의 Magic Formula, 항력, 가감속/조향 동특성을 별도 구현한 모델은 아닙니다.

파일
- rc_car_scene.usd: 자체 완결된 최종 장면
- preview.png: 최종 장면 화면
- tire_closeup.png: 부드러운 타이어 외형 확대 화면
- rc_car_scene.build.json: 형상/물성/변환 설정 기록
- validation_final.json: 실제 검증 결과
- build_config.json: CAD 좌표 변환, 충돌 형상, 카메라 설정
- build_rc_scene.py, rebuild_scene.py, run_usd_python.sh: 재생성 도구
- validate_rc_scene.py: 물리 검증 도구
- source/rc_car_fusion.obj, source/rc_car_fusion.mtl: Fusion 내보내기 원본
- source/f110_rgb.ini: 사용자 매개변수 원본

원격 Ubuntu에서 재생성할 경우(장면을 다시 열기 전 실행):
bash /home/shin/Desktop/RC_Car_Isaac/run_usd_python.sh /home/shin/Desktop/RC_Car_Isaac/rebuild_scene.py
