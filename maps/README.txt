맵 선택 방법
1. File > Open: ../rc_car_vehicle_only.usd
2. Content 브라우저 경로: /home/shin/Desktop/RC_Car_Isaac/maps
3. 원하는 .usdc 파일 하나를 Stage 패널의 /World 아래로 드래그 앤 드롭.
4. Reference/Payload를 묻는 경우 Reference 선택. 추가한 맵의 Transform은 translate/rotate=(0,0,0), scale=(1,1,1)로 유지.
5. 다른 맵으로 바꾸려면 추가한 맵 prim 전체를 삭제한 뒤 새 맵 추가. 맵마다 바닥 포함.
6. File > Save As로 조합한 씬 저장. 원본 vehicle_only 씬은 재사용 권장.

rc_car_scene.usd 및 rc_car_scene_with_0821.usd에는 이미 0821 맵이 포함되어 있음.
이 씬에 다른 맵을 추가하면 두 맵과 바닥이 겹치므로 차량 전용 씬에서 시작하는 것이 편함.

맵: 0521, 0814, 0821, 0821-1, 0822, ifac, ifac1, ifac2.
일부 데이터 폴더의 맵은 같으므로 8개 파일이 모두 서로 다른 코스인 것은 아님.
details/ 아래에 맵별 추정 중심선 JSON과 미리보기 PNG 포함.
0821/track.usdc는 기존 조합 씬에서 사용하는 참조 경로이므로 유지.
차량 형상/위치/센서 설정은 변경하지 않았음.
