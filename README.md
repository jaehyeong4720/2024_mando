1. **gps_data_parsing.py**
   - `ublox_gps fix` 데이터를 받아 위경도 값 및 UTM 좌표로 토픽을 발행
     - 구독: `/ublox_gps_base/fix`, `/ublox_gps_rover/navrelposned`
     - 발행: `/gps_lat`, `/gps_lon`, `/utm_x_topic`, `/utm_y_topic`
   - `ublox_gps navrelposned` 데이터를 받아 헤딩값으로 토픽을 발행
     - 발행: `/gps_heading_topic`

2. **joy.py**
   - 메인 코드로부터 `/main_cmd_joy` 명령을 받아 조이스틱 기능을 구현
   - 아두이노로 속도 명령을 바로 전송
     - 구독: `/joy`
     - 발행: `/cmd_vel_steer`

3. **main.py**
   - UTM 좌표 및 조이스틱 데이터를 구독하여, 미션을 발행할 트리거를 관리
     - 구독: `/joy`, `/utm_x_topic`, `/utm_y_topic`, `/TP_done`, `/PP_done`, `/red_detected`, `/obstacle_detected`, `/stop_line_detected`
     - 발행: `/main_cmd_joy`, `/main_cmd_way`, `/main_cmd_way_num`, `/main_cmd_T_parking`, `/main_cmd_P_parking`, `/main_cmd_stop`
   - 미션 발행 트리거는 지오펜스를 통해 구역으로 관리

4. **obstacle.py**
   - 라이다 스캔 데이터를 구독하고, 전처리하여 장애물 판별
   - 복잡하고 직관적이지 않아 최적화 필요
     - 구독: `/scan`
     - 발행: `/obstacle_detected`

5. **p_parking.py**
   - 메인으로부터 P파킹 명령을 받으면 상수값으로 P주차를 실행
   - 주차 완료 후, `/PP_done` 토픽을 발행하여 웨이포인트 주행 재개
     - 구독: `/main_cmd_P_parking`
     - 발행: `/cmd_vel_steer`, `/PP_done`

6. **t_parking.py**
   - 메인으로부터 T파킹 명령을 받으면 상수값으로 T주차를 실행
   - 주차 완료 후, `/TP_done` 토픽을 발행하여 웨이포인트 주행 재개
     - 구독: `/main_cmd_T_parking`
     - 발행: `/cmd_vel_steer`, `/TP_done`

7. **stop.py**
   - 메인으로부터 `/main_cmd_stop` 명령을 받아 차량 정지 실행
   - 장애물, 정지선, 신호등 감지 시 동작
     - 구독: `/main_cmd_stop`
     - 발행: `/cmd_vel_steer`

8. **traffic.py**
   - `/usb_cam/image_raw`를 구독하여 신호등의 특정 색상을 감지하고, 감지 결과를 토픽으로 발행
   - 카메라 세팅과 HSV 임계값 조절 필요
     - 구독: `/usb_cam/image_raw`
     - 발행: `/red_detected`

9. **traffic_comp.py**
   - `image_raw` 대신 압축된 파일을 구독하여 대역폭 문제를 해결
   - 실제 사용은 아직이지만 모바일 환경에서 잘 작동할 것으로 예상
     - 구독: 압축된 이미지 데이터

10. **way.py**
   - GPS 데이터를 UTM 좌표로 구독하여 Stanley Method로 Waypoint 주행 가능
   - 차량 전륜 중심에 맞추어 오프셋 조절 필요
   - 아두이노 주기에 맞추어 설정 필요
     - 구독: `/main_cmd_way`, `/main_cmd_way_num`, `/gps_heading_topic`, `/utm_x_topic`, `/utm_y_topic`
     - 발행: `/cmd_vel_steer`, (`/csv_n_done`: 실험 중 사용된 토픽)

11. **white_line.py**
   - 엔코더 기반으로 정지 기능 구현
   - 슬립을 대비해 실제 거리보다 약간 더 적게 설정 권장
   - 코드 내에서 정지 시간과 차량별 엔코더 값을 조절
     - 구독: `/encoder1` (아두이노에서 발행)
     - 발행: `/stop_line_detected` (토픽 이름 변경 필요)
