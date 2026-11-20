# ACS MQTT BMS 연동 (AMR-01)

기존 TABOS BMS RS-485 측정 데이터를 MQTT 브로커로 약 2초마다 보내는 **독립 실행 테스트 브리지**입니다. CAN/리프트/주행 프로그램을 실행하지 않아도 동작합니다.

## 구성
- 파일: `amr_bms_mqtt_periodic_charging.py`
- 배터리: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`, 19200 baud
- 브로커: `211.233.216.202:8883` (MQTT 3.1.1 / TLS client certificate)
- 발행 토픽: `vda5050/v3/HANA_AI/HA-AMR-001/state`
- 발행 주기: 기본 2초 / retain=false / **현재 구현 QoS 1** (인터페이스 정의서의 QoS 0과 차이 있음)
- 충전 판정: 실측 전류 > +0.05 A → true, < -0.05 A → false, 근접 0 A → 직전 판정 유지. 처음부터 0 A 근처라면 판정을 보류합니다.
- 작업 필드(`orderId`, `orderUpdateId`, `lastNodeId`, `lastNodeSequenceId`, `nodeStates`, `edgeStates`, `actionStates`, `instantActionStates`)는 미수행 상태로 채웁니다.

## 실행
인증서와 개인키는 **Git에 올리지 않고** IPC의 `/etc/hana-amr/certs/AMR-01-MQTT`에 설치합니다. 현재 파일은 root:root, 0600/0700 권한이므로 아래처럼 sudo로 실행합니다. 동일한 시리얼 포트를 쓰는 `battery_monitor.py`가 실행 중이면 먼저 종료하세요.

```bash
cd ~/amr_battery
source ~/amr_battery/mqtt_env/bin/activate
sudo ~/amr_battery/mqtt_env/bin/python amr_bms_mqtt_periodic_charging.py
```

Git에서 가져온 파일을 위 경로에 복사한 뒤 실행해야 합니다.

## 주의
이 브리지는 **BMS 필드 중심의 부분 VDA 5050 state**만 발행합니다. `driving`, `operatingMode`, `safetyState` 등 정의서의 필수 항목이 없으므로 ACS 수신부에서 전체 스키마 검증을 수행하면 **거부될 수 있습니다**. 메시지 발행 성공은 ACS 화면 표시에 대한 보증이 아닙니다. 실제 운용용 완전한 state 발행기는 다른 ROS2/CAN/안전 데이터와 통합해야 합니다.

같은 장비의 MQTT 클라이언트 ID `hana-amr-001`를 복수 프로세스에서 동시에 사용하지 마세요. 추가 인스턴스는 기존 연결을 끊을 수 있습니다.
