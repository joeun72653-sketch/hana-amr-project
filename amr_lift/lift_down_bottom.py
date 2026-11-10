#!/usr/bin/env python3
"""ORGO lift: move to BOTTOM using RS-232/Modbus RTU.

Standalone maintenance script. Do not run concurrently with lift_websocket.py
or other serial clients. Verify wheel drive is disabled and the mechanism
is clear before running. Hardware E-stop must remain available.
"""
import signal
import time

from pymodbus.client import ModbusSerialClient

PORT = "/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D20J8ODC-if00-port0"
DEVICE_ID = 1
MAX_TIME = 45.0
MAX_POSITION_DIFF = 20

STOP_STATE = 0xF000
DOWN_STATE = 0xF001
BOTTOM_STATE = 0x0202
TOP_STATE = 0x0404
MID_STATE = 0x0101

client = ModbusSerialClient(
    port=PORT, baudrate=115200, bytesize=8, parity="N",
    stopbits=1, timeout=0.3, retries=0,
)


def handle_signal(signum, frame):
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, handle_signal)
signal.signal(signal.SIGTERM, handle_signal)


def read_status():
    result = client.read_holding_registers(
        address=1, count=7, device_id=DEVICE_ID
    )
    if result.isError() or len(result.registers) != 7:
        raise RuntimeError("상태 조회 실패")
    return list(result.registers)


def send_command(value):
    result = client.write_register(
        address=0, value=value, device_id=DEVICE_ID
    )
    if result.isError():
        raise RuntimeError(f"명령 오류: {result}")


def main():
    connected = False
    bottom_reached = False
    try:
        connected = client.connect()
        if not connected:
            raise RuntimeError("RS232 통신 연결 실패")

        before = read_status()
        print("=== 하강 전 상태 ===", flush=True)
        print(f"운전: 0x{before[0]:04X}")
        print(f"위치 상태: 0x{before[1]:04X}")
        print(f"오류: 0x{before[2]:04X}")
        print(f"Lift 1: {before[5]} Pulse")
        print(f"Lift 2: {before[6]} Pulse")

        if before[0] != STOP_STATE:
            raise RuntimeError("정지 상태가 아닙니다")
        if before[2] != 0:
            raise RuntimeError("리프트 오류가 있습니다")
        if abs(before[5] - before[6]) > MAX_POSITION_DIFF:
            raise RuntimeError("두 리프트 위치 차이 초과")

        if before[1] == BOTTOM_STATE:
            print("이미 BOTTOM 위치입니다.", flush=True)
            bottom_reached = True
        elif before[1] not in (TOP_STATE, MID_STATE):
            raise RuntimeError("예상하지 못한 위치 상태")
        else:
            print("\n[DOWN] BOTTOM 방향 하강 시작", flush=True)
            start1, start2 = before[5], before[6]
            start = time.monotonic()
            send_command(2)

            while True:
                if time.monotonic() - start >= MAX_TIME:
                    raise RuntimeError("최대 운전 시간 초과")

                s = read_status()
                motor_state, position_state, error = s[:3]
                pos1, pos2 = s[5], s[6]

                print(
                    f"L1={pos1} L2={pos2} "
                    f"하강량={start1-pos1}/{start2-pos2} "
                    f"차이={abs(pos1-pos2)} "
                    f"운전=0x{motor_state:04X} "
                    f"위치=0x{position_state:04X} "
                    f"오류=0x{error:04X}", flush=True
                )

                if error != 0:
                    raise RuntimeError("리프트 오류 발생")
                if abs(pos1 - pos2) > MAX_POSITION_DIFF:
                    raise RuntimeError("리프트 위치 편차 초과")
                if pos1 > start1 + 5 or pos2 > start2 + 5:
                    raise RuntimeError("예상과 다른 이동 방향")

                if position_state == BOTTOM_STATE:
                    print("\n[BOTTOM] 양쪽 BOTTOM 감지!", flush=True)
                    bottom_reached = True
                    break
                if motor_state == STOP_STATE:
                    raise RuntimeError("BOTTOM 도달 전 모터 정지")
                if motor_state != DOWN_STATE:
                    raise RuntimeError(
                        f"예상하지 못한 운전 상태: 0x{motor_state:04X}"
                    )
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[중단] 사용자 중단 요청", flush=True)
    except Exception as exc:
        print("\n[ERROR]", exc, flush=True)
    finally:
        if connected:
            try:
                print("\n[STOP] 정지 명령 전송", flush=True)
                send_command(4)
                print("STOP 명령 응답 정상", flush=True)
                time.sleep(0.2)
                after = read_status()
                print("\n=== 최종 상태 ===")
                print(f"운전: 0x{after[0]:04X}")
                print(f"위치: 0x{after[1]:04X}")
                print(f"오류: 0x{after[2]:04X}")
                print(f"Lift 1: {after[5]} Pulse")
                print(f"Lift 2: {after[6]} Pulse")
                if (bottom_reached and after[0] == STOP_STATE
                        and after[1] == BOTTOM_STATE and after[2] == 0):
                    print("\nBOTTOM 도달 및 정지 상태 확인!")
                else:
                    print("\nBOTTOM 완료 조건 미충족")
                    print("추가 구동 전에 상태를 확인하세요.")
            except Exception as exc:
                print("[위험] STOP 또는 상태 확인 실패:", exc)
                print("독립적인 정지 수단으로 안전 확보 필요")
            client.close()


if __name__ == "__main__":
    main()
