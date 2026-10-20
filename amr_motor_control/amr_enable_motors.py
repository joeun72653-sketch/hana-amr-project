
import time
import serial

from amr_keyboard_safe import UCan

PORT = "/dev/ttyUSB0"
BAUD = 115200
NODES = (1, 2)

STEPS = [
    (0x0006, 0x0021, "Ready to Switch On"),
    (0x0007, 0x0023, "Switched On"),
    (0x000F, 0x0027, "Operation Enabled"),
]


def decode_state(status):
    value = status & 0x006F

    states = {
        0x0040: "Switch On Disabled",
        0x0021: "Ready to Switch On",
        0x0023: "Switched On",
        0x0027: "Operation Enabled",
        0x0007: "Quick Stop Active",
        0x0008: "Fault",
        0x000F: "Fault Reaction Active",
    }

    return states.get(value, f"Unknown 0x{value:04X}")


def read_status(can, node):
    return can.read_sdo(
        node, 0x6041, signed=False
    )


def write_controlword(can, node, value):
    payload = (
        b"\x2B\x40\x60\x00"
        + value.to_bytes(2, "little")
        + b"\x00\x00"
    )

    can.send(node, payload)

    response = can.wait_sdo(node, 0x6040)

    if response is None or response[0] != 0x60:
        raise RuntimeError(
            f"Node {node}: Controlword ACK 실패"
        )


def wait_state(can, node, expected):
    deadline = time.monotonic() + 2.0

    while time.monotonic() < deadline:
        status = read_status(can, node)

        if (status & 0x006F) == expected:
            print(
                f"[OK] Node {node}: "
                f"0x{status:04X} "
                f"({decode_state(status)})"
            )
            return

        if (status & 0x004F) == 0x0008:
            raise RuntimeError(
                f"Node {node}: Fault 상태"
            )

        time.sleep(0.1)

    raise TimeoutError(
        f"Node {node}: 상태 전이 시간 초과"
    )


def verify_zero(can):
    for node in NODES:
        target = can.read_sdo(
            node, 0x60FF, signed=True
        )
        actual = can.read_sdo(
            node, 0x606C, signed=True
        )

        print(
            f"[Node {node}] "
            f"Target={target}, Actual={actual}"
        )

        if target != 0 or actual != 0:
            raise RuntimeError(
                f"Node {node}: 속도가 0이 아님"
            )


def main():
    print("=" * 50)
    print("AMR CANopen Operation Enable")
    print("=" * 50)

    with serial.Serial(
        PORT, BAUD, timeout=0.02
    ) as ser:

        can = UCan(ser)
        print("[OK] uCAN 연결 성공")

        print("\n[STEP 1] 속도 0 확인")
        verify_zero(can)

        print("\n[STEP 2] 현재 상태 확인")

        for node in NODES:
            status = read_status(can, node)
            print(
                f"Node {node}: 0x{status:04X} "
                f"({decode_state(status)})"
            )

            if (status & 0x006F) not in (
                0x0040, 0x0021, 0x0023, 0x0027
            ):
                raise RuntimeError(
                    f"Node {node}: 예상하지 못한 상태"
                )

        answer = input(
            "\n기체 고정 및 독립 비상정지 확인 후 "
            "ENABLE 입력: "
        ).strip()

        if answer != "ENABLE":
            print("[CANCEL] 활성화 취소")
            return

        try:
            for control, expected, name in STEPS:
                print(f"\n[STEP] {name}")

                for node in NODES:
                    status = read_status(can, node)

                    if (status & 0x006F) == expected:
                        print(
                            f"[SKIP] Node {node}: "
                            "이미 해당 상태"
                        )
                        continue

                    print(
                        f"[TX] Node {node}: "
                        f"Controlword=0x{control:04X}"
                    )

                    write_controlword(
                        can, node, control
                    )
                    wait_state(
                        can, node, expected
                    )

            print("\n[FINAL] 상태 확인")

            for node in NODES:
                status = read_status(can, node)

                if (status & 0x006F) != 0x0027:
                    raise RuntimeError(
                        f"Node {node}: Enable 실패"
                    )

            verify_zero(can)

            print(
                "\n[SUCCESS] 양쪽 모터 "
                "Operation Enabled"
            )

        except Exception as error:
            print("[CRITICAL]", error)
            print(
                "[ACTION] 구동하지 말고 "
                "드라이버 상태와 독립 비상정지 확인"
            )
            raise


if __name__ == "__main__":
    main()
