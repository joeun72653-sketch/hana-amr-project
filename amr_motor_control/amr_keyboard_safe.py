
import serial
import time
import sys
import tty
import termios
import select

# ==========================================
# AMR CANopen 안전 초기화 + 키보드 제어
# ==========================================

PORT = "/dev/ttyUSB1"
BAUD = 115200
NODES = (1, 2)

SPEED = 30
SDO_TIMEOUT = 1.5

# 실제 전진 송신 비활성화
SIMULATE_FORWARD = True

ZERO = bytes.fromhex(
    "23 FF 60 00 00 00 00 00"
)


class UCan:
    def __init__(self, ser):
        self.ser = ser
        self.buffer = bytearray()

    def send(self, node, data):
        can_id = 0x600 + node

        frame = (
            f"t{can_id:03X}{len(data):X}"
            f"{data.hex().upper()}\r"
        )

        raw = frame.encode("ascii")
        written = self.ser.write(raw)
        self.ser.flush()

        if written != len(raw):
            raise IOError(
                f"Node {node}: CAN 송신 불완전"
            )

        print(
            f"[TX] Node {node}: "
            f"{data.hex(' ').upper()}"
        )

    def receive(self, timeout):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            chunk = self.ser.read(
                self.ser.in_waiting or 1
            )

            if chunk:
                self.buffer.extend(chunk)

            while b"\r" in self.buffer:
                packet, _, remaining = (
                    self.buffer.partition(b"\r")
                )
                self.buffer = bytearray(remaining)

                try:
                    msg = packet.decode(
                        "ascii"
                    ).strip()

                    if not msg.startswith("t"):
                        continue

                    if len(msg) < 5:
                        continue

                    can_id = int(msg[1:4], 16)
                    dlc = int(msg[4], 16)

                    if dlc > 8:
                        continue

                    data = bytes.fromhex(msg[5:])

                    if len(data) != dlc:
                        continue

                    return can_id, data

                except (
                    ValueError,
                    UnicodeDecodeError
                ):
                    continue

            if len(self.buffer) > 4096:
                self.buffer.clear()

        return None

    def wait_sdo(self, node, index):
        expected_id = 0x580 + node
        expected_obj = (
            index.to_bytes(2, "little")
            + b"\x00"
        )

        deadline = (
            time.monotonic() + SDO_TIMEOUT
        )

        while time.monotonic() < deadline:
            result = self.receive(
                max(
                    0,
                    deadline - time.monotonic()
                )
            )

            if result is None:
                break

            can_id, data = result

            if can_id != expected_id:
                continue

            if len(data) < 4:
                continue

            if data[1:4] != expected_obj:
                continue

            if data[0] == 0x80:
                if len(data) < 8:
                    raise RuntimeError(
                        "SDO Abort 길이 오류"
                    )

                code = int.from_bytes(
                    data[4:8], "little"
                )

                raise RuntimeError(
                    f"Node {node}: "
                    f"SDO Abort 0x{code:08X}"
                )

            return data

        raise TimeoutError(
            f"Node {node}: "
            f"0x{index:04X} 응답 없음"
        )

    def read_sdo(
        self, node, index, signed=False
    ):
        request = (
            b"\x40"
            + index.to_bytes(2, "little")
            + b"\x00" * 5
        )

        self.ser.reset_input_buffer()
        self.buffer.clear()

        self.send(node, request)

        data = self.wait_sdo(
            node, index
        )

        sizes = {
            0x4F: 1,
            0x4B: 2,
            0x47: 3,
            0x43: 4
        }

        if data[0] not in sizes:
            raise RuntimeError(
                "지원하지 않는 SDO 응답"
            )

        size = sizes[data[0]]

        if len(data) < 4 + size:
            raise RuntimeError(
                "SDO 데이터 길이 오류"
            )

        return int.from_bytes(
            data[4:4 + size],
            "little",
            signed=signed
        )

    def write_zero(self, node):
        self.ser.reset_input_buffer()
        self.buffer.clear()

        self.send(node, ZERO)

        response = self.wait_sdo(
            node, 0x60FF
        )

        if response[0] != 0x60:
            raise RuntimeError(
                f"Node {node}: "
                "속도 0 기록 실패"
            )

        print(
            f"[OK] Node {node}: "
            "속도 0 기록 성공"
        )

    def stop_all(self):
        print(
            "\n[STOP] 양쪽 모터 정지 명령"
        )

        errors = []

        # 양쪽에 먼저 속도 0 송신
        for node in NODES:
            try:
                self.send(node, ZERO)
            except Exception as e:
                errors.append(
                    f"Node {node}: {e}"
                )

        pending = set(NODES)
        deadline = (
            time.monotonic() + SDO_TIMEOUT
        )

        while pending and time.monotonic() < deadline:
            result = self.receive(
                max(
                    0,
                    deadline - time.monotonic()
                )
            )

            if result is None:
                break

            can_id, data = result
            node = can_id - 0x580

            if node not in pending:
                continue

            if len(data) < 4:
                continue

            if data[1:4] != b"\xFF\x60\x00":
                continue

            if data[0] == 0x60:
                print(
                    f"[OK] Node {node}: "
                    "정지 명령 기록 성공"
                )
                pending.remove(node)

            elif data[0] == 0x80:
                errors.append(
                    f"Node {node}: SDO Abort"
                )
                pending.remove(node)

        for node in pending:
            errors.append(
                f"Node {node}: 정지 응답 없음"
            )

        # 목표 속도 및 실제 속도 확인
        for node in NODES:
            try:
                target = self.read_sdo(
                    node, 0x60FF, signed=True
                )

                actual = self.read_sdo(
                    node, 0x606C, signed=True
                )

                print(
                    f"[Node {node}] "
                    f"Target={target}, "
                    f"Actual={actual}"
                )

                if target != 0 or actual != 0:
                    errors.append(
                        f"Node {node}: "
                        "정지 미확인"
                    )

            except Exception as e:
                errors.append(
                    f"Node {node}: {e}"
                )

        if errors:
            for error in errors:
                print("[CRITICAL]", error)

            raise RuntimeError(
                "양쪽 모터 정지 검증 실패"
            )

        print(
            "[SUCCESS] 양쪽 모터 정지 확인"
        )

    def safe_start(self):
        print(
            "\n[STEP 1] 안전 초기화"
        )

        # 양쪽 목표 속도 0 설정
        # 한 노드가 실패해도 다른 노드에
        # 정지 명령을 시도
        errors = []

        for node in NODES:
            try:
                self.write_zero(node)
            except Exception as e:
                errors.append(str(e))

        if errors:
            raise RuntimeError(
                "초기 속도 0 설정 실패: "
                + "; ".join(errors)
            )

        print(
            "\n[STEP 2] 모터 상태 확인"
        )

        for node in NODES:
            status = self.read_sdo(
                node, 0x6041
            )

            target = self.read_sdo(
                node, 0x60FF, signed=True
            )

            actual = self.read_sdo(
                node, 0x606C, signed=True
            )

            mode = self.read_sdo(
                node, 0x6061
            )

            print(
                f"\n[Node {node}]"
                f"\n Statusword: 0x{status:04X}"
                f"\n Target Velocity: {target}"
                f"\n Actual Velocity: {actual}"
                f"\n Operation Mode: {mode}"
            )

            if (status & 0x006F) != 0x0027:
                raise RuntimeError(
                    f"Node {node}: "
                    "Operation Enabled 아님"
                )

            if target != 0 or actual != 0:
                raise RuntimeError(
                    f"Node {node}: "
                    "정지 상태가 아님"
                )

            if mode != 3:
                raise RuntimeError(
                    f"Node {node}: "
                    "Profile Velocity Mode 아님"
                )

        print(
            "\n[SUCCESS] 안전 초기화 완료"
        )


class AMRController:
    def __init__(self, can):
        self.can = can
        self.moving = False
        self.fault = False

    def forward(self):
        if self.fault:
            print(
                "\n[BLOCKED] 오류 잠금 상태"
            )
            return

        if self.moving:
            return

        if not SIMULATE_FORWARD:
            raise RuntimeError(
                "실제 전진 명령 비활성화"
            )

        print(
            "\n[SIM] W키 전진 시작"
        )
        print(
            f"[SIM] Node 1 = -{SPEED}"
        )
        print(
            f"[SIM] Node 2 = +{SPEED}"
        )

        self.moving = True

    def inject_fault(self):
        print(
            "\n[ERROR] 통신 오류 모의 발생"
        )

        self.fault = True
        self.moving = False

        try:
            self.can.stop_all()
        except Exception as e:
            print(
                "[CRITICAL] 정지 확인 실패:",
                e
            )

        print(
            "[STATE] FAULT_LOCKED"
        )

    def show_status(self):
        print(
            "\n========== AMR STATUS =========="
        )
        print(
            "Moving (simulated):",
            self.moving
        )
        print(
            "Fault locked:",
            self.fault
        )
        print(
            "================================"
        )


def get_key():
    if select.select(
        [sys.stdin], [], [], 0.02
    )[0]:
        return sys.stdin.read(1).lower()

    return None


def main():
    print("=" * 50)
    print(
        "AMR 안전 초기화 + W/Q 통합 제어"
    )
    print("=" * 50)

    print("W: 전진 모의 실행")
    print("Q: 실제 CAN 정지 및 종료")
    print("E: 통신 오류 모의 발생")
    print("P: 상태 확인")

    with serial.Serial(
        PORT, BAUD, timeout=0.02
    ) as ser:

        can = UCan(ser)
        controller = AMRController(can)

        print("[OK] uCAN 연결 성공")

        try:
            # 시작 시 안전 초기화
            can.safe_start()

            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)

            try:
                tty.setcbreak(fd)

                print(
                    "\n[READY] W / E / P / Q"
                )

                while True:
                    key = get_key()

                    if key == "w":
                        controller.forward()

                    elif key == "e":
                        controller.inject_fault()

                    elif key == "p":
                        controller.show_status()

                    elif key == "q":
                        print("\n[KEY] Q")
                        break

            finally:
                termios.tcsetattr(
                    fd, termios.TCSADRAIN, old
                )

        finally:
            try:
                can.stop_all()
            except Exception as e:
                print(
                    "[CRITICAL] 종료 시 정지 실패:",
                    e
                )
                print(
                    "[ACTION] 하드웨어 비상정지 확인"
                )

    print(
        "\n[EXIT] 프로그램 종료"
    )


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print(
            "\n[INFO] 사용자 중단"
        )

    except Exception as e:
        print("[ERROR]", e)
