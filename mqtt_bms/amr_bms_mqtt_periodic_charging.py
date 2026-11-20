#!/usr/bin/env python3
"""AMR-01 TABOS BMS -> MQTT periodic BMS-only state (ACS integration test).

Uses site-observed current polarity: positive = charging, negative = not charging.
At currents near zero, holds the last established state, or withholds a new
message until polarity is established. Does not invent motor/E-stop state.
Still a partial VDA 5050 state; strict validators may reject it.
Never issues CAN, lift or navigation commands.
"""
import argparse
import json
import math
import ssl
import time
from datetime import datetime, timezone
from pathlib import Path

import serial
import paho.mqtt.client as mqtt

BMS_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
REQUEST = bytes.fromhex('AF FA 61 05 01 61 47 00 0F AF A0')
TOPIC = 'vda5050/v3/HANA_AI/HA-AMR-001/state'
CHARGING_CURRENT_DEADBAND_A = 0.05  # Based on observed polarity; adjustable after field checks


def infer_charging(current, previous=None):
    """Infer from empirically confirmed charger current polarity.

    A small deadband retains the last known state. None means unknown.
    """
    if current > CHARGING_CURRENT_DEADBAND_A:
        return True
    if current < -CHARGING_CURRENT_DEADBAND_A:
        return False
    return previous



def read_bms():
    """Reads and validates the same 17-byte TABOS frame as battery_monitor.py."""
    with serial.Serial(BMS_PORT, 19200, timeout=0.1, write_timeout=1) as ser:
        time.sleep(0.2)
        for _ in range(3):
            ser.reset_input_buffer()
            ser.write(REQUEST)
            ser.flush()
            buf = bytearray()
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                buf.extend(ser.read(ser.in_waiting or 1))
                while len(buf) >= 17:
                    idx = buf.find(b'\xaf\xfa')
                    if idx < 0:
                        buf[:] = b'\xaf' if buf[-1:] == b'\xaf' else b''
                        break
                    if idx:
                        del buf[:idx]
                    if len(buf) < 17:
                        break
                    frame = bytes(buf[:17])
                    if (frame[2:6] == bytes.fromhex('61 0B 03 61')
                            and frame[15:17] == b'\xaf\xa0'
                            and (sum(frame[2:14]) & 0xff) == frame[14]):
                        voltage = int.from_bytes(frame[6:8], 'big') / 100
                        current = int.from_bytes(frame[8:10], 'big', signed=True) / 100
                        soc = int.from_bytes(frame[10:12], 'big')
                        if 0 <= soc <= 100 and math.isfinite(voltage) and math.isfinite(current):
                            return dict(stateOfCharge=soc,
                                        batteryVoltage=voltage,
                                        batteryCurrent=current)
                    del buf[0]
            time.sleep(0.2)
    raise RuntimeError('No valid TABOS frame')


def make_payload(seq, measured):
    return {
        'headerId': seq % (1 << 32),
        'timestamp': datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z'),
        'version': '3.0.0',
        'manufacturer': 'HANA_AI',
        'serialNumber': 'HA-AMR-001',
        # No order is being executed in this BMS-only test.
        'orderId': '',
        'orderUpdateId': 0,
        'lastNodeId': '',
        'lastNodeSequenceId': 0,
        'nodeStates': [],
        'edgeStates': [],
        'actionStates': [],
        'instantActionStates': [],
        'powerSupply': measured,
    }


def new_client(cert_dir):
    # Paho MQTT 2.x: avoid deprecated Callback API v1 warning.
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id='hana-amr-001',
                         protocol=mqtt.MQTTv311, clean_session=True)
    base = Path(cert_dir)
    client.tls_set(ca_certs=str(base / 'ca.crt'),
                   certfile=str(base / 'hana-amr-001.crt'),
                   keyfile=str(base / 'hana-amr-001.key'),
                   cert_reqs=ssl.CERT_REQUIRED)
    client.tls_insecure_set(False)
    return client


def main():
    parser = argparse.ArgumentParser(description='Publish TABOS BMS to ACS MQTT every 2s')
    parser.add_argument('--cert-dir', default='/etc/hana-amr/certs/AMR-01-MQTT')
    parser.add_argument('--host', default='211.233.216.202')
    parser.add_argument('--port', type=int, default=8883)
    parser.add_argument('--interval', type=float, default=2.0)
    args = parser.parse_args()
    if args.interval < 1.0:
        parser.error('interval must be >= 1 second')

    client = None
    charging = None
    seq = int(time.time()) & 0xffffffff
    print(f'[BMS MQTT] period={args.interval:g}s topic={TOPIC}', flush=True)
    print('[NOTE] Charging inferred from measured current polarity; other VDA required fields (driving/operatingMode/safetyState) are not available in BMS-only mode.', flush=True)
    try:
        while True:
            started = time.monotonic()
            try:
                measured = read_bms()
                charging = infer_charging(measured['batteryCurrent'], charging)
                if charging is None:
                    print('[BMS] Charging polarity unknown near 0 A; state not published', flush=True)
                    raise ValueError('Charging polarity not yet established')
                measured['charging'] = charging
                payload = make_payload(seq, measured)
                if client is None:
                    client = new_client(args.cert_dir)
                    client.connect(args.host, args.port, keepalive=30)
                    client.loop_start()
                    print('[MQTT] connected', flush=True)
                info = client.publish(TOPIC, json.dumps(payload, separators=(',', ':')),
                                      qos=1, retain=False)
                info.wait_for_publish(timeout=5.0)
                if not info.is_published():
                    raise TimeoutError('PUBACK not received within 5 seconds')
                print(f'[MQTT PUBACK] SOC={measured["stateOfCharge"]}% '
                      f'V={measured["batteryVoltage"]:.2f} '
                      f'I={measured["batteryCurrent"]:+.2f} '
                      f'charging={measured["charging"]} seq={seq}', flush=True)
                seq = (seq + 1) & 0xffffffff
            except (OSError, serial.SerialException, RuntimeError, ValueError, TimeoutError) as exc:
                print(f'[ERROR] {type(exc).__name__}: {exc}', flush=True)
                if client is not None:
                    try:
                        client.disconnect()
                        client.loop_stop()
                    except Exception:
                        pass
                    client = None
            time.sleep(max(0.0, args.interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print('\n[BMS MQTT] stopped', flush=True)
    finally:
        if client is not None:
            client.disconnect()
            client.loop_stop()


if __name__ == '__main__':
    main()