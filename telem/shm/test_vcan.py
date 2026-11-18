#!/usr/bin/env python3
"""Linux integration test; uses an existing vcan0, never configures hardware."""
import math
import os
import pty
import tty
from pathlib import Path
import socket
import struct
import subprocess
import sys
import time

from read_shm import SensorShmReader


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: python3 test_vcan.py PATH_TO_WRITE_SHM')
    master, slave = pty.openpty()
    tty.setraw(slave)
    env = dict(os.environ, SHM_STEERING_SCALE='0.1', SHM_STEERING_OFFSET='-100',
               SHM_BRAKE_SCALE='0.25', SHM_BRAKE_OFFSET='-10',
               SHM_CURRENT_OFFSET_A='0',
               SHM_GPS_DEVICE=os.ttyname(slave))
    # Test owns /sensor_shm; do not run alongside a production writer.
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as can:
        can.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
        can.bind(('vcan0',))
        proc = subprocess.Popen([sys.argv[1], 'vcan0'], env=env)
        reader = None
        try:
            time.sleep(0.15)
            assert proc.poll() is None, 'writer exited during startup'
            reader = SensorShmReader()
            assert reader.available
            assert Path('/dev/shm/sensor_shm').stat().st_size == 100

            def wait_for(predicate):
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    snap = reader.read_snapshot_dict()
                    if snap and predicate(snap):
                        return snap
                    time.sleep(0.005)
                raise AssertionError('snapshot condition timed out')

            def send(can_id, body):
                data = bytes([0, len(body)]) + body
                size = next(n for n in [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
                            if n >= len(data))
                can.send(struct.pack('=IBBBB64s', can_id, size, 1, 0, 0, data))

            # Fragment an RMC sentence to exercise nonblocking UART accumulation.
            os.write(master, b'$GPRMC,123519,A,4807.038,N,01131.000,E,')
            time.sleep(0.03)
            os.write(master, b'10.0,84.4,230394,,,A*00\r\n')
            gps = wait_for(lambda s: s['gps']['ts'] != 0)['gps']
            assert math.isclose(gps['lat'], 48.1173, abs_tol=1e-5)
            assert math.isclose(gps['long'], 11.5166667, abs_tol=1e-5)
            assert math.isclose(gps['speed'], 5.14444, rel_tol=1e-5)
            initial = wait_for(lambda s: s['global_ts'] > 0)
            assert math.isnan(initial['motor']['rpm']) and initial['motor']['ts'] == 0
            assert math.isnan(initial['steering']['turn_angle'])
            send(0x210, struct.pack('<IHHHH', 0xfffffff0, 600, 300, 1500, 0))
            front = wait_for(lambda s: s['rpm_front']['rpm_left'] == 600)
            assert front['rpm_front']['ts'] == 0xfffffff0
            assert front['steering']['ts'] == 0xfffffff0
            assert front['rpm_front']['rpm_right'] == 300
            assert front['steering']['turn_angle'] == 50
            assert front['rpm_back']['ts'] == 0
            send(0x220, struct.pack('<IHHHH', 123456, 150, 450, 200, 0))
            back = wait_for(lambda s: s['rpm_back']['rpm_right'] == 450)
            assert back['rpm_back']['ts'] == 123456
            assert back['rpm_back']['rpm_left'] == 150
            assert back['steering']['brake_pressure'] == 40
            assert back['rpm_front'] == front['rpm_front']
            assert back['steering']['ts'] == front['steering']['ts']
            assert math.isclose(back['filtered']['speed'], 300 * math.pi * 0.58166 / 60, rel_tol=1e-5)
            send(0x230, struct.pack('<Iffff', 654321, 48, 12.5, 600, 1000))
            power = wait_for(lambda s: s['power']['current'] == 12.5)
            assert power['power']['ts'] == 654321
            assert power['power']['voltage'] == 48
            assert power['rpm_front'] == front['rpm_front']
            assert power['rpm_back'] == back['rpm_back']
            send(0x210, struct.pack('<IHHHH', 20, 0, 0, 2000, 0))
            updated = wait_for(lambda s: s['steering']['turn_angle'] == 100)
            assert updated['rpm_front']['ts'] == 20
            assert updated['steering']['ts'] == 20
            assert (updated['rpm_front']['ts'] - front['rpm_front']['ts']) % 2**32 == 36
            assert updated['rpm_front']['rpm_left'] == 0
            assert updated['rpm_front']['rpm_right'] == 0
            assert updated['steering']['brake_pressure'] == 40
            assert updated['power'] == power['power']
            assert updated['rpm_back'] == back['rpm_back']
            assert updated['seq'] % 2 == 0
            print('PASS: Python layout, conversions, independent node caches and timestamps')
        finally:
            if reader:
                reader.close()
            os.close(master)
            os.close(slave)
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
                raise AssertionError('writer did not stop cleanly')


if __name__ == '__main__':
    main()
