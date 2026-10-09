#!/usr/bin/env python3
"""One-time setup of the WT901C-TTL IMU. Run on the Pi (imu_node must not run):

    python3 ~/NALA/scripts/setup_imu.py              # IMU still at the factory baud rate 9600
    python3 ~/NALA/scripts/setup_imu.py --from 115200  # IMU already at another baud rate

Writes into the IMU (it keeps the settings after power-off):
  output = acceleration (0x51) + angular velocity (0x52) only, rate and baud rate from config/imu.yaml.
Then it checks the frames at the new baud rate.
Register values: WitMotion WT901 protocol (write = FF AA <register> <low> <high>, unlock = FF AA 69 88 B5).
If something goes wrong, the WitMotion PC software can reset the IMU.
"""

import argparse
import os
import sys
import time

import serial
import yaml

RSW, RRATE, BAUD, SAVE = 0x02, 0x03, 0x04, 0x00             # registers
OUTPUT_ACC_GYRO = 0x0006                                     # RSW bits: 1 = acceleration, 2 = angular velocity
RATE_CODES = {10: 0x06, 20: 0x07, 50: 0x08, 100: 0x09, 200: 0x0B}
BAUD_CODES = {9600: 0x02, 19200: 0x03, 38400: 0x04, 57600: 0x05, 115200: 0x06, 230400: 0x07}
UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])


def write(port, register, value):
    port.write(bytes([0xFF, 0xAA, register, value & 0xFF, value >> 8]))
    port.flush()
    time.sleep(0.1)


def count_frames(port, seconds):
    """Count frames with a good checksum per type for some seconds."""
    port.reset_input_buffer()
    data = b''
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        data += port.read(port.in_waiting or 1)
    counts = {}
    i = 0
    while len(data) - i >= 11:
        frame = data[i:i + 11]
        if frame[0] == 0x55 and sum(frame[:10]) & 0xFF == frame[10]:
            counts[frame[1]] = counts.get(frame[1], 0) + 1
            i += 11
        else:
            i += 1
    return {hex(kind): n / seconds for kind, n in sorted(counts.items())}


def main():
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(repo, 'config', 'imu.yaml')) as f:
        config = yaml.safe_load(f)
    params = config['imu_node']['ros__parameters']
    rate = config['setup_imu']['ros__parameters']['output_rate_hz']
    target = params['baud_rate']
    parser = argparse.ArgumentParser(description='One-time WT901 IMU setup')
    parser.add_argument('--port', default=params['serial_port'])
    parser.add_argument('--from', dest='old', type=int, default=9600, help='current IMU baud rate')
    args = parser.parse_args()

    with serial.Serial(args.port, args.old, timeout=0.1) as port:
        found = count_frames(port, 2.0)
        print(f'{args.port} at {args.old} baud, frames per second by type: {found}')
        if not found:
            sys.exit('No IMU frames. Check the wiring, the port and --from (the current baud rate).')
        port.write(UNLOCK)
        time.sleep(0.1)
        write(port, RSW, OUTPUT_ACC_GYRO)
        write(port, RRATE, RATE_CODES[rate])
        write(port, SAVE, 0)
        port.write(UNLOCK)
        time.sleep(0.1)
        write(port, BAUD, BAUD_CODES[target])
    time.sleep(0.5)
    with serial.Serial(args.port, target, timeout=0.1) as port:
        port.write(UNLOCK)                                    # save again at the new baud rate
        time.sleep(0.1)
        write(port, SAVE, 0)
        found = count_frames(port, 3.0)
    print(f'{args.port} at {target} baud, frames per second by type: {found}')
    if set(found) == {'0x51', '0x52'} and all(abs(n - rate) < 0.2 * rate for n in found.values()):
        print(f'OK: only acceleration and angular velocity, {rate} Hz, {target} baud.')
    else:
        sys.exit(f'Not as expected (want 0x51 and 0x52 at {rate} Hz). Run again with --from {target}, '
                 'or reset the IMU with the WitMotion PC software.')


if __name__ == '__main__':
    main()
