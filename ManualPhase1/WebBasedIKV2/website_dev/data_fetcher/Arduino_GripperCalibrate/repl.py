"""
repl.py — minimal terminal client for Arduino_GripperCalibrate.ino.

Each keystroke is forwarded as a single byte to the Arduino, matching the
sketch's per-byte command parser. Arduino output is mirrored to the terminal
and appended to a session log.

Run on the Pi (over SSH is fine):

    python3 repl.py --port /dev/ttyUSB0

Exit with Ctrl-C.
"""

import argparse
import datetime as dt
import select
import sys
import termios
import time
import tty

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--log', default='gripper_calibration.log')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(2)
    print(f'connected {args.port} @ {args.baud}  (Ctrl-C to quit)')
    print(f'logging to {args.log}')

    log_fp = open(args.log, 'a', buffering=1)
    log_fp.write(f'\n=== session {dt.datetime.now().isoformat()} ===\n')

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            r, _, _ = select.select([sys.stdin, ser], [], [], 0.1)
            if sys.stdin in r:
                ch = sys.stdin.read(1)
                if not ch:
                    break
                ser.write(ch.encode())
            if ser in r and ser.in_waiting:
                data = ser.read(ser.in_waiting).decode('ascii', errors='replace')
                sys.stdout.write(data)
                sys.stdout.flush()
                log_fp.write(data)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        ser.close()
        log_fp.close()
        print('\nbye')


if __name__ == '__main__':
    main()
