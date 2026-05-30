"""
calibrate_gripper.py — measure full-sweep time for the DC gripper motor.

Talks directly to the Arduino over serial, bypassing the ROS bridge.
Stop `bridge_node` first so the port isn't in use.

Commands (type letter + Enter):
  f   motor "forward"           — direction A
  b   motor "backward"          — direction B
  p   pause                     — stop motor + pause timer
  s   start recording           — arm timer (zeros accumulator)
  r   reset                     — stop motor, zero accumulator, disarm
  q   quit

Wire-flip safe: the timer counts whenever the motor is running and recording
is armed, regardless of whether you used `f` or `b` to start it. So if your
"forward" wire is actually closing the gripper, the measurement is still
correct — you just label the direction yourself.

Typical session:
  b   → motor runs in direction B; jog gripper to one mechanical limit
  p   → stop at the limit
  s   → arm timer (resets to 0)
  f   → motor runs in direction A; let it travel to the opposite limit
  p   → stop at the other limit; script prints elapsed_ms and ms_per_percent

Then set the ROS param on bridge_node:
  ros2 param set /arduino_bridge gripper_ms_per_percent <value>
"""

import argparse
import queue
import sys
import threading
import time

import serial


# Arduino brakes any pulse longer than MAX_PULSE_MS (5000 ms). We send 4000 ms
# pulses and refresh every REFRESH_S so the motor stays on smoothly without
# letting the brake fire between refreshes.
CHUNK_MS  = 4000
REFRESH_S = 1.0

# Arm stays still during calibration — packet still needs the 6 joint angles.
HOME_ANGLES = [90] * 6


def build_packet(pulse_ms: int) -> bytes:
    payload  = HOME_ANGLES + [int(pulse_ms)]
    checksum = sum(payload) & 0xFFFF
    return (' '.join(str(v) for v in payload) + f' {checksum}\n').encode('ascii')


class GripperCalibrator:
    def __init__(self, ser: serial.Serial):
        self.ser         = ser
        self.dir         = 0          # -1, 0, +1
        self.recording   = False
        self.accumulated = 0.0        # seconds
        self.run_started = None       # time.monotonic() when current motion started

    def _send(self, pulse_ms: int):
        self.ser.write(build_packet(pulse_ms))

    def _start_motor(self, direction: int, label: str):
        if self.dir == direction:
            return
        self.dir = direction
        self._send(direction * CHUNK_MS)
        if self.recording and self.run_started is None:
            self.run_started = time.monotonic()
            print(f'{label} — timer running')
        else:
            print(label)

    def _stop_motor(self, verbose: bool = True):
        if self.dir == 0:
            if verbose:
                print('paused (motor already idle)')
            return
        self.dir = 0
        self._send(0)
        if self.recording and self.run_started is not None:
            elapsed = time.monotonic() - self.run_started
            self.accumulated += elapsed
            self.run_started = None
            ms_total      = self.accumulated * 1000.0
            ms_per_pct    = ms_total / 100.0
            print(
                f'paused. elapsed={elapsed:.3f}s  '
                f'total={self.accumulated:.3f}s ({ms_total:.0f} ms)  '
                f'=> gripper_ms_per_percent ≈ {ms_per_pct:.2f}'
            )
        elif verbose:
            print('paused')

    def refresh(self):
        """Re-issue the current pulse so the motor doesn't auto-brake."""
        if self.dir != 0:
            self._send(self.dir * CHUNK_MS)

    def handle(self, line: str) -> bool:
        c = line.strip().lower()
        if c == 'f':
            self._start_motor(+1, 'forward')
        elif c == 'b':
            self._start_motor(-1, 'backward')
        elif c == 'p':
            self._stop_motor()
        elif c == 's':
            self._stop_motor(verbose=False)        # ensure timer base is clean
            self.recording   = True
            self.accumulated = 0.0
            self.run_started = None
            print('recording armed (timer = 0)')
        elif c == 'r':
            self._stop_motor(verbose=False)
            self.recording   = False
            self.accumulated = 0.0
            self.run_started = None
            print('reset — disarmed, timer = 0')
        elif c in ('q', 'quit', 'exit'):
            self._stop_motor(verbose=False)
            return False
        elif c == '':
            pass
        else:
            print(f'unknown: {c!r}  (commands: f b p s r q)')
        return True


def _stdin_reader(q: queue.Queue):
    for line in sys.stdin:
        q.put(line)
    q.put(None)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f'Could not open {args.port}: {exc}', file=sys.stderr)
        print('Is bridge_node still running? Stop it first.', file=sys.stderr)
        sys.exit(1)

    time.sleep(2)   # let Arduino reset
    print(f'Connected {args.port} @ {args.baud}')
    print('commands: f=fwd  b=back  p=pause  s=start  r=reset  q=quit')

    cal = GripperCalibrator(ser)
    q: queue.Queue = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(q,), daemon=True).start()

    last_refresh = 0.0
    try:
        while True:
            try:
                item = q.get(timeout=0.1)
            except queue.Empty:
                item = ''
            if item is None:
                break
            if item and not cal.handle(item):
                break

            now = time.monotonic()
            if now - last_refresh > REFRESH_S:
                cal.refresh()
                last_refresh = now

            while ser.in_waiting:
                msg = ser.readline().decode('ascii', errors='replace').strip()
                if msg:
                    print(f'[arduino] {msg}')
    except KeyboardInterrupt:
        pass
    finally:
        cal._stop_motor(verbose=False)
        ser.close()
        print('bye')


if __name__ == '__main__':
    main()
