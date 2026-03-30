#!/usr/bin/env python3
"""
Motor direction calibration tool for VECTOR NAV.
Tests each motor individually and reports encoder direction.

Run inside the Docker container:
  docker exec -it vector-control python3 /ros2_ws/src/vector_control/vector_control/calibrate_motors.py
"""

import time
import threading
import glob as _glob

import gpiozero
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button
from gpiozero.pins.lgpio import LGPIOFactory

# --- Auto-detect gpiochip ---
for _chip in sorted(
    set(int(c.replace('/dev/gpiochip', ''))
        for c in _glob.glob('/dev/gpiochip*')),
    reverse=True,
):
    try:
        gpiozero.Device.pin_factory = LGPIOFactory(chip=_chip)
        break
    except Exception:
        continue

# --- Pin definitions (matching test_motors.py on Pi) ---
MOTORS = {
    'LF': {'pwm': 12, 'in1': 6,  'in2': 5},
    'LR': {'pwm': 18, 'in1': 26, 'in2': 19},
    'RF': {'pwm': 16, 'in1': 20, 'in2': 21},
    'RR': {'pwm': 22, 'in1': 17, 'in2': 27},
}

ENCODERS = {
    'LF': {'a': 25, 'b': 4},
    'LR': {'a': 24, 'b': 14},
    'RF': {'a': 15, 'b': 7},
    'RR': {'a': 10, 'b': 9},
}

STBY_PINS = [13, 23]

PWM_FREQ = 1000
TEST_SPEED = 0.4   # 40% duty
TEST_DURATION = 1.5  # seconds


def main():
    print('=' * 50)
    print(' VECTOR NAV — Motor Direction Calibration')
    print('=' * 50)
    print()
    print('This will spin each motor one at a time.')
    print('Observe which direction the wheel turns and')
    print('report whether it matches the expected direction.')
    print()
    print('Convention (looking from BEHIND the robot):')
    print('  FORWARD = wheel pushes robot forward')
    print('  LF/LR = left side,  RF/RR = right side')
    print()

    # Enable standby pins
    stbys = []
    for pin in STBY_PINS:
        s = DigitalOutputDevice(pin)
        s.on()
        stbys.append(s)

    results = {}

    for name, pins in MOTORS.items():
        print(f'--- Testing {name} ---')

        # Setup motor
        pwm = PWMOutputDevice(pins['pwm'], frequency=PWM_FREQ)
        in1 = DigitalOutputDevice(pins['in1'])
        in2 = DigitalOutputDevice(pins['in2'])

        # Setup encoder
        enc_pins = ENCODERS[name]
        ticks = 0
        lock = threading.Lock()
        phase_b = Button(enc_pins['b'], pull_up=True, bounce_time=None)
        phase_a = Button(enc_pins['a'], pull_up=True, bounce_time=None)

        def on_tick():
            nonlocal ticks
            direction = 1 if phase_b.is_pressed else -1
            with lock:
                ticks += direction

        phase_a.when_pressed = on_tick

        # --- Test: IN1=on, IN2=off (what set_speed(+) does) ---
        print(f'  Spinning {name}: IN1=ON, IN2=OFF (positive speed)...')
        ticks = 0
        in1.on()
        in2.off()
        pwm.value = TEST_SPEED
        time.sleep(TEST_DURATION)
        pwm.value = 0
        in1.off()
        in2.off()

        with lock:
            fwd_ticks = ticks

        print(f'  Encoder ticks: {fwd_ticks}')
        if fwd_ticks > 0:
            print(f'  → Encoder reads POSITIVE (forward)')
        elif fwd_ticks < 0:
            print(f'  → Encoder reads NEGATIVE (backward)')
        else:
            print(f'  → No encoder ticks detected!')

        time.sleep(0.5)

        # --- Test: IN1=off, IN2=on (what set_speed(-) does) ---
        print(f'  Spinning {name}: IN1=OFF, IN2=ON (negative speed)...')
        ticks = 0
        in1.off()
        in2.on()
        pwm.value = TEST_SPEED
        time.sleep(TEST_DURATION)
        pwm.value = 0
        in1.off()
        in2.off()

        with lock:
            rev_ticks = ticks

        print(f'  Encoder ticks: {rev_ticks}')
        if rev_ticks < 0:
            print(f'  → Encoder reads NEGATIVE (backward)')
        elif rev_ticks > 0:
            print(f'  → Encoder reads POSITIVE (forward)')
        else:
            print(f'  → No encoder ticks detected!')

        print()

        # Ask user
        while True:
            answer = input(
                f'  Did {name} move FORWARD with positive speed? (y/n/skip): '
            ).strip().lower()
            if answer in ('y', 'n', 's', 'skip'):
                break

        results[name] = {
            'fwd_ticks': fwd_ticks,
            'rev_ticks': rev_ticks,
            'correct': answer == 'y',
            'skipped': answer in ('s', 'skip'),
        }

        # Cleanup motor/encoder
        pwm.close()
        in1.close()
        in2.close()
        phase_a.close()
        phase_b.close()

        time.sleep(0.5)

    # --- Summary ---
    print()
    print('=' * 50)
    print(' RESULTS')
    print('=' * 50)
    print(f'  {"Motor":<6} {"Fwd Ticks":>10} {"Rev Ticks":>10} {"Direction":>12}')
    print('  ' + '-' * 42)
    for name, r in results.items():
        if r['skipped']:
            status = 'SKIPPED'
        elif r['correct']:
            status = 'OK'
        else:
            status = 'REVERSED'
        print(f'  {name:<6} {r["fwd_ticks"]:>10} {r["rev_ticks"]:>10} {status:>12}')

    # --- Suggest fixes ---
    reversed_motors = [n for n, r in results.items() if not r['correct'] and not r['skipped']]
    if reversed_motors:
        print()
        print('  Motors with wrong direction:', ', '.join(reversed_motors))
        print('  Fix: swap IN1/IN2 pins for these motors in motor_driver_node.py')
    else:
        print()
        print('  All motors OK!')

    # Cleanup
    for s in stbys:
        s.off()
        s.close()


if __name__ == '__main__':
    main()
