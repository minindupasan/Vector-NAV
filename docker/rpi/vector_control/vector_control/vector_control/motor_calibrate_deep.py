#!/usr/bin/env python3
"""
Deep motor calibration for VECTOR NAV.

Tests each motor at multiple PWM levels in both directions.
Reports ticks, RPM, and flags asymmetries or defects.

Run inside the Docker container:
  docker exec -it vector-control python3 \
    /ros2_ws/src/vector_control/vector_control/motor_calibrate_deep.py
"""

import math
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

# --- Pin definitions ---
MOTORS = {
    'LF': {'pwm': 12, 'in1': 6,  'in2': 5},
    'LR': {'pwm': 18, 'in1': 26, 'in2': 19},
    'RF': {'pwm': 16, 'in1': 20, 'in2': 21},
    'RR': {'pwm': 22, 'in1': 17, 'in2': 27},
}

ENCODERS = {
    'LF': {'a': 4,  'b': 25},
    'LR': {'a': 24, 'b': 14},
    'RF': {'a': 15, 'b': 7},
    'RR': {'a': 10, 'b': 9},
}

STBY_PINS = [13, 23]
PWM_FREQ = 1000
TICKS_PER_REV = 225
WHEEL_RADIUS = 0.0325

# Test at these PWM duty cycles
PWM_LEVELS = [0.2, 0.3, 0.4, 0.5, 0.7, 1.0]
TEST_DURATION = 2.0  # seconds per test
SETTLE_TIME = 0.5    # seconds between tests


def run_motor_test(pwm_dev, in1, in2, phase_a, phase_b, pwm_val, direction, duration):
    """Run a single motor test. Returns (ticks, rpm, m_per_s)."""
    ticks = 0
    lock = threading.Lock()

    def on_tick():
        nonlocal ticks
        with lock:
            ticks += 1

    phase_a.when_pressed = on_tick

    if direction == 'FWD':
        in1.on(); in2.off()
    else:
        in1.off(); in2.on()

    ticks = 0
    pwm_dev.value = pwm_val
    time.sleep(duration)
    pwm_dev.value = 0
    in1.off(); in2.off()

    phase_a.when_pressed = None

    with lock:
        t = ticks

    revs = t / TICKS_PER_REV
    rpm = (revs / duration) * 60.0
    m_per_s = (revs * 2.0 * math.pi * WHEEL_RADIUS) / duration

    return t, rpm, m_per_s


def main():
    print('=' * 70)
    print(' VECTOR NAV — Deep Motor Calibration')
    print('=' * 70)
    print()
    print(f' Test duration: {TEST_DURATION}s per direction per PWM level')
    print(f' PWM levels: {PWM_LEVELS}')
    print(f' Ticks/rev: {TICKS_PER_REV}, Wheel radius: {WHEEL_RADIUS}m')
    print()

    # Enable standby
    stbys = []
    for pin in STBY_PINS:
        s = DigitalOutputDevice(pin)
        s.on()
        stbys.append(s)

    all_results = {}

    for motor_name in ['LF', 'LR', 'RF', 'RR']:
        pins = MOTORS[motor_name]
        enc_pins = ENCODERS[motor_name]

        pwm_dev = PWMOutputDevice(pins['pwm'], frequency=PWM_FREQ)
        in1 = DigitalOutputDevice(pins['in1'])
        in2 = DigitalOutputDevice(pins['in2'])
        phase_b = Button(enc_pins['b'], pull_up=True, bounce_time=None)
        phase_a = Button(enc_pins['a'], pull_up=True, bounce_time=None)

        print(f'{"=" * 70}')
        print(f' Motor: {motor_name}  (PWM pin {pins["pwm"]}, '
              f'ENC A={enc_pins["a"]} B={enc_pins["b"]})')
        print(f'{"=" * 70}')
        print(f' {"PWM":>5} | {"Dir":>4} | {"Ticks":>7} | {"RPM":>8} | {"m/s":>7} | Notes')
        print(f' {"-" * 5}-+-{"-" * 4}-+-{"-" * 7}-+-{"-" * 8}-+-{"-" * 7}-+-------')

        motor_results = []

        for pwm_val in PWM_LEVELS:
            for direction in ['FWD', 'REV']:
                t, rpm, mps = run_motor_test(
                    pwm_dev, in1, in2, phase_a, phase_b,
                    pwm_val, direction, TEST_DURATION
                )

                notes = []
                if t == 0:
                    notes.append('NO TICKS!')

                note_str = ', '.join(notes) if notes else ''
                print(f' {pwm_val:>5.1f} | {direction:>4} | {t:>7} | {rpm:>8.1f} | {mps:>7.3f} | {note_str}')

                motor_results.append({
                    'pwm': pwm_val,
                    'dir': direction,
                    'ticks': t,
                    'rpm': rpm,
                    'mps': mps,
                })

                time.sleep(SETTLE_TIME)

        all_results[motor_name] = motor_results
        print()

        # Cleanup — disarm callbacks before closing to avoid race with
        # gpiozero's internal hold thread (causes AttributeError crash).
        phase_a.when_pressed = None
        phase_a.when_released = None
        phase_b.when_pressed = None
        phase_b.when_released = None
        time.sleep(0.1)  # let pending lgpio callbacks drain
        pwm_dev.close()
        in1.close()
        in2.close()
        phase_a.close()
        phase_b.close()
        time.sleep(0.5)

    # --- Summary / Analysis ---
    print()
    print('=' * 70)
    print(' ANALYSIS')
    print('=' * 70)
    print()

    # Compare FWD vs REV symmetry for each motor
    print(' Direction symmetry (FWD RPM vs REV RPM at each PWM):')
    print(f' {"Motor":>5} | {"PWM":>5} | {"FWD RPM":>8} | {"REV RPM":>8} | {"Ratio":>6} | Status')
    print(f' {"-" * 5}-+-{"-" * 5}-+-{"-" * 8}-+-{"-" * 8}-+-{"-" * 6}-+-------')

    for motor_name in ['LF', 'LR', 'RF', 'RR']:
        results = all_results[motor_name]
        for pwm_val in PWM_LEVELS:
            fwd = [r for r in results if r['pwm'] == pwm_val and r['dir'] == 'FWD'][0]
            rev = [r for r in results if r['pwm'] == pwm_val and r['dir'] == 'REV'][0]

            if fwd['rpm'] > 0 and rev['rpm'] > 0:
                ratio = min(fwd['rpm'], rev['rpm']) / max(fwd['rpm'], rev['rpm'])
                status = 'OK' if ratio > 0.8 else 'ASYMMETRIC' if ratio > 0.5 else 'DEFECT?'
            elif fwd['rpm'] == 0 and rev['rpm'] == 0:
                ratio = 0.0
                status = 'DEAD!'
            else:
                ratio = 0.0
                status = 'ONE-DIR ONLY!'

            print(f' {motor_name:>5} | {pwm_val:>5.1f} | {fwd["rpm"]:>8.1f} | {rev["rpm"]:>8.1f} | {ratio:>6.2f} | {status}')

    # Compare motors against each other at same PWM
    print()
    print(' Motor-to-motor comparison (RPM at each PWM, FWD):')
    print(f' {"PWM":>5} |', end='')
    for m in ['LF', 'LR', 'RF', 'RR']:
        print(f' {m:>8} |', end='')
    print(' Spread')
    print(f' {"-" * 5}-+', end='')
    for _ in ['LF', 'LR', 'RF', 'RR']:
        print(f'-{"-" * 8}-+', end='')
    print('--------')

    for pwm_val in PWM_LEVELS:
        rpms = {}
        print(f' {pwm_val:>5.1f} |', end='')
        for motor_name in ['LF', 'LR', 'RF', 'RR']:
            results = all_results[motor_name]
            fwd = [r for r in results if r['pwm'] == pwm_val and r['dir'] == 'FWD'][0]
            rpms[motor_name] = fwd['rpm']
            print(f' {fwd["rpm"]:>8.1f} |', end='')

        vals = [v for v in rpms.values() if v > 0]
        if len(vals) >= 2:
            spread = (max(vals) - min(vals)) / max(vals) * 100
            print(f' {spread:>5.1f}%')
        else:
            print(f'   N/A')

    # Minimum PWM to start moving
    print()
    print(' Minimum PWM to start (deadzone):')
    for motor_name in ['LF', 'LR', 'RF', 'RR']:
        results = all_results[motor_name]
        min_fwd = None
        min_rev = None
        for r in results:
            if r['dir'] == 'FWD' and r['rpm'] > 0 and (min_fwd is None or r['pwm'] < min_fwd):
                min_fwd = r['pwm']
            if r['dir'] == 'REV' and r['rpm'] > 0 and (min_rev is None or r['pwm'] < min_rev):
                min_rev = r['pwm']
        fwd_str = f'{min_fwd:.1f}' if min_fwd else 'NEVER'
        rev_str = f'{min_rev:.1f}' if min_rev else 'NEVER'
        print(f'   {motor_name}: FWD={fwd_str}, REV={rev_str}')

    # Cleanup
    for s in stbys:
        s.off()
        s.close()

    print()
    print('Done!')


if __name__ == '__main__':
    main()
