#!/usr/bin/env python3
"""
Automated PID Tuning — LF Motor Only (Ziegler-Nichols relay method).

Procedure:
  1. Detect encoder direction automatically
  2. Find ultimate gain (Ku) and oscillation period (Tu) using relay feedback
  3. Compute PID gains via Ziegler-Nichols
  4. Validate with step response tests (0 hold, forward, reverse)
  5. Print final gains ready for use

Usage:
  python3 pid_autotune_lf.py
"""

import math
import threading
import time
import sys

import glob as _glob
import gpiozero
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button
from gpiozero.pins.lgpio import LGPIOFactory

# ---------- GPIO chip auto-detect ----------
_factory_set = False
_available = sorted(_glob.glob('/dev/gpiochip*'))
_chip_nums = sorted(
    set(int(c.replace('/dev/gpiochip', '')) for c in _available),
    reverse=True,
)
for _chip in _chip_nums:
    try:
        gpiozero.Device.pin_factory = LGPIOFactory(chip=_chip)
        _factory_set = True
        break
    except Exception:
        continue

if not _factory_set:
    raise RuntimeError(f'Cannot open any gpiochip. Available: {_available}')

# ---------- LF Motor pins ----------
MD1_PWMA = 12;  MD1_AIN1 = 6;  MD1_AIN2 = 5;  MD1_STBY = 13
ENC_LF_A = 4;   ENC_LF_B = 25

# ---------- Robot constants ----------
WHEEL_RADIUS = 0.0325
TICKS_PER_REV = 225
METERS_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
PWM_FREQ = 1000
CONTROL_HZ = 50
DT = 1.0 / CONTROL_HZ


class HBridgeMotor:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, frequency=PWM_FREQ)
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)

    def set_speed(self, speed_frac):
        if speed_frac > 0:
            self.in1.on();  self.in2.off()
        elif speed_frac < 0:
            self.in1.off(); self.in2.on()
        else:
            self.in1.off(); self.in2.off()
        self.pwm.value = min(abs(speed_frac), 1.0)

    def stop(self):
        self.in1.off(); self.in2.off(); self.pwm.value = 0

    def close(self):
        self.stop()
        self.pwm.close(); self.in1.close(); self.in2.close()


class SignedEncoder:
    def __init__(self, pin_a, pin_b):
        self._count = 0
        self._lock = threading.Lock()
        self._phase_b = Button(pin_b, pull_up=True, bounce_time=None)
        self._phase_a = Button(pin_a, pull_up=True, bounce_time=None)
        self._phase_a.when_pressed = self._on_rising_a

    def _on_rising_a(self):
        direction = -1 if self._phase_b.is_pressed else 1
        with self._lock:
            self._count += direction

    def get_and_reset(self):
        with self._lock:
            c = self._count
            self._count = 0
        return c

    def close(self):
        self._phase_a.close()
        self._phase_b.close()


def read_velocity(encoder, enc_flip):
    """Read encoder and return velocity in m/s."""
    ticks = encoder.get_and_reset() * enc_flip
    return ticks * METERS_PER_TICK / DT


def run_pid(motor, encoder, enc_flip, kp, ki, kd, target_vel, duration,
            label="", integral_limit=1.0):
    """Run PID loop for a fixed duration. Returns (times, targets, measurements)."""
    integral = 0.0
    prev_error = 0.0
    times = []
    targets = []
    measurements = []

    steps = int(duration / DT)
    for i in range(steps):
        t0 = time.monotonic()

        vel = read_velocity(encoder, enc_flip)
        error = target_vel - vel

        integral += error * DT
        integral = max(-integral_limit, min(integral_limit, integral))

        derivative = (error - prev_error) / DT
        prev_error = error

        output = kp * error + ki * integral + kd * derivative
        pwm = max(-1.0, min(1.0, output))
        motor.set_speed(pwm)

        times.append(i * DT)
        targets.append(target_vel)
        measurements.append(vel)

        elapsed = time.monotonic() - t0
        if DT - elapsed > 0:
            time.sleep(DT - elapsed)

    motor.stop()
    return times, targets, measurements


def compute_metrics(targets, measurements, settle_start=0.5):
    """Compute step response metrics from the last portion of data."""
    settle_idx = int(settle_start / DT)
    if settle_idx >= len(measurements):
        settle_idx = len(measurements) // 2

    settled = measurements[settle_idx:]
    tgt = targets[settle_idx:]

    if not settled or not tgt:
        return {'steady_state_error': float('inf'), 'overshoot': float('inf'),
                'oscillation': float('inf')}

    target = tgt[0]
    errors = [abs(m - target) for m in settled]
    mean_error = sum(errors) / len(errors)

    if abs(target) > 0.001:
        peak = max(settled) if target > 0 else min(settled)
        overshoot = abs(peak - target) / abs(target) * 100.0
    else:
        overshoot = max(abs(m) for m in settled) / 0.01 * 100.0 if settled else 0

    # Oscillation: std dev of settled measurements
    mean_vel = sum(settled) / len(settled)
    variance = sum((m - mean_vel) ** 2 for m in settled) / len(settled)
    oscillation = math.sqrt(variance)

    return {
        'steady_state_error': mean_error,
        'overshoot': overshoot,
        'oscillation': oscillation,
    }


def detect_encoder_direction(motor, encoder):
    """Drive motor forward briefly, check if encoder reads positive."""
    print("\n[1/4] Detecting encoder direction...")
    # Flush encoder
    encoder.get_and_reset()
    time.sleep(0.1)
    encoder.get_and_reset()

    # Drive forward at moderate speed
    motor.set_speed(0.4)
    time.sleep(0.5)

    ticks = encoder.get_and_reset()
    motor.stop()
    time.sleep(0.3)

    if ticks > 0:
        print(f"  Encoder reads +{ticks} ticks for forward drive → direction OK")
        return 1
    elif ticks < 0:
        print(f"  Encoder reads {ticks} ticks for forward drive → FLIPPED")
        return -1
    else:
        print("  WARNING: No encoder ticks detected! Check wiring.")
        print("  Assuming normal direction, continuing...")
        return 1


def find_ultimate_gain(motor, encoder, enc_flip):
    """
    Relay auto-tune: apply a bang-bang relay around target=0.
    Measure the oscillation period and amplitude to find Ku and Tu.
    """
    print("\n[2/4] Finding ultimate gain (relay method)...")
    print("  Applying relay feedback — motor will oscillate. This is normal.")

    relay_amplitude = 0.3  # PWM relay output
    target = 0.0

    # Flush encoder
    encoder.get_and_reset()
    time.sleep(0.1)
    encoder.get_and_reset()

    zero_crossings = []
    velocities = []
    prev_sign = 0
    duration = 6.0  # seconds
    steps = int(duration / DT)

    for i in range(steps):
        t0 = time.monotonic()
        vel = read_velocity(encoder, enc_flip)
        velocities.append(vel)

        error = target - vel
        # Relay: bang-bang control
        if error > 0:
            pwm = relay_amplitude
            current_sign = 1
        else:
            pwm = -relay_amplitude
            current_sign = -1

        motor.set_speed(pwm)

        # Detect zero crossings (after initial transient)
        if i > int(1.0 / DT) and prev_sign != 0 and current_sign != prev_sign:
            zero_crossings.append(i * DT)
        prev_sign = current_sign

        elapsed = time.monotonic() - t0
        if DT - elapsed > 0:
            time.sleep(DT - elapsed)

    motor.stop()
    time.sleep(0.3)

    # Compute Tu from zero crossings (period = 2 * avg half-period)
    if len(zero_crossings) < 4:
        print("  WARNING: Not enough oscillations detected.")
        print("  Using conservative default gains.")
        return None, None

    half_periods = []
    for j in range(1, len(zero_crossings)):
        half_periods.append(zero_crossings[j] - zero_crossings[j - 1])

    tu = 2.0 * (sum(half_periods) / len(half_periods))

    # Compute amplitude of oscillation (after transient)
    transient_idx = int(1.0 / DT)
    osc_vels = velocities[transient_idx:]
    if osc_vels:
        a_osc = (max(osc_vels) - min(osc_vels)) / 2.0
    else:
        a_osc = 0.01

    # Ku = 4 * relay_amplitude / (pi * a_osc)
    if a_osc > 0.0001:
        ku = (4.0 * relay_amplitude) / (math.pi * a_osc)
    else:
        print("  WARNING: Oscillation amplitude too small.")
        return None, None

    print(f"  Zero crossings: {len(zero_crossings)}")
    print(f"  Oscillation period Tu = {tu:.4f} s")
    print(f"  Oscillation amplitude  = {a_osc:.4f} m/s")
    print(f"  Ultimate gain Ku = {ku:.3f}")

    return ku, tu


def ziegler_nichols_pid(ku, tu):
    """Compute PID gains from Ku and Tu (classic ZN method)."""
    kp = 0.6 * ku
    ki = 2.0 * kp / tu
    kd = kp * tu / 8.0
    return kp, ki, kd


def ziegler_nichols_no_overshoot(ku, tu):
    """ZN variant: less aggressive, reduced overshoot."""
    kp = 0.2 * ku
    ki = 2.0 * kp / tu
    kd = kp * tu / 3.0
    return kp, ki, kd


def validate_gains(motor, encoder, enc_flip, kp, ki, kd, label):
    """Run step response tests and print results."""
    print(f"\n  Testing {label}: Kp={kp:.3f}  Ki={ki:.3f}  Kd={kd:.3f}")

    # Test 1: Hold at 0 (resist disturbance)
    print("    Hold at 0 m/s (2s)...", end=" ", flush=True)
    _, tgt, meas = run_pid(motor, encoder, enc_flip, kp, ki, kd, 0.0, 2.0)
    m = compute_metrics(tgt, meas, settle_start=0.5)
    print(f"err={m['steady_state_error']:.4f}  osc={m['oscillation']:.4f}")
    time.sleep(0.3)

    # Test 2: Step to 0.15 m/s
    print("    Step to +0.15 m/s (2s)...", end=" ", flush=True)
    _, tgt, meas = run_pid(motor, encoder, enc_flip, kp, ki, kd, 0.15, 2.0)
    m_fwd = compute_metrics(tgt, meas, settle_start=0.8)
    print(f"err={m_fwd['steady_state_error']:.4f}  overshoot={m_fwd['overshoot']:.1f}%  osc={m_fwd['oscillation']:.4f}")
    time.sleep(0.3)

    # Test 3: Step to -0.15 m/s
    print("    Step to -0.15 m/s (2s)...", end=" ", flush=True)
    _, tgt, meas = run_pid(motor, encoder, enc_flip, kp, ki, kd, -0.15, 2.0)
    m_rev = compute_metrics(tgt, meas, settle_start=0.8)
    print(f"err={m_rev['steady_state_error']:.4f}  overshoot={m_rev['overshoot']:.1f}%  osc={m_rev['oscillation']:.4f}")
    time.sleep(0.3)

    # Test 4: Return to 0
    print("    Return to 0 m/s (2s)...", end=" ", flush=True)
    _, tgt, meas = run_pid(motor, encoder, enc_flip, kp, ki, kd, 0.0, 2.0)
    m_zero = compute_metrics(tgt, meas, settle_start=0.5)
    print(f"err={m_zero['steady_state_error']:.4f}  osc={m_zero['oscillation']:.4f}")
    time.sleep(0.3)

    score = (m['steady_state_error'] + m_fwd['steady_state_error'] +
             m_rev['steady_state_error'] + m_zero['steady_state_error'])
    return score


def refine_gains(motor, encoder, enc_flip, kp, ki, kd):
    """Try small variations around the computed gains, pick the best."""
    print("\n[4/4] Refining gains...")

    best_kp, best_ki, best_kd = kp, ki, kd
    best_score = validate_gains(motor, encoder, enc_flip, kp, ki, kd, "baseline")

    # Try scaling each gain
    scales = [0.6, 0.8, 1.2, 1.5]
    for s in scales:
        test_kp = kp * s
        score = validate_gains(motor, encoder, enc_flip, test_kp, ki, kd,
                               f"Kp×{s}")
        if score < best_score:
            best_score = score
            best_kp = test_kp

    for s in scales:
        test_ki = ki * s
        score = validate_gains(motor, encoder, enc_flip, best_kp, test_ki, kd,
                               f"Ki×{s}")
        if score < best_score:
            best_score = score
            best_ki = test_ki

    for s in [0.5, 0.8, 1.2, 2.0]:
        test_kd = kd * s
        score = validate_gains(motor, encoder, enc_flip, best_kp, best_ki, test_kd,
                               f"Kd×{s}")
        if score < best_score:
            best_score = score
            best_kd = test_kd

    return best_kp, best_ki, best_kd, best_score


def main():
    print("=" * 65)
    print("  PID AUTO-TUNER — LF Motor")
    print("=" * 65)

    stby = DigitalOutputDevice(MD1_STBY)
    stby.on()
    motor = HBridgeMotor(MD1_PWMA, MD1_AIN1, MD1_AIN2)
    encoder = SignedEncoder(ENC_LF_A, ENC_LF_B)

    try:
        # Step 1: Detect encoder direction
        enc_flip = detect_encoder_direction(motor, encoder)

        # Step 2: Find Ku and Tu via relay method
        ku, tu = find_ultimate_gain(motor, encoder, enc_flip)

        if ku is not None and tu is not None:
            # Step 3: Compute candidate gains
            print("\n[3/4] Computing PID gains (Ziegler-Nichols)...")

            kp_classic, ki_classic, kd_classic = ziegler_nichols_pid(ku, tu)
            kp_safe, ki_safe, kd_safe = ziegler_nichols_no_overshoot(ku, tu)

            print(f"  Classic ZN:       Kp={kp_classic:.3f}  Ki={ki_classic:.3f}  Kd={kd_classic:.3f}")
            print(f"  No-overshoot ZN:  Kp={kp_safe:.3f}  Ki={ki_safe:.3f}  Kd={kd_safe:.3f}")

            # Validate both, pick better
            print("\n  --- Evaluating Classic ZN ---")
            score_classic = validate_gains(motor, encoder, enc_flip,
                                           kp_classic, ki_classic, kd_classic, "Classic")
            time.sleep(0.5)

            print("\n  --- Evaluating No-Overshoot ZN ---")
            score_safe = validate_gains(motor, encoder, enc_flip,
                                        kp_safe, ki_safe, kd_safe, "No-overshoot")
            time.sleep(0.5)

            if score_classic < score_safe:
                kp, ki, kd = kp_classic, ki_classic, kd_classic
                print(f"\n  Classic ZN wins (score={score_classic:.4f} vs {score_safe:.4f})")
            else:
                kp, ki, kd = kp_safe, ki_safe, kd_safe
                print(f"\n  No-overshoot ZN wins (score={score_safe:.4f} vs {score_classic:.4f})")
        else:
            # Fallback: conservative defaults
            print("\n[3/4] Using conservative default gains...")
            kp, ki, kd = 3.0, 1.0, 0.02

        # Step 4: Refine
        kp, ki, kd, final_score = refine_gains(motor, encoder, enc_flip, kp, ki, kd)

        # Final results
        print("\n" + "=" * 65)
        print("  AUTO-TUNE COMPLETE")
        print("=" * 65)
        print(f"  Encoder flip: {'yes (-1)' if enc_flip == -1 else 'no (1)'}")
        if ku is not None:
            print(f"  Ku = {ku:.3f}   Tu = {tu:.4f} s")
        print(f"")
        print(f"  ┌─────────────────────────────┐")
        print(f"  │  Kp = {kp:8.4f}              │")
        print(f"  │  Ki = {ki:8.4f}              │")
        print(f"  │  Kd = {kd:8.4f}              │")
        print(f"  └─────────────────────────────┘")
        print(f"  Score: {final_score:.4f} (lower = better)")
        print(f"")
        print(f"  To use in pid_tune_lf.py:")
        print(f"    p {kp:.4f} {ki:.4f} {kd:.4f}")
        print(f"")
        print(f"  For motor_driver_node.py:")
        print(f"    kp: {kp:.4f}")
        print(f"    ki: {ki:.4f}")
        print(f"    kd: {kd:.4f}")
        print("=" * 65)

        # Final demo: hold at 0 for 3 seconds
        print("\n  Final demo: holding at 0 m/s for 3s (try pushing the wheel)...")
        run_pid(motor, encoder, enc_flip, kp, ki, kd, 0.0, 3.0, label="hold")
        print("  Done.")

    except KeyboardInterrupt:
        print("\n  >> Interrupted")
    finally:
        motor.close()
        encoder.close()
        stby.off()
        stby.close()
        print("  >> Cleanup complete.")


if __name__ == '__main__':
    main()
