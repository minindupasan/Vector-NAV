#!/usr/bin/env python3
"""
PID Velocity Tuning Script — LF Motor Only.

Tunes PID so the motor:
  1. Tracks commanded velocity precisely
  2. Actively resists free movement when target = 0 (holds still)

Uses proper quadrature decoding (Phase A + Phase B) for signed direction.

Usage:
  Run this script, then type target velocity in m/s (e.g. 0.1, -0.1, 0).
  Press Enter to send. Type 'q' to quit.
  Type 'p <kp> <ki> <kd>' to change PID gains live.
"""

import math
import threading
import time
import sys
import select

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
WHEEL_RADIUS = 0.0325   # meters
TICKS_PER_REV = 225      # 11 PPR × 20.5:1 gearbox
METERS_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
MAX_SPEED = 0.5          # m/s (for PWM normalization)
PWM_FREQ = 1000
CONTROL_HZ = 50
DT = 1.0 / CONTROL_HZ


class HBridgeMotor:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, frequency=PWM_FREQ)
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)

    def set_speed(self, speed_frac):
        """Set motor speed: -1.0 … +1.0 (positive = forward)."""
        if speed_frac > 0:
            self.in1.on();  self.in2.off()
        elif speed_frac < 0:
            self.in1.off(); self.in2.on()
        else:
            self.in1.off(); self.in2.off()
        self.pwm.value = min(abs(speed_frac), 1.0)

    def brake(self):
        """Active brake: both inputs high, PWM full."""
        self.in1.on(); self.in2.on()
        self.pwm.value = 1.0

    def stop(self):
        self.in1.off(); self.in2.off(); self.pwm.value = 0

    def close(self):
        self.stop()
        self.pwm.close(); self.in1.close(); self.in2.close()


class SignedEncoder:
    """Quadrature encoder with direction from Phase B on Phase A rising edge."""

    def __init__(self, pin_a, pin_b):
        self._count = 0
        self._lock = threading.Lock()

        self._phase_b = Button(pin_b, pull_up=True, bounce_time=None)
        self._phase_a = Button(pin_a, pull_up=True, bounce_time=None)
        self._phase_a.when_pressed = self._on_rising_a

    def _on_rising_a(self):
        # On Phase A rising edge: Phase B high = forward, low = reverse
        direction = -1 if self._phase_b.is_pressed else 1
        with self._lock:
            self._count += direction

    def get_and_reset(self):
        """Return signed tick count since last call."""
        with self._lock:
            c = self._count
            self._count = 0
        return c

    def close(self):
        self._phase_a.close()
        self._phase_b.close()


def main():
    # ---------- Init hardware ----------
    stby = DigitalOutputDevice(MD1_STBY)
    stby.on()

    motor = HBridgeMotor(MD1_PWMA, MD1_AIN1, MD1_AIN2)
    encoder = SignedEncoder(ENC_LF_A, ENC_LF_B)

    # ---------- PID state ----------
    kp = 10.0
    ki = 2.0
    kd = 0.05
    integral = 0.0
    prev_error = 0.0
    integral_limit = 1.0  # anti-windup clamp

    target_vel = 0.0  # m/s
    measured_vel = 0.0
    enc_flip = 1  # set to -1 if encoder direction is wrong

    print("=" * 65)
    print("  PID Velocity Tuner — LF Motor")
    print("=" * 65)
    print(f"  PID: Kp={kp}  Ki={ki}  Kd={kd}")
    print(f"  Wheel: radius={WHEEL_RADIUS}m, {TICKS_PER_REV} ticks/rev")
    print(f"  Control rate: {CONTROL_HZ} Hz")
    print("-" * 65)
    print("  Commands:")
    print("    <velocity>       — set target m/s (e.g. 0.1, -0.05, 0)")
    print("    p <kp> <ki> <kd> — change PID gains")
    print("    f                — flip encoder direction")
    print("    r                — reset PID integral")
    print("    q                — quit")
    print("-" * 65)

    log_counter = 0
    running = True

    try:
        while running:
            loop_start = time.monotonic()

            # --- Check for user input (non-blocking) ---
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line == 'q':
                    break
                elif line == 'r':
                    integral = 0.0
                    prev_error = 0.0
                    print("  >> PID reset")
                elif line == 'f':
                    enc_flip *= -1
                    integral = 0.0
                    prev_error = 0.0
                    print(f"  >> Encoder direction flipped (now {'inverted' if enc_flip == -1 else 'normal'})")
                elif line.startswith('p '):
                    parts = line.split()
                    if len(parts) == 4:
                        kp = float(parts[1])
                        ki = float(parts[2])
                        kd = float(parts[3])
                        integral = 0.0
                        prev_error = 0.0
                        print(f"  >> PID gains: Kp={kp}  Ki={ki}  Kd={kd}")
                    else:
                        print("  >> Usage: p <kp> <ki> <kd>")
                else:
                    try:
                        target_vel = float(line)
                        integral = 0.0
                        prev_error = 0.0
                        print(f"  >> Target velocity: {target_vel:.3f} m/s")
                    except ValueError:
                        print(f"  >> Unknown command: {line}")

            # --- Read encoder (signed ticks) ---
            ticks = encoder.get_and_reset() * enc_flip
            distance = ticks * METERS_PER_TICK
            measured_vel = distance / DT

            # --- PID computation ---
            error = target_vel - measured_vel

            integral += error * DT
            integral = max(-integral_limit, min(integral_limit, integral))

            derivative = (error - prev_error) / DT
            prev_error = error

            pid_output = kp * error + ki * integral + kd * derivative

            # Clamp to [-1, 1]
            pwm = max(-1.0, min(1.0, pid_output))

            # --- Drive motor ---
            motor.set_speed(pwm)

            # --- Logging (every 5 cycles = 10 Hz) ---
            log_counter += 1
            if log_counter >= 5:
                log_counter = 0
                rpm = measured_vel / (2.0 * math.pi * WHEEL_RADIUS) * 60.0
                target_rpm = target_vel / (2.0 * math.pi * WHEEL_RADIUS) * 60.0
                print(
                    f"  tgt:{target_vel:+.3f} m/s ({target_rpm:+6.1f} RPM) | "
                    f"meas:{measured_vel:+.3f} m/s ({rpm:+6.1f} RPM) | "
                    f"err:{error:+.4f} | "
                    f"P:{kp*error:+.3f} I:{ki*integral:+.3f} D:{kd*derivative:+.3f} | "
                    f"pwm:{pwm:+.3f} | ticks:{ticks:+d}"
                )

            # --- Maintain loop rate ---
            elapsed = time.monotonic() - loop_start
            sleep_time = DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n  >> Interrupted")
    finally:
        print("  >> Stopping motor...")
        motor.close()
        encoder.close()
        stby.off()
        stby.close()
        print("  >> Done.")


if __name__ == '__main__':
    main()
