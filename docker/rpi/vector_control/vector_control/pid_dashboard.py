#!/usr/bin/env python3
"""
PID Tuning Dashboard — All 4 Motors.

Web-based real-time dashboard for tuning PID controllers on each motor
individually. Provides live RPM plots, error visualization, and
interactive gain adjustment.

Usage:
    python3 pid_dashboard.py [--port 5000] [--host 0.0.0.0]

Then open http://<pi-ip>:5000 in a browser.
"""

import argparse
import math
import threading
import time
import json
import signal
import sys

import glob as _glob
import gpiozero
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button
from gpiozero.pins.lgpio import LGPIOFactory

from flask import Flask, render_template_string
from flask_socketio import SocketIO

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

# ---------- Pin definitions ----------
# Motor Driver 1 (LF = Channel A, LR = Channel B)
MD1_PWMA = 12;  MD1_AIN1 = 6;   MD1_AIN2 = 5;   MD1_STBY = 13
MD1_PWMB = 18;  MD1_BIN1 = 26;  MD1_BIN2 = 19

# Motor Driver 2 (RF = Channel A, RR = Channel B)
MD2_PWMA = 16;  MD2_AIN1 = 20;  MD2_AIN2 = 21;  MD2_STBY = 23
MD2_PWMB = 22;  MD2_BIN1 = 17;  MD2_BIN2 = 27

# Encoders (Phase A, Phase B)
ENC_LF_A = 4;   ENC_LF_B = 25
ENC_LR_A = 24;  ENC_LR_B = 14
ENC_RF_A = 15;  ENC_RF_B = 7
ENC_RR_A = 10;  ENC_RR_B = 9

# ---------- Constants ----------
WHEEL_RADIUS = 0.034
TICKS_PER_REV = 225
METERS_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV
MAX_SPEED = 1.0  # actual no-load top speed (~0.95 m/s); keeps ff scaling accurate
PWM_FREQ = 1000
CONTROL_HZ = 50
DT = 1.0 / CONTROL_HZ
RPM_FACTOR = 60.0 / (2.0 * math.pi * WHEEL_RADIUS)
EMA_ALPHA = 0.35

MOTOR_NAMES = ['LF', 'LR', 'RF', 'RR']
MOTOR_LABELS = ['Left Front', 'Left Rear', 'Right Front', 'Right Rear']
MOTOR_COLORS = ['#3b82f6', '#8b5cf6', '#10b981', '#f59e0b']


class HBridgeMotor:
    def __init__(self, pwm_pin, in1_pin, in2_pin):
        self.pwm = PWMOutputDevice(pwm_pin, frequency=PWM_FREQ)
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)

    def set_speed(self, speed_frac):
        if speed_frac > 0:
            self.in1.on(); self.in2.off()
        elif speed_frac < 0:
            self.in1.off(); self.in2.on()
        else:
            self.in1.off(); self.in2.off()
        self.pwm.value = min(abs(speed_frac), 1.0)

    def brake(self):
        # Active short-brake: both H-bridge inputs HIGH with PWM at 100%.
        # Motor terminals are shorted through the H-bridge → strong resistance
        # to any external rotation. This is the only reliable way to "hold
        # position" with a magnitude-only encoder, since PID can't resolve
        # disturbance direction at zero velocity.
        self.in1.on(); self.in2.on()
        self.pwm.value = 1.0

    def stop(self):
        self.in1.off(); self.in2.off(); self.pwm.value = 0

    def close(self):
        self.stop()
        self.pwm.close(); self.in1.close(); self.in2.close()


class PulseEncoder:
    """Magnitude-only encoder: counts Phase A rising edges.

    Phase B direction reading is unreliable in Python at high RPM because
    gpiozero callbacks have 1–5ms latency. By the time _on_rising_a fires,
    Phase B has already transitioned, giving the wrong sign. Instead we
    count pulses for magnitude and let the control loop supply direction
    from the actual PWM sign applied the previous cycle — the same approach
    used in motor_driver_node.py.
    """

    def __init__(self, pin_a, pin_b):
        self._count = 0
        self._lock = threading.Lock()
        self._phase_a = Button(pin_a, pull_up=True, bounce_time=None)
        self._phase_a.when_pressed = self._on_rising
        # Phase B kept for completeness / future hardware use
        self._phase_b = Button(pin_b, pull_up=True, bounce_time=None)

    def _on_rising(self):
        with self._lock:
            self._count += 1

    def get_and_reset(self, direction_sign=1):
        """Return signed tick count. direction_sign comes from the caller (PWM sign)."""
        with self._lock:
            c = self._count
            self._count = 0
        return c * direction_sign

    def close(self):
        self._phase_a.close()
        self._phase_b.close()


class PIDController:
    """
    PI(D) controller with feedforward and conditional-integration anti-windup.

    Output: u = ff + Kp·e + Ki·∫e dt + Kd·de/dt   clamped to ±output_limit.

    Anti-windup uses *conditional integration* (Åström & Hägglund 1995, §3.5):
    the integrator is paused when the unsaturated output already exceeds the
    actuator limit AND further integration would push it deeper into
    saturation (i.e. error has the same sign as the saturated output). When
    error reverses, integration resumes immediately so the controller can
    pull back. This is the standard textbook scheme — no ad-hoc clamping of
    the integral against ki.
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, setpoint, measured, dt, ff=0.0):
        error = setpoint - measured
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0

        p_term = self.kp * error
        d_term = self.kd * derivative
        i_term = self.ki * self._integral

        # Conditional integration: integrate iff the output is unsaturated,
        # OR integration would reduce saturation (error opposes current output).
        u_pre = ff + p_term + i_term + d_term
        if abs(u_pre) < self.output_limit or u_pre * error < 0:
            self._integral += error * dt
            i_term = self.ki * self._integral

        output = ff + p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))

        self._prev_error = error
        return output, error, p_term, i_term, d_term

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()


# ---------- Flask app ----------
app = Flask(__name__)
app.config['SECRET_KEY'] = 'vectornav-pid'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ---------- Global state ----------
motors = []
encoders = []
pids = []
stby_pins = []
motor_enabled = [False, False, False, False]
target_rpm = [0.0, 0.0, 0.0, 0.0]
enc_flip = [1, 1, 1, 1]
v_filtered = [0.0, 0.0, 0.0, 0.0]
motor_dir = [1, 1, 1, 1]   # actual PWM direction from previous cycle (+1 or -1)
autotune_active = [False, False, False, False]  # bypass control loop stop during autotune
min_pwm = 0.18
control_running = False
hw_initialized = False


def init_hardware():
    global motors, encoders, pids, stby_pins, hw_initialized

    stby1 = DigitalOutputDevice(MD1_STBY)
    stby2 = DigitalOutputDevice(MD2_STBY)
    stby1.on()
    stby2.on()
    stby_pins = [stby1, stby2]

    motors = [
        HBridgeMotor(MD1_PWMA, MD1_AIN1, MD1_AIN2),  # LF
        HBridgeMotor(MD1_PWMB, MD1_BIN1, MD1_BIN2),  # LR
        HBridgeMotor(MD2_PWMA, MD2_AIN1, MD2_AIN2),  # RF
        HBridgeMotor(MD2_PWMB, MD2_BIN1, MD2_BIN2),  # RR
    ]

    encoders = [
        PulseEncoder(ENC_LF_A, ENC_LF_B),
        PulseEncoder(ENC_LR_A, ENC_LR_B),
        PulseEncoder(ENC_RF_A, ENC_RF_B),
        PulseEncoder(ENC_RR_A, ENC_RR_B),
    ]

    pids = [PIDController() for _ in range(4)]
    hw_initialized = True
    print("[HW] Hardware initialized")


def shutdown_hardware():
    global hw_initialized, control_running
    control_running = False
    time.sleep(0.1)
    for m in motors:
        m.close()
    for e in encoders:
        e.close()
    for s in stby_pins:
        s.off()
        s.close()
    hw_initialized = False
    print("[HW] Hardware shutdown")


def control_loop():
    global control_running
    control_running = True
    cycle = 0

    while control_running:
        loop_start = time.monotonic()

        data = {'t': time.time(), 'motors': {}}

        for i in range(4):
            # Use PREVIOUS cycle's actual PWM direction for encoder sign.
            # Reading Phase B in the callback is unreliable above ~50 RPM
            # due to Python callback latency — the pulse has already passed.
            direction_sign = motor_dir[i] * enc_flip[i]
            ticks = encoders[i].get_and_reset(direction_sign)
            distance = ticks * METERS_PER_TICK
            raw_vel = distance / DT
            v_filtered[i] = EMA_ALPHA * raw_vel + (1.0 - EMA_ALPHA) * v_filtered[i]

            measured_vel = v_filtered[i]
            measured_rpm = measured_vel * RPM_FACTOR
            target_vel = target_rpm[i] / RPM_FACTOR  # RPM -> m/s

            error = 0.0
            p_term = i_term = d_term = 0.0
            pwm = 0.0

            if motor_enabled[i]:
                if abs(target_rpm[i]) < 0.5:
                    # Position hold via active brake. With a magnitude-only
                    # encoder, PID cannot resolve disturbance direction at
                    # zero velocity, so we shore-circuit the motor terminals
                    # through the H-bridge instead.
                    motors[i].brake()
                    pids[i].reset()
                    v_filtered[i] = 0.0
                    error = -measured_vel
                    pwm = 1.0  # display only
                else:
                    # Standard PI(D) + feed-forward velocity controller
                    ff = target_vel / MAX_SPEED
                    pwm, error, p_term, i_term, d_term = pids[i].compute(
                        target_vel, measured_vel, DT, ff=ff)

                    # Stall-only stiction kick: only when the motor is at rest
                    # AND the controller wants to move in the target direction.
                    # Avoids the limit-cycle behavior of an unconditional kick.
                    if (abs(measured_vel) < 0.02
                            and pwm * target_vel > 0
                            and abs(pwm) < min_pwm):
                        pwm = math.copysign(min_pwm, pwm)

                    motors[i].set_speed(pwm)
                    if abs(pwm) > 0.05:
                        motor_dir[i] = 1 if pwm > 0 else -1
            elif not autotune_active[i]:
                motors[i].stop()
                pids[i].reset()

            error_rpm = (target_vel - measured_vel) * RPM_FACTOR if motor_enabled[i] else 0.0

            data['motors'][MOTOR_NAMES[i]] = {
                'rpm': round(measured_rpm, 2),
                'target_rpm': round(target_rpm[i], 2),
                'error': round(error_rpm, 2),
                'pwm': round(pwm, 4),
                'p': round(p_term, 4),
                'i': round(i_term, 4),
                'd': round(d_term, 4),
                'ticks': ticks,
                'enabled': motor_enabled[i],
                'kp': pids[i].kp,
                'ki': pids[i].ki,
                'kd': pids[i].kd,
            }

        # Send data at ~20 Hz (every 2-3 control cycles at 50 Hz)
        cycle += 1
        if cycle % 2 == 0:
            socketio.emit('motor_data', data)

        elapsed = time.monotonic() - loop_start
        sleep_time = DT - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


# ---------- Socket events ----------
@socketio.on('connect')
def on_connect():
    print("[WS] Client connected")
    # Send current state
    state = {}
    for i in range(4):
        state[MOTOR_NAMES[i]] = {
            'enabled': motor_enabled[i],
            'target_rpm': target_rpm[i],
            'kp': pids[i].kp,
            'ki': pids[i].ki,
            'kd': pids[i].kd,
            'enc_flip': enc_flip[i],
        }
    socketio.emit('state', state)


@socketio.on('set_target')
def on_set_target(data):
    idx = MOTOR_NAMES.index(data['motor'])
    target_rpm[idx] = float(data['rpm'])
    pids[idx].reset()


@socketio.on('set_pid')
def on_set_pid(data):
    idx = MOTOR_NAMES.index(data['motor'])
    pids[idx].set_gains(
        float(data['kp']),
        float(data['ki']),
        float(data['kd']),
    )


@socketio.on('toggle_motor')
def on_toggle(data):
    idx = MOTOR_NAMES.index(data['motor'])
    motor_enabled[idx] = data['enabled']
    if not data['enabled']:
        motors[idx].stop()
        pids[idx].reset()
        v_filtered[idx] = 0.0
        motor_dir[idx] = 1


@socketio.on('flip_encoder')
def on_flip(data):
    idx = MOTOR_NAMES.index(data['motor'])
    enc_flip[idx] *= -1
    pids[idx].reset()
    v_filtered[idx] = 0.0
    socketio.emit('enc_flipped', {'motor': data['motor'], 'flip': enc_flip[idx]})


@socketio.on('emergency_stop')
def on_estop(_=None):
    for i in range(4):
        motor_enabled[i] = False
        target_rpm[i] = 0.0
        motors[i].stop()
        pids[i].reset()
        v_filtered[i] = 0.0
        motor_dir[i] = 1
    socketio.emit('stopped', {})


@socketio.on('copy_pid')
def on_copy_pid(data):
    src = MOTOR_NAMES.index(data['from'])
    kp, ki, kd = pids[src].kp, pids[src].ki, pids[src].kd
    for name in data['to']:
        idx = MOTOR_NAMES.index(name)
        pids[idx].set_gains(kp, ki, kd)
    socketio.emit('pid_copied', {'kp': kp, 'ki': ki, 'kd': kd, 'to': data['to']})


@socketio.on('set_all_target')
def on_set_all_target(data):
    rpm = float(data['rpm'])
    for i in range(4):
        target_rpm[i] = rpm
        pids[i].reset()


@socketio.on('enable_all')
def on_enable_all(data):
    enabled = data['enabled']
    for i in range(4):
        motor_enabled[i] = enabled
        if not enabled:
            motors[i].stop()
            pids[i].reset()
            v_filtered[i] = 0.0


# ---------- Auto-Tune (Skogestad SIMC, λ-tuning) ----------
autotune_running = {}  # motor_name -> True/False


def _at_run_pid(motor_idx, target_vel, kp, ki, kd, duration, warmup=0.5):
    """
    Run the production PI(D)+FF control law with the given gains for
    `duration` seconds and return performance metrics:

        (mse, max_overshoot_frac, mean_ss_error, mean_pwm_chatter)

    measured over the post-warmup window. Mirrors PIDController.compute()
    exactly — same FF formula, same conditional-integration anti-windup,
    same stiction kick — so validation results match production behavior.
    """
    integral = 0.0
    prev_err = 0.0
    v_filt = 0.0
    ff = target_vel / MAX_SPEED

    steps = int(duration / DT)
    warmup_steps = int(warmup / DT)

    errors_sq = []
    overshoot = 0.0
    ss_errors = []
    pwm_history = []

    encoders[motor_idx].get_and_reset(1)

    for step in range(steps):
        t0 = time.monotonic()

        # Velocity feedback (magnitude only — autotune always runs forward)
        ticks = encoders[motor_idx].get_and_reset(1)
        v_raw = ticks * METERS_PER_TICK / DT
        v_filt = EMA_ALPHA * v_raw + (1.0 - EMA_ALPHA) * v_filt

        err = target_vel - v_filt
        deriv = (err - prev_err) / DT

        # Conditional integration anti-windup (matches PIDController.compute)
        u_pre = ff + kp * err + ki * integral + kd * deriv
        if abs(u_pre) < 1.0 or u_pre * err < 0:
            integral += err * DT
        prev_err = err

        u = ff + kp * err + ki * integral + kd * deriv
        pwm = max(-1.0, min(1.0, u))

        # Stall-only stiction kick (matches control loop)
        if (v_filt < 0.02 and target_vel > 0
                and 0 < pwm < min_pwm):
            pwm = min_pwm

        motors[motor_idx].set_speed(pwm)

        if step >= warmup_steps:
            errors_sq.append(err * err)
            overshoot = max(overshoot, v_filt - target_vel)
            ss_errors.append(err)
            pwm_history.append(pwm)

        elapsed = time.monotonic() - t0
        if DT - elapsed > 0:
            time.sleep(DT - elapsed)

    motors[motor_idx].stop()
    time.sleep(0.25)

    mse = sum(errors_sq) / len(errors_sq) if errors_sq else 999.0
    os_frac = max(0.0, overshoot) / (target_vel + 1e-9)
    n_ss = max(1, int(1.0 / DT))
    mean_ss = (sum(ss_errors[-n_ss:]) / min(n_ss, len(ss_errors))
               if ss_errors else 0.0)

    chatter = 0.0
    if len(pwm_history) > 5:
        diffs = [abs(pwm_history[k] - pwm_history[k-1])
                 for k in range(1, len(pwm_history))]
        chatter = sum(diffs) / len(diffs)

    return mse, os_frac, mean_ss, chatter


def autotune_motor(motor_idx):
    """
    SIMC λ-tuned PI controller for DC motor velocity control.

    Three-phase deterministic procedure — produces ONE answer, no candidate
    competition, no empirical tournament:

      1. FOPDT identification via step response.
            K = v_ss / step_pwm                  (DC gain, m/s per PWM unit)
            L = first time v ≥ 5 % of v_ss        (apparent transport delay)
            τ = first time v ≥ 63.2 % of v_ss − L  (dominant time constant)

      2. Skogestad SIMC λ-tuning rules for FOPDT plants:
            τ_c = max(τ, 8L)        — closed-loop time constant; 8L floor
                                       handles dead-time-dominated systems
            Kp  = τ / (K · (τ_c + L))
            Ti  = min(τ, 4·(τ_c + L))
            Ki  = Kp / Ti
            Kd  = 0  (D term amplifies encoder quantisation noise without
                      benefit for first-order velocity loops)

      3. Validation at low / mid / high setpoints + brake hold check.
            Reports MSE, overshoot, steady-state error, PWM chatter — purely
            informational; gains are not adjusted from the validation data.

    References
        Skogestad, S. (2003). Simple analytic rules for model reduction and
            PID controller tuning. J. Process Control 13, 291–309.
        Åström, K. J. & Hägglund, T. (1995). PID Controllers: Theory, Design
            and Tuning, 2nd ed., ISA.
    """
    name = MOTOR_NAMES[motor_idx]
    autotune_running[name] = True
    autotune_active[motor_idx] = True

    motor_enabled[motor_idx] = False
    motors[motor_idx].stop()
    pids[motor_idx].reset()
    v_filtered[motor_idx] = 0.0
    time.sleep(0.4)

    def emit(msg, phase=None, progress=None):
        socketio.emit('autotune_status', {
            'motor': name, 'msg': msg, 'phase': phase, 'progress': progress,
        })

    try:
        # ── Phase 1: FOPDT identification ─────────────────────────────────
        emit('Phase 1/3 — System identification (step response)...',
             'identify', 5)

        STEP_PWM = 0.55           # well above stiction (~0.18) for clean ramp
        STEP_DUR = 6.0            # ≫ 5τ_typical, enough to settle
        N_STEPS  = int(STEP_DUR / DT)

        motors[motor_idx].stop()
        time.sleep(0.5)
        encoders[motor_idx].get_and_reset(1)

        motors[motor_idx].set_speed(STEP_PWM)
        t_start = time.monotonic()

        samples = []   # (t_since_start, raw_velocity)
        for k in range(N_STEPS):
            t0 = time.monotonic()
            ticks = encoders[motor_idx].get_and_reset(1)
            v_raw = ticks * METERS_PER_TICK / DT
            samples.append((t0 - t_start, v_raw))

            if k % 50 == 0:
                emit(f'Step ID  t={k*DT:.2f}s  v={v_raw:.3f} m/s',
                     'identify', 5 + int(35 * k / N_STEPS))

            elapsed = time.monotonic() - t0
            if DT - elapsed > 0:
                time.sleep(DT - elapsed)

        motors[motor_idx].stop()
        time.sleep(0.5)

        if len(samples) < 50:
            raise RuntimeError(f'Step ID failed: only {len(samples)} samples')

        # Smooth raw velocities for analysis (matches production EMA)
        smoothed = []
        v_e = 0.0
        for t, v in samples:
            v_e = EMA_ALPHA * v + (1.0 - EMA_ALPHA) * v_e
            smoothed.append((t, v_e))

        # Steady-state v_ss: median over last 25 % of samples
        tail = sorted(v for _, v in smoothed[int(0.75 * len(smoothed)):])
        v_ss = tail[len(tail) // 2] if tail else 0.0

        if v_ss < 0.05:
            raise RuntimeError(
                f'Motor did not respond to PWM={STEP_PWM} '
                f'(v_ss={v_ss:.4f} m/s) — check wiring and standby pin')

        # DC gain
        K = v_ss / STEP_PWM

        # Transport delay L: first crossing of 5 % v_ss
        L = 0.05  # fallback
        for t, v in smoothed:
            if v >= 0.05 * v_ss:
                L = max(0.005, t)
                break

        # Time constant τ: first crossing of 63.2 % v_ss, minus L
        tau = STEP_DUR * 0.3  # fallback
        for t, v in smoothed:
            if v >= 0.632 * v_ss:
                tau = max(0.02, t - L)
                break

        emit(f'Plant identified  K={K:.4f} m/s/PWM  τ={tau*1000:.0f}ms  '
             f'L={L*1000:.0f}ms  v_ss={v_ss:.3f} m/s',
             'identify', 40)

        # ── Phase 2: SIMC λ-tuning ────────────────────────────────────────
        emit('Phase 2/3 — Computing PI gains (Skogestad SIMC)...',
             'compute', 45)

        tau_c = max(tau, 8.0 * L)         # robust default — settling ≈ 3τ_c
        Kp    = tau / (K * (tau_c + L))
        Ti    = min(tau, 4.0 * (tau_c + L))
        Ki    = Kp / Ti
        Kd    = 0.0

        emit(f'SIMC  τ_c={tau_c*1000:.0f}ms  Ti={Ti*1000:.0f}ms  →  '
             f'Kp={Kp:.5f}  Ki={Ki:.5f}  Kd=0',
             'compute', 55)

        # ── Phase 3: Validation ────────────────────────────────────────────
        emit('Phase 3/3 — Validating at low/mid/high setpoints...',
             'validate', 60)

        targets = [v_ss * 0.20, v_ss * 0.50, v_ss * 0.80]
        validation = []
        for j, target in enumerate(targets):
            emit(f'  @ {target:.2f} m/s ({target*RPM_FACTOR:.1f} RPM)...',
                 'validate', 60 + j * 10)
            mse, os_frac, ss_err, chatter = _at_run_pid(
                motor_idx, target, Kp, Ki, Kd, duration=4.0, warmup=1.5)
            validation.append({
                'target_mps':   round(target, 4),
                'target_rpm':   round(target * RPM_FACTOR, 1),
                'mse':          round(mse, 6),
                'overshoot_pct': round(os_frac * 100, 1),
                'ss_err_rpm':   round(ss_err * RPM_FACTOR, 2),
                'chatter':      round(chatter, 5),
            })
            emit(f'    MSE={mse:.5f}  OS={os_frac*100:.1f}%  '
                 f'SS_err={ss_err*RPM_FACTOR:+.2f} RPM  '
                 f'chatter={chatter:.4f}',
                 'validate')
            time.sleep(0.4)

        # Brake hold sanity check (active brake at zero target)
        emit('  Brake hold check...', 'validate', 92)
        encoders[motor_idx].get_and_reset(1)
        motors[motor_idx].brake()
        time.sleep(1.0)
        hold_ticks = abs(encoders[motor_idx].get_and_reset(1))
        motors[motor_idx].stop()
        hold_pass = hold_ticks <= 2

        # Apply gains
        pids[motor_idx].set_gains(Kp, Ki, Kd)

        emit(f'Done — Kp={Kp:.5f}  Ki={Ki:.5f}  Kd=0   '
             f'hold={"OK" if hold_pass else f"WARN ({hold_ticks} ticks)"}',
             'done', 100)

        socketio.emit('autotune_done', {
            'motor': name,
            'kp':    round(Kp, 5),
            'ki':    round(Ki, 5),
            'kd':    0.0,
            'plant': {
                'K':    round(K,   5),
                'tau':  round(tau, 5),
                'L':    round(L,   5),
                'v_ss': round(v_ss, 4),
            },
            'tau_c':      round(tau_c, 5),
            'validation': validation,
            'hold_pass':  hold_pass,
        })

    except Exception as e:
        motors[motor_idx].stop()
        emit(f'Error: {e}', 'error', 0)
        socketio.emit('autotune_done', {
            'motor': name, 'kp': 0.0, 'ki': 0.0, 'kd': 0.0, 'error': str(e),
        })
    finally:
        autotune_active[motor_idx] = False
        autotune_running[name] = False





@socketio.on('autotune')
def on_autotune(data):
    name = data['motor']
    if autotune_running.get(name):
        socketio.emit('autotune_status', {
            'motor': name, 'msg': 'Already running!', 'phase': 'error'})
        return
    idx = MOTOR_NAMES.index(name)
    thread = threading.Thread(target=autotune_motor, args=(idx,), daemon=True)
    thread.start()


@socketio.on('autotune_all')
def on_autotune_all(_=None):
    # Run motors one at a time — sequential so step responses don't overlap.
    def run_all():
        for i, name in enumerate(MOTOR_NAMES):
            if autotune_running.get(name):
                socketio.emit('autotune_status', {
                    'motor': name, 'msg': 'Skipped — already running',
                    'phase': 'skip', 'progress': 0,
                })
                continue
            socketio.emit('autotune_status', {
                'motor': name, 'msg': f'Starting ({i+1}/4)…',
                'phase': 'start', 'progress': 0,
            })
            autotune_motor(i)
            time.sleep(1.0)
    threading.Thread(target=run_all, daemon=True).start()


# ---------- HTML Template ----------
@app.route('/')
def index():
    return render_template_string(DASHBOARD_HTML)


DASHBOARD_HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>VECTOR NAV — PID Tuning Dashboard</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.4/socket.io.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
:root {
    --bg: #0f1117;
    --bg2: #1a1d28;
    --bg3: #242837;
    --border: #2e3348;
    --text: #e4e7f1;
    --text2: #8b8fa8;
    --accent: #3b82f6;
    --green: #10b981;
    --red: #ef4444;
    --orange: #f59e0b;
    --purple: #8b5cf6;
    --lf: #3b82f6;
    --lr: #8b5cf6;
    --rf: #10b981;
    --rr: #f59e0b;
}

* { margin: 0; padding: 0; box-sizing: border-box; }

body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    overflow-x: hidden;
}

.header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 16px 24px;
    background: var(--bg2);
    border-bottom: 1px solid var(--border);
}

.header h1 {
    font-size: 20px;
    font-weight: 700;
    letter-spacing: 1px;
}

.header h1 span { color: var(--accent); }

.header-status {
    display: flex;
    align-items: center;
    gap: 16px;
}

.status-dot {
    width: 10px; height: 10px;
    border-radius: 50%;
    background: var(--green);
    animation: pulse 2s infinite;
}

.status-dot.disconnected { background: var(--red); animation: none; }

@keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.4; }
}

.global-controls {
    display: flex;
    gap: 12px;
    align-items: center;
    padding: 12px 24px;
    background: var(--bg2);
    border-bottom: 1px solid var(--border);
    flex-wrap: wrap;
}

.btn {
    padding: 8px 16px;
    border: 1px solid var(--border);
    border-radius: 8px;
    background: var(--bg3);
    color: var(--text);
    cursor: pointer;
    font-size: 13px;
    font-weight: 500;
    transition: all 0.2s;
}

.btn:hover { background: var(--border); }

.btn-danger {
    background: var(--red);
    border-color: var(--red);
    color: white;
    font-weight: 700;
}

.btn-danger:hover { background: #dc2626; }

.btn-success {
    background: var(--green);
    border-color: var(--green);
    color: white;
}

.btn-success:hover { background: #059669; }

.global-rpm-input {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-left: auto;
}

.global-rpm-input label { font-size: 13px; color: var(--text2); }

.global-rpm-input input {
    width: 100px;
    padding: 6px 10px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    font-size: 14px;
    text-align: center;
}

.dashboard {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 16px;
    padding: 16px 24px;
}

@media (max-width: 1200px) {
    .dashboard { grid-template-columns: 1fr; }
}

.motor-card {
    background: var(--bg2);
    border: 1px solid var(--border);
    border-radius: 12px;
    overflow: hidden;
}

.motor-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 12px 16px;
    border-bottom: 1px solid var(--border);
}

.motor-title {
    display: flex;
    align-items: center;
    gap: 10px;
}

.motor-dot {
    width: 12px; height: 12px;
    border-radius: 50%;
}

.motor-title h3 { font-size: 15px; font-weight: 600; }
.motor-title span { font-size: 12px; color: var(--text2); }

.motor-toggle {
    position: relative;
    width: 48px; height: 26px;
}

.motor-toggle input { opacity: 0; width: 0; height: 0; }

.toggle-slider {
    position: absolute;
    inset: 0;
    background: var(--bg);
    border-radius: 13px;
    cursor: pointer;
    transition: 0.3s;
    border: 1px solid var(--border);
}

.toggle-slider::before {
    content: '';
    position: absolute;
    width: 20px; height: 20px;
    left: 2px; bottom: 2px;
    background: var(--text2);
    border-radius: 50%;
    transition: 0.3s;
}

.motor-toggle input:checked + .toggle-slider {
    background: var(--green);
    border-color: var(--green);
}

.motor-toggle input:checked + .toggle-slider::before {
    transform: translateX(22px);
    background: white;
}

.motor-body { padding: 12px 16px; }

.metrics-row {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 8px;
    margin-bottom: 12px;
}

.metric {
    text-align: center;
    padding: 8px 4px;
    background: var(--bg);
    border-radius: 8px;
    border: 1px solid var(--border);
}

.metric-label {
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    color: var(--text2);
    margin-bottom: 4px;
}

.metric-value {
    font-size: 20px;
    font-weight: 700;
    font-variant-numeric: tabular-nums;
}

.metric-unit {
    font-size: 10px;
    color: var(--text2);
}

.target-row {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-bottom: 12px;
}

.target-row label {
    font-size: 12px;
    color: var(--text2);
    white-space: nowrap;
}

.target-input {
    flex: 1;
    padding: 6px 10px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--text);
    font-size: 14px;
    text-align: center;
}

.target-input:focus {
    outline: none;
    border-color: var(--accent);
}

.target-presets {
    display: flex;
    gap: 4px;
}

.preset-btn {
    padding: 4px 8px;
    font-size: 11px;
    border: 1px solid var(--border);
    border-radius: 4px;
    background: var(--bg);
    color: var(--text2);
    cursor: pointer;
}

.preset-btn:hover { background: var(--bg3); color: var(--text); }

.pid-controls {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 10px;
    margin-bottom: 12px;
}

.pid-param {
    display: flex;
    flex-direction: column;
    gap: 4px;
}

.pid-label {
    display: flex;
    justify-content: space-between;
    align-items: center;
}

.pid-label span {
    font-size: 12px;
    font-weight: 600;
}

.pid-val {
    font-size: 12px;
    color: var(--accent);
    font-variant-numeric: tabular-nums;
    min-width: 45px;
    text-align: right;
}

.pid-slider {
    -webkit-appearance: none;
    appearance: none;
    width: 100%;
    height: 6px;
    background: var(--bg);
    border-radius: 3px;
    outline: none;
}

.pid-slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 16px; height: 16px;
    border-radius: 50%;
    background: var(--accent);
    cursor: pointer;
}

.pid-input {
    width: 100%;
    padding: 4px 6px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 4px;
    color: var(--text);
    font-size: 12px;
    text-align: center;
}

.pid-actions {
    display: flex;
    gap: 6px;
    margin-bottom: 12px;
    flex-wrap: wrap;
}

.pid-actions .btn { font-size: 11px; padding: 4px 10px; }

.btn-autotune {
    background: linear-gradient(135deg, #6366f1, #8b5cf6);
    border-color: #6366f1;
    color: white;
    font-weight: 600;
}

.btn-autotune:hover { background: linear-gradient(135deg, #4f46e5, #7c3aed); }

.btn-autotune:disabled {
    opacity: 0.5;
    cursor: not-allowed;
}

.autotune-status {
    display: none;
    margin-bottom: 10px;
    padding: 8px 12px;
    background: var(--bg);
    border: 1px solid #6366f1;
    border-radius: 8px;
    font-size: 12px;
}

.autotune-status .at-msg { color: var(--text); margin-bottom: 6px; }

.at-progress-bar {
    height: 4px;
    background: var(--border);
    border-radius: 2px;
    overflow: hidden;
}

.at-progress-fill {
    height: 100%;
    background: linear-gradient(90deg, #6366f1, #8b5cf6);
    border-radius: 2px;
    transition: width 0.3s;
    width: 0%;
}

.chart-container {
    height: 200px;
    background: var(--bg);
    border-radius: 8px;
    border: 1px solid var(--border);
    padding: 8px;
    margin-bottom: 8px;
}

.pid-chart-container {
    height: 140px;
    background: var(--bg);
    border-radius: 8px;
    border: 1px solid var(--border);
    padding: 8px;
}

.bottom-bar {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 8px 24px;
    background: var(--bg2);
    border-top: 1px solid var(--border);
    font-size: 12px;
    color: var(--text2);
    position: fixed;
    bottom: 0;
    left: 0;
    right: 0;
}

.config-export {
    display: flex;
    gap: 8px;
}

/* Robot diagram */
.robot-diagram {
    display: none; /* Hidden on small screens, shown in header on wide */
}

/* Motor model / Bode */
.model-bar {
    display: flex;
    align-items: center;
    gap: 14px;
    padding: 8px 24px;
    background: #12151f;
    border-bottom: 1px solid var(--border);
    flex-wrap: wrap;
    font-size: 12px;
    color: var(--text2);
}
.model-bar label { color: var(--text2); white-space: nowrap; }
.model-bar input {
    width: 80px;
    padding: 4px 8px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 5px;
    color: var(--text);
    font-size: 12px;
    text-align: center;
}
.model-tf {
    font-family: monospace;
    font-size: 11px;
    color: #6366f1;
    padding: 3px 8px;
    background: #1e1f35;
    border-radius: 4px;
    border: 1px solid #3d3f70;
}
.bode-toggle {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 6px 12px;
    margin: 8px 0 0 0;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    cursor: pointer;
    font-size: 12px;
    color: var(--text2);
    width: 100%;
    text-align: left;
    transition: all 0.2s;
}
.bode-toggle:hover { background: var(--bg3); color: var(--text); }
.bode-toggle .arrow { transition: transform 0.2s; }
.bode-toggle.open .arrow { transform: rotate(90deg); }
.bode-section { padding: 0; }
.bode-content {
    display: none;
    padding: 10px 0 4px 0;
}
.bode-metrics {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 6px;
    margin-bottom: 8px;
}
.bode-metric {
    text-align: center;
    padding: 6px 4px;
    background: var(--bg);
    border-radius: 6px;
    border: 1px solid var(--border);
}
.bode-metric-label { font-size: 9px; text-transform: uppercase; letter-spacing: 0.5px; color: var(--text2); margin-bottom: 2px; }
.bode-metric-value { font-size: 14px; font-weight: 700; font-variant-numeric: tabular-nums; }
.bode-mag-container { height: 160px; background: var(--bg); border-radius: 6px; border: 1px solid var(--border); padding: 6px; margin-bottom: 6px; }
.bode-phase-container { height: 130px; background: var(--bg); border-radius: 6px; border: 1px solid var(--border); padding: 6px; }
</style>
</head>
<body>

<div class="header">
    <h1><span>VECTOR</span>NAV — PID Tuning Dashboard</h1>
    <div class="header-status">
        <div class="status-dot" id="statusDot"></div>
        <span id="statusText" style="font-size:13px;">Connected</span>
    </div>
</div>

<div class="global-controls">
    <button class="btn btn-danger" onclick="emergencyStop()" title="Stop all motors immediately">
        &#x26A0; EMERGENCY STOP
    </button>
    <button class="btn btn-success" onclick="enableAll(true)">Enable All</button>
    <button class="btn" onclick="enableAll(false)">Disable All</button>
    <button class="btn" onclick="resetAllPID()">Reset All PID</button>
    <button class="btn btn-autotune" onclick="autotuneAll()">Auto-Tune All (ZN)</button>
    <button class="btn" onclick="exportConfig()">Export YAML</button>
    <div class="global-rpm-input">
        <label>All Motors RPM:</label>
        <input type="number" id="globalRpm" value="0" step="5" min="-300" max="300">
        <button class="btn" onclick="setAllTarget()">Set</button>
    </div>
</div>

<div class="model-bar">
    <strong style="color:var(--text)">Motor Model</strong>
    <span style="color:var(--text2)">JGA25-370 12V 280RPM</span>
    <label>Km (m/s/PWM)
        <input type="number" id="modelKm" value="1.0" step="0.01" min="0.1" max="5" onchange="onModelChange()">
    </label>
    <label>τ (s)
        <input type="number" id="modelTau" value="0.15" step="0.005" min="0.01" max="2" onchange="onModelChange()">
    </label>
    <div class="model-tf" id="modelTf">G(s) = 1.00000 / (0.15000s + 1)</div>
    <span style="color:var(--text2);font-size:11px;">Bode plots update live with PID sliders</span>
</div>

<div class="dashboard" id="dashboard"></div>

<div class="bottom-bar">
    <span>Control: 50 Hz | Data: ~25 Hz | Wheel: R=34mm, 225 ticks/rev</span>
    <div class="config-export">
        <span id="loopRate">Loop: --</span>
    </div>
</div>

<script>
const MOTORS = ['LF', 'LR', 'RF', 'RR'];
const LABELS = ['Left Front', 'Left Rear', 'Right Front', 'Right Rear'];
const COLORS = ['#3b82f6', '#8b5cf6', '#10b981', '#f59e0b'];
const MAX_POINTS = 200;

const socket = io();
const charts = {};
const pidCharts = {};
const motorData = {};

// Init data store
MOTORS.forEach(m => {
    motorData[m] = {
        times: [], rpms: [], targets: [], errors: [],
        ps: [], is: [], ds: [], pwms: []
    };
});

// Build cards
const dashboard = document.getElementById('dashboard');
MOTORS.forEach((m, idx) => {
    const card = document.createElement('div');
    card.className = 'motor-card';
    card.id = `card-${m}`;
    card.innerHTML = `
        <div class="motor-header">
            <div class="motor-title">
                <div class="motor-dot" style="background:${COLORS[idx]}"></div>
                <div>
                    <h3>${LABELS[idx]}</h3>
                    <span>${m} Motor</span>
                </div>
            </div>
            <label class="motor-toggle">
                <input type="checkbox" id="toggle-${m}" onchange="toggleMotor('${m}', this.checked)">
                <div class="toggle-slider"></div>
            </label>
        </div>
        <div class="motor-body">
            <div class="metrics-row">
                <div class="metric">
                    <div class="metric-label">Current RPM</div>
                    <div class="metric-value" id="rpm-${m}" style="color:${COLORS[idx]}">0.0</div>
                    <div class="metric-unit">RPM</div>
                </div>
                <div class="metric">
                    <div class="metric-label">Target RPM</div>
                    <div class="metric-value" id="trpm-${m}">0.0</div>
                    <div class="metric-unit">RPM</div>
                </div>
                <div class="metric">
                    <div class="metric-label">Error</div>
                    <div class="metric-value" id="err-${m}" style="font-size:16px;">0.0</div>
                    <div class="metric-unit">RPM</div>
                </div>
                <div class="metric">
                    <div class="metric-label">PWM</div>
                    <div class="metric-value" id="pwm-${m}" style="font-size:16px;">0.00</div>
                    <div class="metric-unit">duty</div>
                </div>
            </div>
            <div class="target-row">
                <label>Target RPM:</label>
                <input type="number" class="target-input" id="target-${m}" value="0" step="5" min="-300" max="300"
                       onchange="setTarget('${m}', this.value)">
                <div class="target-presets">
                    <button class="preset-btn" onclick="setTarget('${m}', 0)">0</button>
                    <button class="preset-btn" onclick="setTarget('${m}', 30)">30</button>
                    <button class="preset-btn" onclick="setTarget('${m}', 60)">60</button>
                    <button class="preset-btn" onclick="setTarget('${m}', 100)">100</button>
                    <button class="preset-btn" onclick="setTarget('${m}', 150)">150</button>
                    <button class="preset-btn" onclick="setTarget('${m}', -60)">-60</button>
                </div>
            </div>
            <div class="pid-controls">
                <div class="pid-param">
                    <div class="pid-label">
                        <span>Kp</span>
                        <span class="pid-val" id="kpval-${m}">0.00000</span>
                    </div>
                    <input type="range" class="pid-slider" id="kp-${m}" min="0" max="5" step="0.00001" value="0"
                           oninput="updatePID('${m}')">
                    <input type="number" class="pid-input" id="kpin-${m}" value="0.00000" step="0.00001" min="0" max="20"
                           onchange="setPIDFromInput('${m}')">
                </div>
                <div class="pid-param">
                    <div class="pid-label">
                        <span>Ki</span>
                        <span class="pid-val" id="kival-${m}">0.00000</span>
                    </div>
                    <input type="range" class="pid-slider" id="ki-${m}" min="0" max="2" step="0.00001" value="0"
                           oninput="updatePID('${m}')">
                    <input type="number" class="pid-input" id="kiin-${m}" value="0.00000" step="0.00001" min="0" max="20"
                           onchange="setPIDFromInput('${m}')">
                </div>
                <div class="pid-param">
                    <div class="pid-label">
                        <span>Kd</span>
                        <span class="pid-val" id="kdval-${m}">0.00000</span>
                    </div>
                    <input type="range" class="pid-slider" id="kd-${m}" min="0" max="1" step="0.00001" value="0"
                           oninput="updatePID('${m}')">
                    <input type="number" class="pid-input" id="kdin-${m}" value="0.00000" step="0.00001" min="0" max="5"
                           onchange="setPIDFromInput('${m}')">
                </div>
            </div>
            <div class="pid-actions">
                <button class="btn" onclick="resetPID('${m}')">Reset PID</button>
                <button class="btn" onclick="flipEncoder('${m}')">Flip Encoder</button>
                <button class="btn" onclick="copyPIDFrom('${m}')">Copy to All</button>
                <button class="btn btn-autotune" id="atbtn-${m}" onclick="startAutotune('${m}')">Auto-Tune (ZN)</button>
            </div>
            <div class="autotune-status" id="atstatus-${m}">
                <div class="at-msg" id="atmsg-${m}">Initializing...</div>
                <div class="at-progress-bar">
                    <div class="at-progress-fill" id="atprog-${m}"></div>
                </div>
            </div>
            <div class="chart-container">
                <canvas id="chart-${m}"></canvas>
            </div>
            <div class="pid-chart-container">
                <canvas id="pidchart-${m}"></canvas>
            </div>
            <div class="bode-section">
                <button class="bode-toggle" id="bodetoggle-${m}" onclick="toggleBode('${m}')">
                    <span class="arrow">▶</span> Bode Plot — Frequency Response
                </button>
                <div class="bode-content" id="bode-content-${m}">
                    <div class="bode-metrics">
                        <div class="bode-metric">
                            <div class="bode-metric-label">Phase Margin</div>
                            <div class="bode-metric-value" id="bode-pm-${m}" style="color:#10b981">—</div>
                        </div>
                        <div class="bode-metric">
                            <div class="bode-metric-label">Gain Margin</div>
                            <div class="bode-metric-value" id="bode-gm-${m}" style="color:#3b82f6">—</div>
                        </div>
                        <div class="bode-metric">
                            <div class="bode-metric-label">Bandwidth</div>
                            <div class="bode-metric-value" id="bode-bw-${m}" style="color:#f59e0b">—</div>
                        </div>
                        <div class="bode-metric">
                            <div class="bode-metric-label">Crossover</div>
                            <div class="bode-metric-value" id="bode-gc-${m}" style="color:#8b5cf6">—</div>
                        </div>
                    </div>
                    <div class="bode-mag-container">
                        <canvas id="bode-mag-${m}"></canvas>
                    </div>
                    <div class="bode-phase-container">
                        <canvas id="bode-phase-${m}"></canvas>
                    </div>
                </div>
            </div>
        </div>
    `;
    dashboard.appendChild(card);
});

// Create charts
MOTORS.forEach((m, idx) => {
    const ctx = document.getElementById(`chart-${m}`).getContext('2d');
    charts[m] = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'Actual RPM',
                    data: [],
                    borderColor: COLORS[idx],
                    borderWidth: 2,
                    pointRadius: 0,
                    tension: 0.3,
                    fill: false,
                },
                {
                    label: 'Target RPM',
                    data: [],
                    borderColor: '#ffffff44',
                    borderWidth: 1.5,
                    borderDash: [6, 4],
                    pointRadius: 0,
                    tension: 0,
                    fill: false,
                },
                {
                    label: 'Error',
                    data: [],
                    borderColor: '#ef444488',
                    borderWidth: 1,
                    pointRadius: 0,
                    tension: 0.3,
                    fill: {
                        target: 'origin',
                        above: 'rgba(239,68,68,0.08)',
                        below: 'rgba(239,68,68,0.08)',
                    },
                },
            ],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: { intersect: false, mode: 'index' },
            scales: {
                x: { display: false },
                y: {
                    grid: { color: '#2e334822' },
                    ticks: { color: '#8b8fa8', font: { size: 10 } },
                },
            },
            plugins: {
                legend: {
                    display: true,
                    position: 'top',
                    labels: {
                        color: '#8b8fa8',
                        font: { size: 10 },
                        boxWidth: 12,
                        padding: 8,
                    },
                },
            },
        },
    });

    // PID component chart
    const pctx = document.getElementById(`pidchart-${m}`).getContext('2d');
    pidCharts[m] = new Chart(pctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'P',
                    data: [],
                    borderColor: '#3b82f6',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    tension: 0.3,
                },
                {
                    label: 'I',
                    data: [],
                    borderColor: '#10b981',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    tension: 0.3,
                },
                {
                    label: 'D',
                    data: [],
                    borderColor: '#f59e0b',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    tension: 0.3,
                },
                {
                    label: 'PWM',
                    data: [],
                    borderColor: '#ef4444',
                    borderWidth: 1.5,
                    pointRadius: 0,
                    tension: 0.3,
                    borderDash: [4, 2],
                },
            ],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: { intersect: false, mode: 'index' },
            scales: {
                x: { display: false },
                y: {
                    grid: { color: '#2e334822' },
                    ticks: { color: '#8b8fa8', font: { size: 10 } },
                    min: -1.2,
                    max: 1.2,
                },
            },
            plugins: {
                legend: {
                    display: true,
                    position: 'top',
                    labels: {
                        color: '#8b8fa8',
                        font: { size: 10 },
                        boxWidth: 10,
                        padding: 6,
                    },
                },
            },
        },
    });
});

// Socket handlers
let lastDataTime = 0;

socket.on('connect', () => {
    document.getElementById('statusDot').classList.remove('disconnected');
    document.getElementById('statusText').textContent = 'Connected';
});

socket.on('disconnect', () => {
    document.getElementById('statusDot').classList.add('disconnected');
    document.getElementById('statusText').textContent = 'Disconnected';
});

socket.on('state', (state) => {
    MOTORS.forEach(m => {
        if (state[m]) {
            document.getElementById(`toggle-${m}`).checked = state[m].enabled;
            document.getElementById(`target-${m}`).value = state[m].target_rpm;
            updateSliders(m, state[m].kp, state[m].ki, state[m].kd);
        }
    });
});

socket.on('motor_data', (data) => {
    const now = performance.now() / 1000;
    document.getElementById('loopRate').textContent =
        `Data: ${((now - lastDataTime) * 1000).toFixed(0)}ms`;
    lastDataTime = now;

    MOTORS.forEach(m => {
        const d = data.motors[m];
        if (!d) return;

        // Update metrics
        document.getElementById(`rpm-${m}`).textContent = d.rpm.toFixed(1);
        document.getElementById(`trpm-${m}`).textContent = d.target_rpm.toFixed(1);
        document.getElementById(`err-${m}`).textContent = d.error.toFixed(1);
        document.getElementById(`pwm-${m}`).textContent = d.pwm.toFixed(2);

        // Color error by magnitude
        const errEl = document.getElementById(`err-${m}`);
        const absErr = Math.abs(d.error);
        if (absErr < 5) errEl.style.color = '#10b981';
        else if (absErr < 15) errEl.style.color = '#f59e0b';
        else errEl.style.color = '#ef4444';

        // Push chart data
        const md = motorData[m];
        const t = md.times.length;
        md.times.push(t);
        md.rpms.push(d.rpm);
        md.targets.push(d.target_rpm);
        md.errors.push(d.error);
        md.ps.push(d.p);
        md.is.push(d.i);
        md.ds.push(d.d);
        md.pwms.push(d.pwm);

        // Trim
        if (md.times.length > MAX_POINTS) {
            md.times.shift(); md.rpms.shift(); md.targets.shift();
            md.errors.shift(); md.ps.shift(); md.is.shift();
            md.ds.shift(); md.pwms.shift();
        }

        // Update RPM chart
        const c = charts[m];
        c.data.labels = md.times;
        c.data.datasets[0].data = md.rpms;
        c.data.datasets[1].data = md.targets;
        c.data.datasets[2].data = md.errors;
        c.update('none');

        // Update PID chart
        const pc = pidCharts[m];
        pc.data.labels = md.times;
        pc.data.datasets[0].data = md.ps;
        pc.data.datasets[1].data = md.is;
        pc.data.datasets[2].data = md.ds;
        pc.data.datasets[3].data = md.pwms;
        pc.update('none');
    });
});

socket.on('stopped', () => {
    MOTORS.forEach(m => {
        document.getElementById(`toggle-${m}`).checked = false;
        document.getElementById(`target-${m}`).value = 0;
    });
    document.getElementById('globalRpm').value = 0;
});

socket.on('enc_flipped', (data) => {
    console.log(`Encoder ${data.motor} flipped: ${data.flip}`);
});

socket.on('pid_copied', (data) => {
    data.to.forEach(m => {
        updateSliders(m, data.kp, data.ki, data.kd);
    });
});

// Control functions
function toggleMotor(m, enabled) {
    socket.emit('toggle_motor', { motor: m, enabled: enabled });
}

function setTarget(m, rpm) {
    rpm = parseFloat(rpm) || 0;
    document.getElementById(`target-${m}`).value = rpm;
    socket.emit('set_target', { motor: m, rpm: rpm });
}

function updatePID(m) {
    const kp = parseFloat(document.getElementById(`kp-${m}`).value);
    const ki = parseFloat(document.getElementById(`ki-${m}`).value);
    const kd = parseFloat(document.getElementById(`kd-${m}`).value);
    document.getElementById(`kpval-${m}`).textContent = kp.toFixed(5);
    document.getElementById(`kival-${m}`).textContent = ki.toFixed(5);
    document.getElementById(`kdval-${m}`).textContent = kd.toFixed(5);
    document.getElementById(`kpin-${m}`).value = kp.toFixed(5);
    document.getElementById(`kiin-${m}`).value = ki.toFixed(5);
    document.getElementById(`kdin-${m}`).value = kd.toFixed(5);
    socket.emit('set_pid', { motor: m, kp, ki, kd });
}

function setPIDFromInput(m) {
    const kp = parseFloat(document.getElementById(`kpin-${m}`).value) || 0;
    const ki = parseFloat(document.getElementById(`kiin-${m}`).value) || 0;
    const kd = parseFloat(document.getElementById(`kdin-${m}`).value) || 0;
    updateSliders(m, kp, ki, kd);
    socket.emit('set_pid', { motor: m, kp, ki, kd });
}

function updateSliders(m, kp, ki, kd) {
    document.getElementById(`kp-${m}`).value = kp;
    document.getElementById(`ki-${m}`).value = ki;
    document.getElementById(`kd-${m}`).value = kd;
    document.getElementById(`kpin-${m}`).value = parseFloat(kp).toFixed(5);
    document.getElementById(`kiin-${m}`).value = parseFloat(ki).toFixed(5);
    document.getElementById(`kdin-${m}`).value = parseFloat(kd).toFixed(5);
    document.getElementById(`kpval-${m}`).textContent = parseFloat(kp).toFixed(5);
    document.getElementById(`kival-${m}`).textContent = parseFloat(ki).toFixed(5);
    document.getElementById(`kdval-${m}`).textContent = parseFloat(kd).toFixed(5);
}

function resetPID(m) {
    updateSliders(m, 0.0, 0.0, 0.0);
    socket.emit('set_pid', { motor: m, kp: 0.0, ki: 0.0, kd: 0.0 });
}

function flipEncoder(m) {
    socket.emit('flip_encoder', { motor: m });
}

function copyPIDFrom(m) {
    const others = MOTORS.filter(x => x !== m);
    socket.emit('copy_pid', { from: m, to: others });
}

function emergencyStop() {
    socket.emit('emergency_stop');
}

function enableAll(en) {
    MOTORS.forEach(m => {
        document.getElementById(`toggle-${m}`).checked = en;
    });
    socket.emit('enable_all', { enabled: en });
}

function setAllTarget() {
    const rpm = parseFloat(document.getElementById('globalRpm').value) || 0;
    MOTORS.forEach(m => {
        document.getElementById(`target-${m}`).value = rpm;
    });
    socket.emit('set_all_target', { rpm });
}

function resetAllPID() {
    MOTORS.forEach(m => resetPID(m));
}

function exportConfig() {
    let yaml = '# PID gains tuned via dashboard\nmotor_driver:\n  ros__parameters:\n';
    // Use LF as reference (or show all)
    const gains = {};
    MOTORS.forEach(m => {
        gains[m] = {
            kp: parseFloat(document.getElementById(`kpin-${m}`).value),
            ki: parseFloat(document.getElementById(`kiin-${m}`).value),
            kd: parseFloat(document.getElementById(`kdin-${m}`).value),
        };
    });

    // Check if all same
    const allSame = MOTORS.every(m =>
        gains[m].kp === gains.LF.kp &&
        gains[m].ki === gains.LF.ki &&
        gains[m].kd === gains.LF.kd
    );

    if (allSame) {
        yaml += `    pid_kp: ${gains.LF.kp}\n`;
        yaml += `    pid_ki: ${gains.LF.ki}\n`;
        yaml += `    pid_kd: ${gains.LF.kd}\n`;
    } else {
        MOTORS.forEach(m => {
            yaml += `    # ${m}\n`;
            yaml += `    pid_kp_${m.toLowerCase()}: ${gains[m].kp}\n`;
            yaml += `    pid_ki_${m.toLowerCase()}: ${gains[m].ki}\n`;
            yaml += `    pid_kd_${m.toLowerCase()}: ${gains[m].kd}\n`;
        });
    }

    // Copy to clipboard and show
    navigator.clipboard.writeText(yaml).then(() => {
        alert('YAML copied to clipboard!\n\n' + yaml);
    }).catch(() => {
        prompt('Copy this YAML config:', yaml);
    });
}

// ---------- Auto-Tune ----------
function startAutotune(m) {
    if (!confirm(`Run Ziegler-Nichols auto-tune on ${m}?\n\nThe motor will oscillate for ~10 seconds. Make sure the wheel is free to spin.`))
        return;
    document.getElementById(`atbtn-${m}`).disabled = true;
    document.getElementById(`atbtn-${m}`).textContent = 'Tuning...';
    document.getElementById(`atstatus-${m}`).style.display = 'block';
    document.getElementById(`atmsg-${m}`).textContent = 'Starting auto-tune...';
    document.getElementById(`atprog-${m}`).style.width = '0%';
    document.getElementById(`toggle-${m}`).checked = false;
    socket.emit('autotune', { motor: m });
}

function autotuneAll() {
    if (!confirm('Run Ziegler-Nichols auto-tune on ALL 4 motors sequentially?\n\nThis will take ~45 seconds. Make sure all wheels are free to spin.'))
        return;
    MOTORS.forEach(m => {
        document.getElementById(`atbtn-${m}`).disabled = true;
        document.getElementById(`atbtn-${m}`).textContent = 'Queued...';
        document.getElementById(`atstatus-${m}`).style.display = 'block';
        document.getElementById(`atmsg-${m}`).textContent = 'Waiting in queue...';
        document.getElementById(`atprog-${m}`).style.width = '0%';
        document.getElementById(`toggle-${m}`).checked = false;
    });
    socket.emit('autotune_all');
}

socket.on('autotune_status', (data) => {
    const m = data.motor;
    document.getElementById(`atmsg-${m}`).textContent = data.msg;
    if (data.progress !== null && data.progress !== undefined) {
        document.getElementById(`atprog-${m}`).style.width = data.progress + '%';
    }
    document.getElementById(`atbtn-${m}`).textContent = 'Tuning...';
});

socket.on('autotune_done', (data) => {
    const m = data.motor;
    document.getElementById(`atbtn-${m}`).disabled = false;
    document.getElementById(`atbtn-${m}`).textContent = 'Auto-Tune (ZN)';
    if (data.error) {
        document.getElementById(`atmsg-${m}`).textContent = 'Failed: ' + data.error;
    } else {
        document.getElementById(`atmsg-${m}`).textContent =
            `Done! Kp=${data.kp.toFixed(5)}  Ki=${data.ki.toFixed(5)}  Kd=${data.kd.toFixed(5)}`;
        document.getElementById(`atprog-${m}`).style.width = '100%';
        updateSliders(m, data.kp, data.ki, data.kd);
    }
    // Hide status after 8 seconds
    setTimeout(() => {
        document.getElementById(`atstatus-${m}`).style.display = 'none';
    }, 8000);
});

// Keyboard shortcut: Escape = emergency stop
document.addEventListener('keydown', (e) => {
    if (e.code === 'Escape') {
        emergencyStop();
    }
});

// ─────────────────────────────────────────────────────────────
// Bode Plot — JGA25-370 motor model + PID frequency response
//
// Plant (velocity loop):  G(s) = Km / (τ·s + 1)
//   Km  = 1.0 m/s per PWM  (280 RPM × 2π × 0.034 m / 60 ≈ 1.0 m/s at full duty)
//   τ   = 0.15 s            (empirical mechanical time constant for JGA25-370)
//
// Controller:  C(s) = Kp + Ki/s + Kd·s
//
// Open-loop:   L(s) = C(s)·G(s)
// Closed-loop: T(s) = L(s) / (1 + L(s))
//
// Metrics derived:
//   Phase Margin  = 180° + ∠L(jω) at gain crossover (|L|=0 dB)
//   Gain Margin   = −|L(jω)| dB at phase crossover (∠L = −180°)
//   Bandwidth     = closed-loop −3 dB frequency
//   Crossover     = gain crossover frequency (Hz)
// ─────────────────────────────────────────────────────────────

const bodeChartsMag   = {};
const bodeChartsPhase = {};
const bodeOpen = {};   // which motors have bode panel open

// Complex arithmetic helpers
function cMul(a, b) { return {re: a.re*b.re - a.im*b.im, im: a.re*b.im + a.im*b.re}; }
function cAdd(a, b) { return {re: a.re+b.re, im: a.im+b.im}; }
function cDiv(a, b) {
    const d = b.re*b.re + b.im*b.im;
    return {re:(a.re*b.re+a.im*b.im)/d, im:(a.im*b.re-a.re*b.im)/d};
}
function cMag(a)   { return Math.sqrt(a.re*a.re + a.im*a.im); }
function cPhase(a) { return Math.atan2(a.im, a.re) * 180/Math.PI; }
function cMagDb(a) { const m = cMag(a); return m > 1e-12 ? 20*Math.log10(m) : -240; }

function evalPlant(omega, Km, tau) {
    // G(jω) = Km / (1 + jω·τ)
    return cDiv({re: Km, im: 0}, {re: 1, im: omega*tau});
}

function evalPID(omega, kp, ki, kd) {
    // C(jω) = kp + ki/(jω) + kd·jω
    //       = kp  +  j·(kd·ω − ki/ω)
    if (omega < 1e-9) return {re: 1e9, im: 0};
    return {re: kp, im: kd*omega - ki/omega};
}

function computeBode(kp, ki, kd) {
    const Km  = parseFloat(document.getElementById('modelKm').value)  || 1.0;
    const tau = parseFloat(document.getElementById('modelTau').value) || 0.15;

    const N = 400;
    const fMin = 0.01, fMax = 200;   // Hz — wide enough for fast motors
    const freqs = [], logFreqs = [];
    const magPlant = [], phasePlant = [];
    const magOL = [],    phaseOL = [];
    const magCL = [],    phaseCL = [];

    for (let i = 0; i < N; i++) {
        const f = fMin * Math.pow(fMax/fMin, i/(N-1));
        const omega = 2*Math.PI*f;
        freqs.push(f);
        logFreqs.push(f.toFixed(4));

        const G = evalPlant(omega, Km, tau);
        const C = evalPID(omega, kp, ki, kd);
        const L = cMul(C, G);
        const T = cDiv(L, cAdd(L, {re:1, im:0}));

        magPlant.push(cMagDb(G));
        phasePlant.push(cPhase(G));
        magOL.push(cMagDb(L));
        phaseOL.push(cPhase(L));
        magCL.push(cMagDb(T));
        phaseCL.push(cPhase(T));
    }

    // Gain crossover: where |L(jω)| crosses 0 dB (high→low)
    let phaseMargin = null, gainCrossFreq = null;
    for (let i = 0; i < N-1; i++) {
        if (magOL[i] >= 0 && magOL[i+1] < 0) {
            const t = magOL[i] / (magOL[i] - magOL[i+1]);
            gainCrossFreq = freqs[i] + t*(freqs[i+1]-freqs[i]);
            const ph = phaseOL[i] + t*(phaseOL[i+1]-phaseOL[i]);
            phaseMargin = 180 + ph;
            break;
        }
    }

    // Phase crossover: where ∠L(jω) crosses −180°
    let gainMargin = null, phaseCrossFreq = null;
    for (let i = 0; i < N-1; i++) {
        if (phaseOL[i] >= -180 && phaseOL[i+1] < -180) {
            const t = (phaseOL[i]+180) / (phaseOL[i] - phaseOL[i+1]);
            phaseCrossFreq = freqs[i] + t*(freqs[i+1]-freqs[i]);
            const mg = magOL[i] + t*(magOL[i+1]-magOL[i]);
            gainMargin = -mg;
            break;
        }
    }

    // Bandwidth: closed-loop −3 dB (where |T| drops below −3 dB)
    let bandwidth = null;
    const bw3db = magCL[0] - 3;
    for (let i = 1; i < N; i++) {
        if (magCL[i] < bw3db) {
            const t = (magCL[i-1]-bw3db) / (magCL[i-1]-magCL[i]);
            bandwidth = freqs[i-1] + t*(freqs[i]-freqs[i-1]);
            break;
        }
    }

    return { freqs, logFreqs,
             magPlant, phasePlant,
             magOL, phaseOL,
             magCL, phaseCL,
             phaseMargin, gainCrossFreq,
             gainMargin, phaseCrossFreq,
             bandwidth };
}

function buildBodeCharts(m) {
    const magCtx   = document.getElementById(`bode-mag-${m}`).getContext('2d');
    const phaseCtx = document.getElementById(`bode-phase-${m}`).getContext('2d');

    const baseOpts = {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
            x: {
                type: 'logarithmic',
                min: 0.01, max: 200,
                grid: {color:'#2e334833'},
                ticks: {
                    color:'#8b8fa8', font:{size:9},
                    callback: v => [0.01,0.1,1,10,100].includes(Number(v.toFixed(2))) ? v+'Hz' : '',
                },
            },
        },
        plugins: {
            legend: { display:true, position:'top',
                labels:{color:'#8b8fa8', font:{size:9}, boxWidth:10, padding:6} },
        },
        interaction: {intersect:false, mode:'index'},
    };

    bodeChartsMag[m] = new Chart(magCtx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {label:'Plant G', data:[], borderColor:'#6366f1', borderWidth:1.5,
                 pointRadius:0, tension:0, borderDash:[4,3]},
                {label:'Open-loop L', data:[], borderColor:'#f59e0b', borderWidth:2,
                 pointRadius:0, tension:0},
                {label:'Closed-loop T', data:[], borderColor:'#10b981', borderWidth:2,
                 pointRadius:0, tension:0},
            ],
        },
        options: {
            ...baseOpts,
            scales: {
                ...baseOpts.scales,
                y: {
                    grid:{color:'#2e334833'},
                    ticks:{color:'#8b8fa8', font:{size:9},
                           callback: v => v+'dB'},
                    title:{display:true, text:'Magnitude (dB)',
                           color:'#8b8fa8', font:{size:9}},
                },
            },
            plugins: {
                ...baseOpts.plugins,
                annotation: {},   // placeholder; real annotation via custom drawing
            },
        },
    });

    bodeChartsPhase[m] = new Chart(phaseCtx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {label:'Plant G', data:[], borderColor:'#6366f133', borderWidth:1.5,
                 pointRadius:0, tension:0, borderDash:[4,3]},
                {label:'Open-loop L', data:[], borderColor:'#f59e0b', borderWidth:2,
                 pointRadius:0, tension:0},
                {label:'−180° line', data:[], borderColor:'#ef444488', borderWidth:1,
                 pointRadius:0, tension:0, borderDash:[6,3]},
            ],
        },
        options: {
            ...baseOpts,
            scales: {
                ...baseOpts.scales,
                y: {
                    min: -270, max: 90,
                    grid:{color:'#2e334833'},
                    ticks:{color:'#8b8fa8', font:{size:9},
                           callback: v => v+'°'},
                    title:{display:true, text:'Phase (°)',
                           color:'#8b8fa8', font:{size:9}},
                },
            },
        },
    });
}

function renderBode(m) {
    const kp = parseFloat(document.getElementById(`kpin-${m}`).value) || 0;
    const ki = parseFloat(document.getElementById(`kiin-${m}`).value) || 0;
    const kd = parseFloat(document.getElementById(`kdin-${m}`).value) || 0;

    const d = computeBode(kp, ki, kd);

    // Update magnitude chart
    const mc = bodeChartsMag[m];
    mc.data.labels = d.freqs;
    mc.data.datasets[0].data = d.freqs.map((f,i)=>({x:f,y:d.magPlant[i]}));
    mc.data.datasets[1].data = d.freqs.map((f,i)=>({x:f,y:d.magOL[i]}));
    mc.data.datasets[2].data = d.freqs.map((f,i)=>({x:f,y:d.magCL[i]}));
    mc.update('none');

    // Update phase chart (−180° reference line)
    const pc = bodeChartsPhase[m];
    pc.data.labels = d.freqs;
    pc.data.datasets[0].data = d.freqs.map((f,i)=>({x:f,y:d.phasePlant[i]}));
    pc.data.datasets[1].data = d.freqs.map((f,i)=>({x:f,y:d.phaseOL[i]}));
    pc.data.datasets[2].data = d.freqs.map(f=>({x:f,y:-180}));
    pc.update('none');

    // Update metrics
    const pmEl = document.getElementById(`bode-pm-${m}`);
    const gmEl = document.getElementById(`bode-gm-${m}`);
    const bwEl = document.getElementById(`bode-bw-${m}`);
    const gcEl = document.getElementById(`bode-gc-${m}`);

    if (d.phaseMargin !== null) {
        const pm = d.phaseMargin;
        pmEl.textContent = pm.toFixed(1) + '°';
        pmEl.style.color = pm >= 45 ? '#10b981' : pm >= 30 ? '#f59e0b' : '#ef4444';
    } else {
        pmEl.textContent = '∞';
        pmEl.style.color = '#10b981';
    }

    if (d.gainMargin !== null) {
        const gm = d.gainMargin;
        gmEl.textContent = gm.toFixed(1) + ' dB';
        gmEl.style.color = gm >= 10 ? '#10b981' : gm >= 6 ? '#f59e0b' : '#ef4444';
    } else {
        gmEl.textContent = '∞';
        gmEl.style.color = '#10b981';
    }

    if (d.bandwidth !== null) {
        bwEl.textContent = d.bandwidth < 1
            ? (d.bandwidth*1000).toFixed(1)+' mHz'
            : d.bandwidth.toFixed(2)+' Hz';
        bwEl.style.color = '#f59e0b';
    } else {
        bwEl.textContent = '< 0.01 Hz';
        bwEl.style.color = '#8b8fa8';
    }

    if (d.gainCrossFreq !== null) {
        gcEl.textContent = d.gainCrossFreq < 1
            ? (d.gainCrossFreq*1000).toFixed(1)+' mHz'
            : d.gainCrossFreq.toFixed(2)+' Hz';
        gcEl.style.color = '#8b5cf6';
    } else {
        gcEl.textContent = '< 0.01 Hz';
        gcEl.style.color = '#8b8fa8';
    }
}

function toggleBode(m) {
    const content = document.getElementById(`bode-content-${m}`);
    const btn = document.getElementById(`bodetoggle-${m}`);
    if (bodeOpen[m]) {
        content.style.display = 'none';
        btn.classList.remove('open');
        bodeOpen[m] = false;
    } else {
        content.style.display = 'block';
        btn.classList.add('open');
        bodeOpen[m] = true;
        if (!bodeChartsMag[m]) {
            buildBodeCharts(m);
        }
        renderBode(m);
    }
}

function onModelChange() {
    const Km  = parseFloat(document.getElementById('modelKm').value)  || 1.0;
    const tau = parseFloat(document.getElementById('modelTau').value) || 0.15;
    document.getElementById('modelTf').textContent =
        `G(s) = ${Km.toFixed(5)} / (${tau.toFixed(5)}s + 1)`;
    MOTORS.forEach(m => { if (bodeOpen[m]) renderBode(m); });
}

// Hook into updatePID and setPIDFromInput to live-refresh open Bode panels
const _origUpdatePID = updatePID;
updatePID = function(m) { _origUpdatePID(m); if (bodeOpen[m]) renderBode(m); };
const _origSetPIDFromInput = setPIDFromInput;
setPIDFromInput = function(m) { _origSetPIDFromInput(m); if (bodeOpen[m]) renderBode(m); };
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description='VECTOR NAV PID Tuning Dashboard')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=5000, help='Port to listen on')
    args = parser.parse_args()

    init_hardware()

    # Start control loop in background thread
    ctrl_thread = threading.Thread(target=control_loop, daemon=True)
    ctrl_thread.start()

    def signal_handler(sig, frame):
        print("\n[!] Shutting down...")
        shutdown_hardware()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"\n{'='*60}")
    print(f"  VECTOR NAV — PID Tuning Dashboard")
    print(f"  http://{args.host}:{args.port}")
    print(f"  Press Ctrl+C or ESC in browser to emergency stop")
    print(f"{'='*60}\n")

    try:
        socketio.run(app, host=args.host, port=args.port,
                     allow_unsafe_werkzeug=True, log_output=False)
    finally:
        shutdown_hardware()


if __name__ == '__main__':
    main()
