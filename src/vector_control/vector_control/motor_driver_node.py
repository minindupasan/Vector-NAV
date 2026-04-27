#!/usr/bin/env python3
"""
Motor Driver & Encoder Odometry Node for VECTOR NAV (Raspberry Pi).

Subscribes: /cmd_vel  (geometry_msgs/Twist)
Publishes:  /wheel/odom (nav_msgs/Odometry)
            /tf        (odom → base_link)

Hardware:
  - Two TB6612FNG motor drivers (4 motors, skid-steer as diff-drive)
  - Quadrature encoders on each wheel (Phase A + Phase B)
  - MPU6500 + HMC5883L IMU on I2C bus 1

Pin mapping matches test_motors.py.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros

import glob as _glob
import os
import gpiozero
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button
from gpiozero.pins.lgpio import LGPIOFactory

# Auto-detect the correct gpiochip.
# On Pi 5: /dev/gpiochip4 → gpiochip0 (symlink). In Docker the symlink
# may not exist, so we scan all available chips.
_factory_set = False
_available = sorted(_glob.glob('/dev/gpiochip*'))
# Try chips in descending order (Pi 5 chip4 > Pi 4 chip0)
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
    raise RuntimeError(
        f'Cannot open any gpiochip. Available: {_available}. '
        f'Make sure /dev/gpiochip* devices are passed to the container '
        f'and the container runs with --privileged.'
    )


# ---------------------------------------------------------------------------
# Pin definitions (matching test_motors.py)
# ---------------------------------------------------------------------------
# Motor Driver 1 (LF = Channel A, LR = Channel B)
# AIN1/AIN2 and BIN1/BIN2 swapped to correct motor direction
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

PWM_FREQ = 1000


class HBridgeMotor:
    """Single motor on a TB6612FNG H-bridge."""

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
        """Active short-brake: both H-bridge inputs HIGH with PWM at 100%."""
        self.in1.on(); self.in2.on()
        self.pwm.value = 1.0

    def stop(self):
        self.in1.off(); self.in2.off(); self.pwm.value = 0

    def close(self):
        self.stop()
        self.pwm.close(); self.in1.close(); self.in2.close()


class PIDController:
    """
    PI(D) controller with feedforward and conditional-integration anti-windup.

    Anti-windup uses conditional integration: the integrator is paused when
    the unsaturated output already exceeds the actuator limit AND further
    integration would push it deeper into saturation.
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

        # Conditional integration anti-windup
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


class QuadratureEncoder:
    """Encoder using Phase A pulse counting (magnitude only).

    At high RPM, Python callback latency causes Phase B direction reads
    to be unreliable. Instead, we count pulses for magnitude and use
    the motor command direction externally.
    """

    def __init__(self, pin_a, pin_b, ticks_per_rev, wheel_radius):
        self.ticks_per_rev = ticks_per_rev
        self.wheel_radius = wheel_radius
        self.meters_per_tick = (2.0 * math.pi * wheel_radius) / ticks_per_rev

        self._count = 0
        self._lock = threading.Lock()

        # Phase A triggers counting on rising edge (magnitude only)
        self._phase_a = Button(pin_a, pull_up=True, bounce_time=None, hold_time=None)
        self._phase_a.when_pressed = self._on_rising

        # Phase B kept for low-speed direction (hand-turning detection)
        self._phase_b = Button(pin_b, pull_up=True, bounce_time=None, hold_time=None)

    def _on_rising(self):
        with self._lock:
            self._count += 1

    def get_and_reset(self, direction_sign=1):
        """Return (ticks, distance_meters) since last call.

        direction_sign: +1 or -1, set by the caller based on motor command
        or low-speed Phase B reading.
        """
        with self._lock:
            c = self._count
            self._count = 0
        t = c * direction_sign
        return t, t * self.meters_per_tick

    def close(self):
        self._phase_a.close()
        self._phase_b.close()



def yaw_to_quaternion(yaw):
    """Convert yaw angle (rad) to geometry_msgs Quaternion."""
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # ---------- Parameters ----------
        self.declare_parameter('wheel_separation', 0.178)
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('ticks_per_rev', 225)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('max_motor_speed', 1.0)  # m/s at wheel
        self.declare_parameter('min_linear_speed', 0.07) # m/s min command
        self.declare_parameter('min_angular_speed', 1.0) # rad/s min command
        self.declare_parameter('min_pwm', 0.18)  # minimum PWM to overcome static friction
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)  # EKF publishes tf
        self.declare_parameter('max_accel', 2.0)  # m/s² acceleration limit
        self.declare_parameter('log_interval', 1.0)  # RPM log interval (s)
        self.declare_parameter('rear_rotation_scale', 1.0)  # scale rear wheel targets during rotation (0.7–1.0)

        # PID gains (tuned via dashboard)
        self.declare_parameter('pid_kp_lf', 0.37633)
        self.declare_parameter('pid_ki_lf', 6.25631)
        self.declare_parameter('pid_kd_lf', 0.0)
        self.declare_parameter('pid_kp_lr', 0.42333)
        self.declare_parameter('pid_ki_lr', 10.55251)
        self.declare_parameter('pid_kd_lr', 0.0)
        self.declare_parameter('pid_kp_rf', 0.39689)
        self.declare_parameter('pid_ki_rf', 6.59491)
        self.declare_parameter('pid_kd_rf', 0.0)
        self.declare_parameter('pid_kp_rr', 0.65185)
        self.declare_parameter('pid_ki_rr', 8.12396)
        self.declare_parameter('pid_kd_rr', 0.0)

        # EMA Filter alpha
        self.declare_parameter('ema_alpha', 0.35)

        # Encoder direction flips (1 or -1)
        self.declare_parameter('encoder_flip_lf', 1)
        self.declare_parameter('encoder_flip_lr', 1)
        self.declare_parameter('encoder_flip_rf', 1)
        self.declare_parameter('encoder_flip_rr', 1)

        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        ticks = self.get_parameter('ticks_per_rev').value
        self.control_rate = self.get_parameter('control_rate').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_speed = self.get_parameter('max_motor_speed').value
        self.min_linear_v = self.get_parameter('min_linear_speed').value
        self.min_angular_v = self.get_parameter('min_angular_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.max_accel = self.get_parameter('max_accel').value
        self.log_interval = self.get_parameter('log_interval').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.rear_rotation_scale = self.get_parameter('rear_rotation_scale').value

        self.enc_flips = [
            self.get_parameter('encoder_flip_lf').value,
            self.get_parameter('encoder_flip_lr').value,
            self.get_parameter('encoder_flip_rf').value,
            self.get_parameter('encoder_flip_rr').value,
        ]

        # ---------- Hardware: standby pins ----------
        self.stby1 = DigitalOutputDevice(MD1_STBY)
        self.stby2 = DigitalOutputDevice(MD2_STBY)
        self.stby1.on()
        self.stby2.on()

        # ---------- Motors ----------
        self.motor_lf = HBridgeMotor(MD1_PWMA, MD1_AIN1, MD1_AIN2)
        self.motor_lr = HBridgeMotor(MD1_PWMB, MD1_BIN1, MD1_BIN2)
        self.motor_rf = HBridgeMotor(MD2_PWMA, MD2_AIN1, MD2_AIN2)
        self.motor_rr = HBridgeMotor(MD2_PWMB, MD2_BIN1, MD2_BIN2)

        # ---------- Encoders ----------
        self.enc_lf = QuadratureEncoder(ENC_LF_A, ENC_LF_B, ticks, self.wheel_rad)
        self.enc_lr = QuadratureEncoder(ENC_LR_A, ENC_LR_B, ticks, self.wheel_rad)
        self.enc_rf = QuadratureEncoder(ENC_RF_A, ENC_RF_B, ticks, self.wheel_rad)
        self.enc_rr = QuadratureEncoder(ENC_RR_A, ENC_RR_B, ticks, self.wheel_rad)

        # ---------- PID Controllers ----------
        self.pid_lf = PIDController(
            self.get_parameter('pid_kp_lf').value,
            self.get_parameter('pid_ki_lf').value,
            self.get_parameter('pid_kd_lf').value)
        self.pid_lr = PIDController(
            self.get_parameter('pid_kp_lr').value,
            self.get_parameter('pid_ki_lr').value,
            self.get_parameter('pid_kd_lr').value)
        self.pid_rf = PIDController(
            self.get_parameter('pid_kp_rf').value,
            self.get_parameter('pid_ki_rf').value,
            self.get_parameter('pid_kd_rf').value)
        self.pid_rr = PIDController(
            self.get_parameter('pid_kp_rr').value,
            self.get_parameter('pid_ki_rr').value,
            self.get_parameter('pid_kd_rr').value)

        self.pids = [self.pid_lf, self.pid_lr, self.pid_rf, self.pid_rr]

        # ---------- Velocity filtering (EMA) ----------
        self.v_filtered = [0.0, 0.0, 0.0, 0.0]  # [LF, LR, RF, RR]

        # ---------- Joint state tracking (cumulative wheel positions in rad) ----------
        self.joint_names = [
            'wheel_fl_joint', 'wheel_rl_joint',   # left
            'wheel_fr_joint', 'wheel_rr_joint',   # right
        ]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.rad_per_tick = (2.0 * math.pi) / ticks

        # ---------- Odometry state ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ---------- Velocity command ----------
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = self.get_clock().now()

        # ---------- Motor direction tracking ----------
        # Encoder reads magnitude only; direction comes from the actual PWM
        # sign sent to the motor on the PREVIOUS cycle (not the target).
        self._motor_dir = [1, 1, 1, 1]  # [LF, LR, RF, RR]

        # Velocity ramping state (smooth acceleration/deceleration)
        self._ramped_linear = 0.0
        self._ramped_angular = 0.0

        # RPM diagnostic logging
        self._log_accum = 0.0

        # ---------- ROS interfaces ----------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, qos)

        self.pub_odom = self.create_publisher(Odometry, '/wheel/odom', qos)
        self.pub_joint = self.create_publisher(JointState, '/joint_states', qos)

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------- Control loop timer ----------
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Motor driver started — sep={self.wheel_sep:.3f}m, '
            f'rad={self.wheel_rad:.3f}m, ticks/rev={ticks}, '
            f'max_speed={self.max_speed}m/s, min_pwm={self.min_pwm}, '
            f'rate={self.control_rate}Hz')

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------
    @staticmethod
    def _ramp_towards(current, target, max_delta):
        """Move current towards target by at most max_delta."""
        diff = target - current
        if abs(diff) <= max_delta:
            return target
        return current + math.copysign(max_delta, diff)

    # ------------------------------------------------------------------
    def _control_loop(self):
        now = self.get_clock().now()
        dt = self.dt

        # --- Timeout: stop if no cmd_vel received recently ---
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.cmd_linear = 0.0
            self.cmd_angular = 0.0

        # --- Enforce minimum speeds for moving commands ---
        linear_v = self.cmd_linear
        angular_v = self.cmd_angular

        if 0.001 < abs(linear_v) < self.min_linear_v:
            linear_v = math.copysign(self.min_linear_v, linear_v)
        if 0.001 < abs(angular_v) < self.min_angular_v:
            angular_v = math.copysign(self.min_angular_v, angular_v)

        # --- Smooth velocity ramping (acceleration limiter) ---
        accel_delta = self.max_accel * dt
        ang_accel_delta = self.max_accel / (self.wheel_sep / 2.0) * dt
        self._ramped_linear = self._ramp_towards(
            self._ramped_linear, linear_v, accel_delta)
        self._ramped_angular = self._ramp_towards(
            self._ramped_angular, angular_v, ang_accel_delta)

        # --- Diff-drive inverse kinematics: target wheel speeds (m/s) ---
        v_left_target = self._ramped_linear - (self._ramped_angular * self.wheel_sep / 2.0)
        v_right_target = self._ramped_linear + (self._ramped_angular * self.wheel_sep / 2.0)

        # --- Read encoders using PREVIOUS cycle's actual motor direction + flip logic ---
        # The encoder counts magnitude only.
        t_lf, d_lf = self.enc_lf.get_and_reset(self._motor_dir[0] * self.enc_flips[0])
        t_lr, d_lr = self.enc_lr.get_and_reset(self._motor_dir[1] * self.enc_flips[1])
        t_rf, d_rf = self.enc_rf.get_and_reset(self._motor_dir[2] * self.enc_flips[2])
        t_rr, d_rr = self.enc_rr.get_and_reset(self._motor_dir[3] * self.enc_flips[3])

        # Velocity filtering (EMA)
        raw_vels = [d_lf / dt, d_lr / dt, d_rf / dt, d_rr / dt]
        for i in range(4):
            self.v_filtered[i] = self.ema_alpha * raw_vels[i] + (1.0 - self.ema_alpha) * self.v_filtered[i]

        # --- PID Control & Feed-forward ---
        # Reduce rear wheel targets during rotation to prevent slip caused by
        # lower normal force on the rear axle. Scale blends smoothly between
        # pure linear (no reduction) and pure rotation (full reduction).
        rotation_ratio = abs(self._ramped_angular) / (
            abs(self._ramped_angular) + abs(self._ramped_linear) / (self.wheel_rad + 1e-9) + 1e-9)
        rear_scale = 1.0 - (1.0 - self.rear_rotation_scale) * rotation_ratio

        targets = [v_left_target, v_left_target * rear_scale,
                   v_right_target, v_right_target * rear_scale]
        motors = [self.motor_lf, self.motor_lr, self.motor_rf, self.motor_rr]
        pwms = [0.0, 0.0, 0.0, 0.0]

        for i in range(4):
            target_vel = targets[i]
            measured_vel = self.v_filtered[i]

            if abs(target_vel) < 0.001:
                # Position hold via active brake at zero velocity
                motors[i].brake()
                self.pids[i].reset()
                self.v_filtered[i] = 0.0
                pwms[i] = 1.0  # (indicator)
            else:
                # Standard PI(D) + feed-forward
                ff = target_vel / self.max_speed
                pwm, error, p, it, d = self.pids[i].compute(target_vel, measured_vel, dt, ff=ff)

                # Stall-only stiction kick
                if abs(measured_vel) < 0.02 and pwm * target_vel > 0 and abs(pwm) < self.min_pwm:
                    pwm = math.copysign(self.min_pwm, pwm)

                motors[i].set_speed(pwm)
                pwms[i] = pwm

                # Update motor direction tracking for next cycle's encoder reads
                if abs(pwm) > 0.05:
                    self._motor_dir[i] = 1 if pwm > 0 else -1

        # Odometry: average per side (using filtered velocities for smoother odom)
        v_left_meas = (self.v_filtered[0] + self.v_filtered[1]) / 2.0
        v_right_meas = (self.v_filtered[2] + self.v_filtered[3]) / 2.0
        
        # Distances per cycle for pose integration
        # Note: ticks (t_lf etc) are already signed by enc_flips and _motor_dir
        d_left_step = (d_lf + d_lr) / 2.0
        d_right_step = (d_rf + d_rr) / 2.0

        # Joint positions for RViz
        # Left wheels: negative rotation = forward (URDF Y-axis, left side)
        # Right wheels: positive rotation = forward (joint frame flipped)
        self.joint_positions[0] += -t_lf * self.rad_per_tick  # wheel_fl
        self.joint_positions[1] += -t_lr * self.rad_per_tick  # wheel_rl
        self.joint_positions[2] +=  t_rf * self.rad_per_tick  # wheel_fr
        self.joint_positions[3] +=  t_rr * self.rad_per_tick  # wheel_rr

        # --- Periodic diagnostic logging ---
        self._log_accum += dt
        if self._log_accum >= self.log_interval:
            self._log_accum = 0.0
            if abs(self._ramped_linear) > 0.001 or abs(self._ramped_angular) > 0.001:
                rpm_f = 60.0 / (2.0 * math.pi * self.wheel_rad)
                self.get_logger().info(
                    f'RPM  LF:{self.v_filtered[0]*rpm_f:6.1f}  '
                    f'LR:{self.v_filtered[1]*rpm_f:6.1f}  '
                    f'RF:{self.v_filtered[2]*rpm_f:6.1f}  '
                    f'RR:{self.v_filtered[3]*rpm_f:6.1f}  '
                    f'| tgt L:{v_left_target*rpm_f:5.1f} R:{v_right_target*rpm_f:5.1f}'
                    f'| PWM {pwms[0]:+.2f} {pwms[1]:+.2f} {pwms[2]:+.2f} {pwms[3]:+.2f}')

        # --- Forward kinematics: update odometry ---
        d_center = (d_left_step + d_right_step) / 2.0
        d_theta = (d_right_step - d_left_step) / self.wheel_sep

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v_linear = (v_left_meas + v_right_meas) / 2.0
        v_angular = (v_right_meas - v_left_meas) / self.wheel_sep

        # --- Publish Odometry ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        # Pose covariance (x, y, yaw significant)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw

        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular

        # Twist covariance
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[35] = 0.03  # vyaw

        self.pub_odom.publish(odom)

        # --- Publish joint states (wheel positions for RViz URDF) ---
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joint_names
        js.position = list(self.joint_positions)
        js.velocity = [
            self.v_filtered[0] / self.wheel_rad,   # wheel_fl rad/s
            self.v_filtered[1] / self.wheel_rad,   # wheel_rl
            self.v_filtered[2] / self.wheel_rad,   # wheel_fr
            self.v_filtered[3] / self.wheel_rad,   # wheel_rr
        ]
        self.pub_joint.publish(js)

        # --- Optionally broadcast TF ---
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quaternion(self.theta)
            self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('Shutting down motor driver...')
        for m in [self.motor_lf, self.motor_lr, self.motor_rf, self.motor_rr]:
            m.close()
        for e in [self.enc_lf, self.enc_lr, self.enc_rf, self.enc_rr]:
            e.close()
        self.stby1.off(); self.stby1.close()
        self.stby2.off(); self.stby2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
