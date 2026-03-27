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

    def stop(self):
        self.in1.off(); self.in2.off(); self.pwm.value = 0

    def close(self):
        self.stop()
        self.pwm.close(); self.in1.close(); self.in2.close()


class QuadratureEncoder:
    """Quadrature encoder using Phase A (interrupt) + Phase B (direction)."""

    def __init__(self, pin_a, pin_b, ticks_per_rev, wheel_radius):
        self.ticks_per_rev = ticks_per_rev
        self.wheel_radius = wheel_radius
        self.meters_per_tick = (2.0 * math.pi * wheel_radius) / ticks_per_rev

        self._ticks = 0
        self._lock = threading.Lock()

        # Phase B for direction sensing
        self._phase_b = Button(pin_b, pull_up=True, bounce_time=None)

        # Phase A triggers counting on rising edge
        self._phase_a = Button(pin_a, pull_up=True, bounce_time=None)
        self._phase_a.when_pressed = self._on_rising

    def _on_rising(self):
        # If B is high when A rises → one direction; B low → other
        direction = 1 if self._phase_b.is_pressed else -1
        with self._lock:
            self._ticks += direction

    def get_and_reset(self):
        """Return (ticks, distance_meters) since last call and reset counter."""
        with self._lock:
            t = self._ticks
            self._ticks = 0
        return t, t * self.meters_per_tick

    def close(self):
        self._phase_a.close()
        self._phase_b.close()


class PIDController:
    """Simple PID with anti-windup clamp."""

    def __init__(self, kp, ki, kd, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        self._integral += error * dt
        # Anti-windup
        self._integral = max(-self.output_limit, min(self.output_limit, self._integral))
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


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
        self.declare_parameter('max_motor_speed', 0.5)  # m/s at wheel
        self.declare_parameter('pid_kp', 0.15)
        self.declare_parameter('pid_ki', 0.05)
        self.declare_parameter('pid_kd', 0.0)
        self.declare_parameter('use_pid', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)  # EKF publishes tf

        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        ticks = self.get_parameter('ticks_per_rev').value
        self.control_rate = self.get_parameter('control_rate').value
        self.cmd_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_speed = self.get_parameter('max_motor_speed').value
        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        self.use_pid = self.get_parameter('use_pid').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

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

        # ---------- PID controllers (one per side) ----------
        if self.use_pid:
            self.pid_left = PIDController(kp, ki, kd)
            self.pid_right = PIDController(kp, ki, kd)

        # ---------- Odometry state ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ---------- Velocity command ----------
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = self.get_clock().now()

        # ---------- ROS interfaces ----------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, qos)

        self.pub_odom = self.create_publisher(Odometry, '/wheel/odom', qos)

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ---------- Control loop timer ----------
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(
            f'Motor driver started — sep={self.wheel_sep:.3f}m, '
            f'rad={self.wheel_rad:.3f}m, ticks/rev={ticks}, '
            f'max_speed={self.max_speed}m/s, pid={self.use_pid}, '
            f'rate={self.control_rate}Hz')

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def _control_loop(self):
        now = self.get_clock().now()
        dt = self.dt

        # --- Timeout: stop if no cmd_vel received recently ---
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.cmd_linear = 0.0
            self.cmd_angular = 0.0

        # --- Diff-drive inverse kinematics: target wheel speeds (m/s) ---
        v_left_target = self.cmd_linear - (self.cmd_angular * self.wheel_sep / 2.0)
        v_right_target = self.cmd_linear + (self.cmd_angular * self.wheel_sep / 2.0)

        # --- Read encoders for odometry (average front+rear per side) ---
        # LF encoder direction is inverted in hardware, negate it
        _, d_lf = self.enc_lf.get_and_reset()
        _, d_lr = self.enc_lr.get_and_reset()
        _, d_rf = self.enc_rf.get_and_reset()
        _, d_rr = self.enc_rr.get_and_reset()

        d_left = (-d_lf + d_lr) / 2.0
        d_right = (d_rf + d_rr) / 2.0

        v_left_meas = d_left / dt
        v_right_meas = d_right / dt

        # --- Feed-forward + PID correction ---
        # Feed-forward: direct velocity-to-PWM mapping (does the heavy lifting)
        ff_left = v_left_target / self.max_speed
        ff_right = v_right_target / self.max_speed

        if self.use_pid:
            if abs(v_left_target) < 0.001 and abs(v_right_target) < 0.001:
                # Dead stop — reset PID to avoid windup
                pwm_left = 0.0
                pwm_right = 0.0
                self.pid_left.reset()
                self.pid_right.reset()
            else:
                pid_left = self.pid_left.compute(v_left_target, v_left_meas, dt)
                pid_right = self.pid_right.compute(v_right_target, v_right_meas, dt)
                pwm_left = ff_left + pid_left
                pwm_right = ff_right + pid_right
        else:
            pwm_left = ff_left
            pwm_right = ff_right

        pwm_left = max(-1.0, min(1.0, pwm_left))
        pwm_right = max(-1.0, min(1.0, pwm_right))

        # Drive motors
        self.motor_lf.set_speed(pwm_left)
        self.motor_lr.set_speed(pwm_left)
        self.motor_rf.set_speed(pwm_right)
        self.motor_rr.set_speed(pwm_right)

        # --- Forward kinematics: update odometry ---
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_sep

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
