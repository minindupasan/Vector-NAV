#!/usr/bin/env python3
"""
IMU Publisher Node for VECTOR NAV (Raspberry Pi).

Reads MPU6500 (accel + gyro) and HMC5883L (magnetometer) over I2C,
applies Madgwick AHRS filter, and publishes:
  /imu                (sensor_msgs/Imu)           — fused orientation + raw gyro/accel
  /imu/mag            (sensor_msgs/MagneticField)  — raw magnetometer

The /imu topic feeds into robot_localization EKF for vyaw fusion.
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3

from vector_control.madgwick import MadgwickAHRS

try:
    import smbus2 as smbus
except ImportError:
    import smbus


# ---------------------------------------------------------------------------
# Register map
# ---------------------------------------------------------------------------
MPU6500_ADDR        = 0x68
MPU6500_WHO_AM_I    = 0x75
MPU6500_PWR_MGMT_1  = 0x6B
MPU6500_ACCEL_CONFIG = 0x1C
MPU6500_GYRO_CONFIG  = 0x1B
MPU6500_ACCEL_XOUT_H = 0x3B
MPU6500_GYRO_XOUT_H  = 0x43
MPU6500_INT_PIN_CFG   = 0x37

HMC5883L_ADDR     = 0x1E
HMC5883L_CONFIG_A  = 0x00
HMC5883L_CONFIG_B  = 0x01
HMC5883L_MODE      = 0x02
HMC5883L_DATA_OUT  = 0x03

GRAVITY = 9.80665  # m/s^2


class IMUReader:
    """Low-level I2C reader for MPU6500 + HMC5883L."""

    def __init__(self, bus_num=1, accel_range=2, gyro_range=250):
        """
        Args:
            bus_num: I2C bus number (1 on Raspberry Pi).
            accel_range: Full-scale range in g (2, 4, 8, 16).
            gyro_range: Full-scale range in deg/s (250, 500, 1000, 2000).
        """
        self.bus = smbus.SMBus(bus_num)
        self.mpu_ok = False
        self.mag_ok = False

        # Scale factors
        accel_scales = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
        gyro_scales = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
        self.accel_scale = accel_scales.get(accel_range, 16384.0)
        self.gyro_scale = gyro_scales.get(gyro_range, 131.0)

        # Config register values
        accel_configs = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}
        gyro_configs = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}
        self.accel_config_val = accel_configs.get(accel_range, 0x00)
        self.gyro_config_val = gyro_configs.get(gyro_range, 0x00)

        # Gyro bias (computed during calibration)
        self.gyro_bias = [0.0, 0.0, 0.0]

    def init_mpu6500(self):
        """Wake up MPU6500 and configure ranges."""
        try:
            who = self.bus.read_byte_data(MPU6500_ADDR, MPU6500_WHO_AM_I)
            # Wake up
            self.bus.write_byte_data(MPU6500_ADDR, MPU6500_PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            # Set accel and gyro ranges
            self.bus.write_byte_data(MPU6500_ADDR, MPU6500_ACCEL_CONFIG, self.accel_config_val)
            self.bus.write_byte_data(MPU6500_ADDR, MPU6500_GYRO_CONFIG, self.gyro_config_val)
            # Enable I2C bypass for magnetometer
            self.bus.write_byte_data(MPU6500_ADDR, MPU6500_INT_PIN_CFG, 0x02)
            time.sleep(0.01)
            self.mpu_ok = True
            return True, who
        except OSError as e:
            self.mpu_ok = False
            return False, str(e)

    def init_hmc5883l(self):
        """Initialize HMC5883L magnetometer in continuous mode."""
        try:
            # 8 samples avg, 75 Hz output, normal measurement
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x78)
            # Gain = 1090 LSB/Gauss
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0x20)
            # Continuous measurement mode
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_MODE, 0x00)
            time.sleep(0.01)
            self.mag_ok = True
            return True
        except OSError:
            self.mag_ok = False
            return False

    def calibrate_gyro(self, samples=200, delay=0.005):
        """Compute gyro bias from stationary samples. Robot must be still."""
        sx = sy = sz = 0.0
        count = 0
        for _ in range(samples):
            try:
                gx, gy, gz = self._read_gyro_raw()
                sx += gx; sy += gy; sz += gz
                count += 1
            except OSError:
                pass
            time.sleep(delay)
        if count > 0:
            self.gyro_bias = [sx / count, sy / count, sz / count]

    def _read_raw_block(self, addr, reg, length):
        return self.bus.read_i2c_block_data(addr, reg, length)

    def _read_gyro_raw(self):
        """Read raw gyro in deg/s (before bias removal)."""
        data = self._read_raw_block(MPU6500_ADDR, MPU6500_GYRO_XOUT_H, 6)
        gx = struct.unpack('>h', bytes(data[0:2]))[0] / self.gyro_scale
        gy = struct.unpack('>h', bytes(data[2:4]))[0] / self.gyro_scale
        gz = struct.unpack('>h', bytes(data[4:6]))[0] / self.gyro_scale
        return gx, gy, gz

    def read_accel(self):
        """Read accelerometer in m/s^2."""
        data = self._read_raw_block(MPU6500_ADDR, MPU6500_ACCEL_XOUT_H, 6)
        ax = struct.unpack('>h', bytes(data[0:2]))[0] / self.accel_scale * GRAVITY
        ay = struct.unpack('>h', bytes(data[2:4]))[0] / self.accel_scale * GRAVITY
        az = struct.unpack('>h', bytes(data[4:6]))[0] / self.accel_scale * GRAVITY
        return ax, ay, az

    def read_gyro(self):
        """Read gyroscope in rad/s (bias-corrected)."""
        gx, gy, gz = self._read_gyro_raw()
        deg2rad = math.pi / 180.0
        return (
            (gx - self.gyro_bias[0]) * deg2rad,
            (gy - self.gyro_bias[1]) * deg2rad,
            (gz - self.gyro_bias[2]) * deg2rad,
        )

    def read_mag(self):
        """Read magnetometer in Tesla (HMC5883L byte order: X, Z, Y)."""
        data = self._read_raw_block(HMC5883L_ADDR, HMC5883L_DATA_OUT, 6)
        # HMC5883L: X_H X_L Z_H Z_L Y_H Y_L
        raw_x = struct.unpack('>h', bytes(data[0:2]))[0]
        raw_z = struct.unpack('>h', bytes(data[2:4]))[0]
        raw_y = struct.unpack('>h', bytes(data[4:6]))[0]
        # Convert from Gauss to Tesla (1 Gauss = 1e-4 Tesla)
        # 1090 LSB/Gauss at gain 0x20
        gauss_to_tesla = 1e-4
        mx = (raw_x / 1090.0) * gauss_to_tesla
        my = (raw_y / 1090.0) * gauss_to_tesla
        mz = (raw_z / 1090.0) * gauss_to_tesla
        return mx, my, mz

    def close(self):
        self.bus.close()


class IMUPublisherNode(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # ---------- Parameters ----------
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('accel_range', 2)       # g
        self.declare_parameter('gyro_range', 250)       # deg/s
        self.declare_parameter('madgwick_beta', 0.1)
        self.declare_parameter('calibrate_samples', 500)
        self.declare_parameter('gyro_deadband', 0.005) # rad/s
        self.declare_parameter('use_magnetometer', True)
        self.declare_parameter('frame_id', 'imu_link')

        bus_num = self.get_parameter('i2c_bus').value
        self.rate = self.get_parameter('publish_rate').value
        accel_range = self.get_parameter('accel_range').value
        gyro_range = self.get_parameter('gyro_range').value
        beta = self.get_parameter('madgwick_beta').value
        cal_samples = self.get_parameter('calibrate_samples').value
        self.deadband = self.get_parameter('gyro_deadband').value
        self.use_mag = self.get_parameter('use_magnetometer').value
        self.frame_id = self.get_parameter('frame_id').value

        # ---------- Hardware ----------
        self.imu = IMUReader(bus_num, accel_range, gyro_range)

        ok, who = self.imu.init_mpu6500()
        if ok:
            self.get_logger().info(f'MPU6500 OK (WHO_AM_I=0x{who:02X})')
        else:
            self.get_logger().error(f'MPU6500 init failed: {who}')
            raise RuntimeError('MPU6500 not found')

        if self.use_mag:
            if self.imu.init_hmc5883l():
                self.get_logger().info('HMC5883L magnetometer OK')
            else:
                self.get_logger().warn('HMC5883L not found — using 6-DOF mode')
                self.use_mag = False

        # ---------- Calibrate gyro ----------
        self.get_logger().info(f'Calibrating gyro ({cal_samples} samples) — keep robot still...')
        self.imu.calibrate_gyro(samples=cal_samples)
        bias = self.imu.gyro_bias
        self.get_logger().info(
            f'Gyro bias: [{bias[0]:.4f}, {bias[1]:.4f}, {bias[2]:.4f}] deg/s')

        # ---------- Madgwick filter ----------
        self.dt = 1.0 / self.rate
        self.ahrs = MadgwickAHRS(beta=beta, sample_period=self.dt)

        # ---------- Publishers ----------
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_imu = self.create_publisher(Imu, '/imu', qos)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', qos)

        # ---------- Covariance (static, diagonal) ----------
        # Orientation: Madgwick uncertainty
        self.orientation_cov = [0.0] * 9
        self.orientation_cov[0] = 0.0025  # roll
        self.orientation_cov[4] = 0.0025  # pitch
        self.orientation_cov[8] = 0.0025  # yaw

        # Gyro noise
        self.gyro_cov = [0.0] * 9
        self.gyro_cov[0] = 0.0003
        self.gyro_cov[4] = 0.0003
        self.gyro_cov[8] = 0.0003

        # Accel noise
        self.accel_cov = [0.0] * 9
        self.accel_cov[0] = 0.01
        self.accel_cov[4] = 0.01
        self.accel_cov[8] = 0.01

        # Mag covariance
        self.mag_cov = [0.0] * 9
        self.mag_cov[0] = 1e-6
        self.mag_cov[4] = 1e-6
        self.mag_cov[8] = 1e-6

        # ---------- Timer ----------
        self.timer = self.create_timer(self.dt, self._publish)
        self.get_logger().info(
            f'IMU publisher started — {self.rate}Hz, beta={beta}, '
            f'deadband={self.deadband}rad/s, mag={"ON" if self.use_mag else "OFF"}')

    # ------------------------------------------------------------------
    def _publish(self):
        now = self.get_clock().now()

        try:
            ax, ay, az = self.imu.read_accel()
            gx, gy, gz = self.imu.read_gyro()
        except OSError as e:
            self.get_logger().warn(f'IMU read error: {e}', throttle_duration_sec=2.0)
            return

        # Apply deadband to gyro
        if abs(gx) < self.deadband: gx = 0.0
        if abs(gy) < self.deadband: gy = 0.0
        if abs(gz) < self.deadband: gz = 0.0

        mx = my = mz = None
        if self.use_mag:
            try:
                mx, my, mz = self.imu.read_mag()
            except OSError:
                mx = my = mz = None

        # --- Update Madgwick filter ---
        if mx is not None:
            # Magnetometer is in Tesla for the message, but Madgwick
            # just needs direction — pass raw values (normalised internally)
            self.ahrs.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt=self.dt)
        else:
            self.ahrs.update(gx, gy, gz, ax, ay, az, dt=self.dt)

        w, qx, qy, qz = self.ahrs.quaternion

        # --- Publish sensor_msgs/Imu ---
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation = Quaternion(x=qx, y=qy, z=qz, w=w)
        msg.orientation_covariance = self.orientation_cov

        msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)
        msg.angular_velocity_covariance = self.gyro_cov

        msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
        msg.linear_acceleration_covariance = self.accel_cov

        self.pub_imu.publish(msg)

        # --- Publish MagneticField (if available) ---
        if mx is not None:
            mag_msg = MagneticField()
            mag_msg.header.stamp = now.to_msg()
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field = Vector3(x=mx, y=my, z=mz)
            mag_msg.magnetic_field_covariance = self.mag_cov
            self.pub_mag.publish(mag_msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('Shutting down IMU publisher...')
        self.imu.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
