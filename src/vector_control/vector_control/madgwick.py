"""
Madgwick AHRS filter — 9-DOF (accel + gyro + mag) and 6-DOF (accel + gyro).

Reference: S. Madgwick, "An efficient orientation filter for inertial and
inertial/magnetic sensor arrays", 2010.

All inputs/outputs use the NED (North-East-Down) or ENU convention depending
on how the caller orients axes.  The filter itself is frame-agnostic — it
just tracks a quaternion that rotates the sensor frame into the earth frame.
"""

import math


class MadgwickAHRS:
    """Madgwick Attitude and Heading Reference System."""

    def __init__(self, beta=0.1, sample_period=0.01):
        """
        Args:
            beta: Filter gain — higher = faster convergence but more noise.
                  Typical: 0.033 (slow) to 0.5 (aggressive). Default 0.1.
            sample_period: Expected dt between updates (seconds). Overridden
                           if dt is passed to update().
        """
        self.beta = beta
        self.sample_period = sample_period
        # Quaternion: [w, x, y, z] — initially identity (sensor = earth)
        self.q = [1.0, 0.0, 0.0, 0.0]

    def update(self, gx, gy, gz, ax, ay, az, mx=None, my=None, mz=None, dt=None):
        """
        Update the orientation estimate.

        Args:
            gx, gy, gz: Gyroscope in rad/s.
            ax, ay, az: Accelerometer in m/s^2 (or g — only direction matters).
            mx, my, mz: Magnetometer (optional). If None, 6-DOF mode is used.
            dt: Time step in seconds. Uses sample_period if None.
        """
        if dt is None:
            dt = self.sample_period

        q0, q1, q2, q3 = self.q

        if mx is not None and my is not None and mz is not None:
            self._update_9dof(gx, gy, gz, ax, ay, az, mx, my, mz, dt)
        else:
            self._update_6dof(gx, gy, gz, ax, ay, az, dt)

    # ------------------------------------------------------------------
    # 6-DOF update (accel + gyro only — no yaw reference)
    # ------------------------------------------------------------------
    def _update_6dof(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q

        # Normalise accelerometer
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm < 1e-10:
            return  # avoid division by zero
        ax /= norm; ay /= norm; az /= norm

        # Gradient descent corrective step
        f0 = 2.0 * (q1 * q3 - q0 * q2) - ax
        f1 = 2.0 * (q0 * q1 + q2 * q3) - ay
        f2 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az

        j_t_f0 = -2.0 * q2 * f0 + 2.0 * q1 * f1
        j_t_f1 =  2.0 * q3 * f0 + 2.0 * q0 * f1 - 4.0 * q1 * f2
        j_t_f2 = -2.0 * q0 * f0 + 2.0 * q3 * f1 - 4.0 * q2 * f2
        j_t_f3 =  2.0 * q1 * f0 + 2.0 * q2 * f1

        # Normalise step
        step_norm = math.sqrt(j_t_f0**2 + j_t_f1**2 + j_t_f2**2 + j_t_f3**2)
        if step_norm < 1e-10:
            step_norm = 1.0
        j_t_f0 /= step_norm
        j_t_f1 /= step_norm
        j_t_f2 /= step_norm
        j_t_f3 /= step_norm

        # Quaternion rate of change (gyro) minus beta * gradient step
        q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * j_t_f0
        q_dot1 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy) - self.beta * j_t_f1
        q_dot2 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx) - self.beta * j_t_f2
        q_dot3 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx) - self.beta * j_t_f3

        # Integrate
        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        # Normalise quaternion
        norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        self.q = [q0 / norm, q1 / norm, q2 / norm, q3 / norm]

    # ------------------------------------------------------------------
    # 9-DOF update (accel + gyro + mag)
    # ------------------------------------------------------------------
    def _update_9dof(self, gx, gy, gz, ax, ay, az, mx, my, mz, dt):
        q0, q1, q2, q3 = self.q

        # Normalise accelerometer
        norm_a = math.sqrt(ax * ax + ay * ay + az * az)
        if norm_a < 1e-10:
            return
        ax /= norm_a; ay /= norm_a; az /= norm_a

        # Normalise magnetometer
        norm_m = math.sqrt(mx * mx + my * my + mz * mz)
        if norm_m < 1e-10:
            # Fall back to 6-DOF
            self._update_6dof(gx, gy, gz, ax * norm_a, ay * norm_a, az * norm_a, dt)
            return
        mx /= norm_m; my /= norm_m; mz /= norm_m

        # Reference direction of Earth's magnetic field
        _2q0mx = 2.0 * q0 * mx; _2q0my = 2.0 * q0 * my; _2q0mz = 2.0 * q0 * mz
        _2q1mx = 2.0 * q1 * mx
        hx = (mx * q0*q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1*q1
              + 2.0 * q1 * my * q2 + 2.0 * q1 * mz * q3
              - mx * q2*q2 - mx * q3*q3)
        hy = (_2q0mx * q3 + my * q0*q0 - _2q0mz * q1 + _2q1mx * q2
              - my * q1*q1 + my * q2*q2 + 2.0 * q2 * mz * q3
              - my * q3*q3)
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = (-_2q0mx * q2 + _2q0my * q1 + mz * q0*q0 + _2q1mx * q3
                - mz * q1*q1 + 2.0 * q2 * my * q3 - mz * q2*q2 + mz * q3*q3)

        # Gradient descent corrective step
        _2q0 = 2.0 * q0; _2q1 = 2.0 * q1; _2q2 = 2.0 * q2; _2q3 = 2.0 * q3
        _4bx = 2.0 * _2bx; _4bz = 2.0 * _2bz
        _8bx = 2.0 * _4bx; _8bz = 2.0 * _4bz
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3

        s0 = (-_2q2 * (2.0*(q1*q3 - q0*q2) - ax)
              + _2q1 * (2.0*(q0*q1 + q2*q3) - ay)
              - _2bz * q2 * (_2bx*(0.5 - q2q2 - q3q3) + _2bz*(q1*q3 - q0*q2) - mx)
              + (-_2bx * q3 + _2bz * q1) * (_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
              + _2bx * q2 * (_2bx*(q0*q2 + q1*q3) + _2bz*(0.5 - q1q1 - q2q2) - mz))

        s1 = (_2q3 * (2.0*(q1*q3 - q0*q2) - ax)
              + _2q0 * (2.0*(q0*q1 + q2*q3) - ay)
              - 4.0 * q1 * (1.0 - 2.0*(q1q1 + q2q2) - az)
              + _2bz * q3 * (_2bx*(0.5 - q2q2 - q3q3) + _2bz*(q1*q3 - q0*q2) - mx)
              + (_2bx * q2 + _2bz * q0) * (_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
              + (_2bx * q3 - _4bz * q1) * (_2bx*(q0*q2 + q1*q3) + _2bz*(0.5 - q1q1 - q2q2) - mz))

        s2 = (-_2q0 * (2.0*(q1*q3 - q0*q2) - ax)
              + _2q3 * (2.0*(q0*q1 + q2*q3) - ay)
              - 4.0 * q2 * (1.0 - 2.0*(q1q1 + q2q2) - az)
              + (-_4bx * q2 - _2bz * q0) * (_2bx*(0.5 - q2q2 - q3q3) + _2bz*(q1*q3 - q0*q2) - mx)
              + (_2bx * q1 + _2bz * q3) * (_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
              + (_2bx * q0 - _4bz * q2) * (_2bx*(q0*q2 + q1*q3) + _2bz*(0.5 - q1q1 - q2q2) - mz))

        s3 = (_2q1 * (2.0*(q1*q3 - q0*q2) - ax)
              + _2q2 * (2.0*(q0*q1 + q2*q3) - ay)
              + (-_4bx * q3 + _2bz * q1) * (_2bx*(0.5 - q2q2 - q3q3) + _2bz*(q1*q3 - q0*q2) - mx)
              + (-_2bx * q0 + _2bz * q2) * (_2bx*(q1*q2 - q0*q3) + _2bz*(q0*q1 + q2*q3) - my)
              + _2bx * q1 * (_2bx*(q0*q2 + q1*q3) + _2bz*(0.5 - q1q1 - q2q2) - mz))

        # Normalise step magnitude
        step_norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if step_norm < 1e-10:
            step_norm = 1.0
        s0 /= step_norm; s1 /= step_norm; s2 /= step_norm; s3 /= step_norm

        # Quaternion rate from gyroscope, minus beta * gradient step
        q_dot0 = 0.5 * (-q1*gx - q2*gy - q3*gz) - self.beta * s0
        q_dot1 = 0.5 * ( q0*gx + q2*gz - q3*gy) - self.beta * s1
        q_dot2 = 0.5 * ( q0*gy - q1*gz + q3*gx) - self.beta * s2
        q_dot3 = 0.5 * ( q0*gz + q1*gy - q2*gx) - self.beta * s3

        # Integrate
        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        # Normalise
        norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
        self.q = [q0/norm, q1/norm, q2/norm, q3/norm]

    # ------------------------------------------------------------------
    # Convenience getters
    # ------------------------------------------------------------------
    @property
    def quaternion(self):
        """Return (w, x, y, z)."""
        return tuple(self.q)

    def get_euler(self):
        """Return (roll, pitch, yaw) in radians."""
        w, x, y, z = self.q
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
