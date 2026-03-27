#!/usr/bin/env python3
"""
Motor, Encoder & IMU Test Script for VECTOR NAV
Uses gpiozero (lgpio backend) for Raspberry Pi 5 compatibility.

Pinout:
  I2C (Bus 1):
    SDA=GPIO2 (Pin 3), SCL=GPIO3 (Pin 5)

  Motor Driver 1 (LF + LR):
    PWMA=GPIO12, AIN1=GPIO5, AIN2=GPIO6, STBY=GPIO13
    PWMB=GPIO18, BIN1=GPIO19, BIN2=GPIO26

  Motor Driver 2 (RF + RR):
    PWMA=GPIO16, AIN1=GPIO20, AIN2=GPIO21, STBY=GPIO23
    PWMB=GPIO22, BIN1=GPIO17, BIN2=GPIO27

  Encoders:
    LF: PhaseA=GPIO4,  PhaseB=GPIO25
    LR: PhaseA=GPIO24, PhaseB=GPIO14
    RF: PhaseA=GPIO15, PhaseB=GPIO7
    RR: PhaseA=GPIO10, PhaseB=GPIO9
"""

import time
import struct
import math
import threading

from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button

try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        print("ERROR: smbus not found. Install with: sudo apt install python3-smbus2")
        exit(1)

# ---------------------------------------------------------------------------
# Pin Definitions
# ---------------------------------------------------------------------------

# Motor Driver 1 (LF = Channel A, LR = Channel B)
MD1_PWMA = 12   # LF PWM
MD1_AIN1 = 5    # LF direction
MD1_AIN2 = 6
MD1_STBY = 13   # Driver 1 standby
MD1_BIN1 = 19   # LR direction
MD1_BIN2 = 26
MD1_PWMB = 18   # LR PWM

# Motor Driver 2 (RF = Channel A, RR = Channel B)
MD2_PWMA = 16   # RF PWM
MD2_AIN1 = 20   # RF direction
MD2_AIN2 = 21
MD2_STBY = 23   # Driver 2 standby
MD2_BIN1 = 17   # RR direction
MD2_BIN2 = 27
MD2_PWMB = 22   # RR PWM

# Encoders (Phase A, Phase B)
ENC_LF_A = 4
ENC_LF_B = 25
ENC_LR_A = 24
ENC_LR_B = 14
ENC_RF_A = 15
ENC_RF_B = 7
ENC_RR_A = 10
ENC_RR_B = 9

# PWM frequency for motors (Hz)
PWM_FREQ = 1000

# ---------------------------------------------------------------------------
# I2C / IMU Definitions
# ---------------------------------------------------------------------------
I2C_BUS = 1

# MPU6500 (accel + gyro)
MPU6500_ADDR = 0x68
MPU6500_WHO_AM_I = 0x75
MPU6500_PWR_MGMT_1 = 0x6B
MPU6500_ACCEL_XOUT_H = 0x3B
MPU6500_GYRO_XOUT_H = 0x43
MPU6500_INT_PIN_CFG = 0x37  # to enable I2C bypass for magnetometer

# HMC5883L (magnetometer)
HMC5883L_ADDR = 0x1E
HMC5883L_CONFIG_A = 0x00
HMC5883L_CONFIG_B = 0x01
HMC5883L_MODE = 0x02
HMC5883L_DATA_OUT = 0x03
HMC5883L_ID_A = 0x0A

# ---------------------------------------------------------------------------
# Encoder tracking
# ---------------------------------------------------------------------------
encoder_counts = {"LF": 0, "LR": 0, "RF": 0, "RR": 0}
encoder_lock = threading.Lock()


def make_encoder_callback(name):
    def callback():
        with encoder_lock:
            encoder_counts[name] += 1
    return callback


def get_encoder_counts():
    with encoder_lock:
        return dict(encoder_counts)


def reset_encoder_counts():
    with encoder_lock:
        for key in encoder_counts:
            encoder_counts[key] = 0


# ---------------------------------------------------------------------------
# Motor control helpers
# ---------------------------------------------------------------------------
class Motor:
    def __init__(self, name, pwm_pin, in1_pin, in2_pin):
        self.name = name
        self.pwm = PWMOutputDevice(pwm_pin, frequency=PWM_FREQ)
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)

    def forward(self, speed):
        """Run motor forward at given speed (0-100)."""
        self.in1.on()
        self.in2.off()
        self.pwm.value = speed / 100.0

    def backward(self, speed):
        """Run motor backward at given speed (0-100)."""
        self.in1.off()
        self.in2.on()
        self.pwm.value = speed / 100.0

    def stop(self):
        """Stop the motor."""
        self.in1.off()
        self.in2.off()
        self.pwm.value = 0

    def cleanup(self):
        self.stop()
        self.pwm.close()
        self.in1.close()
        self.in2.close()


# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------
def setup():
    # Standby pins — HIGH to enable drivers
    stby1 = DigitalOutputDevice(MD1_STBY)
    stby2 = DigitalOutputDevice(MD2_STBY)
    stby1.on()
    stby2.on()

    # Create motor objects
    motors = {
        "LF": Motor("LF", MD1_PWMA, MD1_AIN1, MD1_AIN2),
        "LR": Motor("LR", MD1_PWMB, MD1_BIN1, MD1_BIN2),
        "RF": Motor("RF", MD2_PWMA, MD2_AIN1, MD2_AIN2),
        "RR": Motor("RR", MD2_PWMB, MD2_BIN1, MD2_BIN2),
    }

    # Encoder inputs — use Button (input with pull-up, edge detection)
    encoders = {
        "LF": Button(ENC_LF_A, pull_up=True, bounce_time=None),
        "LR": Button(ENC_LR_A, pull_up=True, bounce_time=None),
        "RF": Button(ENC_RF_A, pull_up=True, bounce_time=None),
        "RR": Button(ENC_RR_A, pull_up=True, bounce_time=None),
    }
    for name, enc in encoders.items():
        enc.when_pressed = make_encoder_callback(name)

    return motors, encoders, stby1, stby2


# ---------------------------------------------------------------------------
# Motor test routines
# ---------------------------------------------------------------------------
def test_individual_motors(motors, speed=50, duration=2):
    """Test each motor one at a time."""
    print("\n=== Individual Motor Test ===")
    for name, motor in motors.items():
        reset_encoder_counts()

        print(f"\n  [{name}] Forward at {speed}% for {duration}s...")
        motor.forward(speed)
        time.sleep(duration)
        motor.stop()
        counts = get_encoder_counts()
        print(f"  [{name}] Encoder ticks: {counts[name]}")

        time.sleep(0.5)
        reset_encoder_counts()

        print(f"  [{name}] Backward at {speed}% for {duration}s...")
        motor.backward(speed)
        time.sleep(duration)
        motor.stop()
        counts = get_encoder_counts()
        print(f"  [{name}] Encoder ticks: {counts[name]}")

        time.sleep(0.5)

    print("\n  Individual motor test complete.")


def test_all_forward(motors, speed=50, duration=3):
    """Drive all motors forward simultaneously."""
    print(f"\n=== All Motors Forward at {speed}% for {duration}s ===")
    reset_encoder_counts()

    for motor in motors.values():
        motor.forward(speed)

    time.sleep(duration)

    for motor in motors.values():
        motor.stop()

    counts = get_encoder_counts()
    print(f"  Encoder ticks: LF={counts['LF']}  LR={counts['LR']}  "
          f"RF={counts['RF']}  RR={counts['RR']}")


def test_all_backward(motors, speed=50, duration=3):
    """Drive all motors backward simultaneously."""
    print(f"\n=== All Motors Backward at {speed}% for {duration}s ===")
    reset_encoder_counts()

    for motor in motors.values():
        motor.backward(speed)

    time.sleep(duration)

    for motor in motors.values():
        motor.stop()

    counts = get_encoder_counts()
    print(f"  Encoder ticks: LF={counts['LF']}  LR={counts['LR']}  "
          f"RF={counts['RF']}  RR={counts['RR']}")


def test_spin(motors, speed=50, duration=3):
    """Spin in place — left side forward, right side backward."""
    print(f"\n=== Spin Test (CW) at {speed}% for {duration}s ===")
    reset_encoder_counts()

    motors["LF"].forward(speed)
    motors["LR"].forward(speed)
    motors["RF"].backward(speed)
    motors["RR"].backward(speed)

    time.sleep(duration)

    for motor in motors.values():
        motor.stop()

    counts = get_encoder_counts()
    print(f"  Encoder ticks: LF={counts['LF']}  LR={counts['LR']}  "
          f"RF={counts['RF']}  RR={counts['RR']}")


def test_speed_ramp(motors, duration=5):
    """Ramp speed from 0 to 100 and back down."""
    print(f"\n=== Speed Ramp Test (all motors forward) ===")
    reset_encoder_counts()

    # Ramp up
    for speed in range(0, 101, 10):
        print(f"  Speed: {speed}%")
        for motor in motors.values():
            motor.forward(speed)
        time.sleep(duration / 20)

    # Ramp down
    for speed in range(100, -1, -10):
        print(f"  Speed: {speed}%")
        for motor in motors.values():
            motor.forward(speed)
        time.sleep(duration / 20)

    for motor in motors.values():
        motor.stop()

    counts = get_encoder_counts()
    print(f"  Encoder ticks: LF={counts['LF']}  LR={counts['LR']}  "
          f"RF={counts['RF']}  RR={counts['RR']}")


def test_standby(motors, stby1, stby2):
    """Test standby pin — motors should not move when STBY is LOW."""
    print("\n=== Standby Pin Test ===")

    print("  Setting STBY LOW (disabled)...")
    stby1.off()
    stby2.off()
    reset_encoder_counts()

    for motor in motors.values():
        motor.forward(70)
    time.sleep(1)

    counts = get_encoder_counts()
    print(f"  Encoder ticks (should be ~0): LF={counts['LF']}  LR={counts['LR']}  "
          f"RF={counts['RF']}  RR={counts['RR']}")

    for motor in motors.values():
        motor.stop()

    print("  Setting STBY HIGH (enabled)...")
    stby1.on()
    stby2.on()
    time.sleep(0.5)


def test_encoders_manual():
    """Read encoders for 5 seconds — spin wheels by hand to verify."""
    print("\n=== Manual Encoder Test ===")
    print("  Spin each wheel by hand. Watching for 5 seconds...")
    reset_encoder_counts()

    time.sleep(5)

    counts = get_encoder_counts()
    print(f"  Encoder ticks: LF={counts['LF']}  LR={counts['LR']}  "
          f"RF={counts['RF']}  RR={counts['RR']}")

    for name, count in counts.items():
        status = "OK" if count > 0 else "NO TICKS - check wiring"
        print(f"  [{name}] {status}")


# ---------------------------------------------------------------------------
# IMU (MPU6500 + HMC5883L)
# ---------------------------------------------------------------------------
class IMU:
    def __init__(self, bus_num=I2C_BUS):
        self.bus = smbus.SMBus(bus_num)
        self.mpu_ok = False
        self.mag_ok = False

    def init_mpu6500(self):
        """Wake up MPU6500 and enable I2C bypass for magnetometer access."""
        try:
            who = self.bus.read_byte_data(MPU6500_ADDR, MPU6500_WHO_AM_I)
            print(f"  MPU6500 WHO_AM_I: 0x{who:02X} (expected 0x70 for MPU6500)")
            # Wake up (clear sleep bit)
            self.bus.write_byte_data(MPU6500_ADDR, MPU6500_PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            # Enable I2C bypass so HMC5883L is visible on the main I2C bus
            self.bus.write_byte_data(MPU6500_ADDR, MPU6500_INT_PIN_CFG, 0x02)
            time.sleep(0.01)
            self.mpu_ok = True
            print("  MPU6500 initialized OK")
        except OSError as e:
            print(f"  MPU6500 init FAILED: {e}")
            self.mpu_ok = False

    def init_hmc5883l(self):
        """Initialize HMC5883L magnetometer."""
        try:
            # Read identification registers (should be 'H', '4', '3')
            id_a = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_ID_A)
            id_b = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_ID_A + 1)
            id_c = self.bus.read_byte_data(HMC5883L_ADDR, HMC5883L_ID_A + 2)
            print(f"  HMC5883L ID: {chr(id_a)}{chr(id_b)}{chr(id_c)} (expected H43)")
            # 8 samples avg, 15 Hz output, normal measurement
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_CONFIG_A, 0x70)
            # Gain = 1090 LSB/Gauss (default)
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_CONFIG_B, 0x20)
            # Continuous measurement mode
            self.bus.write_byte_data(HMC5883L_ADDR, HMC5883L_MODE, 0x00)
            time.sleep(0.01)
            self.mag_ok = True
            print("  HMC5883L initialized OK")
        except OSError as e:
            print(f"  HMC5883L init FAILED: {e}")
            self.mag_ok = False

    def read_accel(self):
        """Read accelerometer X, Y, Z in g."""
        data = self.bus.read_i2c_block_data(MPU6500_ADDR, MPU6500_ACCEL_XOUT_H, 6)
        ax = struct.unpack('>h', bytes(data[0:2]))[0] / 16384.0
        ay = struct.unpack('>h', bytes(data[2:4]))[0] / 16384.0
        az = struct.unpack('>h', bytes(data[4:6]))[0] / 16384.0
        return ax, ay, az

    def read_gyro(self):
        """Read gyroscope X, Y, Z in deg/s."""
        data = self.bus.read_i2c_block_data(MPU6500_ADDR, MPU6500_GYRO_XOUT_H, 6)
        gx = struct.unpack('>h', bytes(data[0:2]))[0] / 131.0
        gy = struct.unpack('>h', bytes(data[2:4]))[0] / 131.0
        gz = struct.unpack('>h', bytes(data[4:6]))[0] / 131.0
        return gx, gy, gz

    def read_mag(self):
        """Read magnetometer X, Y, Z in Gauss."""
        data = self.bus.read_i2c_block_data(HMC5883L_ADDR, HMC5883L_DATA_OUT, 6)
        # HMC5883L byte order: X_H, X_L, Z_H, Z_L, Y_H, Y_L
        mx = struct.unpack('>h', bytes(data[0:2]))[0] / 1090.0
        mz = struct.unpack('>h', bytes(data[2:4]))[0] / 1090.0
        my = struct.unpack('>h', bytes(data[4:6]))[0] / 1090.0
        return mx, my, mz

    def read_heading(self):
        """Compute compass heading in degrees from magnetometer."""
        mx, my, _ = self.read_mag()
        heading = math.atan2(my, mx)
        if heading < 0:
            heading += 2 * math.pi
        return math.degrees(heading)

    def close(self):
        self.bus.close()


def test_imu_detect():
    """Scan I2C bus and check for MPU6500 and HMC5883L."""
    print("\n=== IMU I2C Detection ===")
    imu = IMU()
    imu.init_mpu6500()
    imu.init_hmc5883l()
    imu.close()
    return imu.mpu_ok, imu.mag_ok


def test_imu_accel_gyro(duration=5):
    """Read accelerometer and gyroscope for a few seconds."""
    print(f"\n=== IMU Accelerometer & Gyroscope Test ({duration}s) ===")
    imu = IMU()
    imu.init_mpu6500()
    if not imu.mpu_ok:
        imu.close()
        return

    print("  Keep the robot still for baseline, then tilt/rotate it.")
    print(f"  {'Time':>5s}  {'Ax':>7s} {'Ay':>7s} {'Az':>7s}  {'Gx':>8s} {'Gy':>8s} {'Gz':>8s}")
    print("  " + "-" * 58)

    start = time.time()
    try:
        while time.time() - start < duration:
            ax, ay, az = imu.read_accel()
            gx, gy, gz = imu.read_gyro()
            elapsed = time.time() - start
            print(f"  {elapsed:5.1f}  {ax:+7.3f} {ay:+7.3f} {az:+7.3f}  "
                  f"{gx:+8.2f} {gy:+8.2f} {gz:+8.2f}")
            time.sleep(0.2)
    except OSError as e:
        print(f"  Read error: {e}")

    # Sanity check — stationary Az should be ~1.0g
    ax, ay, az = imu.read_accel()
    magnitude = math.sqrt(ax**2 + ay**2 + az**2)
    print(f"\n  Accel magnitude: {magnitude:.3f} g (expected ~1.0 when stationary)")
    if 0.8 < magnitude < 1.2:
        print("  Accelerometer: OK")
    else:
        print("  Accelerometer: UNEXPECTED — check orientation or wiring")

    imu.close()


def test_imu_magnetometer(duration=5):
    """Read magnetometer and compute heading for a few seconds."""
    print(f"\n=== IMU Magnetometer Test ({duration}s) ===")
    imu = IMU()
    imu.init_mpu6500()  # needed to enable I2C bypass
    imu.init_hmc5883l()
    if not imu.mag_ok:
        imu.close()
        return

    print("  Rotate the robot slowly to see heading change.")
    print(f"  {'Time':>5s}  {'Mx':>7s} {'My':>7s} {'Mz':>7s}  {'Heading':>8s}")
    print("  " + "-" * 42)

    start = time.time()
    try:
        while time.time() - start < duration:
            mx, my, mz = imu.read_mag()
            heading = imu.read_heading()
            elapsed = time.time() - start
            print(f"  {elapsed:5.1f}  {mx:+7.3f} {my:+7.3f} {mz:+7.3f}  {heading:7.1f} deg")
            time.sleep(0.3)
    except OSError as e:
        print(f"  Read error: {e}")

    imu.close()


def test_imu_full(duration=10):
    """Read all IMU sensors simultaneously."""
    print(f"\n=== Full IMU Test — All 9 Axes ({duration}s) ===")
    imu = IMU()
    imu.init_mpu6500()
    imu.init_hmc5883l()
    if not imu.mpu_ok:
        print("  Cannot proceed without MPU6500.")
        imu.close()
        return

    print("  Move and rotate the robot to exercise all axes.")
    header = (f"  {'T':>4s}  "
              f"{'Ax':>6s} {'Ay':>6s} {'Az':>6s}  "
              f"{'Gx':>7s} {'Gy':>7s} {'Gz':>7s}")
    if imu.mag_ok:
        header += f"  {'Mx':>6s} {'My':>6s} {'Mz':>6s}  {'Hdg':>6s}"
    print(header)
    print("  " + "-" * (len(header) - 2))

    start = time.time()
    try:
        while time.time() - start < duration:
            ax, ay, az = imu.read_accel()
            gx, gy, gz = imu.read_gyro()
            elapsed = time.time() - start
            line = (f"  {elapsed:4.1f}  "
                    f"{ax:+6.2f} {ay:+6.2f} {az:+6.2f}  "
                    f"{gx:+7.1f} {gy:+7.1f} {gz:+7.1f}")
            if imu.mag_ok:
                mx, my, mz = imu.read_mag()
                heading = imu.read_heading()
                line += f"  {mx:+6.2f} {my:+6.2f} {mz:+6.2f}  {heading:5.1f}°"
            print(line)
            time.sleep(0.25)
    except OSError as e:
        print(f"  Read error: {e}")

    imu.close()
    print("\n  Full IMU test complete.")


# ---------------------------------------------------------------------------
# Main menu
# ---------------------------------------------------------------------------
def main():
    motors, encoders, stby1, stby2 = setup()
    print("VECTOR NAV — Motor, Encoder & IMU Test")
    print("=" * 40)

    menu = """
Select a test:
  --- Motors & Encoders ---
  1. Individual motor test
  2. All motors forward
  3. All motors backward
  4. Spin in place (CW)
  5. Speed ramp test
  6. Standby pin test
  7. Manual encoder test (spin wheels by hand)
  --- IMU ---
  8. IMU I2C detection
  9. Accelerometer & Gyroscope test
  10. Magnetometer test
  11. Full IMU test (all 9 axes)
  --- All ---
  12. Run ALL tests
  0. Quit
"""

    try:
        while True:
            print(menu)
            choice = input("Enter choice: ").strip()

            if choice == "1":
                test_individual_motors(motors)
            elif choice == "2":
                test_all_forward(motors)
            elif choice == "3":
                test_all_backward(motors)
            elif choice == "4":
                test_spin(motors)
            elif choice == "5":
                test_speed_ramp(motors)
            elif choice == "6":
                test_standby(motors, stby1, stby2)
            elif choice == "7":
                test_encoders_manual()
            elif choice == "8":
                test_imu_detect()
            elif choice == "9":
                test_imu_accel_gyro()
            elif choice == "10":
                test_imu_magnetometer()
            elif choice == "11":
                test_imu_full()
            elif choice == "12":
                test_encoders_manual()
                test_standby(motors, stby1, stby2)
                test_individual_motors(motors)
                test_all_forward(motors)
                test_all_backward(motors)
                test_spin(motors)
                test_speed_ramp(motors)
                test_imu_detect()
                test_imu_accel_gyro()
                test_imu_magnetometer()
                test_imu_full()
                print("\n=== ALL TESTS COMPLETE ===")
            elif choice == "0":
                break
            else:
                print("Invalid choice.")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Cleaning up...")
        for motor in motors.values():
            motor.cleanup()
        for enc in encoders.values():
            enc.close()
        stby1.off()
        stby2.off()
        stby1.close()
        stby2.close()
        print("Done.")


if __name__ == "__main__":
    main()
