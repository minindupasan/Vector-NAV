#!/usr/bin/env python3
"""
Wheel direction test for VECTOR NAV.
Drives each wheel one at a time and publishes /joint_states + /wheel/odom
so you can verify RViz matches real-world direction.

Run inside the Docker container while hw_slam.launch.py is running on Jetson:
  docker exec -it vector-control python3 \
    /ros2_ws/src/vector_control/vector_control/wheel_test.py

Each wheel spins for 2 seconds. Watch RViz and the real robot.
Report if the RViz wheel matches the real wheel direction.
"""

import math
import time
import threading
import glob as _glob

import gpiozero
from gpiozero import PWMOutputDevice, DigitalOutputDevice, Button
from gpiozero.pins.lgpio import LGPIOFactory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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

# Maps motor name to URDF joint name
JOINT_MAP = {
    'LF': 'wheel_fl_joint',
    'LR': 'wheel_rl_joint',
    'RF': 'wheel_fr_joint',
    'RR': 'wheel_rr_joint',
}

# Negate for RViz: LF (raw inverted), RF/RR (URDF right-side mirrored)
# LR is normal raw AND left-side URDF → no negation needed
ENCODER_INVERTED = {'LF', 'LR', 'RF', 'RR'}

STBY_PINS = [13, 23]
PWM_FREQ = 1000
TEST_SPEED = 0.3
TEST_DURATION = 2.0
TICKS_PER_REV = 225


def main():
    rclpy.init()
    node = Node('wheel_test')
    pub = node.create_publisher(JointState, '/joint_states', 10)

    joint_names = ['wheel_fl_joint', 'wheel_rl_joint', 'wheel_fr_joint', 'wheel_rr_joint']
    joint_positions = [0.0, 0.0, 0.0, 0.0]
    rad_per_tick = (2.0 * math.pi) / TICKS_PER_REV

    # Enable standby
    stbys = []
    for pin in STBY_PINS:
        s = DigitalOutputDevice(pin)
        s.on()
        stbys.append(s)

    print('=' * 55)
    print(' VECTOR NAV — Wheel Direction Test (with RViz)')
    print('=' * 55)
    print()
    print('Watch both the REAL wheel and the RViz wheel.')
    print('Convention: FORWARD = wheel pushes robot forward')
    print()

    test_order = ['LF', 'LR', 'RF', 'RR']

    for motor_name in test_order:
        pins = MOTORS[motor_name]
        enc_pins = ENCODERS[motor_name]
        joint_name = JOINT_MAP[motor_name]
        joint_idx = joint_names.index(joint_name)
        inverted = motor_name in ENCODER_INVERTED

        print(f'--- Testing {motor_name} ({joint_name}) ---')
        if inverted:
            print(f'  (encoder inverted — ticks will be negated)')

        # Setup motor
        pwm = PWMOutputDevice(pins['pwm'], frequency=PWM_FREQ)
        in1 = DigitalOutputDevice(pins['in1'])
        in2 = DigitalOutputDevice(pins['in2'])

        # Setup encoder
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

        # Spin forward
        print(f'  Spinning {motor_name} FORWARD for {TEST_DURATION}s...')
        ticks = 0
        in1.on()
        in2.off()
        pwm.value = TEST_SPEED

        start = time.time()
        while time.time() - start < TEST_DURATION:
            with lock:
                raw = ticks
            # Apply inversion
            corrected = -raw if inverted else raw
            joint_positions[joint_idx] = corrected * rad_per_tick

            js = JointState()
            js.header.stamp = node.get_clock().now().to_msg()
            js.name = joint_names
            js.position = list(joint_positions)
            pub.publish(js)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.02)

        pwm.value = 0
        in1.off()
        in2.off()

        with lock:
            raw_ticks = ticks
        corrected_ticks = -raw_ticks if inverted else raw_ticks

        print(f'  Raw encoder ticks: {raw_ticks}')
        print(f'  Corrected ticks:   {corrected_ticks}')
        print(f'  → {"POSITIVE (forward)" if corrected_ticks > 0 else "NEGATIVE (backward)" if corrected_ticks < 0 else "ZERO"}')
        print()

        while True:
            answer = input(
                f'  Did REAL {motor_name} move FORWARD? (y/n): '
            ).strip().lower()
            if answer in ('y', 'n'):
                break
        real_fwd = answer == 'y'

        while True:
            answer = input(
                f'  Did RViz {joint_name} move FORWARD (same direction)? (y/n): '
            ).strip().lower()
            if answer in ('y', 'n'):
                break
        rviz_fwd = answer == 'y'

        if real_fwd and rviz_fwd:
            print(f'  ✓ {motor_name}: Real=FWD, RViz=FWD — OK')
        elif real_fwd and not rviz_fwd:
            print(f'  ✗ {motor_name}: Real=FWD, RViz=BWD — encoder inversion wrong')
        elif not real_fwd and rviz_fwd:
            print(f'  ✗ {motor_name}: Real=BWD, RViz=FWD — motor pins swapped')
        else:
            print(f'  ✗ {motor_name}: Real=BWD, RViz=BWD — motor pins AND encoder need fixing')

        print()

        # Reset joint position for next test
        joint_positions[joint_idx] = 0.0

        # Cleanup
        pwm.close()
        in1.close()
        in2.close()
        phase_a.close()
        phase_b.close()
        time.sleep(0.5)

    # Cleanup
    for s in stbys:
        s.off()
        s.close()

    node.destroy_node()
    rclpy.shutdown()
    print('Done! Fix any issues reported above in motor_driver_node.py')


if __name__ == '__main__':
    main()
