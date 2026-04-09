#!/bin/bash
# Control SLLiDAR motor via serial protocol commands
# Usage: lidar_power.sh [on|off]

PORT="/dev/ttyUSB0"

case "$1" in
    off)
        python3 -c "
import serial, struct, time

s = serial.Serial('$PORT', 115200, timeout=1)
time.sleep(0.1)

# Send STOP command: sync(0xA5) + cmd(0x25)
s.write(b'\xA5\x25')
time.sleep(0.1)

# Send SET_MOTOR_PWM(0xF0) with PWM=0
# Format: sync(0xA5) + cmd(0xF0) + size(2) + pwm_le16(0x0000) + checksum
payload = struct.pack('<H', 0)  # PWM = 0, little-endian uint16
size = len(payload)
packet = bytes([0xA5, 0xF0, size]) + payload
checksum = 0
for b in packet:
    checksum ^= b
packet += bytes([checksum])
s.write(packet)

# Also try DTR
s.setDTR(True)

time.sleep(0.1)
s.close()
print('LiDAR motor stopped')
"
        ;;
    on)
        python3 -c "
import serial, struct, time

s = serial.Serial('$PORT', 115200, timeout=1)
time.sleep(0.1)

# Send RESET command to reinitialize
s.write(b'\xA5\x40')
time.sleep(0.5)

# Clear DTR to allow motor
s.setDTR(False)
time.sleep(0.1)

# Send SET_MOTOR_PWM with default speed (660)
payload = struct.pack('<H', 660)
size = len(payload)
packet = bytes([0xA5, 0xF0, size]) + payload
checksum = 0
for b in packet:
    checksum ^= b
packet += bytes([checksum])
s.write(packet)

time.sleep(0.1)
s.close()
print('LiDAR motor started')
"
        ;;
    *)
        echo "Usage: $0 [on|off]"
        exit 1
        ;;
esac
