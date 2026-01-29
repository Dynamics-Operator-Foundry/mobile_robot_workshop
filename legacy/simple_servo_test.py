#!/usr/bin/env python3
"""Simple servo test without full initialization."""

import serial
import time

port = "/dev/ttyUSB0"
baud = 115200

# LX16A command to set motor mode (simplified)
def make_packet(servo_id, cmd, params=[]):
    """Create a command packet."""
    length = 3 + len(params)
    packet = [0x55, 0x55, servo_id, length, cmd] + params
    checksum = (~sum(packet[2:]) & 0xFF)
    packet.append(checksum)
    return bytes(packet)

def motor_mode(ser, servo_id, speed):
    """Set motor mode with speed (-1000 to 1000)."""
    # Command 29: SERVO_MOTOR_MODE_WRITE
    speed = max(-1000, min(1000, speed))
    if speed < 0:
        speed = 65536 + speed
    
    low_byte = speed & 0xFF
    high_byte = (speed >> 8) & 0xFF
    packet = make_packet(servo_id, 29, [0x01, low_byte, high_byte, 0x00])
    ser.write(packet)
    time.sleep(0.01)

print("Opening serial port...")
ser = serial.Serial(port, baudrate=baud, timeout=0.5)
time.sleep(0.1)

print("Testing servos 2 and 3...")

try:
    print("\nSpinning forward for 3 seconds...")
    motor_mode(ser, 2, 300)
    motor_mode(ser, 3, 300)
    time.sleep(3)
    
    print("Spinning backward for 3 seconds...")
    motor_mode(ser, 2, -300)
    motor_mode(ser, 3, -300)
    time.sleep(3)
    
    print("Stopping...")
    motor_mode(ser, 2, 0)
    motor_mode(ser, 3, 0)
    
    print("Test complete!")
    
except KeyboardInterrupt:
    print("\nStopped by user")
    motor_mode(ser, 2, 0)
    motor_mode(ser, 3, 0)
finally:
    ser.close()
