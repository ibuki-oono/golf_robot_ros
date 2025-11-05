#!/usr/bin/env python3
import serial
import time

# Configure serial port
serial_port = '/dev/ttyACM0'
baudrate = 115200  # adjust if needed

try:
    ser = serial.Serial(serial_port, baudrate, timeout=1)
    time.sleep(2)  # wait for Arduino / MCU to initialize
except serial.SerialException as e:
    print(f"Failed to open serial port {serial_port}: {e}")
    exit(1)

# Build command
cmd = "VEL,1666,1666,0\n"

# Write to serial
while True:    
    ser.write(cmd.encode('utf-8'))
    print(f"Sent: {cmd.strip()}")
    time.sleep(0.01)  # send command every second

# Close serial
ser.close()
