import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

try:
    while True:
        # Forward
        ser.write("VEL,2000,2000,0\n".encode())
        ser.flush()
        time.sleep(1)

        # Stop
        ser.write("VEL,0,0,0\n".encode())
        ser.flush()
        time.sleep(1)

        # Reverse
        ser.write("VEL,-2000,-2000,0\n".encode())
        ser.flush()
        time.sleep(1)

        # Stop again
        ser.write("VEL,0,0,0\n".encode())
        ser.flush()
        time.sleep(1)

except KeyboardInterrupt:
    ser.write("VEL,0,0,0\n".encode())
    ser.flush()
finally:
    ser.close()
