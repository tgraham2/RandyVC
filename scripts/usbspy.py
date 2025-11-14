#!/usr/bin/env python3
import serial

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.05)

try:
    while True:
        data = ser.read(64)
        if data:
            print(" ".join(f"{b:02X}" for b in data))
except KeyboardInterrupt:
    pass
finally:
    ser.close()
