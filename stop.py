#!/usr/bin/env python3

import serial

ser = serial.Serial('/dev/cu.usbserial-A50285BI', 115200, timeout=5)
print(ser.name)

ser.write(b'$');
ser.write(b"stoplds$");

