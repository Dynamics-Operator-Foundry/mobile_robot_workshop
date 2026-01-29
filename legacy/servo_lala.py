#!/usr/bin/env python3
# raw_serial_dump.py
import serial, time, sys

port = "/dev/ttyUSB0"   # set to your adapter
baud = 115200           # try 115200, 9600, or whatever your servo expects
timeout = 0.5

try:
    s = serial.Serial(port, baudrate=baud, timeout=timeout)
except Exception as e:
    print("Can't open serial:", e)
    sys.exit(1)

print("Opened", port, "baud", baud)
print("Listening for 5 seconds... press Ctrl-C to stop")
start = time.time()
buf = b''
try:
    while time.time() - start < 5:
        b = s.read(1)
        if b:
            buf += b
            # print bytes as hex grouped
            print("Got:", b.hex(), end=' ', flush=True)
    print("\n\nFull capture (hex):", buf.hex())
finally:
    s.close()