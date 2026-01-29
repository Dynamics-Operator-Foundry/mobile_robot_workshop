#!/usr/bin/env python3
from inputs import devices
import socket
import json

# ------------------------------
# CONFIGURATION
# ------------------------------
UDP_IP = "255.255.255.255"   # Broadcast; or set to a specific IP (e.g. "192.168.1.42")
UDP_PORT = 5005               # Any open UDP port on your LAN

# ------------------------------
# FIND JOYSTICK DEVICE
# ------------------------------
joys = devices.gamepads
if not joys:
    print("❌ No joystick found. Make sure RadioMaster is in USB joystick mode.")
    exit(1)

joy = joys[0]
print(f"✅ Connected to: {joy.name}")
print(f"📡 Sending raw UDP packets to {UDP_IP}:{UDP_PORT}")
print("Press Ctrl+C to stop.\n")

# ------------------------------
# SET UP UDP SOCKET
# ------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# ------------------------------
# MAIN LOOP
# ------------------------------
axis_values = {}

try:
    while True:
        events = joy.read()
        for e in events:
            if e.ev_type == "Absolute":
                # Store raw value directly
                axis_values[e.code] = e.state

                # Send JSON via UDP
                message = json.dumps(axis_values).encode('utf-8')
                sock.sendto(message, (UDP_IP, UDP_PORT))

                # Optional: print live updates
                print(axis_values, end="\r")

except KeyboardInterrupt:
    print("Stopped.")
    sock.close()
