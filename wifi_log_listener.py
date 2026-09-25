# FOR TESTING PURPOSES ONLY, TO REPLACE LATER

#!/usr/bin/env python3
"""Prints UDP lines sent by MotorC's wifi_log.c.

Usage: python wifi_log_listener.py [port]
(port defaults to 4210, matching LOG_SERVER_PORT in CMakeLists.txt)
"""
import socket
import sys

port = int(sys.argv[1]) if len(sys.argv) > 1 else 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Without a timeout, recvfrom() blocks at the OS level and Python never gets
# a chance to check for Ctrl+C until a packet arrives -- especially on
# Windows. Timing out periodically lets KeyboardInterrupt actually land.
sock.settimeout(1.0)

try:
    sock.bind(("0.0.0.0", port))
except OSError as e:
    print(f"Could not bind UDP port {port}: {e}")
    print("A previous run of this script may still be holding the port --")
    print("check Task Manager for a leftover python.exe and end it, then retry.")
    sock.close()
    sys.exit(1)

print(f"Listening for MotorC log lines on UDP port {port}... (Ctrl+C to stop)")

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue
        print(f"[{addr[0]}] {data.decode('utf-8', errors='replace')}")
except KeyboardInterrupt:
    print("\nStopping.")
finally:
    sock.close()
