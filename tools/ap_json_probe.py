#!/usr/bin/env python3
"""Smoke-test the ArduPilot JSON bridge without running ArduPilot.

Sends one JSON-interface servo packet (magic 18458, hover throttle) to the
sim's UDP port and prints the JSON state line that comes back. Run the Godot
scene with SITLManager.enabled_ardupilot = true first.

Usage: python ap_json_probe.py [host] [port] [count]
"""
import argparse
import json
import socket
import struct
import sys
import time

parser = argparse.ArgumentParser(description="Smoke-test the ArduPilot JSON bridge without running ArduPilot.")
parser.add_argument("host", nargs="?", default="127.0.0.1", help="sim host (default: 127.0.0.1)")
parser.add_argument("port", nargs="?", type=int, default=9002, help="sim UDP port (default: 9002)")
parser.add_argument("count", nargs="?", type=int, default=10,
                    help="number of frames to send (default: 10)")
args = parser.parse_args()
host, port, count = args.host, args.port, args.count

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

MAGIC = 18458
RATE  = 400
pwm   = [1500, 1500, 1500, 1500] + [1000] * 12  # hover-ish on a quad

for frame in range(1, count + 1):
    pkt = struct.pack("<HHI16H", MAGIC, RATE, frame, *pwm)
    try:
        sock.sendto(pkt, (host, port))
        data, addr = sock.recvfrom(4096)
    except (socket.timeout, ConnectionRefusedError, ConnectionResetError):
        print(f"No sim listening on {host}:{port} — start the Godot demo with the "
              f"ArduPilot JSON bridge enabled (SITLManager.enabled_ardupilot)")
        sys.exit(1)
    line = data.decode(errors="replace").strip()
    state = json.loads(line)
    print(f"frame {frame}: t={state['timestamp']:.4f}s "
          f"pos={state['position']} att={state['attitude']}")
    time.sleep(0.0025)

print("OK — bridge is replying with lockstep JSON state.")
