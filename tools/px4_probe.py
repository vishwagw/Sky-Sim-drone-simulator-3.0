#!/usr/bin/env python3
"""Smoke-test the PX4 bridge without running PX4.

Connects to the sim's TCP port 4560 (as PX4 SITL would), prints the message
IDs streaming from the sim (expect HIL_SENSOR=107 every tick, HIL_GPS=113,
HEARTBEAT=0), and replies with a HIL_ACTUATOR_CONTROLS frame so the sim
reports the PX4 bridge as the active control source.

Usage: python px4_probe.py [host] [port] [seconds]
"""
import argparse
import socket
import struct
import sys
import time

parser = argparse.ArgumentParser(description="Smoke-test the PX4 bridge without running PX4.")
parser.add_argument("host", nargs="?", default="127.0.0.1", help="sim host (default: 127.0.0.1)")
parser.add_argument("port", nargs="?", type=int, default=4560, help="sim TCP port (default: 4560)")
parser.add_argument("seconds", nargs="?", type=float, default=3.0,
                    help="how long to listen, in seconds (default: 3.0)")
args = parser.parse_args()
host, port, dur = args.host, args.port, args.seconds

X25_INIT = 0xFFFF
def x25(data: bytes, crc: int = X25_INIT) -> int:
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

def mav2_frame(msg_id: int, payload: bytes, crc_extra: int, seq: int) -> bytes:
    hdr = struct.pack("<BBBBBBBBBB", 0xFD, len(payload), 0, 0, seq, 1, 200,
                      msg_id & 0xFF, (msg_id >> 8) & 0xFF, (msg_id >> 16) & 0xFF)
    crc = x25(hdr[1:] + payload)
    crc = x25(bytes([crc_extra]), crc)
    return hdr + payload + struct.pack("<H", crc)

def hil_actuator_controls(t_us: int, motors) -> bytes:
    # wire order: time_usec u64, flags u64, controls f32[16], mode u8
    controls = list(motors) + [0.0] * (16 - len(motors))
    payload = struct.pack("<QQ16fB", t_us, 0, *controls, 129)
    return mav2_frame(93, payload, 47, 0)

try:
    sock = socket.create_connection((host, port), timeout=5.0)
except (ConnectionRefusedError, socket.timeout):
    print(f"No sim listening on {host}:{port} — start the Godot demo with the PX4 bridge enabled")
    sys.exit(1)
sock.settimeout(1.0)
print(f"connected to sim at {host}:{port}")

counts = {}
buf = b""
t_end = time.time() + dur
sent_reply = False
while time.time() < t_end:
    try:
        chunk = sock.recv(4096)
    except socket.timeout:
        continue
    if not chunk:
        print("sim closed the connection")
        break
    buf += chunk
    # scan for MAVLink v2 frames
    while len(buf) >= 12:
        if buf[0] != 0xFD:
            buf = buf[1:]
            continue
        plen = buf[1]
        need = 10 + plen + 2
        if len(buf) < need:
            break
        msg_id = buf[7] | (buf[8] << 8) | (buf[9] << 16)
        counts[msg_id] = counts.get(msg_id, 0) + 1
        if msg_id == 107 and not sent_reply:
            t_us = struct.unpack("<Q", buf[10:18])[0]
            sock.sendall(hil_actuator_controls(t_us, [0.6, 0.6, 0.6, 0.6]))
            sent_reply = True
            print("sent HIL_ACTUATOR_CONTROLS (4 motors @ 0.6)")
        buf = buf[need:]

names = {0: "HEARTBEAT", 107: "HIL_SENSOR", 113: "HIL_GPS"}
print("received:", {names.get(k, k): v for k, v in sorted(counts.items())})
ok = counts.get(107, 0) > 0
print("OK — sim is streaming HIL sensors." if ok
      else "FAIL — no HIL_SENSOR received; is enabled_px4 on and the scene running?")
