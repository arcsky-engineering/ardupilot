#!/usr/bin/env python3
"""Enable NAV-CLOCK, NAV-COV, NAV-SAT, NAV-SIG on the u-blox via the
gps_debug tunnel's TCP socket. No u-center needed.

Run with the tunnel already started in gps_debug (defaults: TCP 127.0.0.1:2001,
NOT locked, recording RX to .ubx).

Sends UBX-CFG-MSG 3-byte form (set rate on current port) to enable each
message at rate=1 (every nav epoch). Sends in order: CLOCK -> COV -> SAT -> SIG
so the most informative ones get on first.

Optionally re-sends every 30 sec to defeat any host-side auto-disable.
"""
import argparse
import socket
import struct
import sys
import time

# UBX class/id for the messages we want
TARGETS = [
    ("NAV-CLOCK", 0x01, 0x22),
    ("NAV-COV",   0x01, 0x36),
    ("NAV-SAT",   0x01, 0x35),
    ("NAV-SIG",   0x01, 0x43),
]


def ubx_frame(cls, mid, payload):
    """Build a UBX frame: sync(2) + cls(1) + id(1) + len(2) + payload + ck(2)."""
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xb5\x62" + body + bytes([ck_a, ck_b])


def cfg_msg_enable(target_cls, target_id, rate=1):
    """3-byte CFG-MSG: set rate on current port (UART1 in our case)."""
    payload = bytes([target_cls, target_id, rate])
    return ubx_frame(0x06, 0x01, payload)


def send_enables(sock, rate=1):
    for name, c, i in TARGETS:
        frame = cfg_msg_enable(c, i, rate)
        sock.sendall(frame)
        print(f"  sent CFG-MSG enable {name}  (rate={rate})")
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2001)
    ap.add_argument("--repeat-sec", type=float, default=0.0,
                    help="If >0, re-send enables every N seconds forever.")
    ap.add_argument("--rate", type=int, default=1,
                    help="Message rate (1 = every nav epoch; 5 = every 5th).")
    args = ap.parse_args()

    print(f"Connecting to {args.host}:{args.port}...")
    sock = socket.create_connection((args.host, args.port), timeout=5.0)
    print("Connected. Sending enables...")
    send_enables(sock, rate=args.rate)

    if args.repeat_sec > 0:
        print(f"\nRe-sending every {args.repeat_sec}s. Ctrl-C to stop.")
        try:
            while True:
                time.sleep(args.repeat_sec)
                send_enables(sock, rate=args.rate)
                print(f"  --- re-sent at {time.strftime('%H:%M:%S')}")
        except KeyboardInterrupt:
            print("\nStopped.")
    sock.close()


if __name__ == "__main__":
    main()
