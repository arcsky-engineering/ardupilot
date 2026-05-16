#!/usr/bin/env python3
"""Glitch-zoom v4 — focus on u-center vs gnssperiph command separation.

Goals:
  - List ALL ACK-ACK / ACK-NAK in absolute time order
  - Decode every CFG-MSG correctly (8-byte form is per-port rates)
  - Find when each "new" UBX message (NAV-SAT, NAV-SIG, NAV-CLOCK, NAV-COV)
    first appears in the stream — that's the moment u-center successfully
    enabled it
  - Look at the chronological *order of u-center commands* leading up to the
    glitch — is one of them the trigger?
"""
import os
import struct
from collections import Counter

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-184552.ubx"

MSG = {
    (0x01, 0x07): "NAV-PVT", (0x01, 0x04): "NAV-DOP", (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x22): "NAV-CLOCK", (0x01, 0x35): "NAV-SAT", (0x01, 0x43): "NAV-SIG",
    (0x01, 0x36): "NAV-COV", (0x01, 0x61): "NAV-EOE", (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x14): "NAV-HPPOSLLH", (0x01, 0x02): "NAV-POSLLH", (0x01, 0x12): "NAV-VELNED",
    (0x01, 0x34): "NAV-ORB",
    (0x02, 0x15): "RXM-RAWX", (0x02, 0x13): "RXM-SFRBX", (0x02, 0x31): "RXM-INTF",
    (0x05, 0x00): "ACK-ACK", (0x05, 0x01): "ACK-NAK",
    (0x06, 0x00): "CFG-PRT", (0x06, 0x01): "CFG-MSG", (0x06, 0x04): "CFG-RST",
    (0x06, 0x06): "CFG-DAT", (0x06, 0x08): "CFG-RATE", (0x06, 0x09): "CFG-CFG",
    (0x06, 0x13): "CFG-ANT", (0x06, 0x24): "CFG-NAV5", (0x06, 0x31): "CFG-TP5",
    (0x06, 0x3e): "CFG-GNSS",
    (0x06, 0x8a): "CFG-VALSET", (0x06, 0x8b): "CFG-VALGET", (0x06, 0x8c): "CFG-VALDEL",
    (0x0a, 0x04): "MON-VER", (0x0a, 0x09): "MON-HW", (0x0a, 0x0b): "MON-IO",
    (0x0a, 0x36): "MON-COMMS", (0x0a, 0x38): "MON-RF", (0x0a, 0x37): "MON-HW3",
    (0x0a, 0x31): "MON-SPAN", (0x0d, 0x03): "TIM-TM2",
}


def iter_ubx(data):
    i = 0
    while i < len(data) - 7:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls = data[i + 2]; mid = data[i + 3]
            length = data[i + 4] | (data[i + 5] << 8)
            if i + 8 + length > len(data):
                i += 1; continue
            payload = data[i + 6:i + 6 + length]
            ck_a = ck_b = 0
            for b in data[i + 2:i + 6 + length]:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if data[i + 6 + length] == ck_a and data[i + 7 + length] == ck_b:
                yield i, cls, mid, payload
                i += 8 + length
                continue
        i += 1


with open(FILE, "rb") as f:
    data = f.read()

records = []
for off, cls, mid, pl in iter_ubx(data):
    name = MSG.get((cls, mid), f"UNK_{cls:02x}{mid:02x}")
    records.append((off, name, pl))

pvt_anchors = []
for off, name, pl in records:
    if name == "NAV-PVT" and len(pl) >= 92:
        iTOW = struct.unpack("<I", pl[0:4])[0]
        pvt_anchors.append((off, iTOW))


def off_to_t(off):
    if not pvt_anchors:
        return None
    if off <= pvt_anchors[0][0]:
        return pvt_anchors[0][1] / 1000.0
    if off >= pvt_anchors[-1][0]:
        return pvt_anchors[-1][1] / 1000.0
    lo, hi = 0, len(pvt_anchors) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if pvt_anchors[mid][0] <= off:
            lo = mid
        else:
            hi = mid
    o0, t0 = pvt_anchors[lo]
    o1, t1 = pvt_anchors[hi]
    if o1 == o0: return t0 / 1000.0
    frac = (off - o0) / (o1 - o0)
    return (t0 + frac * (t1 - t0)) / 1000.0


t_start = pvt_anchors[0][1] / 1000

# Find event
window = []
event_t = None
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000.0
    numSV = pl[23]
    window = [(tt, n) for tt, n in window if t - tt <= 3.0]
    window.append((t, numSV))
    rmax = max(n for _, n in window)
    if rmax >= 10 and numSV + 10 <= rmax and event_t is None:
        event_t = t
        break

print(f"Event @ iTOW {event_t:.3f}s (+{event_t - t_start:.1f}s into capture)\n")


# === When did each "new" message first appear? ===
print("=== First appearance of each message type (capture time) ===")
first_seen = {}
for off, name, pl in records:
    if name not in first_seen:
        t = off_to_t(off)
        if t is None: continue
        first_seen[name] = t - t_start
interesting = ["NAV-PVT", "NAV-DOP", "NAV-TIMEGPS", "NAV-CLOCK", "NAV-SAT", "NAV-SIG", "NAV-COV",
               "MON-HW", "MON-IO", "MON-SPAN", "MON-VER", "MON-COMMS", "NAV-EOE"]
for n in interesting:
    if n in first_seen:
        t = first_seen[n]
        rel_event = t - (event_t - t_start)
        print(f"  {n:<15}  first@ +{t:>6.1f}s   (event-rel {rel_event:+6.1f}s)")
    else:
        print(f"  {n:<15}  NOT PRESENT")


# === Decode every CFG-MSG properly ===
def decode_cfg_msg(pl):
    if len(pl) == 2:
        return f"POLL cls=0x{pl[0]:02x} id=0x{pl[1]:02x}"
    if len(pl) == 3:
        target = MSG.get((pl[0], pl[1]), f"UNK_{pl[0]:02x}{pl[1]:02x}")
        return f"SET {target} rate={pl[2]} (current port)"
    if len(pl) == 8:
        target = MSG.get((pl[0], pl[1]), f"UNK_{pl[0]:02x}{pl[1]:02x}")
        rates = [pl[2], pl[3], pl[4], pl[5], pl[6], pl[7]]
        return f"SET {target} rates(I2C/UART1/UART2/USB/SPI/?)={rates}"
    return f"len={len(pl)} hex={pl[:8].hex()}"


# === All ACK-ACK and ACK-NAK in capture-time order, decoded ===
print("\n=== ALL ACK responses in chronological order ===")
print(f"  {'rel-start':>9}  {'rel-event':>9}  {'ack':>7}  target")
n_ack = 0
n_nak = 0
for off, name, pl in records:
    if name not in ("ACK-ACK", "ACK-NAK"):
        continue
    t = off_to_t(off)
    if t is None: continue
    target = MSG.get((pl[0], pl[1]), f"UNK_{pl[0]:02x}{pl[1]:02x}") if len(pl) >= 2 else "?"
    if name == "ACK-ACK":
        n_ack += 1
        mark = " <- ACK"
    else:
        n_nak += 1
        mark = ""
    print(f"  +{t - t_start:>7.2f}s   {t - event_t:>+8.2f}s  {name[-3:]:>7}  for {target}{mark}")
print(f"\n  Total: ACK={n_ack}, NAK={n_nak}")


# === Show every CFG-MSG sent host->receiver in chronological order ===
# Match each CFG-MSG to the subsequent ACK/NAK to know if it succeeded.
print("\n=== ALL host-issued CFG-MSG commands, decoded ===")
acks_by_target = []
for off, name, pl in records:
    if name in ("ACK-ACK", "ACK-NAK") and len(pl) >= 2:
        t = off_to_t(off)
        acks_by_target.append((t, name, pl[0], pl[1]))

n_match = 0
for off, name, pl in records:
    if name != "CFG-MSG":
        continue
    t = off_to_t(off)
    if t is None: continue
    decoded = decode_cfg_msg(pl)
    # Find next ACK after this CFG-MSG, for class=0x06, id=0x01 (CFG-MSG)
    ack_status = "?"
    for at, an, ac, ai in acks_by_target:
        if at < t: continue
        if at - t > 2.0: break
        if ac == 0x06 and ai == 0x01:
            ack_status = "ACK" if an == "ACK-ACK" else "NAK"
            break
    print(f"  +{t - t_start:>7.2f}s ({t - event_t:>+7.2f}s) [{ack_status}] {decoded}")


# === Also: when did NAV-SAT, NAV-SIG, NAV-CLOCK, NAV-COV first arrive,
# vs when did u-center likely send the enable command for each? ===
# Their first appearance == they were just enabled.

# === Look at NAV-CLOCK timeline relative to event ===
print("\n=== NAV-CLOCK summary across whole capture ===")
clock_samples = []
for off, name, pl in records:
    if name != "NAV-CLOCK" or len(pl) < 20:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    clkB = struct.unpack("<i", pl[4:8])[0]
    clkD = struct.unpack("<i", pl[8:12])[0]
    tAcc = struct.unpack("<I", pl[12:16])[0]
    fAcc = struct.unpack("<I", pl[16:20])[0]
    clock_samples.append((iTOW / 1000, clkB, clkD, tAcc, fAcc))

if clock_samples:
    print(f"  Samples: {len(clock_samples)}")
    print(f"  First sample at +{clock_samples[0][0] - t_start:.1f}s, last at +{clock_samples[-1][0] - t_start:.1f}s")
    print(f"  {'rel-event':>9}  {'clkB(ns)':>12}  {'clkD(ns/s)':>10}  {'tAcc(ns)':>10}  {'fAcc(ps/s)':>10}")
    for it, b, d, ta, fa in clock_samples:
        rel = it - event_t
        if abs(rel) > 60: continue
        print(f"  {rel:>+8.2f}s  {b:>+12d}  {d:>+10d}  {ta:>10d}  {fa:>10d}")
