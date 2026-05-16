#!/usr/bin/env python3
"""Trace message lifecycle + check if the AGC transition happened despite no
visible glitch (silent bug fire).

Also examines:
  - MON-HW agcCnt across the entire capture (did it shift?)
  - Per-30s NAV-PVT sat count summary (any transient sat dips?)
  - Per-10s message-rate bins for NAV-SAT/SIG/CLOCK/COV
"""
import os
import struct
from collections import Counter, defaultdict

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-185749.ubx"


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

print(f"=== FILE: {os.path.basename(FILE)} ({len(data):,} bytes) ===\n")

NAMES = {(0x01,0x07):"NAV-PVT",(0x01,0x35):"NAV-SAT",(0x01,0x43):"NAV-SIG",
         (0x01,0x22):"NAV-CLOCK",(0x01,0x36):"NAV-COV",(0x0a,0x09):"MON-HW"}

records = []
for off, cls, mid, pl in iter_ubx(data):
    name = NAMES.get((cls, mid))
    if name:
        records.append((off, name, pl))

pvt_anchors = []
for off, name, pl in records:
    if name == "NAV-PVT" and len(pl) >= 92:
        iTOW = struct.unpack("<I", pl[0:4])[0]
        pvt_anchors.append((off, iTOW))
t_start = pvt_anchors[0][1] / 1000
t_end = pvt_anchors[-1][1] / 1000
print(f"Capture span: {t_end - t_start:.1f}s")

# === MON-HW across entire capture (looking for agc transitions) ===
print(f"\n=== MON-HW across entire capture (every sample) ===")
print(f"  {'rel_t':>8}  noise  agcCnt  aStat  aPow  jam")
hw_samples = []
for off, name, pl in records:
    if name != "MON-HW" or len(pl) < 60:
        continue
    # No iTOW in MON-HW; estimate by stream position
    # Find nearest PVT anchor
    if off <= pvt_anchors[0][0]:
        t = pvt_anchors[0][1] / 1000
    elif off >= pvt_anchors[-1][0]:
        t = pvt_anchors[-1][1] / 1000
    else:
        lo, hi = 0, len(pvt_anchors) - 1
        while lo + 1 < hi:
            m = (lo + hi) // 2
            if pvt_anchors[m][0] <= off: lo = m
            else: hi = m
        o0, t0 = pvt_anchors[lo]; o1, t1 = pvt_anchors[hi]
        t = (t0 + (t1 - t0) * (off - o0) / (o1 - o0)) / 1000
    rel = t - t_start
    n = struct.unpack("<H", pl[16:18])[0]
    agc = struct.unpack("<H", pl[18:20])[0]
    aS = pl[20]; aP = pl[21]; jam = pl[45]
    hw_samples.append((rel, n, agc, aS, aP, jam))

# Look for agcCnt transitions
prev_agc = None
transitions = []
for rel, n, agc, aS, aP, jam in hw_samples:
    if prev_agc is not None and agc != prev_agc:
        transitions.append((rel, prev_agc, agc))
    prev_agc = agc
print(f"\n  Total MON-HW samples: {len(hw_samples)}")
print(f"  agcCnt transitions detected: {len(transitions)}")
for rel, before, after in transitions:
    print(f"    +{rel:>7.1f}s  agcCnt {before} -> {after}")

# Show MON-HW sampled every ~30s
print(f"\n  MON-HW every ~30s:")
last_print = -100
for rel, n, agc, aS, aP, jam in hw_samples:
    if rel - last_print >= 30:
        print(f"    +{rel:>7.1f}s  noise={n:>3}  agcCnt={agc:>4}  aS={aS}  aP={aP}  jam={jam:>2}")
        last_print = rel

# === Look for transient sat-count dips in NAV-PVT (small bugs that didn't trip threshold) ===
print(f"\n=== NAV-PVT sat-count dips (any drop >= 4 from baseline) ===")
prev_sv = None
recent_max = 0
window = []
dips = []
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    rel = t - t_start
    sv = pl[23]
    gSpd = struct.unpack("<i", pl[60:64])[0]
    hAcc = struct.unpack("<I", pl[40:44])[0] / 1000
    window = [(tt, n) for tt, n in window if rel - tt <= 5.0]
    window.append((rel, sv))
    rmax = max(n for _, n in window)
    if rmax >= 10 and rmax - sv >= 4:
        dips.append((rel, rmax, sv, gSpd, hAcc))

if dips:
    print(f"  Total dips: {len(dips)}")
    for rel, rmax, sv, gSpd, hAcc in dips[:30]:
        print(f"    +{rel:>7.2f}s  sv {rmax}->{sv}  gSpd={gSpd}  hAcc={hAcc:.2f}")
    if len(dips) > 30:
        print(f"    ... ({len(dips) - 30} more)")
else:
    print(f"  No sat dips >= 4 detected. Capture appears CLEAN.")

# === gSpeed jumps >300 mm/s (phantom velocity detector) ===
print(f"\n=== gSpeed jumps >= 300 mm/s ===")
prev_gspd = None
jumps = []
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    rel = t - t_start
    gSpd = struct.unpack("<i", pl[60:64])[0]
    if prev_gspd is not None and abs(gSpd - prev_gspd) >= 300:
        jumps.append((rel, prev_gspd, gSpd))
    prev_gspd = gSpd
if jumps:
    for rel, b, a in jumps[:30]:
        print(f"    +{rel:>7.2f}s  gSpd {b} -> {a} mm/s")
    if len(jumps) > 30:
        print(f"    ... ({len(jumps) - 30} more)")
else:
    print(f"  No gSpeed jumps >= 300 mm/s.")
