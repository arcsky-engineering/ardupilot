#!/usr/bin/env python3
"""Glitch-zoom v5 — confirms diag-message enables AND zooms the event.

Hypothesis under test: clicking the new "Enable NAV-SAT/SIG/CLOCK/COV" button
triggered the bug immediately. Look at:
  - When each diagnostic message first appeared in the stream
  - Whether the bug's signature fired and when (relative to those first-appearances)
  - The NAV-CLOCK data through the event (clock bias/drift) — this is the key
    new data we've never captured before
  - NAV-COV through the event (receiver's own covariance estimate)
  - NAV-SAT per-sat tracking quality through the event
  - MON-HW agcCnt transitions across the whole capture
"""
import os
import struct
from collections import Counter

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-192235.ubx"

NAMES = {(0x01, 0x07): "NAV-PVT", (0x01, 0x04): "NAV-DOP", (0x01, 0x20): "NAV-TIMEGPS",
         (0x01, 0x22): "NAV-CLOCK", (0x01, 0x35): "NAV-SAT", (0x01, 0x43): "NAV-SIG",
         (0x01, 0x36): "NAV-COV", (0x01, 0x03): "NAV-STATUS",
         (0x05, 0x00): "ACK-ACK", (0x05, 0x01): "ACK-NAK",
         (0x06, 0x01): "CFG-MSG", (0x06, 0x8b): "CFG-VALGET",
         (0x06, 0x00): "CFG-PRT", (0x06, 0x08): "CFG-RATE",
         (0x06, 0x24): "CFG-NAV5", (0x06, 0x31): "CFG-TP5", (0x06, 0x3e): "CFG-GNSS",
         (0x0a, 0x04): "MON-VER", (0x0a, 0x09): "MON-HW", (0x0a, 0x0b): "MON-IO",
         (0x0a, 0x31): "MON-SPAN", (0x0d, 0x03): "TIM-TM2"}


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

records = []
for off, cls, mid, pl in iter_ubx(data):
    name = NAMES.get((cls, mid), f"UNK_{cls:02x}{mid:02x}")
    records.append((off, name, pl))

print(f"=== INVENTORY ({len(records)} total) ===")
ctr = Counter(r[1] for r in records)
for n, c in ctr.most_common(25):
    print(f"  {n:<15} {c}")

pvt_anchors = []
for off, name, pl in records:
    if name == "NAV-PVT" and len(pl) >= 92:
        iTOW = struct.unpack("<I", pl[0:4])[0]
        pvt_anchors.append((off, iTOW))
if not pvt_anchors:
    print("\nNo NAV-PVT — cannot derive time axis.")
    raise SystemExit
t_start = pvt_anchors[0][1] / 1000
t_end = pvt_anchors[-1][1] / 1000
print(f"\nCapture span (NAV-PVT): {t_end - t_start:.1f}s")


def off_to_t(off):
    if off <= pvt_anchors[0][0]: return pvt_anchors[0][1] / 1000.0
    if off >= pvt_anchors[-1][0]: return pvt_anchors[-1][1] / 1000.0
    lo, hi = 0, len(pvt_anchors) - 1
    while lo + 1 < hi:
        m = (lo + hi) // 2
        if pvt_anchors[m][0] <= off: lo = m
        else: hi = m
    o0, t0 = pvt_anchors[lo]; o1, t1 = pvt_anchors[hi]
    if o1 == o0: return t0 / 1000.0
    return (t0 + (t1 - t0) * (off - o0) / (o1 - o0)) / 1000.0


# === First-appearance for each diagnostic message ===
print(f"\n=== First appearance of each message type ===")
first_seen = {}
for off, name, pl in records:
    if name not in first_seen:
        first_seen[name] = off_to_t(off) - t_start
interesting = ["NAV-PVT", "NAV-DOP", "NAV-TIMEGPS", "MON-HW", "MON-IO", "MON-SPAN",
               "MON-VER", "NAV-CLOCK", "NAV-SAT", "NAV-SIG", "NAV-COV"]
for n in interesting:
    if n in first_seen:
        print(f"  {n:<15}  first @ +{first_seen[n]:>6.2f}s")
    else:
        print(f"  {n:<15}  NOT PRESENT")

# === Find glitch ===
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
        print(f"\n=== GLITCH @ iTOW {t:.3f}s (+{t - t_start:.1f}s into capture, numSV {rmax}->{numSV}) ===")
        break

if event_t is None:
    print("\nNo sat-cliff glitch found by sat-drop rule. Checking soft-fire signatures...")
    # Look for gSpd jump
    prev_g = None
    biggest = (0, 0, 0)
    for off, name, pl in records:
        if name != "NAV-PVT" or len(pl) < 92:
            continue
        iTOW = struct.unpack("<I", pl[0:4])[0]
        t = iTOW / 1000.0
        g = struct.unpack("<i", pl[60:64])[0]
        if prev_g is not None:
            d = abs(g - prev_g)
            if d > biggest[0]:
                biggest = (d, t, prev_g, g)
        prev_g = g
    if biggest[0] > 0:
        print(f"  Biggest gSpd jump: {biggest[2]} -> {biggest[3]} mm/s at +{biggest[1] - t_start:.2f}s")
        if biggest[0] >= 200:
            event_t = biggest[1]
            print(f"  Using this as event reference")

if event_t is None:
    print("\nUsing midpoint of capture as reference.")
    event_t = (pvt_anchors[0][1] + pvt_anchors[-1][1]) / 2 / 1000

# === MON-HW across the entire capture (every agc transition) ===
print(f"\n=== MON-HW: agcCnt transitions across whole capture ===")
prev_agc = None
for off, name, pl in records:
    if name != "MON-HW" or len(pl) < 60:
        continue
    t = off_to_t(off) - t_start
    agc = struct.unpack("<H", pl[18:20])[0]
    if prev_agc is not None and agc != prev_agc:
        print(f"  +{t:>7.2f}s   agcCnt {prev_agc} -> {agc}")
    prev_agc = agc
print(f"  (final agcCnt: {prev_agc})")

# === NAV-PVT zoom -3s to +5s ===
print(f"\n=== NAV-PVT, -3s to +5s around event ===")
print(f"  {'rel':>8}  sv  fix  gSpd  hAcc  vAcc  pDOP")
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    rel = t - event_t
    if rel < -3 or rel > 5: continue
    fix = pl[20]; sv = pl[23]
    hAcc = struct.unpack("<I", pl[40:44])[0] / 1000
    vAcc = struct.unpack("<I", pl[44:48])[0] / 1000
    g = struct.unpack("<i", pl[60:64])[0]
    pDOP = struct.unpack("<H", pl[76:78])[0] * 0.01
    marker = " <- EVENT" if abs(rel) < 0.21 else ""
    print(f"  {rel:>+7.2f}s  {sv:>2}  {fix:>3}  {g:>5}  {hAcc:>5.2f}  {vAcc:>5.2f}  {pDOP:>5.2f}{marker}")

# === NAV-CLOCK around event — THE PRIZE ===
print(f"\n=== NAV-CLOCK around event (clkB ns, clkD ns/s, tAcc ns, fAcc ps/s) ===")
print(f"  {'rel':>8}  {'clkB':>12}  {'clkD':>10}  {'tAcc':>10}  {'fAcc':>10}")
for off, name, pl in records:
    if name != "NAV-CLOCK" or len(pl) < 20:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    rel = t - event_t
    if abs(rel) > 20: continue
    clkB = struct.unpack("<i", pl[4:8])[0]
    clkD = struct.unpack("<i", pl[8:12])[0]
    tAcc = struct.unpack("<I", pl[12:16])[0]
    fAcc = struct.unpack("<I", pl[16:20])[0]
    marker = " <- EVENT" if abs(rel) < 0.6 else ""
    print(f"  {rel:>+7.2f}s  {clkB:>+12d}  {clkD:>+10d}  {tAcc:>10d}  {fAcc:>10d}{marker}")

# === NAV-COV around event ===
print(f"\n=== NAV-COV around event ===")
print(f"  {'rel':>8}  posValid velValid  sigNN  sigEE  sigDD  sigVN  sigVE  sigVD")
for off, name, pl in records:
    if name != "NAV-COV" or len(pl) < 64:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    rel = t - event_t
    if abs(rel) > 20: continue
    pV = pl[5]; vV = pl[6]
    pNN = struct.unpack("<f", pl[16:20])[0]
    pEE = struct.unpack("<f", pl[28:32])[0]
    pDD = struct.unpack("<f", pl[36:40])[0]
    vNN = struct.unpack("<f", pl[40:44])[0]
    vEE = struct.unpack("<f", pl[52:56])[0]
    vDD = struct.unpack("<f", pl[60:64])[0]
    marker = " <- EVENT" if abs(rel) < 0.6 else ""
    print(f"  {rel:>+7.2f}s  {pV:>8}  {vV:>8}  {pNN:>5.2f}  {pEE:>5.2f}  {pDD:>5.2f}  {vNN:>5.2f}  {vEE:>5.2f}  {vDD:>5.2f}{marker}")

# === NAV-SAT around event ===
print(f"\n=== NAV-SAT around event ===")
print(f"  {'rel':>8}  numSv  tracking  searching  used  used-by-gnss")
for off, name, pl in records:
    if name != "NAV-SAT" or len(pl) < 8:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    rel = t - event_t
    if abs(rel) > 5: continue
    numSvs = pl[5]
    used = tr = se = 0
    by_g = Counter()
    for i in range(numSvs):
        o = 8 + i * 12
        if o + 12 > len(pl): break
        gnss = pl[o]
        flags = struct.unpack("<I", pl[o + 8:o + 12])[0]
        q = flags & 0x7
        u = (flags >> 3) & 0x1
        if q >= 4: tr += 1
        elif q in (1, 2): se += 1
        if u:
            used += 1
            by_g[gnss] += 1
    gn = {0:"GPS",1:"SBA",2:"GAL",3:"BDS",5:"QZ",6:"GLO",7:"NVC"}
    bys = ",".join(f"{gn.get(g,g)}={c}" for g, c in sorted(by_g.items()))
    marker = " <- EVENT" if abs(rel) < 0.6 else ""
    print(f"  {rel:>+7.2f}s  {numSvs:>5}  {tr:>8}  {se:>9}  {used:>4}  {bys}{marker}")
