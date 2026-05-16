#!/usr/bin/env python3
"""Analyze a clean (no-glitch) capture for baseline signatures and any
sub-threshold anomalies we should know about.

Specifically:
  - Confirm bug really didn't fire (no agcCnt jump, no sat cliff, no phantom vel)
  - Establish baselines for NAV-CLOCK (clkB linearity, clkD, tAcc, fAcc)
  - Establish baseline NAV-COV (pos & vel covariance)
  - Look for any sub-threshold transients (gSpd jumps below cliff threshold,
    micro sat dips of 1-3 sats, fAcc spikes, etc.)
  - Identify when the user clicked the button (first appearance of new msgs)
  - Time of any MON-HW agcCnt change
"""
import os
import struct
from collections import Counter

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-192509.ubx"

NAMES = {(0x01, 0x07): "NAV-PVT", (0x01, 0x04): "NAV-DOP", (0x01, 0x20): "NAV-TIMEGPS",
         (0x01, 0x22): "NAV-CLOCK", (0x01, 0x35): "NAV-SAT", (0x01, 0x43): "NAV-SIG",
         (0x01, 0x36): "NAV-COV",
         (0x05, 0x00): "ACK-ACK", (0x05, 0x01): "ACK-NAK",
         (0x06, 0x01): "CFG-MSG",
         (0x0a, 0x09): "MON-HW", (0x0a, 0x0b): "MON-IO", (0x0a, 0x31): "MON-SPAN"}


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

print("=== INVENTORY (top) ===")
ctr = Counter(r[1] for r in records)
for n, c in ctr.most_common(20):
    print(f"  {n:<15} {c}")

pvt_anchors = []
for off, name, pl in records:
    if name == "NAV-PVT" and len(pl) >= 92:
        iTOW = struct.unpack("<I", pl[0:4])[0]
        pvt_anchors.append((off, iTOW))
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


# === First appearance of each msg type — tells us when button was clicked ===
print("\n=== First appearance of each message ===")
first_seen = {}
for off, name, pl in records:
    if name not in first_seen:
        first_seen[name] = off_to_t(off) - t_start
for n in ["NAV-PVT","NAV-DOP","NAV-TIMEGPS","MON-HW","MON-IO","MON-SPAN",
          "NAV-CLOCK","NAV-SAT","NAV-SIG","NAV-COV"]:
    if n in first_seen:
        print(f"  {n:<14}  +{first_seen[n]:>6.2f}s")
    else:
        print(f"  {n:<14}  NOT PRESENT")

# === MON-HW agcCnt across capture ===
print("\n=== MON-HW agcCnt history ===")
agc_changes = []
prev = None
agc_summary = Counter()
last_agc_t = None
for off, name, pl in records:
    if name != "MON-HW" or len(pl) < 60: continue
    t = off_to_t(off) - t_start
    n = struct.unpack("<H", pl[16:18])[0]
    agc = struct.unpack("<H", pl[18:20])[0]
    agc_summary[agc] += 1
    if prev is not None and agc != prev:
        agc_changes.append((t, prev, agc))
    prev = agc
    last_agc_t = t
print(f"  Total MON-HW samples: {sum(agc_summary.values())}")
print(f"  agcCnt distribution: {dict(agc_summary)}")
print(f"  agcCnt transitions: {len(agc_changes)}")
for t, b, a in agc_changes:
    print(f"    +{t:>7.2f}s  {b} -> {a}")

# === Sat-cliff detection (same threshold as before) ===
print("\n=== Sat-cliff events (drop >= 10 from rolling 3s max) ===")
window = []
cliffs = []
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92: continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    sv = pl[23]
    window = [(tt, n) for tt, n in window if t - tt <= 3.0]
    window.append((t, sv))
    rmax = max(n for _, n in window)
    if rmax >= 10 and sv + 10 <= rmax:
        cliffs.append((t - t_start, rmax, sv))
print(f"  Hard cliffs: {len(cliffs)}")

# === Soft-fire detection: gSpd jumps >= 100 mm/s, with categorization ===
print("\n=== gSpd jumps across capture (>= 100 mm/s) ===")
prev_g = None
prev_t = None
jumps = []
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92: continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    g = struct.unpack("<i", pl[60:64])[0]
    sv = pl[23]
    if prev_g is not None and abs(g - prev_g) >= 100:
        jumps.append((t - t_start, prev_g, g, sv))
    prev_g = g
print(f"  Total: {len(jumps)}")
buckets = Counter()
for _, b, a, _ in jumps:
    delta = abs(a - b)
    if delta < 200: buckets["100-200"] += 1
    elif delta < 500: buckets["200-500"] += 1
    elif delta < 1000: buckets["500-1000"] += 1
    else: buckets[">=1000"] += 1
print(f"  By magnitude: {dict(buckets)}")
big_jumps = [j for j in jumps if abs(j[2] - j[1]) >= 300]
if big_jumps:
    print(f"  Jumps >= 300 mm/s (potential phantom velocity):")
    for t, b, a, sv in big_jumps:
        print(f"    +{t:>7.2f}s  gSpd {b} -> {a}  (sv={sv})")
else:
    print(f"  No jumps >= 300 mm/s")

# === Micro sat dips (drop of 4-9 sats; below cliff threshold) ===
print("\n=== Micro sat dips (drop 4-9 sats, sub-threshold) ===")
window = []
dips = []
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92: continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000
    sv = pl[23]
    window = [(tt, n) for tt, n in window if t - tt <= 3.0]
    window.append((t, sv))
    rmax = max(n for _, n in window)
    drop = rmax - sv
    if rmax >= 10 and 4 <= drop <= 9:
        dips.append((t - t_start, rmax, sv))
# group consecutive
print(f"  Total dip samples: {len(dips)}")
if dips:
    # group runs within 1.5s
    groups = [[dips[0]]]
    for d in dips[1:]:
        if d[0] - groups[-1][-1][0] < 1.5:
            groups[-1].append(d)
        else:
            groups.append([d])
    print(f"  Grouped into {len(groups)} dip events:")
    for g in groups[:25]:
        worst = min(x[2] for x in g)
        biggest = max(x[1] for x in g)
        print(f"    +{g[0][0]:>7.2f}s..+{g[-1][0]:>6.2f}s  ({len(g)} samples)  sv {biggest} -> {worst}")

# === NAV-CLOCK baseline ===
print("\n=== NAV-CLOCK baseline (10s windows) ===")
print(f"  {'window':>12}  {'clkB_min':>13}  {'clkB_max':>13}  {'clkD_avg':>10}  {'tAcc_avg':>10}  {'fAcc_avg':>10}  {'fAcc_max':>10}")
clock_samples = []
for off, name, pl in records:
    if name != "NAV-CLOCK" or len(pl) < 20: continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000 - t_start
    clkB = struct.unpack("<i", pl[4:8])[0]
    clkD = struct.unpack("<i", pl[8:12])[0]
    tAcc = struct.unpack("<I", pl[12:16])[0]
    fAcc = struct.unpack("<I", pl[16:20])[0]
    clock_samples.append((t, clkB, clkD, tAcc, fAcc))

if clock_samples:
    # 30-second bins
    bin_size = 30
    cur_bin = []
    cur_bin_start = clock_samples[0][0]
    for s in clock_samples:
        if s[0] - cur_bin_start >= bin_size:
            ts = [x[0] for x in cur_bin]
            clkBs = [x[1] for x in cur_bin]
            clkDs = [x[2] for x in cur_bin]
            tAccs = [x[3] for x in cur_bin]
            fAccs = [x[4] for x in cur_bin]
            print(f"  +{ts[0]:>4.0f}-{ts[-1]:>4.0f}s   {min(clkBs):>+13d}  {max(clkBs):>+13d}  {sum(clkDs)//len(clkDs):>+10d}  {sum(tAccs)//len(tAccs):>10d}  {sum(fAccs)//len(fAccs):>10d}  {max(fAccs):>10d}")
            cur_bin_start = s[0]
            cur_bin = []
        cur_bin.append(s)
    if cur_bin:
        ts = [x[0] for x in cur_bin]
        clkBs = [x[1] for x in cur_bin]
        clkDs = [x[2] for x in cur_bin]
        tAccs = [x[3] for x in cur_bin]
        fAccs = [x[4] for x in cur_bin]
        print(f"  +{ts[0]:>4.0f}-{ts[-1]:>4.0f}s   {min(clkBs):>+13d}  {max(clkBs):>+13d}  {sum(clkDs)//len(clkDs):>+10d}  {sum(tAccs)//len(tAccs):>10d}  {sum(fAccs)//len(fAccs):>10d}  {max(fAccs):>10d}")

    # Look for spikes in fAcc above 800 (anything that might indicate stress)
    fAcc_spikes = [(t, fa) for t, _, _, _, fa in clock_samples if fa >= 800]
    print(f"\n  fAcc spikes >= 800 ps/s: {len(fAcc_spikes)}")
    for t, fa in fAcc_spikes[:20]:
        print(f"    +{t:>7.2f}s  fAcc={fa} ps/s")

# === NAV-COV baseline velocity-covariance ===
print("\n=== NAV-COV velocity covariance baseline ===")
vel_cov_samples = []
for off, name, pl in records:
    if name != "NAV-COV" or len(pl) < 64: continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000 - t_start
    vNN = struct.unpack("<f", pl[40:44])[0]
    vEE = struct.unpack("<f", pl[52:56])[0]
    vDD = struct.unpack("<f", pl[60:64])[0]
    vel_cov_samples.append((t, vNN, vEE, vDD))
if vel_cov_samples:
    vNNs = [s[1] for s in vel_cov_samples]
    vEEs = [s[2] for s in vel_cov_samples]
    vDDs = [s[3] for s in vel_cov_samples]
    print(f"  vNN: min={min(vNNs):.3f}, max={max(vNNs):.3f}, avg={sum(vNNs)/len(vNNs):.3f}")
    print(f"  vEE: min={min(vEEs):.3f}, max={max(vEEs):.3f}, avg={sum(vEEs)/len(vEEs):.3f}")
    print(f"  vDD: min={min(vDDs):.3f}, max={max(vDDs):.3f}, avg={sum(vDDs)/len(vDDs):.3f}")
    # Any velocity covariance spikes above 0.3? (10x baseline)
    vel_spikes = [(t, vN, vE, vD) for t, vN, vE, vD in vel_cov_samples
                  if max(vN, vE, vD) >= 0.3]
    print(f"\n  Vel-cov spikes >= 0.3 m^2/s^2: {len(vel_spikes)}")
    for t, vN, vE, vD in vel_spikes[:20]:
        print(f"    +{t:>7.2f}s  vNN={vN:.3f}  vEE={vE:.3f}  vDD={vD:.3f}")
