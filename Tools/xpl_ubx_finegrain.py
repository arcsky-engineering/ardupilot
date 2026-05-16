#!/usr/bin/env python3
"""Look at NAV-PVT / NAV-SIG data at full sample resolution.
Detect ANY sat-count or signal-count drop, even brief ones."""
from collections import Counter, defaultdict
import struct, os

FILE = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\tcp___127_0_0_1_500_260512_192559.ubx"

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
                yield cls, mid, payload
                i += 8 + length
                continue
        i += 1


with open(FILE, 'rb') as f:
    data = f.read()

BAND_NAMES = {
    (0, 0): 'GPS-L1', (0, 3): 'GPS-L2', (0, 4): 'GPS-L2', (0, 6): 'GPS-L5', (0, 7): 'GPS-L5',
    (1, 0): 'SBAS-L1', (2, 0): 'GAL-E1', (2, 1): 'GAL-E1',
    (2, 3): 'GAL-E5a', (2, 4): 'GAL-E5a', (2, 5): 'GAL-E5b', (2, 6): 'GAL-E5b',
    (3, 0): 'BDS-B1', (3, 1): 'BDS-B1', (3, 2): 'BDS-B2', (3, 3): 'BDS-B2',
    (3, 5): 'BDS-B1C', (3, 7): 'BDS-B2A',
    (5, 0): 'QZSS-L1', (5, 1): 'QZSS-L1S', (5, 4): 'QZSS-L2', (5, 5): 'QZSS-L2',
    (5, 8): 'QZSS-L5', (5, 9): 'QZSS-L5',
    (6, 0): 'GLO-L1', (6, 2): 'GLO-L2', (7, 0): 'NAVIC-L5',
}

pvt = []
sig = []

for cls, mid, payload in iter_ubx(data):
    if cls == 0x01 and mid == 0x07 and len(payload) >= 92:
        iTOW = struct.unpack('<I', payload[0:4])[0]
        pvt.append((iTOW, payload[23], payload[20]))  # numSV, fix_type
    elif cls == 0x01 and mid == 0x43 and len(payload) >= 8:
        iTOW = struct.unpack('<I', payload[0:4])[0]
        numSigs = payload[5]
        band_count = Counter()
        used_count = Counter()
        for i in range(numSigs):
            off = 8 + i * 16
            if off + 16 > len(payload): break
            gnssId = payload[off]
            sigId = payload[off + 2]
            sigFlags = struct.unpack('<H', payload[off + 10:off + 12])[0]
            used = (sigFlags >> 3) & 0x1
            band = BAND_NAMES.get((gnssId, sigId), f"?{gnssId}{sigId}")
            band_count[band] += 1
            if used: used_count[band] += 1
        sig.append((iTOW, numSigs, sum(used_count.values()), band_count, used_count))

print(f"NAV-PVT samples: {len(pvt)}")
print(f"NAV-SIG samples: {len(sig)}")

# Look at every NAV-PVT for ANY sample-to-sample drop >= 5
print(f"\n=== NAV-PVT consecutive drops (any >= 5) ===")
prev = None
found = 0
for iTOW, numSV, fix in pvt:
    if prev is not None and prev[1] - numSV >= 5:
        print(f"  DROP t={iTOW/1000:.1f}s: numSV {prev[1]} -> {numSV}  (delta -{prev[1]-numSV})")
        found += 1
    prev = (iTOW, numSV, fix)
if not found:
    print(f"  No drops >= 5 sats in any consecutive NAV-PVT pair ({len(pvt)} samples)")

# Look at every NAV-SIG for ANY drop in total signals >= 5
print(f"\n=== NAV-SIG total signal drops (any >= 5) ===")
prev = None
found = 0
for iTOW, totSig, totUsed, band_count, used_count in sig:
    if prev is not None and prev[1] - totSig >= 5:
        print(f"  DROP t={iTOW/1000:.1f}s: totSig {prev[1]} -> {totSig}  (delta -{prev[1]-totSig})")
        found += 1
    prev = (iTOW, totSig, totUsed, band_count, used_count)
if not found:
    print(f"  No drops >= 5 total signals in any consecutive NAV-SIG pair ({len(sig)} samples)")

# Look at per-band sat count drops
print(f"\n=== Per-band drops between consecutive NAV-SIG samples ===")
print(f"  Showing any drop of >= 2 sats in any band (excluding small fluctuations)")
prev_bands = None
for iTOW, totSig, totUsed, band_count, used_count in sig:
    if prev_bands is not None:
        for band, prev_n in prev_bands.items():
            curr_n = band_count.get(band, 0)
            if prev_n - curr_n >= 2:
                print(f"  t={iTOW/1000:.1f}s  {band}: {prev_n} -> {curr_n}  (delta -{prev_n-curr_n})")
    prev_bands = band_count

# Print full NAV-PVT timeline (numSV every sample)
print(f"\n=== Full NAV-PVT numSV histogram ===")
sv_hist = Counter(p[1] for p in pvt)
for sv, n in sorted(sv_hist.items()):
    print(f"  numSV={sv:3d}: {n:5d} samples ({n/len(pvt)*100:.1f}%)")

# Full NAV-SIG total signal histogram
print(f"\n=== Full NAV-SIG totSig histogram ===")
sig_hist = Counter(s[1] for s in sig)
for n, cnt in sorted(sig_hist.items()):
    print(f"  totSig={n:3d}: {cnt:5d} samples ({cnt/len(sig)*100:.1f}%)")

# Look at NAV-SIG totals at every sample around minutes 4 and 8 if in window
print(f"\n=== Per-second sampling of full NAV-PVT timeline ===")
last_sec = -1
for iTOW, numSV, fix in pvt:
    sec = iTOW // 1000
    if sec != last_sec and sec % 10 == 0:  # every 10s sample
        # find matching NAV-SIG
        matching = [s for s in sig if abs(s[0] - iTOW) < 500]
        if matching:
            totSig, totUsed = matching[0][1], matching[0][2]
            print(f"  t={sec}s (iTOW): numSV={numSV} fix={fix} totSig={totSig} totUsed={totUsed}")
        last_sec = sec

# Also check used signals trend
print(f"\n=== NAV-SIG: totSig vs totUsed every 30s ===")
last_sec = -1
for iTOW, totSig, totUsed, bc, uc in sig:
    sec = iTOW // 1000
    if sec - last_sec >= 30:
        print(f"  t={sec}s: tracked={totSig} used_in_fix={totUsed} ({totUsed/totSig*100 if totSig else 0:.0f}%)")
        last_sec = sec
