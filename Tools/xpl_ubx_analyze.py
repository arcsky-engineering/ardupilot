#!/usr/bin/env python3
"""Full analysis of a u-center .ubx capture:
- Inventory all messages and their rates
- Plot NAV-PVT sat count + fix over time
- Track MON-COMMS UART saturation over time
- Look for sudden sat count drops (the dropout event)
- For each tracked signal in NAV-SIG, break down by band over time"""
from collections import Counter, defaultdict
import struct, sys, os

FILE = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\tcp___127_0_0_1_500_260512_192559.ubx"

MSG_NAMES = {
    (0x01, 0x07): "NAV-PVT", (0x01, 0x35): "NAV-SAT", (0x01, 0x43): "NAV-SIG",
    (0x01, 0x61): "NAV-EOE", (0x01, 0x03): "NAV-STATUS", (0x01, 0x04): "NAV-DOP",
    (0x01, 0x14): "NAV-HPPOSLLH", (0x01, 0x12): "NAV-VELNED", (0x01, 0x22): "NAV-CLOCK",
    (0x01, 0x34): "NAV-ORB", (0x01, 0x36): "NAV-COV",
    (0x0a, 0x04): "MON-VER", (0x0a, 0x09): "MON-HW", (0x0a, 0x36): "MON-COMMS",
    (0x0a, 0x38): "MON-RF", (0x0a, 0x37): "MON-HW3", (0x0a, 0x31): "MON-SPAN",
    (0x02, 0x15): "RXM-RAWX", (0x02, 0x13): "RXM-SFRBX", (0x02, 0x31): "RXM-INTF",
    (0x06, 0x8b): "CFG-VALGET",
}
GNSS_NAMES = {0:'GPS', 1:'SBAS', 2:'GAL', 3:'BDS', 5:'QZSS', 6:'GLO', 7:'NAVIC'}
BAND_NAMES = {
    (0, 0): 'GPS-L1CA', (0, 3): 'GPS-L2C', (0, 4): 'GPS-L2C', (0, 6): 'GPS-L5', (0, 7): 'GPS-L5',
    (1, 0): 'SBAS-L1',
    (2, 0): 'GAL-E1', (2, 1): 'GAL-E1', (2, 3): 'GAL-E5a', (2, 4): 'GAL-E5a', (2, 5): 'GAL-E5b', (2, 6): 'GAL-E5b',
    (3, 0): 'BDS-B1', (3, 1): 'BDS-B1', (3, 2): 'BDS-B2', (3, 3): 'BDS-B2', (3, 5): 'BDS-B1C', (3, 7): 'BDS-B2A',
    (5, 0): 'QZSS-L1', (5, 1): 'QZSS-L1S', (5, 4): 'QZSS-L2C', (5, 5): 'QZSS-L2C', (5, 8): 'QZSS-L5', (5, 9): 'QZSS-L5',
    (6, 0): 'GLO-L1', (6, 2): 'GLO-L2',
    (7, 0): 'NAVIC-L5',
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
                yield cls, mid, payload
                i += 8 + length
                continue
        i += 1


with open(FILE, 'rb') as f:
    data = f.read()

print(f"File: {os.path.basename(FILE)}")
print(f"Size: {len(data):,} bytes ({len(data)/1024/1024:.2f} MB)\n")

counts = Counter()
# Time-series collections
pvt_series = []     # (iTOW, numSV, fix_type)
sig_series = []     # (iTOW, total_sigs, by_band {band: count})
sat_series = []     # (iTOW, numSvs, sats_used_in_fix)
comms_series = []   # (idx, [port_name, tx%, tx_peak%, rx%, rx_peak%, tx_pending, rx_pending])
rf_series = []      # (idx, [block_band, noisePerMS, jamInd, agcCnt])

mon_ver = None
msg_idx = 0
for cls, mid, payload in iter_ubx(data):
    msg_idx += 1
    key = (cls, mid)
    name = MSG_NAMES.get(key, f"UNK_{cls:02x}{mid:02x}")
    counts[name] += 1

    if name == "NAV-PVT" and len(payload) >= 92:
        iTOW = struct.unpack('<I', payload[0:4])[0]
        fix_type = payload[20]
        numSV = payload[23]
        pvt_series.append((iTOW, numSV, fix_type))
    elif name == "NAV-SIG" and len(payload) >= 8:
        iTOW = struct.unpack('<I', payload[0:4])[0]
        numSigs = payload[5]
        band_count = Counter()
        used_count = Counter()
        for i in range(numSigs):
            off = 8 + i * 16
            if off + 16 > len(payload): break
            gnssId = payload[off]
            sigId = payload[off + 2]
            cno = payload[off + 6]
            sigFlags = struct.unpack('<H', payload[off + 10:off + 12])[0]
            health = (sigFlags >> 1) & 0x3
            used = (sigFlags >> 3) & 0x1
            band = BAND_NAMES.get((gnssId, sigId), f"{GNSS_NAMES.get(gnssId,'?')}-sig{sigId}")
            band_count[band] += 1
            if used:
                used_count[band] += 1
        sig_series.append((iTOW, numSigs, band_count, used_count))
    elif name == "NAV-SAT" and len(payload) >= 8:
        iTOW = struct.unpack('<I', payload[0:4])[0]
        numSvs = payload[5]
        used = 0
        for i in range(numSvs):
            off = 8 + i * 12
            if off + 12 > len(payload): break
            flags = struct.unpack('<I', payload[off + 8:off + 12])[0]
            if (flags >> 3) & 0x1: used += 1
        sat_series.append((iTOW, numSvs, used))
    elif name == "MON-COMMS" and len(payload) >= 8:
        nPorts = payload[1]
        ports = []
        for i in range(nPorts):
            off = 8 + i * 40
            if off + 40 > len(payload): break
            portId = struct.unpack('<H', payload[off:off + 2])[0]
            txPending = struct.unpack('<H', payload[off + 2:off + 4])[0]
            txUsage = payload[off + 8]
            txPeak = payload[off + 9]
            rxPending = struct.unpack('<H', payload[off + 10:off + 12])[0]
            rxUsage = payload[off + 16]
            rxPeak = payload[off + 17]
            port_name = {0x0000: 'I2C', 0x0100: 'UART1', 0x0200: 'UART2', 0x0300: 'USB', 0x0400: 'SPI', 0x0101: 'CAN'}.get(portId, f'p{portId:04x}')
            ports.append((port_name, txUsage, txPeak, rxUsage, rxPeak, txPending, rxPending))
        comms_series.append((msg_idx, ports))
    elif name == "MON-RF" and len(payload) >= 4:
        nBlocks = payload[1]
        blocks = []
        for i in range(nBlocks):
            off = 4 + i * 24
            if off + 24 > len(payload): break
            blockId = payload[off]
            noisePerMS = struct.unpack('<H', payload[off + 12:off + 14])[0]
            agcCnt = struct.unpack('<H', payload[off + 14:off + 16])[0]
            jamInd = payload[off + 16]
            band_name = {0: 'L1', 1: 'L2/L5'}.get(blockId, f'b{blockId}')
            blocks.append((band_name, noisePerMS, jamInd, agcCnt))
        rf_series.append((msg_idx, blocks))
    elif name == "MON-VER" and mon_ver is None:
        mon_ver = payload

# ==== Output ====
print(f"=== MESSAGE INVENTORY ({sum(counts.values()):,} total) ===")
for name, cnt in counts.most_common(20):
    print(f"  {name:<25} {cnt:>6}")

# Estimate duration from iTOW
if pvt_series and len(pvt_series) > 1:
    dur_ms = pvt_series[-1][0] - pvt_series[0][0]
    print(f"\n=== TIMELINE ===")
    print(f"  NAV-PVT epochs: {len(pvt_series)}")
    print(f"  Duration (iTOW span): {dur_ms/1000:.1f} sec ({dur_ms/60000:.1f} min)")
    print(f"  First iTOW: {pvt_series[0][0]/1000:.1f}s, last iTOW: {pvt_series[-1][0]/1000:.1f}s")

# Firmware version
if mon_ver:
    sw = mon_ver[0:30].split(b'\0')[0].decode('ascii', errors='replace')
    print(f"\n=== MON-VER ===")
    print(f"  SW: {sw}")

# Sat count over time
print(f"\n=== NAV-PVT timeline (sat count + fix) - every ~30s ===")
print(f"  {'t (s, iTOW)':>12} {'numSV':>6} {'fix':>4}")
prev = -100
for iTOW, numSV, fix_type in pvt_series:
    t = iTOW / 1000
    if t - prev >= 30 or t == pvt_series[-1][0]/1000:
        print(f"  {t:>12.0f} {numSV:>6} {fix_type:>4}")
        prev = t

# Detect sudden sat-count drops (>=10 from rolling max)
print(f"\n=== SAT COUNT DROP DETECTION (drop >= 10 from rolling 3s max) ===")
window = []
drops = []
for iTOW, numSV, fix_type in pvt_series:
    t = iTOW / 1000
    window = [(tt, n) for tt, n in window if t - tt <= 3.0]
    window.append((t, numSV))
    recent_max = max(n for _, n in window) if window else 0
    if recent_max >= 10 and numSV + 10 <= recent_max:
        drops.append((t, recent_max, numSV))
# group consecutive
groups = []
if drops:
    cur = [drops[0]]
    for d in drops[1:]:
        if d[0] - cur[-1][0] < 1.5: cur.append(d)
        else: groups.append(cur); cur = [d]
    groups.append(cur)
if groups:
    print(f"  FOUND {len(groups)} drop event(s):")
    for g in groups:
        print(f"    t={g[0][0]:.1f}s  sats {g[0][1]} -> {min(d[2] for d in g)} ({len(g)} samples)")
else:
    print(f"  No drop events detected (threshold: 10 sats below rolling 3s max)")

# Sig breakdown over time
if sig_series:
    print(f"\n=== NAV-SIG: signal breakdown (every ~60s) ===")
    print(f"  {'t (s)':>8} {'totSig':>7} {'usedSig':>8}  Bands present (count tracked / used)")
    prev = -100
    for iTOW, totSig, band_count, used_count in sig_series:
        t = iTOW / 1000
        if t - prev >= 60 or iTOW == sig_series[-1][0]:
            tot_used = sum(used_count.values())
            bands_str = ", ".join(f"{b}={n}/{used_count[b]}" for b, n in sorted(band_count.items()))
            print(f"  {t:>8.0f} {totSig:>7} {tot_used:>8}  {bands_str}")
            prev = t

# Detect L5-specific drops
print(f"\n=== L5-BAND SIGNAL COUNT OVER TIME ===")
print(f"  {'t (s)':>8} {'GPS-L5':>8} {'GAL-E5a':>8} {'QZSS-L5':>8} {'BDS-B2A':>8} {'NAVIC-L5':>8} {'total-L5-band':>14}")
prev = -100
l5_series = []
for iTOW, totSig, band_count, used_count in sig_series:
    t = iTOW / 1000
    gps_l5 = band_count.get('GPS-L5', 0)
    gal_e5a = band_count.get('GAL-E5a', 0)
    qzss_l5 = band_count.get('QZSS-L5', 0)
    bds_b2a = band_count.get('BDS-B2A', 0)
    navic_l5 = band_count.get('NAVIC-L5', 0)
    total_l5 = gps_l5 + gal_e5a + qzss_l5 + bds_b2a + navic_l5
    l5_series.append((t, gps_l5, gal_e5a, qzss_l5, bds_b2a, navic_l5, total_l5))
    if t - prev >= 30 or iTOW == sig_series[-1][0]:
        print(f"  {t:>8.0f} {gps_l5:>8} {gal_e5a:>8} {qzss_l5:>8} {bds_b2a:>8} {navic_l5:>8} {total_l5:>14}")
        prev = t

# MON-COMMS saturation analysis
if comms_series:
    print(f"\n=== MON-COMMS SATURATION CHECK (worst observed values) ===")
    by_port = defaultdict(list)
    for idx, ports in comms_series:
        for port_name, txU, txP, rxU, rxP, txPend, rxPend in ports:
            by_port[port_name].append((txU, txP, rxU, rxP, txPend, rxPend))
    for port_name, samples in by_port.items():
        max_tx = max(s[0] for s in samples)
        max_tx_peak = max(s[1] for s in samples)
        max_rx = max(s[2] for s in samples)
        max_pend_tx = max(s[4] for s in samples)
        mean_tx = sum(s[0] for s in samples) / len(samples)
        print(f"  Port {port_name}: TX usage mean={mean_tx:.0f}% max={max_tx}% peak={max_tx_peak}%, RX max={max_rx}%, TX pending max={max_pend_tx}B  (n={len(samples)})")
    saturated = any(max(s[1] for s in samples) > 95 for samples in by_port.values())
    if saturated:
        print(f"\n  WARNING: at least one port hit >95% peak TX usage -- data loss possible.")
    else:
        print(f"\n  No port exceeded 95% peak. UART has headroom.")

# MON-RF
if rf_series:
    print(f"\n=== MON-RF: noise + jamming per band ===")
    print(f"  Samples: {len(rf_series)}")
    if rf_series[0][1]:
        by_band = defaultdict(list)
        for idx, blocks in rf_series:
            for band, noise, jam, agc in blocks:
                by_band[band].append((noise, jam, agc))
        for band, samples in by_band.items():
            noises = [s[0] for s in samples]
            jams = [s[1] for s in samples]
            print(f"  {band:<8}: noisePerMS mean={sum(noises)/len(noises):.0f} min={min(noises)} max={max(noises)}, jamInd mean={sum(jams)/len(jams):.0f} max={max(jams)}")

print("\n=== READINESS CHECK ===")
checks = [
    ("NAV-PVT (timing+fix)", counts.get("NAV-PVT", 0) > 50),
    ("NAV-SAT", counts.get("NAV-SAT", 0) > 50),
    ("NAV-SIG", counts.get("NAV-SIG", 0) > 50),
    ("MON-COMMS", counts.get("MON-COMMS", 0) > 0),
    ("MON-RF", counts.get("MON-RF", 0) > 0),
    ("MON-VER", counts.get("MON-VER", 0) > 0),
    ("NAV-EOE", counts.get("NAV-EOE", 0) > 0),
    ("Outdoor fix (NAV-PVT shows fix_type>=3)", any(p[2] >= 3 for p in pvt_series)),
    ("Multiple sats tracked", any(p[1] >= 10 for p in pvt_series)),
]
for label, ok in checks:
    print(f"  [{'OK' if ok else 'MISSING'}] {label}")
