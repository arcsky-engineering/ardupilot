#!/usr/bin/env python3
"""Quick sanity check on a u-center .ubx capture: inventory the messages,
verify NAV-SIG and other diagnostics are being captured, and decode one
NAV-SIG sample to confirm per-band signal info is present."""
from collections import Counter, defaultdict
import struct, sys, os

FILE = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\tcp___127_0_0_1_500_260512_190622.ubx"

# UBX class/id → name lookup (relevant subset)
MSG_NAMES = {
    (0x01, 0x07): "NAV-PVT",
    (0x01, 0x35): "NAV-SAT",
    (0x01, 0x43): "NAV-SIG",
    (0x01, 0x61): "NAV-EOE",
    (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x21): "NAV-TIMEUTC",
    (0x01, 0x22): "NAV-CLOCK",
    (0x01, 0x04): "NAV-DOP",
    (0x01, 0x14): "NAV-HPPOSLLH",
    (0x01, 0x12): "NAV-VELNED",
    (0x01, 0x09): "NAV-ODO",
    (0x01, 0x60): "NAV-AOPSTATUS",
    (0x01, 0x42): "NAV-SLAS",
    (0x01, 0x32): "NAV-SBAS",
    (0x01, 0x34): "NAV-ORB",
    (0x01, 0x36): "NAV-COV",
    (0x01, 0x13): "NAV-HPPOSECEF",
    (0x01, 0x01): "NAV-POSECEF",
    (0x01, 0x02): "NAV-POSLLH",
    (0x01, 0x11): "NAV-VELECEF",
    (0x0a, 0x04): "MON-VER",
    (0x0a, 0x09): "MON-HW",
    (0x0a, 0x36): "MON-COMMS",
    (0x0a, 0x38): "MON-RF",
    (0x0a, 0x37): "MON-HW3",
    (0x0a, 0x28): "MON-GNSS",
    (0x0a, 0x32): "MON-BATCH",
    (0x0a, 0x31): "MON-SPAN",
    (0x0a, 0x07): "MON-RXBUF",
    (0x0a, 0x08): "MON-TXBUF",
    (0x02, 0x15): "RXM-RAWX",
    (0x02, 0x13): "RXM-SFRBX",
    (0x02, 0x31): "RXM-INTF",
    (0x04, 0x00): "INF-ERROR",
    (0x04, 0x01): "INF-WARNING",
    (0x04, 0x02): "INF-NOTICE",
    (0x04, 0x03): "INF-TEST",
    (0x04, 0x04): "INF-DEBUG",
    (0x06, 0x8b): "CFG-VALGET",
    (0x05, 0x01): "ACK-ACK",
    (0x05, 0x00): "ACK-NAK",
    (0x27, 0x03): "SEC-UNIQID",
    (0x0d, 0x01): "TIM-TP",
    (0x0d, 0x03): "TIM-TM2",
    (0x10, 0x02): "ESF-MEAS",
    (0x10, 0x10): "ESF-STATUS",
    (0x28, 0x00): "HNR-PVT",
}


def iter_ubx(data):
    """Yield (class, id, payload) for each valid UBX message in the stream."""
    i = 0
    while i < len(data) - 7:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls = data[i + 2]
            mid = data[i + 3]
            length = data[i + 4] | (data[i + 5] << 8)
            if i + 8 + length > len(data):
                i += 1
                continue
            payload = data[i + 6:i + 6 + length]
            # checksum verify
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
print(f"Size: {len(data):,} bytes ({len(data)/1024:.1f} KB)\n")

counts = Counter()
nav_pvt_samples = []
nav_sig_samples = []
mon_comms_samples = []
mon_rf_samples = []
nav_sat_samples = []
mon_ver_payload = None
all_messages = []  # for timeline

for cls, mid, payload in iter_ubx(data):
    key = (cls, mid)
    name = MSG_NAMES.get(key, f"UNKNOWN-{cls:02x}{mid:02x}")
    counts[name] += 1
    all_messages.append((name, len(payload)))
    if name == "NAV-PVT" and len(nav_pvt_samples) < 5:
        nav_pvt_samples.append(payload)
    elif name == "NAV-SIG" and len(nav_sig_samples) < 3:
        nav_sig_samples.append(payload)
    elif name == "MON-COMMS" and len(mon_comms_samples) < 3:
        mon_comms_samples.append(payload)
    elif name == "MON-RF" and len(mon_rf_samples) < 3:
        mon_rf_samples.append(payload)
    elif name == "NAV-SAT" and len(nav_sat_samples) < 3:
        nav_sat_samples.append(payload)
    elif name == "MON-VER" and mon_ver_payload is None:
        mon_ver_payload = payload

print(f"=== MESSAGE INVENTORY ({sum(counts.values()):,} total) ===")
for name, cnt in counts.most_common():
    print(f"  {name:<25} {cnt:>6}")

# MON-VER (firmware ID)
if mon_ver_payload:
    sw_ver = mon_ver_payload[0:30].split(b'\0')[0].decode('ascii', errors='replace')
    hw_ver = mon_ver_payload[30:40].split(b'\0')[0].decode('ascii', errors='replace')
    print(f"\n=== MON-VER ===")
    print(f"  SW: {sw_ver}")
    print(f"  HW: {hw_ver}")
    offset = 40
    while offset + 30 <= len(mon_ver_payload):
        ext = mon_ver_payload[offset:offset + 30].split(b'\0')[0].decode('ascii', errors='replace')
        if ext.strip(): print(f"  Ext: {ext}")
        offset += 30

# NAV-PVT sample (sat count + fix + time)
if nav_pvt_samples:
    p = nav_pvt_samples[0]
    iTOW = struct.unpack('<I', p[0:4])[0]
    year, month, day, hour, minute, sec = struct.unpack('<HBBBBB', p[4:11])
    valid = p[11]
    fix_type = p[20]
    flags = p[21]
    numSV = p[23]
    print(f"\n=== NAV-PVT (sample) ===")
    print(f"  Time: {year}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{sec:02d}, iTOW={iTOW}, fix_type={fix_type}, numSV={numSV}, valid=0x{valid:02x}")

# NAV-SIG decode (the critical one)
if nav_sig_samples:
    p = nav_sig_samples[0]
    iTOW = struct.unpack('<I', p[0:4])[0]
    version = p[4]
    numSigs = p[5]
    print(f"\n=== NAV-SIG (sample, iTOW={iTOW}, version={version}, numSigs={numSigs}) ===")
    print(f"  {'gnssId':>6} {'svId':>4} {'sigId':>5} {'cno':>4} {'health':>6} {'qualityInd':>10} {'used':>4}  Band")
    GNSS = {0:'GPS', 1:'SBAS', 2:'GAL', 3:'BDS', 5:'QZSS', 6:'GLO', 7:'NAVIC'}
    BAND = {
        (0, 0): 'L1CA', (0, 3): 'L2C-L', (0, 4): 'L2C-M', (0, 6): 'L5-I', (0, 7): 'L5-Q',
        (1, 0): 'L1CA',
        (2, 0): 'E1-C', (2, 1): 'E1-B', (2, 3): 'E5a-I', (2, 4): 'E5a-Q', (2, 5): 'E5b-I', (2, 6): 'E5b-Q',
        (3, 0): 'B1I-D1', (3, 1): 'B1I-D2', (3, 2): 'B2I-D1', (3, 3): 'B2I-D2', (3, 5): 'B1C', (3, 7): 'B2A',
        (5, 0): 'L1CA', (5, 1): 'L1S', (5, 4): 'L2CM', (5, 5): 'L2CL', (5, 8): 'L5-I', (5, 9): 'L5-Q',
        (6, 0): 'L1OF', (6, 2): 'L2OF',
        (7, 0): 'L5A',
    }
    band_counts = Counter()
    for i in range(numSigs):
        off = 8 + i * 16
        if off + 16 > len(p): break
        gnssId, svId, sigId, freqId, prRes_lo, prRes_hi, cno, qualityInd, corrSource, ionoModel = p[off:off+10]
        sigFlags = struct.unpack('<H', p[off+10:off+12])[0]
        health = (sigFlags >> 1) & 0x3
        used = (sigFlags >> 3) & 0x1
        band = BAND.get((gnssId, sigId), f'sig{sigId}')
        gnss_name = GNSS.get(gnssId, f'gnss{gnssId}')
        if i < 15:
            print(f"  {gnss_name:>6} {svId:>4} {sigId:>5} {cno:>4} {health:>6} {qualityInd:>10} {used:>4}  {band}")
        band_counts[(gnss_name, band)] += 1
    if numSigs > 15: print(f"  ...{numSigs - 15} more signals")
    print(f"\n  Signal mix breakdown:")
    for (gnss, band), n in sorted(band_counts.items()):
        print(f"    {gnss:<6} {band:<8} : {n} sigs")

# MON-COMMS check (UART saturation diagnostic)
if mon_comms_samples:
    p = mon_comms_samples[0]
    print(f"\n=== MON-COMMS (sample) ===")
    version = p[0]
    nPorts = p[1]
    print(f"  version={version}, nPorts={nPorts}")
    for i in range(nPorts):
        off = 8 + i * 40
        if off + 40 > len(p): break
        portId = struct.unpack('<H', p[off:off+2])[0]
        txPending = struct.unpack('<H', p[off+2:off+4])[0]
        txBytes = struct.unpack('<I', p[off+4:off+8])[0]
        txUsage = p[off+8]
        txPeak = p[off+9]
        rxPending = struct.unpack('<H', p[off+10:off+12])[0]
        rxBytes = struct.unpack('<I', p[off+12:off+16])[0]
        rxUsage = p[off+16]
        rxPeak = p[off+17]
        port_name = {0x0000: 'I2C', 0x0100: 'UART1', 0x0200: 'UART2', 0x0300: 'USB', 0x0400: 'SPI'}.get(portId, f'port{portId:04x}')
        print(f"  Port {port_name}: TX usage={txUsage}%/peak{txPeak}% pending={txPending}B  RX usage={rxUsage}%/peak{rxPeak}% pending={rxPending}B")

# MON-RF check (jamming detection diagnostic)
if mon_rf_samples:
    p = mon_rf_samples[0]
    print(f"\n=== MON-RF (sample) ===")
    version = p[0]
    nBlocks = p[1]
    print(f"  version={version}, nBlocks={nBlocks}")
    for i in range(nBlocks):
        off = 4 + i * 24
        if off + 24 > len(p): break
        blockId, jamming, antStatus, antPower = p[off:off+4]
        postStatus = struct.unpack('<I', p[off+4:off+8])[0]
        noisePerMS = struct.unpack('<H', p[off+12:off+14])[0]
        agcCnt = struct.unpack('<H', p[off+14:off+16])[0]
        jamInd = p[off+16]
        ofsI, magI, ofsQ, magQ = struct.unpack('<bBbB', p[off+17:off+21])
        band_name = {0: 'L1', 1: 'L2/L5'}.get(blockId, f'band{blockId}')
        print(f"  RF block {band_name}: noisePerMS={noisePerMS}, jamInd={jamInd}, agcCnt={agcCnt}")

# NAV-SAT count
if nav_sat_samples:
    p = nav_sat_samples[0]
    iTOW = struct.unpack('<I', p[0:4])[0]
    version = p[4]
    numSvs = p[5]
    print(f"\n=== NAV-SAT (sample, iTOW={iTOW}, version={version}, numSvs={numSvs}) ===")
    # decode a few
    for i in range(min(5, numSvs)):
        off = 8 + i * 12
        if off + 12 > len(p): break
        gnssId, svId, cno, elev = p[off:off+4]
        azim = struct.unpack('<h', p[off+4:off+6])[0]
        flags = struct.unpack('<I', p[off+8:off+12])[0]
        used = (flags >> 3) & 0x1
        GNSS = {0:'GPS', 1:'SBAS', 2:'GAL', 3:'BDS', 5:'QZSS', 6:'GLO', 7:'NAVIC'}
        gnss_name = GNSS.get(gnssId, f'gnss{gnssId}')
        print(f"  {gnss_name:>6} sv={svId} cno={cno} elev={elev} azim={azim} used={used}")

# Summary
print("\n=== READINESS CHECK ===")
checks = {
    "NAV-PVT (timing + fix)": counts.get("NAV-PVT", 0) > 0,
    "NAV-SAT (per-SV tracking)": counts.get("NAV-SAT", 0) > 0,
    "NAV-SIG (per-signal/per-band)": counts.get("NAV-SIG", 0) > 0,
    "MON-COMMS (UART saturation check)": counts.get("MON-COMMS", 0) > 0,
    "MON-RF (jamming/noise per band)": counts.get("MON-RF", 0) > 0,
    "MON-VER (firmware ID)": counts.get("MON-VER", 0) > 0,
    "NAV-EOE (epoch marker)": counts.get("NAV-EOE", 0) > 0,
}
for label, ok in checks.items():
    print(f"  [{'OK' if ok else 'MISSING'}] {label}")
