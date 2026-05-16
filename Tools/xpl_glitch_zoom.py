#!/usr/bin/env python3
"""Zoom into the bug-event window of a UBX capture from gps_debug tunnel.

Built specifically for the 2026-05-13 Here4 capture where sat count dropped
30 -> 11 near the end. Looks at:
  - NAV-PVT timeline second-by-second 30s before/after the drop
  - NAV-DOP H/V/PDOP and accuracy estimates around the drop
  - MON-HW noise/AGC/jamInd around the drop
  - MON-SPAN spectrum samples around the drop (compare before vs during)
  - NAV-TIMEGPS leap-second / week / time-accuracy around the drop
  - Any CFG / ACK traffic in the window (host-initiated config that may have
    triggered the bug)
"""
import os
import struct
import sys
from collections import Counter, defaultdict

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-181219.ubx"

MSG = {
    (0x01, 0x07): "NAV-PVT",
    (0x01, 0x04): "NAV-DOP",
    (0x01, 0x20): "NAV-TIMEGPS",
    (0x0a, 0x09): "MON-HW",
    (0x0a, 0x0b): "MON-IO",
    (0x0a, 0x31): "MON-SPAN",
    (0x05, 0x00): "ACK-ACK",
    (0x05, 0x01): "ACK-NAK",
    (0x06, 0x00): "CFG-PRT",
    (0x06, 0x01): "CFG-MSG",
    (0x06, 0x08): "CFG-RATE",
    (0x06, 0x24): "CFG-NAV5",
    (0x06, 0x31): "CFG-TP5",
    (0x06, 0x3e): "CFG-GNSS",
    (0x06, 0x8b): "CFG-VALGET",
}


def iter_ubx(data):
    i = 0
    pos_for_msg = []
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

# First pass: index every message with its file-byte offset AND iTOW if it has one.
# We'll use NAV-PVT samples to assign approximate iTOW timestamps to neighboring
# bytes (MON-* messages don't carry iTOW so we interpolate by stream position).
records = []  # list of (file_offset, name, payload)
for off, cls, mid, pl in iter_ubx(data):
    name = MSG.get((cls, mid), f"UNK_{cls:02x}{mid:02x}")
    records.append((off, name, pl))

# Build an offset -> iTOW table from NAV-PVT records (1 per 200ms)
pvt_anchors = []  # (file_offset, iTOW_ms)
for off, name, pl in records:
    if name == "NAV-PVT" and len(pl) >= 92:
        iTOW = struct.unpack("<I", pl[0:4])[0]
        pvt_anchors.append((off, iTOW))


def off_to_t(off):
    """Approximate t (seconds) at byte-offset `off` by linear interp between PVTs."""
    if not pvt_anchors:
        return None
    if off <= pvt_anchors[0][0]:
        return pvt_anchors[0][1] / 1000.0
    if off >= pvt_anchors[-1][0]:
        return pvt_anchors[-1][1] / 1000.0
    # binary search
    lo, hi = 0, len(pvt_anchors) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if pvt_anchors[mid][0] <= off:
            lo = mid
        else:
            hi = mid
    o0, t0 = pvt_anchors[lo]
    o1, t1 = pvt_anchors[hi]
    if o1 == o0:
        return t0 / 1000.0
    frac = (off - o0) / (o1 - o0)
    return (t0 + frac * (t1 - t0)) / 1000.0


# Locate the glitch — same rolling-3s rule as the GUI worker, but emit the
# precise PVT epoch at which it tripped.
window = []
event_iTOW = None
event_off = None
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    numSV = pl[23]
    t = iTOW / 1000.0
    window = [(tt, n, o) for tt, n, o in window if t - tt <= 3.0]
    window.append((t, numSV, off))
    recent_max = max(n for _, n, _ in window) if window else 0
    if recent_max >= 10 and numSV + 10 <= recent_max and event_iTOW is None:
        event_iTOW = iTOW
        event_off = off
        print(f"GLITCH ENTRY: iTOW={iTOW/1000:.1f}s  numSV {recent_max} -> {numSV}  (file offset 0x{off:x})\n")
        break

if event_iTOW is None:
    print("No glitch detected by rolling rule.")
    sys.exit(0)

t_event = event_iTOW / 1000.0

# === NAV-PVT zoom: 30s before through 30s after the event, per-epoch ===
print("=== NAV-PVT, per-epoch, t-30s through end ===")
print(f"  {'rel_t':>8}  {'numSV':>5}  {'fix':>4}  {'gSpeed':>7}  {'flags':>8}  {'flags2':>8}  {'pAcc(m)':>8}  {'hAcc(m)':>8}  {'vAcc(m)':>8}")
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000.0
    if t < t_event - 30 or t > t_event + 60:
        continue
    fix_type = pl[20]
    flags = pl[21]
    flags2 = pl[22]
    numSV = pl[23]
    hAcc = struct.unpack("<I", pl[40:44])[0] / 1000.0
    vAcc = struct.unpack("<I", pl[44:48])[0] / 1000.0
    gSpeed = struct.unpack("<i", pl[60:64])[0]  # mm/s
    pDOP_raw = struct.unpack("<H", pl[76:78])[0]
    rel = t - t_event
    marker = "  <-- GLITCH" if abs(rel) < 0.21 else ""
    print(f"  {rel:>+7.1f}s  {numSV:>5}  {fix_type:>4}  {gSpeed:>7}  0x{flags:02x}    0x{flags2:02x}     "
          f"{hAcc:>7.2f}  {vAcc:>7.2f}  {pDOP_raw*0.01:>7.2f}{marker}")

# === NAV-DOP zoom ===
print("\n=== NAV-DOP, per-epoch, around event ===")
print(f"  {'rel_t':>8}  {'gDOP':>5}  {'pDOP':>5}  {'tDOP':>5}  {'vDOP':>5}  {'hDOP':>5}  {'nDOP':>5}  {'eDOP':>5}")
for off, name, pl in records:
    if name != "NAV-DOP" or len(pl) < 18:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000.0
    if t < t_event - 30 or t > t_event + 60:
        continue
    g = struct.unpack("<H", pl[4:6])[0] * 0.01
    p = struct.unpack("<H", pl[6:8])[0] * 0.01
    td = struct.unpack("<H", pl[8:10])[0] * 0.01
    v = struct.unpack("<H", pl[10:12])[0] * 0.01
    h = struct.unpack("<H", pl[12:14])[0] * 0.01
    n = struct.unpack("<H", pl[14:16])[0] * 0.01
    e = struct.unpack("<H", pl[16:18])[0] * 0.01
    rel = t - t_event
    marker = "  <-- GLITCH" if abs(rel) < 0.21 else ""
    print(f"  {rel:>+7.1f}s  {g:>5.2f}  {p:>5.2f}  {td:>5.2f}  {v:>5.2f}  {h:>5.2f}  {n:>5.2f}  {e:>5.2f}{marker}")

# === MON-HW around the event ===
print("\n=== MON-HW, around event ===")
print(f"  {'rel_t':>8}  {'noisePerMS':>10}  {'agcCnt':>6}  {'aStatus':>7}  {'aPower':>6}  {'jamInd':>6}  {'flags':>5}")
for off, name, pl in records:
    if name != "MON-HW" or len(pl) < 60:
        continue
    t = off_to_t(off)
    if t is None or t < t_event - 60 or t > t_event + 60:
        continue
    noisePerMS = struct.unpack("<H", pl[16:18])[0]
    agcCnt     = struct.unpack("<H", pl[18:20])[0]
    aStatus    = pl[20]
    aPower     = pl[21]
    flags      = pl[22]
    jamInd     = pl[45]
    rel = t - t_event
    print(f"  {rel:>+7.1f}s  {noisePerMS:>10}  {agcCnt:>6}  {aStatus:>7}  {aPower:>6}  {jamInd:>6}  0x{flags:02x}")

# === MON-SPAN around the event ===
# UBX-MON-SPAN: header 4 bytes (version, numRfBlocks, reserved, reserved), then per block
# (256 bytes spectrum, u4 span, u4 res, u4 center, u1 pga, 3 reserved) = 256+16 = 272 bytes per block
print("\n=== MON-SPAN, around event (count + center freqs) ===")
print(f"  {'rel_t':>8}  {'nBlocks':>7}  {'centers(Hz)':>40}  {'span(Hz)':>30}  {'pga':>20}")
spans_before = []
spans_during = []
for off, name, pl in records:
    if name != "MON-SPAN" or len(pl) < 4:
        continue
    t = off_to_t(off)
    if t is None or t < t_event - 60 or t > t_event + 60:
        continue
    nBlocks = pl[1]
    centers = []
    spans = []
    pgas = []
    spectra = []
    for b in range(nBlocks):
        base = 4 + b * 272
        if base + 272 > len(pl):
            break
        spectrum = pl[base:base + 256]
        span     = struct.unpack("<I", pl[base + 256:base + 260])[0]
        res      = struct.unpack("<I", pl[base + 260:base + 264])[0]
        center   = struct.unpack("<I", pl[base + 264:base + 268])[0]
        pga      = pl[base + 268]
        centers.append(center)
        spans.append(span)
        pgas.append(pga)
        spectra.append((center, list(spectrum)))
    rel = t - t_event
    print(f"  {rel:>+7.1f}s  {nBlocks:>7}  "
          f"{str([c for c in centers]):>40}  "
          f"{str([s for s in spans]):>30}  "
          f"{str(pgas):>20}")
    if rel < -5:
        spans_before.append((rel, spectra, pgas))
    elif 0 <= rel < 30:
        spans_during.append((rel, spectra, pgas))

# Compare a representative "before" vs "during" spectrum block-by-block
if spans_before and spans_during:
    print("\n=== MON-SPAN: spectrum delta (before vs during) ===")
    b_rel, b_blocks, b_pga = spans_before[-1]  # most recent before
    d_rel, d_blocks, d_pga = spans_during[0]   # first during
    print(f"  Using before  @ rel {b_rel:+.1f}s  pga={b_pga}")
    print(f"  Using during  @ rel {d_rel:+.1f}s  pga={d_pga}")
    for i, ((cb, sb), (cd, sd)) in enumerate(zip(b_blocks, d_blocks)):
        if cb != cd:
            print(f"  Block {i}: center mismatch {cb} vs {cd} -- skip")
            continue
        diffs = [d - b for b, d in zip(sb, sd)]
        max_pos = max(diffs)
        max_neg = min(diffs)
        avg = sum(diffs) / len(diffs)
        # bin index of largest changes
        max_pos_bin = diffs.index(max_pos)
        max_neg_bin = diffs.index(max_neg)
        print(f"  Block {i} (center={cb/1e6:.1f} MHz): avg_delta={avg:+.2f}dB  "
              f"max_up={max_pos:+d} @ bin {max_pos_bin}  max_down={max_neg:+d} @ bin {max_neg_bin}")

# === ACK / NAK traffic across the event ===
print("\n=== ACK-ACK / ACK-NAK around the event (full 60s window) ===")
ack_count_before = 0
ack_count_during = 0
nak_count_before = 0
nak_count_during = 0
for off, name, pl in records:
    if name not in ("ACK-ACK", "ACK-NAK"):
        continue
    t = off_to_t(off)
    if t is None:
        continue
    rel = t - t_event
    if rel < -60 or rel > 60:
        continue
    cls_id = (pl[0], pl[1]) if len(pl) >= 2 else (0, 0)
    target = MSG.get(cls_id, f"UNK_{cls_id[0]:02x}{cls_id[1]:02x}")
    print(f"  {rel:>+7.1f}s  {name}  for {target}")
    if rel < 0:
        if name == "ACK-ACK": ack_count_before += 1
        else: nak_count_before += 1
    else:
        if name == "ACK-ACK": ack_count_during += 1
        else: nak_count_during += 1
print(f"  Summary: before  ACK={ack_count_before} NAK={nak_count_before}   during/after  ACK={ack_count_during} NAK={nak_count_during}")

# === CFG traffic around the event ===
print("\n=== CFG traffic in event window (host config commands) ===")
for off, name, pl in records:
    if not name.startswith("CFG-") and not name.startswith("UNK_06"):
        continue
    t = off_to_t(off)
    if t is None:
        continue
    rel = t - t_event
    if rel < -60 or rel > 60:
        continue
    print(f"  {rel:>+7.1f}s  {name}  len={len(pl)}  first8={pl[:8].hex()}")

# === MON-IO ports around the event ===
print("\n=== MON-IO around event (UART error counters) ===")
print(f"  {'rel_t':>8}  rxBytes  txBytes  parityErr  framingErr  overrunErr  brkCnd  rxBusy  txBusy")
for off, name, pl in records:
    if name != "MON-IO" or len(pl) < 20:
        continue
    t = off_to_t(off)
    if t is None or t < t_event - 60 or t > t_event + 60:
        continue
    # MON-IO is per-port, port 0 = I2C, 1 = UART1, 2 = UART2, 3 = USB, 4 = SPI
    # Each port: rxBytes(u4) txBytes(u4) parityErr(u2) framingErr(u2) overrunErr(u2) brkCnd(u2) rxBusy(u1) txBusy(u1) res(2)
    nPorts = len(pl) // 20
    for p in range(nPorts):
        base = p * 20
        if base + 20 > len(pl): break
        rx = struct.unpack("<I", pl[base:base + 4])[0]
        tx = struct.unpack("<I", pl[base + 4:base + 8])[0]
        parErr = struct.unpack("<H", pl[base + 8:base + 10])[0]
        framErr = struct.unpack("<H", pl[base + 10:base + 12])[0]
        ovErr = struct.unpack("<H", pl[base + 12:base + 14])[0]
        brk = struct.unpack("<H", pl[base + 14:base + 16])[0]
        rxBusy = pl[base + 16]
        txBusy = pl[base + 17]
        rel = t - t_event
        if parErr or framErr or ovErr or brk:
            print(f"  {rel:>+7.1f}s p{p}: rx={rx} tx={tx} parityErr={parErr} framingErr={framErr} overrunErr={ovErr} brkCnd={brk} rxBusy={rxBusy} txBusy={txBusy}")
