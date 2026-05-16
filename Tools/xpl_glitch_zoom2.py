#!/usr/bin/env python3
"""Glitch-zoom v2 — for captures with UBX debug messages enabled.

Adds UBX-INF-* decoding (debug/notice/warning/error strings printed by the
u-blox itself), and looks for ANY messages in the +/-10s event window that
don't appear in the steady-state stream.
"""
import os
import struct
from collections import Counter, defaultdict

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-182415.ubx"

MSG = {
    (0x01, 0x07): "NAV-PVT",
    (0x01, 0x04): "NAV-DOP",
    (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x35): "NAV-SAT",
    (0x01, 0x43): "NAV-SIG",
    (0x01, 0x61): "NAV-EOE",
    (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x14): "NAV-HPPOSLLH",
    (0x01, 0x22): "NAV-CLOCK",
    (0x01, 0x34): "NAV-ORB",
    (0x01, 0x36): "NAV-COV",
    (0x02, 0x15): "RXM-RAWX",
    (0x02, 0x13): "RXM-SFRBX",
    (0x02, 0x31): "RXM-INTF",
    (0x04, 0x00): "INF-ERROR",
    (0x04, 0x01): "INF-WARNING",
    (0x04, 0x02): "INF-NOTICE",
    (0x04, 0x03): "INF-TEST",
    (0x04, 0x04): "INF-DEBUG",
    (0x05, 0x00): "ACK-ACK",
    (0x05, 0x01): "ACK-NAK",
    (0x06, 0x00): "CFG-PRT",
    (0x06, 0x01): "CFG-MSG",
    (0x06, 0x08): "CFG-RATE",
    (0x06, 0x24): "CFG-NAV5",
    (0x06, 0x31): "CFG-TP5",
    (0x06, 0x3e): "CFG-GNSS",
    (0x06, 0x8b): "CFG-VALGET",
    (0x06, 0x8a): "CFG-VALSET",
    (0x06, 0x09): "CFG-CFG",
    (0x06, 0x04): "CFG-RST",
    (0x0a, 0x04): "MON-VER",
    (0x0a, 0x09): "MON-HW",
    (0x0a, 0x0b): "MON-IO",
    (0x0a, 0x36): "MON-COMMS",
    (0x0a, 0x38): "MON-RF",
    (0x0a, 0x37): "MON-HW3",
    (0x0a, 0x31): "MON-SPAN",
    (0x0d, 0x03): "TIM-TM2",
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

print(f"=== FILE: {os.path.basename(FILE)} ({len(data):,} bytes) ===\n")

records = []
for off, cls, mid, pl in iter_ubx(data):
    name = MSG.get((cls, mid), f"UNK_{cls:02x}{mid:02x}")
    records.append((off, name, pl))

print(f"=== INVENTORY ({len(records)} total) ===")
ctr = Counter(r[1] for r in records)
for n, c in ctr.most_common(30):
    print(f"  {n:<20} {c}")

# Index NAV-PVT for time conversion
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


# Find glitch
window = []
event_iTOW = None
event_off = None
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    numSV = pl[23]
    t = iTOW / 1000.0
    window = [(tt, n) for tt, n in window if t - tt <= 3.0]
    window.append((t, numSV))
    recent_max = max(n for _, n in window) if window else 0
    if recent_max >= 10 and numSV + 10 <= recent_max and event_iTOW is None:
        event_iTOW = iTOW
        event_off = off
        print(f"\nGLITCH ENTRY: iTOW={iTOW/1000:.3f}s  numSV {recent_max} -> {numSV}  (file offset 0x{off:x})")
        # Position in PVT epochs since start
        idx = next(i for i, (_, t2) in enumerate(pvt_anchors) if t2 == iTOW)
        first_t = pvt_anchors[0][1] / 1000
        rel_to_start = iTOW / 1000 - first_t
        print(f"  PVT epoch #{idx} of {len(pvt_anchors)}")
        print(f"  Capture started at iTOW={first_t:.1f}s, event at +{rel_to_start:.1f}s into capture")
        break

if event_iTOW is None:
    print("\nNo glitch detected.")
    import sys; sys.exit(0)

t_event = event_iTOW / 1000.0

# === Dump ALL UBX-INF-* messages with their timing relative to the event ===
print("\n=== UBX-INF-* messages (all of them, with timing) ===")
inf_count_pre = 0
inf_count_post = 0
inf_count_total = 0
for off, name, pl in records:
    if not name.startswith("INF-"):
        continue
    inf_count_total += 1
    t = off_to_t(off)
    rel = (t - t_event) if t is not None else None
    text = bytes(pl).decode("ascii", errors="replace").rstrip("\x00")
    if rel is None:
        when = "?"
    else:
        when = f"{rel:+8.2f}s"
        if rel < 0: inf_count_pre += 1
        else: inf_count_post += 1
    # Filter to interesting window
    if rel is not None and -30 <= rel <= 30:
        print(f"  [{when}] {name}: {text}")
print(f"\n  Total INF messages: {inf_count_total} (pre-event: {inf_count_pre}, post-event: {inf_count_post})")

# === All INF-WARNING and INF-ERROR regardless of window ===
print("\n=== ALL INF-WARNING/ERROR (entire capture) ===")
for off, name, pl in records:
    if name not in ("INF-WARNING", "INF-ERROR"):
        continue
    t = off_to_t(off)
    rel = (t - t_event) if t is not None else None
    text = bytes(pl).decode("ascii", errors="replace").rstrip("\x00")
    rel_str = f"{rel:+8.2f}s" if rel is not None else "  ?"
    print(f"  [{rel_str}] {name}: {text}")

# === NAV-PVT zoom — same as before but with NAV-SAT/NAV-SIG count too ===
print("\n=== NAV-PVT, per-epoch, -5s to +5s ===")
print(f"  {'rel_t':>8}  {'numSV':>5}  {'fix':>4}  {'gSpd':>7}  {'hAcc':>7}  {'vAcc':>7}")
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000.0
    rel = t - t_event
    if rel < -5 or rel > 5:
        continue
    fix_type = pl[20]
    numSV = pl[23]
    hAcc = struct.unpack("<I", pl[40:44])[0] / 1000.0
    vAcc = struct.unpack("<I", pl[44:48])[0] / 1000.0
    gSpeed = struct.unpack("<i", pl[60:64])[0]
    marker = "  <-- GLITCH" if abs(rel) < 0.21 else ""
    print(f"  {rel:>+7.2f}s  {numSV:>5}  {fix_type:>4}  {gSpeed:>7}  {hAcc:>7.2f}  {vAcc:>7.2f}{marker}")

# === MON-HW around event ===
print("\n=== MON-HW around event ===")
print(f"  {'rel_t':>8}  noise  agcCnt  aStat  aPow  jam  flags")
for off, name, pl in records:
    if name != "MON-HW" or len(pl) < 60:
        continue
    t = off_to_t(off)
    if t is None or abs(t - t_event) > 60:
        continue
    n = struct.unpack("<H", pl[16:18])[0]
    agc = struct.unpack("<H", pl[18:20])[0]
    aS = pl[20]; aP = pl[21]; fl = pl[22]; jam = pl[45]
    rel = t - t_event
    print(f"  {rel:>+7.2f}s  {n:>5}  {agc:>6}  {aS:>5}  {aP:>4}  {jam:>3}  0x{fl:02x}")

# === NAV-SAT / NAV-SIG zoom if present ===
if any(r[1] == "NAV-SAT" for r in records):
    print("\n=== NAV-SAT per-epoch around event (per-sat lock status) ===")
    for off, name, pl in records:
        if name != "NAV-SAT" or len(pl) < 8:
            continue
        iTOW = struct.unpack("<I", pl[0:4])[0]
        t = iTOW / 1000.0
        rel = t - t_event
        if rel < -3 or rel > 3:
            continue
        numSvs = pl[5]
        # per-sat
        used = 0
        tracking = 0
        searching = 0
        by_gnss = Counter()
        bad_health = 0
        for i in range(numSvs):
            o = 8 + i * 12
            if o + 12 > len(pl): break
            gnss = pl[o]; sv = pl[o + 1]; cno = pl[o + 2]
            qual_flags = struct.unpack("<I", pl[o + 8:o + 12])[0]
            quality = qual_flags & 0x7  # 0=no signal, 1=searching, 2=signal acquired, 3=signal detected unusable, 4=code locked, 5/6/7=carrier locked
            sv_used = (qual_flags >> 3) & 0x1
            health = (qual_flags >> 4) & 0x3  # 0=unknown, 1=healthy, 2=unhealthy
            if quality >= 4:
                tracking += 1
            elif quality in (1, 2):
                searching += 1
            if sv_used:
                used += 1
                by_gnss[gnss] += 1
            if health == 2:
                bad_health += 1
        gnss_names = {0:"GPS", 1:"SBAS", 2:"GAL", 3:"BDS", 5:"QZSS", 6:"GLO", 7:"NAVIC"}
        used_str = ",".join(f"{gnss_names.get(g,g)}={c}" for g, c in sorted(by_gnss.items()))
        marker = "  <-- GLITCH" if abs(rel) < 0.21 else ""
        print(f"  {rel:>+7.2f}s  numSvs={numSvs}  tracking={tracking}  searching={searching}  used={used}  badHealth={bad_health}  ({used_str}){marker}")
