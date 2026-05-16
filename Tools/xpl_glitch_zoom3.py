#!/usr/bin/env python3
"""Glitch-zoom v3 — handles captures with NAV-SAT, NAV-SIG, NAV-CLOCK, NAV-COV.

Adds:
  - NAV-CLOCK around event (clock bias / drift / accuracy)
  - NAV-SAT per-sat lock state around the event (which channels lost lock?)
  - NAV-SIG per-signal CN0 around the event (per-band tracking quality)
  - NAV-COV receiver-reported covariance around the event
  - Full CFG/ACK trail through the entire capture (to spot u-center commands)
"""
import os
import struct
from collections import Counter

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-184552.ubx"

MSG = {
    (0x01, 0x07): "NAV-PVT",
    (0x01, 0x04): "NAV-DOP",
    (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x22): "NAV-CLOCK",
    (0x01, 0x35): "NAV-SAT",
    (0x01, 0x43): "NAV-SIG",
    (0x01, 0x36): "NAV-COV",
    (0x01, 0x61): "NAV-EOE",
    (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x14): "NAV-HPPOSLLH",
    (0x01, 0x34): "NAV-ORB",
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
    (0x06, 0x04): "CFG-RST",
    (0x06, 0x08): "CFG-RATE",
    (0x06, 0x09): "CFG-CFG",
    (0x06, 0x24): "CFG-NAV5",
    (0x06, 0x31): "CFG-TP5",
    (0x06, 0x3e): "CFG-GNSS",
    (0x06, 0x8a): "CFG-VALSET",
    (0x06, 0x8b): "CFG-VALGET",
    (0x06, 0x8c): "CFG-VALDEL",
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
for n, c in ctr.most_common(40):
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
    if o1 == o0:
        return t0 / 1000.0
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
        idx = next(i for i, (_, t2) in enumerate(pvt_anchors) if t2 == iTOW)
        first_t = pvt_anchors[0][1] / 1000
        rel_to_start = iTOW / 1000 - first_t
        print(f"\nGLITCH ENTRY: iTOW={iTOW/1000:.3f}s  numSV {recent_max} -> {numSV}  (offset 0x{off:x})")
        print(f"  Epoch #{idx} of {len(pvt_anchors)}, +{rel_to_start:.1f}s into capture")
        break

if event_iTOW is None:
    print("\nNo sat-cliff glitch detected. Looking for other anomalies...")
    # Find biggest gSpeed jump and biggest hAcc jump
    biggest_dgspd = 0
    biggest_dhacc = 0
    biggest_dgspd_t = None
    biggest_dhacc_t = None
    prev_gspd = None
    prev_hacc = None
    prev_t = None
    for off, name, pl in records:
        if name != "NAV-PVT" or len(pl) < 92:
            continue
        iTOW = struct.unpack("<I", pl[0:4])[0]
        t = iTOW / 1000.0
        gSpd = struct.unpack("<i", pl[60:64])[0]
        hAcc = struct.unpack("<I", pl[40:44])[0] / 1000.0
        if prev_gspd is not None:
            d_gspd = abs(gSpd - prev_gspd)
            d_hacc = abs(hAcc - prev_hacc)
            if d_gspd > biggest_dgspd:
                biggest_dgspd = d_gspd
                biggest_dgspd_t = t
            if d_hacc > biggest_dhacc:
                biggest_dhacc = d_hacc
                biggest_dhacc_t = t
        prev_gspd = gSpd; prev_hacc = hAcc; prev_t = t
    print(f"  Biggest gSpeed delta: {biggest_dgspd} mm/s at t={biggest_dgspd_t:.1f}s")
    print(f"  Biggest hAcc delta: {biggest_dhacc:.2f} m at t={biggest_dhacc_t:.1f}s")
    # Set the event time to whatever has the highest signature
    if biggest_dgspd > 200:
        event_iTOW = int(biggest_dgspd_t * 1000)
        print(f"  Using gSpeed-jump location as event time")
    elif biggest_dhacc > 0.3:
        event_iTOW = int(biggest_dhacc_t * 1000)
        print(f"  Using hAcc-jump location as event time")
    else:
        print(f"  No obvious event signature found; using midpoint of capture")
        event_iTOW = int((pvt_anchors[0][1] + pvt_anchors[-1][1]) / 2)

t_event = event_iTOW / 1000.0

# === NAV-PVT zoom -10s to +10s ===
print(f"\n=== NAV-PVT, -10s to +10s, event @ iTOW={t_event:.3f}s ===")
print(f"  {'rel':>8}  {'sv':>3}  {'fix':>3}  {'gSpd':>6}  {'hAcc':>6}  {'vAcc':>6}  {'pDOP':>5}")
for off, name, pl in records:
    if name != "NAV-PVT" or len(pl) < 92:
        continue
    iTOW = struct.unpack("<I", pl[0:4])[0]
    t = iTOW / 1000.0
    rel = t - t_event
    if rel < -10 or rel > 10:
        continue
    fix_type = pl[20]
    numSV = pl[23]
    hAcc = struct.unpack("<I", pl[40:44])[0] / 1000.0
    vAcc = struct.unpack("<I", pl[44:48])[0] / 1000.0
    gSpd = struct.unpack("<i", pl[60:64])[0]
    pDOP = struct.unpack("<H", pl[76:78])[0] * 0.01
    marker = "  <-- EVENT" if abs(rel) < 0.21 else ""
    print(f"  {rel:>+7.2f}s  {numSV:>3}  {fix_type:>3}  {gSpd:>6}  {hAcc:>6.2f}  {vAcc:>6.2f}  {pDOP:>5.2f}{marker}")

# === NAV-CLOCK around event (clkB, clkD, tAcc, fAcc) ===
n_clock = sum(1 for _, n, _ in records if n == "NAV-CLOCK")
if n_clock > 0:
    print(f"\n=== NAV-CLOCK around event (clkB=clock bias ns, clkD=clock drift ns/s) ===")
    print(f"  {'rel':>8}  {'clkB_ns':>12}  {'clkD_nsps':>10}  {'tAcc_ns':>10}  {'fAcc_psps':>10}")
    for off, name, pl in records:
        if name != "NAV-CLOCK" or len(pl) < 20:
            continue
        iTOW = struct.unpack("<I", pl[0:4])[0]
        t = iTOW / 1000.0
        rel = t - t_event
        if rel < -15 or rel > 15:
            continue
        clkB = struct.unpack("<i", pl[4:8])[0]
        clkD = struct.unpack("<i", pl[8:12])[0]
        tAcc = struct.unpack("<I", pl[12:16])[0]
        fAcc = struct.unpack("<I", pl[16:20])[0]
        marker = "  <-- EVENT" if abs(rel) < 0.51 else ""
        print(f"  {rel:>+7.2f}s  {clkB:>+12d}  {clkD:>+10d}  {tAcc:>10d}  {fAcc:>10d}{marker}")
else:
    print("\n(NAV-CLOCK not present)")

# === NAV-COV around event ===
n_cov = sum(1 for _, n, _ in records if n == "NAV-COV")
if n_cov > 0:
    print(f"\n=== NAV-COV around event (position covariance diagonals m^2) ===")
    print(f"  {'rel':>8}  {'posValid':>8}  {'velValid':>8}  {'sigNN':>8}  {'sigEE':>8}  {'sigDD':>8}")
    for off, name, pl in records:
        if name != "NAV-COV" or len(pl) < 64:
            continue
        iTOW = struct.unpack("<I", pl[0:4])[0]
        t = iTOW / 1000.0
        rel = t - t_event
        if rel < -15 or rel > 15:
            continue
        posCovValid = pl[5]
        velCovValid = pl[6]
        # 16 bytes of reserved, then 6 floats pos cov, then 6 floats vel cov
        posCovNN = struct.unpack("<f", pl[16:20])[0]
        posCovNE = struct.unpack("<f", pl[20:24])[0]
        posCovND = struct.unpack("<f", pl[24:28])[0]
        posCovEE = struct.unpack("<f", pl[28:32])[0]
        posCovED = struct.unpack("<f", pl[32:36])[0]
        posCovDD = struct.unpack("<f", pl[36:40])[0]
        marker = "  <-- EVENT" if abs(rel) < 0.51 else ""
        print(f"  {rel:>+7.2f}s  {posCovValid:>8}  {velCovValid:>8}  {posCovNN:>8.3f}  {posCovEE:>8.3f}  {posCovDD:>8.3f}{marker}")
else:
    print("\n(NAV-COV not present)")

# === NAV-SAT around event ===
n_sat = sum(1 for _, n, _ in records if n == "NAV-SAT")
if n_sat > 0:
    print(f"\n=== NAV-SAT around event ===")
    print(f"  {'rel':>8}  {'numSv':>5}  {'tracking':>8}  {'searching':>9}  {'used':>5}  {'used_by_gnss'}")
    for off, name, pl in records:
        if name != "NAV-SAT" or len(pl) < 8:
            continue
        iTOW = struct.unpack("<I", pl[0:4])[0]
        t = iTOW / 1000.0
        rel = t - t_event
        if rel < -15 or rel > 15:
            continue
        numSvs = pl[5]
        used = tracking = searching = 0
        by_gnss = Counter()
        for i in range(numSvs):
            o = 8 + i * 12
            if o + 12 > len(pl): break
            gnss = pl[o]
            flags = struct.unpack("<I", pl[o + 8:o + 12])[0]
            qual = flags & 0x7
            sv_used = (flags >> 3) & 0x1
            if qual >= 4: tracking += 1
            elif qual in (1, 2): searching += 1
            if sv_used:
                used += 1
                by_gnss[gnss] += 1
        gnss_names = {0:"GPS",1:"SBA",2:"GAL",3:"BDS",5:"QZ",6:"GLO",7:"NVC"}
        by_gnss_str = ",".join(f"{gnss_names.get(g,g)}={c}" for g, c in sorted(by_gnss.items()))
        marker = "  <-- EVENT" if abs(rel) < 0.51 else ""
        print(f"  {rel:>+7.2f}s  {numSvs:>5}  {tracking:>8}  {searching:>9}  {used:>5}  {by_gnss_str}{marker}")
else:
    print("\n(NAV-SAT not present)")

# === NAV-SAT detailed delta — find which sats dropped during event ===
if n_sat > 0:
    sat_before = None
    sat_during = None
    for off, name, pl in records:
        if name != "NAV-SAT" or len(pl) < 8:
            continue
        iTOW = struct.unpack("<I", pl[0:4])[0]
        t = iTOW / 1000.0
        rel = t - t_event
        if -3 <= rel <= -0.5:
            sat_before = pl
            sat_before_rel = rel
        elif 0 <= rel <= 1.5 and sat_during is None:
            sat_during = pl
            sat_during_rel = rel
    if sat_before is not None and sat_during is not None:
        print(f"\n=== NAV-SAT per-sat delta (before@{sat_before_rel:+.2f}s vs during@{sat_during_rel:+.2f}s) ===")
        def parse_sat(pl):
            n = pl[5]
            sats = {}
            for i in range(n):
                o = 8 + i * 12
                if o + 12 > len(pl): break
                gnss = pl[o]; sv = pl[o + 1]; cno = pl[o + 2]
                flags = struct.unpack("<I", pl[o + 8:o + 12])[0]
                qual = flags & 0x7
                sv_used = (flags >> 3) & 0x1
                health = (flags >> 4) & 0x3
                sats[(gnss, sv)] = (cno, qual, sv_used, health)
            return sats
        b = parse_sat(sat_before)
        d = parse_sat(sat_during)
        all_keys = sorted(set(b.keys()) | set(d.keys()))
        gnss_names = {0:"GPS",1:"SBA",2:"GAL",3:"BDS",5:"QZSS",6:"GLO",7:"NVC"}
        qual_names = {0:"none",1:"srch",2:"acq",3:"unuse",4:"code",5:"car1",6:"car2",7:"car3"}
        print(f"  {'sat':>10}  {'before':>20}  {'during':>20}  flag")
        for k in all_keys:
            bv = b.get(k)
            dv = d.get(k)
            bstr = f"cno{bv[0]:>2} q{qual_names.get(bv[1],bv[1])} u{bv[2]} h{bv[3]}" if bv else "(absent)"
            dstr = f"cno{dv[0]:>2} q{qual_names.get(dv[1],dv[1])} u{dv[2]} h{dv[3]}" if dv else "(absent)"
            flag = ""
            if bv and dv:
                if bv[2] != dv[2]: flag += " USED-flip"
                if bv[1] != dv[1]: flag += " QUAL-change"
                if abs(bv[0] - dv[0]) > 5: flag += " CN0-drop"
            elif bv and not dv:
                flag = " DISAPPEARED"
            elif dv and not bv:
                flag = " NEW"
            print(f"  {gnss_names.get(k[0],k[0])}{k[1]:>3}     {bstr:>20}   {dstr:>20}  {flag}")

# === MON-HW around event ===
print("\n=== MON-HW around event ===")
print(f"  {'rel':>8}  noise  agcCnt  aStat  aPow  jam")
for off, name, pl in records:
    if name != "MON-HW" or len(pl) < 60:
        continue
    t = off_to_t(off)
    if t is None or abs(t - t_event) > 90:
        continue
    n = struct.unpack("<H", pl[16:18])[0]
    agc = struct.unpack("<H", pl[18:20])[0]
    aS = pl[20]; aP = pl[21]; jam = pl[45]
    rel = t - t_event
    print(f"  {rel:>+7.2f}s  {n:>5}  {agc:>6}  {aS:>5}  {aP:>4}  {jam:>3}")

# === ALL CFG / ACK traffic across the whole capture (which u-center commands fired?) ===
print("\n=== ALL CFG-VALSET, CFG-VALDEL, CFG-CFG (host config commands - u-center? gnssperiph?) ===")
for off, name, pl in records:
    if name not in ("CFG-VALSET", "CFG-VALDEL", "CFG-CFG", "CFG-RST"):
        continue
    t = off_to_t(off)
    rel = (t - t_event) if t is not None else None
    rel_str = f"{rel:+8.2f}s" if rel is not None else "       ?"
    # CFG-VALSET first 4 bytes: version, layers, transaction, reserved.
    # Then list of {keyId(4), value(1-8)} packed.
    hexhead = pl[:16].hex()
    print(f"  [{rel_str}] {name:<12}  len={len(pl):>4}  hex16={hexhead}")

# CFG-MSG occurrence count - by class/id - across capture
print("\n=== CFG-MSG enable/disable summary (which messages were toggled by anyone) ===")
cfg_msg_targets = Counter()
for off, name, pl in records:
    if name != "CFG-MSG":
        continue
    if len(pl) >= 3:
        target_cls = pl[0]
        target_id = pl[1]
        rate = pl[2]
        target_name = MSG.get((target_cls, target_id), f"UNK_{target_cls:02x}{target_id:02x}")
        cfg_msg_targets[(target_name, rate)] += 1
for (tn, rate), c in cfg_msg_targets.most_common():
    print(f"  {tn:<15} rate={rate}  x{c}")

# === ACK / NAK timeline +/- 30s of event ===
print("\n=== ACK/NAK around event (+/-30s) ===")
for off, name, pl in records:
    if name not in ("ACK-ACK", "ACK-NAK"):
        continue
    t = off_to_t(off)
    if t is None: continue
    rel = t - t_event
    if abs(rel) > 30: continue
    cls_id = (pl[0], pl[1]) if len(pl) >= 2 else (0, 0)
    target = MSG.get(cls_id, f"UNK_{cls_id[0]:02x}{cls_id[1]:02x}")
    print(f"  {rel:>+7.2f}s  {name}  for {target}")
