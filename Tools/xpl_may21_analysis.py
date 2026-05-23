#!/usr/bin/env python3
"""Analyze the 2026-05-21 capture sequence to validate the MON-SPAN workaround.

Per user narrative:
  F1  125022  sanity check, MON-SPAN expected to still be on, bug expected
  F2  131053  after disabling MON-SPAN in all layers — bug should NOT fire
  F3  131923  same — bug should NOT fire
  F4  132801  same — bug should NOT fire
  F5  133810  MON-SPAN turned back on in Flash + RAM — interesting if bug doesn't fire

Look for:
  - Hard sat cliff (>=10 sat drop) = full bug fire
  - agcCnt transitions = canonical bug signature even if soft
  - Phantom velocity (gSpeed jump > 200 mm/s with healthy fix prior)
  - Vel-cov spikes >= 0.3 m^2/s^2
  - MON-SPAN message count = is it actually disabled or not?
"""
import os
import struct
from collections import Counter

DESKTOP = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop"

CAPTURES = [
    ("F1", "ublox_node121_20260521-125022.ubx",
     "Sanity check, MON-SPAN still on, bug expected"),
    ("F2", "ublox_node121_20260521-131053.ubx",
     "After disabling MON-SPAN in all layers (RAM + BBR + Flash)"),
    ("F3", "ublox_node121_20260521-131923.ubx",
     "Same condition as F2"),
    ("F4", "ublox_node121_20260521-132801.ubx",
     "Same condition as F2/F3"),
    ("F5", "ublox_node121_20260521-133810.ubx",
     "MON-SPAN re-enabled in Flash + RAM"),
]


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


def parse(path):
    with open(path, "rb") as f:
        data = f.read()
    inv = Counter()
    pvt = []; cov = []; clock = []; hw = []; navstatus = []; mon_span_count = 0
    pvt_anchors = []

    for off, cls, mid, pl in iter_ubx(data):
        if cls == 0x01 and mid == 0x07 and len(pl) >= 92:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            pvt_anchors.append((off, iTOW))

    if not pvt_anchors: return None
    t_start = pvt_anchors[0][1] / 1000
    t_end = pvt_anchors[-1][1] / 1000

    def off_to_t(off):
        if off <= pvt_anchors[0][0]: return pvt_anchors[0][1] / 1000
        if off >= pvt_anchors[-1][0]: return pvt_anchors[-1][1] / 1000
        lo, hi = 0, len(pvt_anchors) - 1
        while lo + 1 < hi:
            m = (lo + hi) // 2
            if pvt_anchors[m][0] <= off: lo = m
            else: hi = m
        o0, t0 = pvt_anchors[lo]; o1, t1 = pvt_anchors[hi]
        if o1 == o0: return t0 / 1000
        return (t0 + (t1 - t0) * (off - o0) / (o1 - o0)) / 1000

    NAMES = {(0x01,0x07):"NAV-PVT",(0x01,0x04):"NAV-DOP",(0x01,0x20):"NAV-TIMEGPS",
             (0x01,0x22):"NAV-CLOCK",(0x01,0x35):"NAV-SAT",(0x01,0x43):"NAV-SIG",
             (0x01,0x36):"NAV-COV",(0x01,0x03):"NAV-STATUS",
             (0x05,0x00):"ACK-ACK",(0x05,0x01):"ACK-NAK",
             (0x06,0x01):"CFG-MSG",(0x06,0x8a):"CFG-VALSET",(0x06,0x8b):"CFG-VALGET",
             (0x0a,0x09):"MON-HW",(0x0a,0x0b):"MON-IO",(0x0a,0x31):"MON-SPAN"}

    for off, cls, mid, pl in iter_ubx(data):
        name = NAMES.get((cls, mid))
        if name: inv[name] += 1

        if cls == 0x01 and mid == 0x07 and len(pl) >= 92:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            fix = pl[20]; sv = pl[23]
            hAcc = struct.unpack("<I", pl[40:44])[0] / 1000
            vAcc = struct.unpack("<I", pl[44:48])[0] / 1000
            g = struct.unpack("<i", pl[60:64])[0]
            pDOP = struct.unpack("<H", pl[76:78])[0] * 0.01
            pvt.append((t, fix, sv, g, hAcc, vAcc, pDOP))
        elif cls == 0x01 and mid == 0x36 and len(pl) >= 64:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            vNN = struct.unpack("<f", pl[40:44])[0]
            vEE = struct.unpack("<f", pl[52:56])[0]
            vDD = struct.unpack("<f", pl[60:64])[0]
            cov.append((t, vNN, vEE, vDD))
        elif cls == 0x01 and mid == 0x22 and len(pl) >= 20:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            clkB = struct.unpack("<i", pl[4:8])[0]
            fAcc = struct.unpack("<I", pl[16:20])[0]
            clock.append((t, clkB, fAcc))
        elif cls == 0x01 and mid == 0x03 and len(pl) >= 16:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            navstatus.append(t)
        elif cls == 0x0a and mid == 0x09 and len(pl) >= 60:
            t = off_to_t(off) - t_start
            n = struct.unpack("<H", pl[16:18])[0]
            agc = struct.unpack("<H", pl[18:20])[0]
            hw.append((t, n, agc))
        elif cls == 0x0a and mid == 0x31:
            mon_span_count += 1

    # Find event by sat-cliff rule
    et = None
    window = []
    for entry in pvt:
        t = entry[0]; sv = entry[2]
        window = [(tt, n) for tt, n in window if t - tt <= 3.0]
        window.append((t, sv))
        rmax = max(n for _, n in window)
        if rmax >= 10 and sv + 10 <= rmax and et is None:
            et = t

    # Soft-fire: gSpd jump >= 300 with healthy prior fix
    soft = False
    if et is None:
        hw2 = []
        healthy = {}
        for i, entry in enumerate(pvt):
            t = entry[0]; hAcc = entry[4]; fix = entry[1]
            hw2 = [(tt, ha, fx) for tt, ha, fx in hw2 if t - tt <= 2.0]
            hw2.append((t, hAcc, fix))
            healthy[i] = (len(hw2) >= 3 and
                         all(ha < 5.0 and fx >= 3 for _, ha, fx in hw2))
        prev = None
        biggest = (0, None)
        for i, entry in enumerate(pvt):
            if prev is not None:
                d = abs(entry[3] - prev)
                if d > biggest[0] and healthy.get(i, False):
                    biggest = (d, entry[0])
            prev = entry[3]
        if biggest[0] >= 300:
            et = biggest[1]
            soft = True

    # agcCnt transitions
    agc_changes = []
    prev = None
    agc_distrib = Counter()
    for r in hw:
        agc_distrib[r[2]] += 1
        if prev is not None and r[2] != prev:
            agc_changes.append((r[0], prev, r[2]))
        prev = r[2]

    # Max gSpd jump (regardless of healthy guard)
    max_jump = 0
    max_jump_t = None
    prev = None
    for entry in pvt:
        if prev is not None:
            d = abs(entry[3] - prev)
            if d > max_jump:
                max_jump = d
                max_jump_t = entry[0]
        prev = entry[3]

    # Max velCov
    max_velcov = 0
    max_velcov_t = None
    for r in cov:
        m = max(r[1], r[2], r[3])
        if m > max_velcov:
            max_velcov = m
            max_velcov_t = r[0]

    # Max gSpd value (peak velocity even if not a jump)
    max_gspd = max((e[3] for e in pvt), default=0)
    # Max hAcc value (during steady state — excluding warm-up)
    healthy_hacc = [e[4] for e in pvt if e[1] >= 3 and e[4] < 50.0]
    max_hAcc_healthy = max(healthy_hacc, default=0)

    # File size in bytes per second of duration
    duration = t_end - t_start
    rate_bps = len(data) / duration if duration > 0 else 0

    return dict(
        path=path, size=len(data), duration=duration, rate_bps=rate_bps,
        inv=dict(inv), mon_span=mon_span_count,
        pvt=pvt, cov=cov, clock=clock, hw=hw, navstatus=navstatus,
        event_t=et, soft_fire=soft, agc_changes=agc_changes,
        agc_distrib=dict(agc_distrib),
        max_jump=max_jump, max_jump_t=max_jump_t,
        max_velcov=max_velcov, max_velcov_t=max_velcov_t,
        max_gspd=max_gspd, max_hAcc_healthy=max_hAcc_healthy,
    )


print("Parsing all May 21 captures...")
results = {}
for label, fname, desc in CAPTURES:
    p = os.path.join(DESKTOP, fname)
    if not os.path.exists(p):
        print(f"  {label}: NOT FOUND")
        continue
    print(f"  {label}: {fname}")
    results[label] = parse(p)

print("\n" + "=" * 90)
print("SEQUENCE SUMMARY")
print("=" * 90)

print(f"\n{'tag':<5} {'duration':>9} {'size':>8} {'bytes/s':>8} {'event':>9} "
      f"{'cliff':>6} {'jump':>7} {'velcov':>8} {'MONSPN':>6} {'AGCs'}")
for label, fname, desc in CAPTURES:
    d = results.get(label)
    if d is None: continue
    et_str = f"+{d['event_t']:.0f}s" if d['event_t'] is not None else "—"
    cliff = "yes" if (d['event_t'] is not None and not d['soft_fire']) else "no"
    agc_seq = "->".join([str(d['agc_changes'][0][1])] + [str(c[2]) for c in d['agc_changes']]) if d['agc_changes'] else "stable"
    print(f"{label:<5} {d['duration']:>8.0f}s {d['size']:>8,} {d['rate_bps']:>8.0f} {et_str:>9} "
          f"{cliff:>6} {d['max_jump']:>7} {d['max_velcov']:>8.3f} {d['mon_span']:>6} {agc_seq}")

print(f"\nLegend: cliff=hard sat drop >=10 sats; jump=largest gSpd delta (mm/s); "
      f"velcov=max velocity covariance (m^2/s^2); MONSPN=count of MON-SPAN msgs")

# Per-capture detail
for label, fname, desc in CAPTURES:
    d = results.get(label)
    if d is None: continue
    print(f"\n--- {label}: {desc} ---")
    print(f"  Duration: {d['duration']:.0f}s   File: {d['size']:,} bytes "
          f"({d['rate_bps']:.0f} bytes/sec)")
    print(f"  MON-SPAN messages: {d['mon_span']} "
          f"({'~' + str(round(d['mon_span']/d['duration'], 1)) + ' Hz' if d['mon_span'] > 0 else 'DISABLED'})")
    print(f"  Sat cliff event: {('HARD at +' + str(round(d['event_t'])) + 's') if (d['event_t'] is not None and not d['soft_fire']) else 'none'}")
    print(f"  agcCnt distribution: {d['agc_distrib']}")
    if d['agc_changes']:
        print(f"  agcCnt transitions:")
        for t, a, b in d['agc_changes']:
            print(f"    +{t:.1f}s  {a} -> {b}")
    else:
        print(f"  agcCnt: no transitions (stayed at {list(d['agc_distrib'].keys())})")
    print(f"  Max gSpd jump: {d['max_jump']} mm/s at +{d['max_jump_t']:.1f}s"
          if d['max_jump_t'] else "")
    print(f"  Max peak gSpd: {d['max_gspd']} mm/s")
    print(f"  Max velCov: {d['max_velcov']:.3f} m^2/s^2 at +{d['max_velcov_t']:.1f}s"
          if d['max_velcov_t'] else "  Max velCov: no NAV-COV data")
    print(f"  Max steady-state hAcc: {d['max_hAcc_healthy']:.2f} m")
    print(f"  NAV-STATUS messages: {len(d['navstatus'])}")
    print(f"  Inventory: {dict(sorted(d['inv'].items(), key=lambda x: -x[1])[:6])}")

    # If event detected, show NAV-PVT around it
    if d['event_t'] is not None:
        et = d['event_t']
        print(f"\n  NAV-PVT around event (-1s to +1s):")
        print(f"  {'rel':>7}  {'sv':>2}  {'fix':>3}  {'gSpd':>6}  {'hAcc':>6}")
        for entry in d['pvt']:
            t = entry[0]; rel = t - et
            if -1 <= rel <= 1:
                m = " <-" if abs(rel) < 0.21 else ""
                print(f"  {rel:>+6.2f}s  {entry[2]:>2}  {entry[1]:>3}  "
                      f"{entry[3]:>6}  {entry[4]:>5.2f}{m}")
