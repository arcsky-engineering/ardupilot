#!/usr/bin/env python3
"""Comprehensive HTML data dump for all Here4 bug captures (10 total: C1-C7, D1-D4).

Goal: emit every plot and table that could possibly help someone draw their
own conclusions. No analysis spin — just data presented cleanly and
consistently across all captures.

Per capture, this generates:
  - Metadata + test conditions
  - Full message-type inventory with counts and average rates
  - Statistics tables for every numeric field (min, max, mean, median, std)
  - Full-capture plots for every signal in every available message type
  - Event-window zoom (-5s to +5s) plots
  - Anomaly listings (gSpd jumps, sat dips, vel-cov spikes, etc.)

Cross-capture comparison plots are at the top.

Writes to:  Desktop/Here4_full_data_dump_20260513.html
"""
import base64
import io
import os
import struct
from collections import Counter
from statistics import mean, median, pstdev

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DESKTOP = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop"

# Capture catalog: (label, file, short title, conditions narrative)
CAPTURES = [
    ("C1", "ublox_node121_20260513-181219.ubx",
     "Unmolested baseline",
     "gps_debug recording only, no u-center, no manual interference. Just the "
     "GNSSPeriph driver running its normal CFG poll cycle. The cleanest natural "
     "trigger we have."),
    ("C2", "ublox_node121_20260513-182415.ubx",
     "Unmolested + UBX_DebugMessages bit",
     "gps_debug recording. GPS_DRV_OPTIONS bit 0x002 enabled (which only causes "
     "GNSSPeriph to emit extra CAN log strings; does not affect the UBX data "
     "stream itself). Node rebooted to apply. Fired ~8 min in."),
    ("C3", "ublox_node121_20260513-184552.ubx",
     "u-center enabled NAV-SAT only",
     "u-center connected via tunnel mid-capture. User enabled only NAV-SAT. "
     "Bug fired 5s after the enable took effect — first evidence that host CFG "
     "activity accelerates the trigger."),
    ("C4", "ublox_node121_20260513-185749.ubx",
     "u-center sequential enables + re-enables",
     "10-minute session. User enabled NAV-SAT/SIG/CLOCK/COV sequentially via "
     "u-center. Messages kept getting disabled by u-center's view-management "
     "behavior; user kept re-enabling. Soft bug fired with a brief agcCnt revert "
     "(3510→3861→3510) — first observation that the alternate AGC state is "
     "transient."),
    ("C6", "ublox_node121_20260513-192235.ubx",
     "Custom GUI button — 4 enables at startup",
     "First use of the new diag-msgs button. Fired all four CFG-MSG enables "
     "(NAV-SAT/SIG/CLOCK/COV) immediately on session start. Hard bug fired 200ms "
     "later, with the largest phantom velocity we've recorded (2,341 mm/s)."),
    ("C7", "ublox_node121_20260513-192509.ubx",
     "Custom GUI button + 10-min observation",
     "Button fired at startup. Softest event yet — single phantom-velocity epoch, "
     "vel-cov spike to 60× baseline, recovered without sat cliff or agcCnt step. "
     "9.5 minutes completely clean afterward."),

    # === Day 2 captures (2026-05-14) — old unused Here4 unit (node 122) ===
    ("D1", "ublox_node122_20260514-191122.ubx",
     "Day 2: Fresh power-on, never-used unit",
     "Old/unused Here4 hardware on carrier-board test stand. First capture after "
     "power-on. Natural fire at ~7:36 with canonical signature (agcCnt 3510 → 3861 "
     "→ 3510)."),
    ("D2", "ublox_node122_20260514-192155.ubx",
     "Day 2: After u-center COLD BOOT ONLY (no config reset)",
     "Cold-boot via UBX-CFG-RST. NO configuration reset. Bug fired AGAIN at ~7:46. "
     "Crucially, the receiver started this capture at agcCnt=3861 (not 3510), and "
     "during the event went to a brand-new value of 4212. Proves cold-boot alone "
     "is insufficient — it does not restore the receiver to the 3510 baseline."),
    ("D3", "ublox_node122_20260514-193327.ubx",
     "Day 2: After configuration RESET + cold boot",
     "Full u-center procedure: configuration reset to defaults, then cold boot. "
     "Receiver re-acquired fix from cold-start, then ran clean for the full "
     "9.5 minutes. No bug. (Note: brief no-fix acquisition window at the very "
     "start of capture is normal warm-up, not a bug event.)"),
    ("D4", "ublox_node122_20260514-194332.ubx",
     "Day 2: Continued after config-reset + cold boot",
     "Continuation session after D3 — no procedure changes. Fully clean across "
     "9.5 minutes. Combined with D3, ~18 min clean operation after the full "
     "treatment, well past the canonical 8-min bug-fire timing."),
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


MSG_NAMES = {(0x01,0x07):"NAV-PVT",(0x01,0x04):"NAV-DOP",(0x01,0x20):"NAV-TIMEGPS",
             (0x01,0x22):"NAV-CLOCK",(0x01,0x35):"NAV-SAT",(0x01,0x43):"NAV-SIG",
             (0x01,0x36):"NAV-COV",(0x01,0x03):"NAV-STATUS",(0x01,0x61):"NAV-EOE",
             (0x05,0x00):"ACK-ACK",(0x05,0x01):"ACK-NAK",
             (0x06,0x00):"CFG-PRT",(0x06,0x01):"CFG-MSG",(0x06,0x04):"CFG-RST",
             (0x06,0x06):"CFG-DAT",(0x06,0x08):"CFG-RATE",(0x06,0x09):"CFG-CFG",
             (0x06,0x13):"CFG-ANT",(0x06,0x24):"CFG-NAV5",(0x06,0x31):"CFG-TP5",
             (0x06,0x3e):"CFG-GNSS",(0x06,0x8a):"CFG-VALSET",(0x06,0x8b):"CFG-VALGET",
             (0x0a,0x04):"MON-VER",(0x0a,0x09):"MON-HW",(0x0a,0x0b):"MON-IO",
             (0x0a,0x36):"MON-COMMS",(0x0a,0x37):"MON-HW3",(0x0a,0x38):"MON-RF",
             (0x0a,0x31):"MON-SPAN",(0x0d,0x03):"TIM-TM2"}


def parse_capture(path):
    """Parse one .ubx file and extract all signal time series."""
    with open(path, "rb") as f:
        data = f.read()

    inventory = Counter()
    nav_pvt = []
    nav_dop = []
    nav_clock = []
    nav_cov = []
    nav_sat = []
    nav_timegps = []
    mon_hw = []
    mon_io_first_last = []  # just first/last for context
    mon_span_count_per_block = []  # (t, [center_freqs], [pgas]) — to show RF stability

    pvt_anchors = []
    for off, cls, mid, pl in iter_ubx(data):
        name = MSG_NAMES.get((cls, mid))
        if name:
            inventory[name] += 1
        if cls == 0x01 and mid == 0x07 and len(pl) >= 92:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            pvt_anchors.append((off, iTOW))

    if not pvt_anchors:
        return None

    t_start = pvt_anchors[0][1] / 1000.0
    t_end = pvt_anchors[-1][1] / 1000.0

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

    for off, cls, mid, pl in iter_ubx(data):
        if cls == 0x01 and mid == 0x07 and len(pl) >= 92:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            fixType = pl[20]; flags = pl[21]; flags2 = pl[22]; numSV = pl[23]
            hAcc = struct.unpack("<I", pl[40:44])[0]
            vAcc = struct.unpack("<I", pl[44:48])[0]
            velN = struct.unpack("<i", pl[48:52])[0]
            velE = struct.unpack("<i", pl[52:56])[0]
            velD = struct.unpack("<i", pl[56:60])[0]
            gSpeed = struct.unpack("<i", pl[60:64])[0]
            headMot = struct.unpack("<i", pl[64:68])[0]
            sAcc = struct.unpack("<I", pl[68:72])[0]
            headAcc = struct.unpack("<I", pl[72:76])[0]
            pDOP = struct.unpack("<H", pl[76:78])[0] * 0.01
            nav_pvt.append((t, fixType, numSV, hAcc/1000.0, vAcc/1000.0,
                            velN, velE, velD, gSpeed, headMot, sAcc, headAcc, pDOP,
                            flags, flags2))
        elif cls == 0x01 and mid == 0x04 and len(pl) >= 18:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            gDOP = struct.unpack("<H", pl[4:6])[0] * 0.01
            pDOP = struct.unpack("<H", pl[6:8])[0] * 0.01
            tDOP = struct.unpack("<H", pl[8:10])[0] * 0.01
            vDOP = struct.unpack("<H", pl[10:12])[0] * 0.01
            hDOP = struct.unpack("<H", pl[12:14])[0] * 0.01
            nDOP = struct.unpack("<H", pl[14:16])[0] * 0.01
            eDOP = struct.unpack("<H", pl[16:18])[0] * 0.01
            nav_dop.append((t, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP))
        elif cls == 0x01 and mid == 0x22 and len(pl) >= 20:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            clkB = struct.unpack("<i", pl[4:8])[0]
            clkD = struct.unpack("<i", pl[8:12])[0]
            tAcc = struct.unpack("<I", pl[12:16])[0]
            fAcc = struct.unpack("<I", pl[16:20])[0]
            nav_clock.append((t, clkB, clkD, tAcc, fAcc))
        elif cls == 0x01 and mid == 0x36 and len(pl) >= 64:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            pV = pl[5]; vV = pl[6]
            pNN = struct.unpack("<f", pl[16:20])[0]
            pNE = struct.unpack("<f", pl[20:24])[0]
            pND = struct.unpack("<f", pl[24:28])[0]
            pEE = struct.unpack("<f", pl[28:32])[0]
            pED = struct.unpack("<f", pl[32:36])[0]
            pDD = struct.unpack("<f", pl[36:40])[0]
            vNN = struct.unpack("<f", pl[40:44])[0]
            vNE = struct.unpack("<f", pl[44:48])[0]
            vND = struct.unpack("<f", pl[48:52])[0]
            vEE = struct.unpack("<f", pl[52:56])[0]
            vED = struct.unpack("<f", pl[56:60])[0]
            vDD = struct.unpack("<f", pl[60:64])[0]
            nav_cov.append((t, pV, vV, pNN, pNE, pND, pEE, pED, pDD,
                           vNN, vNE, vND, vEE, vED, vDD))
        elif cls == 0x01 and mid == 0x35 and len(pl) >= 8:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            numSvs = pl[5]
            used = tr = se = bad_h = 0
            by_g = Counter()
            cno_list = []
            for i in range(numSvs):
                o = 8 + i * 12
                if o + 12 > len(pl): break
                gnss = pl[o]
                cno = pl[o + 2]
                cno_list.append(cno)
                flags = struct.unpack("<I", pl[o + 8:o + 12])[0]
                q = flags & 0x7
                u = (flags >> 3) & 0x1
                h = (flags >> 4) & 0x3
                if q >= 4: tr += 1
                elif q in (1, 2): se += 1
                if u:
                    used += 1
                    by_g[gnss] += 1
                if h == 2: bad_h += 1
            nav_sat.append((t, numSvs, used, tr, se, bad_h, dict(by_g),
                           mean(cno_list) if cno_list else 0,
                           max(cno_list) if cno_list else 0))
        elif cls == 0x01 and mid == 0x20 and len(pl) >= 16:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            week = struct.unpack("<h", pl[8:10])[0]
            leapS = struct.unpack("<b", pl[10:11])[0]
            valid = pl[11]
            tAcc = struct.unpack("<I", pl[12:16])[0]
            nav_timegps.append((t, week, leapS, valid, tAcc))
        elif cls == 0x0a and mid == 0x09 and len(pl) >= 60:
            t = off_to_t(off) - t_start
            noisePerMS = struct.unpack("<H", pl[16:18])[0]
            agcCnt = struct.unpack("<H", pl[18:20])[0]
            aStatus = pl[20]
            aPower = pl[21]
            flags = pl[22]
            jamInd = pl[45]
            mon_hw.append((t, noisePerMS, agcCnt, aStatus, aPower, flags, jamInd))
        elif cls == 0x0a and mid == 0x31 and len(pl) >= 4:
            t = off_to_t(off) - t_start
            nBlocks = pl[1]
            centers = []
            spans = []
            pgas = []
            for b in range(nBlocks):
                base = 4 + b * 272
                if base + 272 > len(pl): break
                span = struct.unpack("<I", pl[base + 256:base + 260])[0]
                center = struct.unpack("<I", pl[base + 264:base + 268])[0]
                pga = pl[base + 268]
                centers.append(center)
                spans.append(span)
                pgas.append(pga)
            mon_span_count_per_block.append((t, centers, spans, pgas))

    # Find event
    window = []
    event_t = None
    for t, _, sv, _, _, _, _, _, _, _, _, _, _, _, _ in nav_pvt:
        window = [(tt, n) for tt, n in window if t - tt <= 3.0]
        window.append((t, sv))
        rmax = max(n for _, n in window)
        if rmax >= 10 and sv + 10 <= rmax:
            event_t = t
            break

    # Soft-fire fallback: biggest gSpd jump
    # GUARD: only count gSpd jumps where the receiver had a healthy fix in the
    # preceding 2 seconds (fixType >= 3 AND hAcc < 5m). This prevents the
    # warm-up phase after a config-reset from being mis-classified as a soft
    # fire — at that point fixType is 0 or hAcc is huge.
    if event_t is None:
        # Precompute a "healthy-recently" flag for each PVT epoch
        healthy_window = []  # list of (t, hAcc, fix)
        healthy_at = {}      # idx -> bool
        for i, entry in enumerate(nav_pvt):
            t, fix, _, _, hAcc, _, _, _, _, _, _, _, _, _, _ = entry
            healthy_window = [(tt, ha, fx) for tt, ha, fx in healthy_window
                             if t - tt <= 2.0]
            healthy_window.append((t, hAcc, fix))
            # "healthy" if all samples in last 2s had fix=3 and hAcc < 5m
            healthy_at[i] = (len(healthy_window) >= 3 and
                            all(ha < 5.0 and fx >= 3 for _, ha, fx in healthy_window))
        prev = None
        biggest = (0, None)
        for i, entry in enumerate(nav_pvt):
            gSpd = entry[8]
            if prev is not None and abs(gSpd - prev) > biggest[0] and healthy_at.get(i, False):
                biggest = (abs(gSpd - prev), entry[0])
            prev = gSpd
        if biggest[0] >= 200:
            event_t = biggest[1]

    return dict(path=path, size=len(data), duration=t_end - t_start,
                inventory=dict(inventory),
                nav_pvt=nav_pvt, nav_dop=nav_dop, nav_clock=nav_clock,
                nav_cov=nav_cov, nav_sat=nav_sat, nav_timegps=nav_timegps,
                mon_hw=mon_hw, mon_span=mon_span_count_per_block,
                event_t=event_t, t_start=t_start, t_end=t_end)


def fig_to_b64(fig, dpi=85):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def _stat(series):
    if not series: return ("—", "—", "—", "—", "—")
    return (f"{min(series):.3f}", f"{max(series):.3f}",
            f"{mean(series):.3f}", f"{median(series):.3f}",
            f"{pstdev(series):.3f}" if len(series) > 1 else "—")


def stats_table_html(d):
    """Per-signal statistics table."""
    rows = []
    rows.append(("NAV-PVT  numSV",         [r[2] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  fixType",       [r[1] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  hAcc (m)",      [r[3] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  vAcc (m)",      [r[4] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  velN (mm/s)",   [r[5] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  velE (mm/s)",   [r[6] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  velD (mm/s)",   [r[7] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  gSpeed (mm/s)", [r[8] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  sAcc (mm/s)",   [r[10] for r in d["nav_pvt"]]))
    rows.append(("NAV-PVT  pDOP",          [r[12] for r in d["nav_pvt"]]))

    if d["nav_dop"]:
        rows.append(("NAV-DOP  gDOP", [r[1] for r in d["nav_dop"]]))
        rows.append(("NAV-DOP  hDOP", [r[5] for r in d["nav_dop"]]))
        rows.append(("NAV-DOP  vDOP", [r[4] for r in d["nav_dop"]]))
        rows.append(("NAV-DOP  tDOP", [r[3] for r in d["nav_dop"]]))
        rows.append(("NAV-DOP  nDOP", [r[6] for r in d["nav_dop"]]))
        rows.append(("NAV-DOP  eDOP", [r[7] for r in d["nav_dop"]]))

    if d["nav_clock"]:
        rows.append(("NAV-CLOCK  clkB (ns)",   [r[1] for r in d["nav_clock"]]))
        rows.append(("NAV-CLOCK  clkD (ns/s)", [r[2] for r in d["nav_clock"]]))
        rows.append(("NAV-CLOCK  tAcc (ns)",   [r[3] for r in d["nav_clock"]]))
        rows.append(("NAV-CLOCK  fAcc (ps/s)", [r[4] for r in d["nav_clock"]]))

    if d["nav_cov"]:
        rows.append(("NAV-COV  posCovNN (m²)",     [r[3] for r in d["nav_cov"]]))
        rows.append(("NAV-COV  posCovEE",          [r[6] for r in d["nav_cov"]]))
        rows.append(("NAV-COV  posCovDD",          [r[8] for r in d["nav_cov"]]))
        rows.append(("NAV-COV  velCovNN (m²/s²)",  [r[9] for r in d["nav_cov"]]))
        rows.append(("NAV-COV  velCovEE",          [r[12] for r in d["nav_cov"]]))
        rows.append(("NAV-COV  velCovDD",          [r[14] for r in d["nav_cov"]]))

    if d["nav_sat"]:
        rows.append(("NAV-SAT  numSvs",   [r[1] for r in d["nav_sat"]]))
        rows.append(("NAV-SAT  used",     [r[2] for r in d["nav_sat"]]))
        rows.append(("NAV-SAT  tracking", [r[3] for r in d["nav_sat"]]))
        rows.append(("NAV-SAT  searching",[r[4] for r in d["nav_sat"]]))
        rows.append(("NAV-SAT  mean CN0", [r[7] for r in d["nav_sat"]]))

    if d["mon_hw"]:
        rows.append(("MON-HW  noisePerMS", [r[1] for r in d["mon_hw"]]))
        rows.append(("MON-HW  agcCnt",     [r[2] for r in d["mon_hw"]]))
        rows.append(("MON-HW  aStatus",    [r[3] for r in d["mon_hw"]]))
        rows.append(("MON-HW  aPower",     [r[4] for r in d["mon_hw"]]))
        rows.append(("MON-HW  jamInd",     [r[6] for r in d["mon_hw"]]))

    html = ["<table><tr><th>Signal</th><th class='num'>n</th><th class='num'>min</th>"
            "<th class='num'>max</th><th class='num'>mean</th><th class='num'>median</th>"
            "<th class='num'>stdev</th></tr>"]
    for name, series in rows:
        if not series:
            html.append(f"<tr><td>{name}</td><td colspan='6'>—</td></tr>")
            continue
        mn, mx, mu, md, sd = _stat(series)
        html.append(f"<tr><td>{name}</td><td class='num'>{len(series)}</td>"
                    f"<td class='num'>{mn}</td><td class='num'>{mx}</td>"
                    f"<td class='num'>{mu}</td><td class='num'>{md}</td>"
                    f"<td class='num'>{sd}</td></tr>")
    html.append("</table>")
    return "\n".join(html)


def inventory_table_html(d):
    rows = sorted(d["inventory"].items(), key=lambda x: -x[1])
    dur = d["duration"]
    html = ["<table><tr><th>Message</th><th class='num'>Count</th>"
            "<th class='num'>Avg rate (Hz)</th></tr>"]
    for name, c in rows:
        rate = c / dur if dur > 0 else 0
        html.append(f"<tr><td>{name}</td><td class='num'>{c}</td>"
                    f"<td class='num'>{rate:.2f}</td></tr>")
    html.append("</table>")
    return "\n".join(html)


# ============================================================
# Plotting helpers (per-capture)
# ============================================================

def vline_event(ax, et, label=True):
    if et is None: return
    ax.axvline(et, color="red", lw=0.8, ls="--", alpha=0.6,
               label=f"event @ +{et:.1f}s" if label else None)


def plot_nav_pvt(d, label):
    if not d["nav_pvt"]: return None
    ts = [r[0] for r in d["nav_pvt"]]
    fig, axes = plt.subplots(5, 1, figsize=(14, 11), sharex=True)
    et = d["event_t"]

    # numSV + fixType
    axes[0].plot(ts, [r[2] for r in d["nav_pvt"]], lw=0.6, color="#1f4e8f", label="numSV")
    axes[0].set_ylabel("numSV used")
    axes[0].set_ylim(0, 35)
    vline_event(axes[0], et)
    ax0b = axes[0].twinx()
    ax0b.plot(ts, [r[1] for r in d["nav_pvt"]], lw=0.6, color="#666", alpha=0.5, label="fixType")
    ax0b.set_ylabel("fixType (gray)", color="#666")
    ax0b.set_ylim(0, 5)
    axes[0].legend(loc="upper left")
    axes[0].grid(alpha=0.3)

    # hAcc + vAcc
    axes[1].plot(ts, [r[3] for r in d["nav_pvt"]], lw=0.6, color="#306030", label="hAcc")
    axes[1].plot(ts, [r[4] for r in d["nav_pvt"]], lw=0.6, color="#603030", label="vAcc")
    axes[1].set_ylabel("accuracy (m)")
    vline_event(axes[1], et, label=False)
    axes[1].legend(loc="upper left")
    axes[1].grid(alpha=0.3)

    # velocity (N/E/D + gSpeed)
    axes[2].plot(ts, [r[5] for r in d["nav_pvt"]], lw=0.4, color="#1f4e8f", alpha=0.5, label="velN")
    axes[2].plot(ts, [r[6] for r in d["nav_pvt"]], lw=0.4, color="#206030", alpha=0.5, label="velE")
    axes[2].plot(ts, [r[7] for r in d["nav_pvt"]], lw=0.4, color="#603030", alpha=0.5, label="velD")
    axes[2].plot(ts, [r[8] for r in d["nav_pvt"]], lw=0.6, color="#a02050", label="gSpeed")
    axes[2].set_ylabel("velocity (mm/s)")
    vline_event(axes[2], et, label=False)
    axes[2].legend(loc="upper left", fontsize=8)
    axes[2].grid(alpha=0.3)

    # sAcc + headAcc
    axes[3].plot(ts, [r[10] for r in d["nav_pvt"]], lw=0.6, color="#806020", label="sAcc")
    axes[3].set_ylabel("speed acc (mm/s)")
    vline_event(axes[3], et, label=False)
    axes[3].legend(loc="upper left")
    axes[3].grid(alpha=0.3)

    # pDOP
    axes[4].plot(ts, [r[12] for r in d["nav_pvt"]], lw=0.6, color="#605020", label="pDOP")
    axes[4].set_ylabel("pDOP")
    axes[4].set_xlabel("capture time (s)")
    vline_event(axes[4], et, label=False)
    axes[4].legend(loc="upper left")
    axes[4].grid(alpha=0.3)

    fig.suptitle(f"{label} — NAV-PVT full timeline", fontsize=12)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_nav_dop(d, label):
    if not d["nav_dop"]: return None
    ts = [r[0] for r in d["nav_dop"]]
    fig, ax = plt.subplots(1, 1, figsize=(14, 4))
    for idx, name in enumerate(["gDOP", "pDOP", "tDOP", "vDOP", "hDOP", "nDOP", "eDOP"]):
        ax.plot(ts, [r[idx + 1] for r in d["nav_dop"]], lw=0.6, label=name)
    vline_event(ax, d["event_t"])
    ax.set_xlabel("capture time (s)")
    ax.set_ylabel("DOP (dimensionless)")
    ax.legend(loc="upper left", ncol=4, fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title(f"{label} — NAV-DOP (all components)")
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_nav_clock(d, label):
    if not d["nav_clock"]: return None
    ts = [r[0] for r in d["nav_clock"]]
    fig, axes = plt.subplots(4, 1, figsize=(14, 9), sharex=True)
    et = d["event_t"]
    axes[0].plot(ts, [r[1] for r in d["nav_clock"]], lw=0.6, color="#404090")
    axes[0].set_ylabel("clkB (ns)")
    vline_event(axes[0], et)
    axes[0].legend(loc="upper right")
    axes[0].grid(alpha=0.3)

    axes[1].plot(ts, [r[2] for r in d["nav_clock"]], lw=0.6, color="#704090")
    axes[1].set_ylabel("clkD (ns/s)")
    vline_event(axes[1], et, label=False)
    axes[1].grid(alpha=0.3)

    axes[2].plot(ts, [r[3] for r in d["nav_clock"]], lw=0.6, color="#a04060")
    axes[2].set_ylabel("tAcc (ns)")
    vline_event(axes[2], et, label=False)
    axes[2].grid(alpha=0.3)

    axes[3].plot(ts, [r[4] for r in d["nav_clock"]], lw=0.6, color="#a08020")
    axes[3].set_ylabel("fAcc (ps/s)")
    axes[3].set_xlabel("capture time (s)")
    vline_event(axes[3], et, label=False)
    axes[3].grid(alpha=0.3)

    fig.suptitle(f"{label} — NAV-CLOCK (clkB, clkD, tAcc, fAcc)", fontsize=12)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_nav_cov(d, label):
    if not d["nav_cov"]: return None
    ts = [r[0] for r in d["nav_cov"]]
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    et = d["event_t"]

    axes[0].plot(ts, [r[3] for r in d["nav_cov"]], lw=0.6, label="posCovNN", color="#1f4e8f")
    axes[0].plot(ts, [r[6] for r in d["nav_cov"]], lw=0.6, label="posCovEE", color="#206030")
    axes[0].plot(ts, [r[8] for r in d["nav_cov"]], lw=0.6, label="posCovDD", color="#603030")
    axes[0].set_ylabel("posCov (m²)")
    axes[0].set_yscale("log")
    vline_event(axes[0], et)
    axes[0].legend(loc="upper left", fontsize=8)
    axes[0].grid(alpha=0.3, which="both")

    axes[1].plot(ts, [r[9] for r in d["nav_cov"]], lw=0.6, label="velCovNN", color="#1f4e8f")
    axes[1].plot(ts, [r[12] for r in d["nav_cov"]], lw=0.6, label="velCovEE", color="#206030")
    axes[1].plot(ts, [r[14] for r in d["nav_cov"]], lw=0.6, label="velCovDD", color="#603030")
    axes[1].axhline(0.3, color="orange", lw=0.5, ls=":", label="0.3 thresh")
    axes[1].set_ylabel("velCov (m²/s²)")
    axes[1].set_yscale("log")
    axes[1].set_xlabel("capture time (s)")
    vline_event(axes[1], et, label=False)
    axes[1].legend(loc="upper left", fontsize=8)
    axes[1].grid(alpha=0.3, which="both")

    fig.suptitle(f"{label} — NAV-COV position + velocity covariance (log scale)",
                 fontsize=12)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_nav_sat(d, label):
    if not d["nav_sat"]: return None
    ts = [r[0] for r in d["nav_sat"]]
    fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    et = d["event_t"]
    axes[0].plot(ts, [r[1] for r in d["nav_sat"]], lw=0.6, color="#1f4e8f", label="numSvs (visible)")
    axes[0].plot(ts, [r[2] for r in d["nav_sat"]], lw=0.6, color="#a02050", label="used")
    axes[0].plot(ts, [r[3] for r in d["nav_sat"]], lw=0.6, color="#206030", label="tracking")
    axes[0].plot(ts, [r[4] for r in d["nav_sat"]], lw=0.6, color="#806020", label="searching")
    axes[0].set_ylabel("sat counts")
    vline_event(axes[0], et)
    axes[0].legend(loc="lower left", fontsize=8)
    axes[0].grid(alpha=0.3)

    # Per-constellation used
    gnss_names = {0: "GPS", 2: "GAL", 3: "BDS", 6: "GLO", 5: "QZSS"}
    colors = {0: "#1f4e8f", 2: "#a02050", 3: "#206030", 6: "#806020", 5: "#a04060"}
    for g, gn in gnss_names.items():
        series = [r[6].get(g, 0) for r in d["nav_sat"]]
        if max(series) > 0:
            axes[1].plot(ts, series, lw=0.7, label=gn, color=colors[g])
    axes[1].set_ylabel("used (per GNSS)")
    vline_event(axes[1], et, label=False)
    axes[1].legend(loc="lower left", fontsize=8, ncol=5)
    axes[1].grid(alpha=0.3)

    axes[2].plot(ts, [r[7] for r in d["nav_sat"]], lw=0.6, color="#406040", label="mean CN0")
    axes[2].plot(ts, [r[8] for r in d["nav_sat"]], lw=0.4, color="#406040", alpha=0.5, label="max CN0")
    axes[2].set_ylabel("CN0 (dB-Hz)")
    axes[2].set_xlabel("capture time (s)")
    vline_event(axes[2], et, label=False)
    axes[2].legend(loc="lower left", fontsize=8)
    axes[2].grid(alpha=0.3)

    fig.suptitle(f"{label} — NAV-SAT detail (counts, constellations, CN0)", fontsize=12)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_mon_hw(d, label):
    if not d["mon_hw"]: return None
    ts = [r[0] for r in d["mon_hw"]]
    fig, axes = plt.subplots(3, 1, figsize=(14, 7), sharex=True)
    et = d["event_t"]
    axes[0].plot(ts, [r[2] for r in d["mon_hw"]], marker="o", ms=3, lw=0.6, color="#106b3e", label="agcCnt")
    axes[0].set_ylabel("agcCnt (0-8191)")
    vline_event(axes[0], et)
    axes[0].legend(loc="upper left")
    axes[0].grid(alpha=0.3)

    axes[1].plot(ts, [r[1] for r in d["mon_hw"]], marker="o", ms=3, lw=0.6, color="#406040", label="noisePerMS")
    axes[1].plot(ts, [r[6] for r in d["mon_hw"]], marker="o", ms=3, lw=0.6, color="#604040", label="jamInd")
    axes[1].set_ylabel("noise / jamInd")
    vline_event(axes[1], et, label=False)
    axes[1].legend(loc="upper left")
    axes[1].grid(alpha=0.3)

    axes[2].plot(ts, [r[3] for r in d["mon_hw"]], marker="o", ms=3, lw=0.6, color="#604020", label="aStatus")
    axes[2].plot(ts, [r[4] for r in d["mon_hw"]], marker="o", ms=3, lw=0.6, color="#206020", label="aPower")
    axes[2].set_ylabel("antenna state")
    axes[2].set_xlabel("capture time (s)")
    axes[2].set_ylim(-0.5, 4.5)
    vline_event(axes[2], et, label=False)
    axes[2].legend(loc="upper left")
    axes[2].grid(alpha=0.3)

    fig.suptitle(f"{label} — MON-HW (agcCnt, noise, jamInd, antenna)", fontsize=12)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_event_zoom(d, label):
    """All available signals zoomed to ±5s of the event."""
    et = d["event_t"]
    if et is None or not d["nav_pvt"]: return None
    rows = 4 + (1 if d["nav_clock"] else 0) + (1 if d["nav_cov"] else 0) + (1 if d["nav_sat"] else 0)
    fig, axes = plt.subplots(rows, 1, figsize=(13, 2.0 * rows), sharex=True)
    if rows == 1: axes = [axes]
    ax_i = 0

    # NAV-PVT: numSV
    ts = [r[0] - et for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 5]
    svs = [r[2] for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 5]
    gss = [r[8] for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 5]
    hAcs = [r[3] for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 5]
    pDOPs = [r[12] for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 5]

    axes[ax_i].plot(ts, svs, marker="o", ms=3, lw=0.6, color="#1f4e8f")
    axes[ax_i].axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
    axes[ax_i].set_ylabel("numSV")
    axes[ax_i].set_ylim(0, 35)
    axes[ax_i].grid(alpha=0.3)
    ax_i += 1

    axes[ax_i].plot(ts, gss, marker="o", ms=3, lw=0.6, color="#a02050")
    axes[ax_i].axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
    axes[ax_i].set_ylabel("gSpeed (mm/s)")
    axes[ax_i].grid(alpha=0.3)
    ax_i += 1

    axes[ax_i].plot(ts, hAcs, marker="o", ms=3, lw=0.6, color="#306030")
    axes[ax_i].axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
    axes[ax_i].set_ylabel("hAcc (m)")
    axes[ax_i].grid(alpha=0.3)
    ax_i += 1

    axes[ax_i].plot(ts, pDOPs, marker="o", ms=3, lw=0.6, color="#605020")
    axes[ax_i].axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
    axes[ax_i].set_ylabel("pDOP")
    axes[ax_i].grid(alpha=0.3)
    ax_i += 1

    if d["nav_clock"]:
        cts = [r[0] - et for r in d["nav_clock"] if et - 5 <= r[0] <= et + 5]
        faccs = [r[4] for r in d["nav_clock"] if et - 5 <= r[0] <= et + 5]
        axes[ax_i].plot(cts, faccs, marker="o", ms=3, lw=0.6, color="#a08020")
        axes[ax_i].axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
        axes[ax_i].set_ylabel("fAcc (ps/s)")
        axes[ax_i].grid(alpha=0.3)
        ax_i += 1

    if d["nav_cov"]:
        cts = [r[0] - et for r in d["nav_cov"] if et - 5 <= r[0] <= et + 5]
        vmax = [max(r[9], r[12], r[14]) for r in d["nav_cov"] if et - 5 <= r[0] <= et + 5]
        axes[ax_i].semilogy(cts, vmax, marker="o", ms=3, lw=0.6, color="#a04020")
        axes[ax_i].axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
        axes[ax_i].axhline(0.3, color="orange", lw=0.5, ls=":")
        axes[ax_i].set_ylabel("max velCov (m²/s²)")
        axes[ax_i].grid(alpha=0.3, which="both")
        ax_i += 1

    if d["nav_sat"]:
        sts = [r[0] - et for r in d["nav_sat"] if et - 5 <= r[0] <= et + 5]
        used = [r[2] for r in d["nav_sat"] if et - 5 <= r[0] <= et + 5]
        tr = [r[3] for r in d["nav_sat"] if et - 5 <= r[0] <= et + 5]
        axes[ax_i].plot(sts, used, marker="o", ms=3, lw=0.6, color="#a02050", label="used")
        axes[ax_i].plot(sts, tr, marker="o", ms=3, lw=0.6, color="#206030", label="tracking")
        axes[ax_i].axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
        axes[ax_i].set_ylabel("sat counts")
        axes[ax_i].legend(loc="lower left", fontsize=8)
        axes[ax_i].grid(alpha=0.3)
        ax_i += 1

    axes[-1].set_xlabel("seconds relative to event")
    fig.suptitle(f"{label} — event zoom (-5s to +5s, red dashed = event entry)",
                 fontsize=12)
    fig.tight_layout()
    return fig_to_b64(fig)


def anomaly_list_html(d):
    """List sub-cliff sat dips and gSpd jumps and vel-cov spikes throughout the capture."""
    parts = []

    # gSpd jumps
    jumps = []
    prev = None
    for entry in d["nav_pvt"]:
        t = entry[0]; g = entry[8]
        if prev is not None and abs(g - prev) >= 200:
            jumps.append((t, prev, g))
        prev = g
    parts.append(f"<p><b>gSpeed jumps ≥ 200 mm/s</b>: {len(jumps)} total</p>")
    if jumps:
        parts.append("<table><tr><th class='num'>t (s)</th><th class='num'>prev → next (mm/s)</th></tr>")
        for t, a, b in jumps[:30]:
            parts.append(f"<tr><td class='num'>{t:.2f}</td><td class='num'>{a} → {b}</td></tr>")
        if len(jumps) > 30:
            parts.append(f"<tr><td colspan='2'>... ({len(jumps) - 30} more)</td></tr>")
        parts.append("</table>")

    # Sat dips
    window = []
    dips = []
    for entry in d["nav_pvt"]:
        t = entry[0]; sv = entry[2]
        window = [(tt, n) for tt, n in window if t - tt <= 3.0]
        window.append((t, sv))
        rmax = max(n for _, n in window)
        if rmax >= 10 and rmax - sv >= 4:
            dips.append((t, rmax, sv))
    # group
    groups = []
    if dips:
        cur = [dips[0]]
        for d2 in dips[1:]:
            if d2[0] - cur[-1][0] < 1.5: cur.append(d2)
            else: groups.append(cur); cur = [d2]
        groups.append(cur)
    parts.append(f"<p><b>Sat dips (drop ≥ 4 from rolling 3s max)</b>: "
                 f"{len(groups)} event(s), {len(dips)} samples</p>")
    if groups:
        parts.append("<table><tr><th class='num'>start t (s)</th><th class='num'>duration (s)</th>"
                     "<th class='num'>worst drop</th></tr>")
        for g in groups[:30]:
            dur = g[-1][0] - g[0][0]
            worst = max(x[1] - x[2] for x in g)
            parts.append(f"<tr><td class='num'>{g[0][0]:.2f}</td><td class='num'>{dur:.2f}</td>"
                         f"<td class='num'>{worst}</td></tr>")
        parts.append("</table>")

    # Vel-cov spikes
    if d["nav_cov"]:
        spikes = []
        for r in d["nav_cov"]:
            m = max(r[9], r[12], r[14])
            if m >= 0.3:
                spikes.append((r[0], m))
        parts.append(f"<p><b>Vel-cov spikes ≥ 0.3 m²/s²</b>: {len(spikes)}</p>")
        if spikes:
            parts.append("<table><tr><th class='num'>t (s)</th><th class='num'>max velCov</th></tr>")
            for t, m in spikes[:30]:
                parts.append(f"<tr><td class='num'>{t:.2f}</td><td class='num'>{m:.3f}</td></tr>")
            parts.append("</table>")

    # agcCnt transitions
    transitions = []
    prev = None
    for r in d["mon_hw"]:
        if prev is not None and r[2] != prev:
            transitions.append((r[0], prev, r[2]))
        prev = r[2]
    parts.append(f"<p><b>MON-HW agcCnt transitions</b>: {len(transitions)}</p>")
    if transitions:
        parts.append("<table><tr><th class='num'>t (s)</th><th class='num'>before</th>"
                     "<th class='num'>after</th></tr>")
        for t, a, b in transitions:
            parts.append(f"<tr><td class='num'>{t:.2f}</td><td class='num'>{a}</td>"
                         f"<td class='num'>{b}</td></tr>")
        parts.append("</table>")

    return "\n".join(parts)


def mon_span_summary_html(d):
    """One-liner about RF stability across the capture."""
    if not d["mon_span"]:
        return "<p>No MON-SPAN data.</p>"
    centers_set = set()
    pgas_seen = set()
    for t, centers, spans, pgas in d["mon_span"]:
        centers_set.add(tuple(centers))
        pgas_seen.add(tuple(pgas))
    parts = [f"<p><b>MON-SPAN samples</b>: {len(d['mon_span'])}</p>"]
    parts.append(f"<p>Distinct center-frequency tuples observed: {len(centers_set)}</p>")
    parts.append(f"<p>Distinct PGA tuples observed: {len(pgas_seen)}</p>")
    if centers_set:
        c = list(centers_set)[0]
        parts.append(f"<p>Center frequencies: {[f'{x/1e6:.2f} MHz' for x in c]}</p>")
    if pgas_seen:
        parts.append(f"<p>PGA values: {sorted(pgas_seen)}</p>")
    return "\n".join(parts)


# ============================================================
# Cross-capture summary plots
# ============================================================

def plot_cross_signal_around_event(captures_data, extractor, ylabel, title, ylim=None, log=False):
    """Grid showing extracted series around each event, sized to fit all captures."""
    n = len(captures_data)
    cols = 5
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 3.0 * rows), sharey=True)
    axes = axes.flatten() if rows * cols > 1 else [axes]
    for ax, (label, d) in zip(axes, captures_data):
        if d is None or d["event_t"] is None:
            ax.set_title(f"{label}: no event")
            ax.set_xticks([])
            continue
        et = d["event_t"]
        ts, ys = extractor(d, et)
        if log:
            ax.semilogy(ts, ys, lw=0.8, marker="o", ms=2)
        else:
            ax.plot(ts, ys, lw=0.8, color="#1f4e8f")
        ax.axvline(0, color="red", lw=0.6, ls="--", alpha=0.5)
        ax.set_title(f"{label}", fontsize=10)
        ax.set_xlabel("rel to event (s)", fontsize=8)
        if ylim: ax.set_ylim(*ylim)
        ax.tick_params(labelsize=8)
        ax.grid(alpha=0.3, which="both" if log else "major")
    # Hide unused axes
    for ax in axes[len(captures_data):]:
        ax.axis("off")
    # Y-label on left column only
    for r in range(rows):
        if r * cols < len(captures_data):
            axes[r * cols].set_ylabel(ylabel, fontsize=9)
    fig.suptitle(title, fontsize=12)
    fig.tight_layout()
    return fig_to_b64(fig)


# ============================================================
# Main
# ============================================================

def main():
    print("Parsing captures...")
    captures_data = []
    for label, fname, _, _ in CAPTURES:
        p = os.path.join(DESKTOP, fname)
        if not os.path.exists(p):
            captures_data.append((label, None))
            continue
        print(f"  {label}: {fname}")
        captures_data.append((label, parse_capture(p)))

    print("Generating cross-capture comparison plots...")
    cross_sats = plot_cross_signal_around_event(
        captures_data,
        lambda d, et: ([r[0] - et for r in d["nav_pvt"] if et - 10 <= r[0] <= et + 10],
                       [r[2] for r in d["nav_pvt"] if et - 10 <= r[0] <= et + 10]),
        "numSV", "Cross-capture: numSV around event", ylim=(0, 35))
    cross_gspd = plot_cross_signal_around_event(
        captures_data,
        lambda d, et: ([r[0] - et for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 5],
                       [r[8] for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 5]),
        "gSpeed (mm/s)", "Cross-capture: gSpeed around event")
    cross_hacc = plot_cross_signal_around_event(
        captures_data,
        lambda d, et: ([r[0] - et for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 30],
                       [r[3] for r in d["nav_pvt"] if et - 5 <= r[0] <= et + 30]),
        "hAcc (m)", "Cross-capture: hAcc around event (5s before, 30s after)")

    print("Generating per-capture plots...")
    per_capture_plots = {}
    for label, d in captures_data:
        if d is None: continue
        print(f"  {label}: nav_pvt, dop, clock, cov, sat, mon_hw, zoom")
        per_capture_plots[label] = {
            "nav_pvt": plot_nav_pvt(d, label),
            "nav_dop": plot_nav_dop(d, label),
            "nav_clock": plot_nav_clock(d, label),
            "nav_cov": plot_nav_cov(d, label),
            "nav_sat": plot_nav_sat(d, label),
            "mon_hw": plot_mon_hw(d, label),
            "zoom": plot_event_zoom(d, label),
        }

    print("Writing HTML...")
    html = []
    html.append("""<!doctype html>
<html><head><meta charset="utf-8">
<title>Here4 GPS Bug — Full Data Dump (2026-05-13)</title>
<style>
body { font-family: -apple-system, "Segoe UI", Arial, sans-serif; max-width: 1280px;
       margin: 24px auto; padding: 0 18px; color: #222; line-height: 1.5; }
h1 { border-bottom: 3px solid #1f4e8f; padding-bottom: 4px; color: #1f4e8f; }
h2 { color: #1f4e8f; margin-top: 36px; border-bottom: 1px solid #ccc;
     padding-bottom: 2px; }
h3 { color: #444; margin-top: 28px; }
h4 { color: #666; margin-top: 20px; margin-bottom: 6px; }
table { border-collapse: collapse; margin: 8px 0; }
th, td { border: 1px solid #c0c0c0; padding: 5px 9px; text-align: left;
         vertical-align: top; font-size: 13px; }
th { background: #eaeef5; }
td.num, th.num { text-align: right; font-family: Consolas, monospace; }
img { max-width: 100%; height: auto; border: 1px solid #ccc; margin: 6px 0; }
.toc { background: #f4f7fc; border: 1px solid #c8d4e8; padding: 14px 22px;
       border-radius: 4px; margin: 18px 0; }
.toc ul { margin: 4px 0; padding-left: 20px; }
.toc a { text-decoration: none; color: #1f4e8f; }
.toc a:hover { text-decoration: underline; }
.capture-box { border-left: 4px solid #1f4e8f; padding: 6px 16px;
               background: #f6f9fd; margin: 16px 0; }
.tag-hard { background: #c83030; color: white; padding: 2px 6px;
            border-radius: 3px; font-size: 11px; }
.tag-soft { background: #d88800; color: white; padding: 2px 6px;
            border-radius: 3px; font-size: 11px; }
.tag-clean { background: #2a8f2a; color: white; padding: 2px 6px;
             border-radius: 3px; font-size: 11px; }
code { background: #f4f4f4; padding: 1px 4px; border-radius: 3px;
       font-size: 12.5px; }
.caption { color: #666; font-size: 12.5px; margin: 4px 0 18px 0; }
.signal-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; }
.section-anchor { display: block; position: relative; top: -20px; visibility: hidden; }
</style></head><body>""")

    html.append("<h1>Here4 GPS Bug — Full Data Dump (10 captures, 2026-05-13 + 2026-05-14)</h1>")
    html.append("""
<p>Comprehensive data dump of all ten captures across two test days. Every available
UBX signal is plotted across the entire capture timeline, with stats tables, anomaly
listings, and event-zoom views. No analysis spin — just data.</p>

<div class="capture-box" style="background:#fff5e8;border-left-color:#d88800">
<h3 style="margin-top:0;color:#a06800">Day 2 headline finding</h3>
<p>D1 → D2 → D3 → D4 was a controlled treatment sequence on a fresh old/unused unit:</p>
<ul>
<li><b>D1</b>: fresh power-on, bug fired naturally at ~7:36 (canonical timing).
agcCnt 3510 → 3861 → 3510 (transient revert).</li>
<li><b>D2</b>: after cold-boot ONLY via UBX-CFG-RST. Bug fired AGAIN at ~7:46.
Critically, agcCnt was already at 3861 BEFORE the bug (not 3510), and went to
a brand-new value 4212 during the event. <b>Cold-boot alone does not restore
the receiver to baseline.</b></li>
<li><b>D3 + D4</b>: after configuration RESET via u-center plus cold-boot.
Combined ~18 minutes clean operation, well past the canonical 8-min trigger.</li>
</ul>
<p><b>The fix is config-reset + cold-boot, not cold-boot alone.</b> This reframes
all the earlier discussion of Unit A's cold-start (which must have included
the config reset).</p>
</div>""")

    # ---------- Table of contents ----------
    html.append('<div class="toc"><h4>Contents</h4><ul>')
    html.append('<li><a href="#crosscap">Cross-capture comparison</a></li>')
    html.append('<li><a href="#captures">Per-capture deep dive</a><ul>')
    for label, fname, title, _ in CAPTURES:
        html.append(f'<li><a href="#{label}">{label}: {title}</a></li>')
    html.append("</ul></li>")
    html.append('<li><a href="#reference">UBX message reference</a></li>')
    html.append('</ul></div>')

    # ---------- Captures overview table ----------
    html.append("<h2>Captures overview</h2>")
    html.append("<table>")
    html.append("<tr><th>Tag</th><th>File</th><th class='num'>Size</th>"
                "<th class='num'>Duration</th><th class='num'>Event @</th>"
                "<th>Test conditions</th></tr>")
    for (label, fname, title, desc), (_, d) in zip(CAPTURES, captures_data):
        if d is None:
            html.append(f"<tr><td>{label}</td><td><code>{fname}</code></td>"
                        f"<td colspan='4'>missing</td></tr>")
            continue
        et = d["event_t"]
        et_str = f"+{et:.1f}s" if et is not None else "—"
        html.append(f"<tr><td><a href='#{label}'><b>{label}</b></a></td>"
                    f"<td><code>{fname}</code></td>"
                    f"<td class='num'>{d['size']:,}</td>"
                    f"<td class='num'>{d['duration']:.0f}s</td>"
                    f"<td class='num'>{et_str}</td>"
                    f"<td>{desc}</td></tr>")
    html.append("</table>")

    # ---------- Cross-capture comparison ----------
    html.append("<span class='section-anchor' id='crosscap'></span>")
    html.append("<h2>Cross-capture comparison</h2>")
    html.append("<h3>numSV around event (-10s to +10s)</h3>")
    html.append(f'<img src="data:image/png;base64,{cross_sats}">')
    html.append("<h3>gSpeed around event (-5s to +5s) — phantom velocity comparison</h3>")
    html.append(f'<img src="data:image/png;base64,{cross_gspd}">')
    html.append("<h3>hAcc around event (-5s to +30s) — recovery tail</h3>")
    html.append(f'<img src="data:image/png;base64,{cross_hacc}">')

    # ---------- Per-capture sections ----------
    html.append("<span class='section-anchor' id='captures'></span>")
    html.append("<h2>Per-capture deep dive</h2>")

    for (label, fname, title, desc), (_, d) in zip(CAPTURES, captures_data):
        html.append(f"<span class='section-anchor' id='{label}'></span>")
        html.append(f"<h3>{label}: {title}</h3>")
        html.append(f"<div class='capture-box'><p><b>File:</b> "
                    f"<code>{fname}</code></p>"
                    f"<p><b>Test conditions:</b> {desc}</p></div>")

        if d is None:
            html.append("<p><i>File not found.</i></p>")
            continue

        # Metadata
        et = d["event_t"]
        et_str = f"+{et:.2f}s into capture" if et is not None else "no event detected"
        html.append("<h4>Metadata</h4>")
        html.append("<table>")
        html.append(f"<tr><th>File size</th><td class='num'>{d['size']:,} bytes</td></tr>")
        html.append(f"<tr><th>Duration</th><td class='num'>{d['duration']:.1f} s</td></tr>")
        html.append(f"<tr><th>Event</th><td>{et_str}</td></tr>")
        html.append("</table>")

        # Inventory
        html.append("<h4>Message inventory</h4>")
        html.append(inventory_table_html(d))

        # Stats
        html.append("<h4>Statistics for every numeric signal</h4>")
        html.append(stats_table_html(d))

        # Anomalies
        html.append("<h4>Anomaly listings</h4>")
        html.append(anomaly_list_html(d))

        # MON-SPAN summary
        html.append("<h4>MON-SPAN RF stability</h4>")
        html.append(mon_span_summary_html(d))

        # Plots
        plots = per_capture_plots.get(label, {})
        if plots.get("zoom"):
            html.append("<h4>Event zoom (-5s to +5s)</h4>")
            html.append(f'<img src="data:image/png;base64,{plots["zoom"]}">')
        if plots.get("nav_pvt"):
            html.append("<h4>NAV-PVT signals (full capture)</h4>")
            html.append(f'<img src="data:image/png;base64,{plots["nav_pvt"]}">')
        if plots.get("nav_dop"):
            html.append("<h4>NAV-DOP signals (full capture)</h4>")
            html.append(f'<img src="data:image/png;base64,{plots["nav_dop"]}">')
        if plots.get("nav_clock"):
            html.append("<h4>NAV-CLOCK signals (full capture)</h4>")
            html.append(f'<img src="data:image/png;base64,{plots["nav_clock"]}">')
        if plots.get("nav_cov"):
            html.append("<h4>NAV-COV signals (full capture)</h4>")
            html.append(f'<img src="data:image/png;base64,{plots["nav_cov"]}">')
        if plots.get("nav_sat"):
            html.append("<h4>NAV-SAT signals (full capture)</h4>")
            html.append(f'<img src="data:image/png;base64,{plots["nav_sat"]}">')
        if plots.get("mon_hw"):
            html.append("<h4>MON-HW signals (full capture)</h4>")
            html.append(f'<img src="data:image/png;base64,{plots["mon_hw"]}">')

    # ---------- Reference ----------
    html.append("<span class='section-anchor' id='reference'></span>")
    html.append("<h2>UBX message reference</h2>")
    html.append("""
<p>All decoding is per the
<b>u-blox F9 HPG L1L5 1.40 Interface Description (UBX-23006991)</b>
PDF on the desktop. Field offsets verified hand-by-hand against PDF pages 128
(MON-HW), 165 (NAV-CLOCK), 166 (NAV-COV), and the Fletcher checksum
specification on page 57.</p>

<h3>UBX frame format</h3>
<pre>+------+------+------+------+----------+------------+--------+
| 0xB5 | 0x62 | cls  | id   | length   | payload    | ck_a   | ck_b
| sync | sync |      |      | 2B LE u2 | length-byt |        |
+------+------+------+------+----------+------------+--------+

Fletcher-16 checksum (spec section 3.4):
  CK_A = 0, CK_B = 0
  For each byte from cls through end-of-payload:
      CK_A = (CK_A + byte) & 0xFF
      CK_B = (CK_B + CK_A) & 0xFF</pre>

<h3>NAV-PVT field offsets (per spec)</h3>
<table>
<tr><th>Offset</th><th>Type</th><th>Name</th><th>Unit</th><th>Notes</th></tr>
<tr><td class='num'>0</td><td>U4</td><td>iTOW</td><td>ms</td><td>GPS time of week</td></tr>
<tr><td class='num'>4</td><td>U2</td><td>year</td><td>—</td><td>UTC year</td></tr>
<tr><td class='num'>6-10</td><td>U1 each</td><td>month, day, hour, min, sec</td><td>—</td><td>UTC</td></tr>
<tr><td class='num'>20</td><td>U1</td><td>fixType</td><td>—</td><td>0=no, 2=2D, 3=3D, 4=GNSS+DR, 5=time-only</td></tr>
<tr><td class='num'>21</td><td>X1</td><td>flags</td><td>—</td></tr>
<tr><td class='num'>23</td><td>U1</td><td>numSV</td><td>—</td><td>sats used</td></tr>
<tr><td class='num'>24-32</td><td>I4 each</td><td>lon, lat</td><td>1e-7 deg</td></tr>
<tr><td class='num'>32-36</td><td>I4</td><td>height</td><td>mm</td></tr>
<tr><td class='num'>36-40</td><td>I4</td><td>hMSL</td><td>mm</td></tr>
<tr><td class='num'>40-44</td><td>U4</td><td>hAcc</td><td>mm</td><td>horiz accuracy</td></tr>
<tr><td class='num'>44-48</td><td>U4</td><td>vAcc</td><td>mm</td><td>vert accuracy</td></tr>
<tr><td class='num'>48-60</td><td>I4 each</td><td>velN, velE, velD</td><td>mm/s</td></tr>
<tr><td class='num'>60-64</td><td>I4</td><td>gSpeed</td><td>mm/s</td><td>2D ground speed</td></tr>
<tr><td class='num'>64-68</td><td>I4</td><td>headMot</td><td>1e-5 deg</td></tr>
<tr><td class='num'>68-72</td><td>U4</td><td>sAcc</td><td>mm/s</td><td>speed accuracy</td></tr>
<tr><td class='num'>72-76</td><td>U4</td><td>headAcc</td><td>1e-5 deg</td></tr>
<tr><td class='num'>76-78</td><td>U2</td><td>pDOP × 100</td><td>—</td></tr>
</table>

<h3>MON-HW field offsets (per spec page 128)</h3>
<table>
<tr><th>Offset</th><th>Type</th><th>Name</th><th>Notes</th></tr>
<tr><td class='num'>16-18</td><td>U2</td><td>noisePerMS</td><td>broadband noise floor</td></tr>
<tr><td class='num'>18-20</td><td>U2</td><td>agcCnt</td><td>0..8191 (100%)</td></tr>
<tr><td class='num'>20</td><td>U1</td><td>aStatus</td><td>0=INIT 1=DONTKNOW 2=OK 3=SHORT 4=OPEN</td></tr>
<tr><td class='num'>21</td><td>U1</td><td>aPower</td><td>0=off 1=on 2=DONTKNOW</td></tr>
<tr><td class='num'>22</td><td>X1</td><td>flags</td><td>bit 0=rtcCalib, bit 1=safeBoot</td></tr>
<tr><td class='num'>45</td><td>U1</td><td>jamInd</td><td>CW jam indicator 0-255</td></tr>
</table>

<h3>NAV-CLOCK field offsets (per spec page 165, same format as NAV2-CLOCK)</h3>
<table>
<tr><th>Offset</th><th>Type</th><th>Name</th><th>Unit</th></tr>
<tr><td class='num'>4-8</td><td>I4</td><td>clkB</td><td>ns (clock bias)</td></tr>
<tr><td class='num'>8-12</td><td>I4</td><td>clkD</td><td>ns/s (clock drift)</td></tr>
<tr><td class='num'>12-16</td><td>U4</td><td>tAcc</td><td>ns (time accuracy)</td></tr>
<tr><td class='num'>16-20</td><td>U4</td><td>fAcc</td><td>ps/s (frequency accuracy)</td></tr>
</table>

<h3>NAV-COV field offsets (per spec page 166)</h3>
<table>
<tr><th>Offset</th><th>Type</th><th>Name</th><th>Unit</th></tr>
<tr><td class='num'>5</td><td>U1</td><td>posCovValid</td><td>flag</td></tr>
<tr><td class='num'>6</td><td>U1</td><td>velCovValid</td><td>flag</td></tr>
<tr><td class='num'>16-20</td><td>R4</td><td>posCovNN</td><td>m²</td></tr>
<tr><td class='num'>28-32</td><td>R4</td><td>posCovEE</td><td>m²</td></tr>
<tr><td class='num'>36-40</td><td>R4</td><td>posCovDD</td><td>m²</td></tr>
<tr><td class='num'>40-44</td><td>R4</td><td>velCovNN</td><td>m²/s²</td></tr>
<tr><td class='num'>52-56</td><td>R4</td><td>velCovEE</td><td>m²/s²</td></tr>
<tr><td class='num'>60-64</td><td>R4</td><td>velCovDD</td><td>m²/s²</td></tr>
</table>

<h3>NAV-SAT per-satellite record (12 bytes after 8-byte header)</h3>
<table>
<tr><th>Offset</th><th>Type</th><th>Name</th><th>Notes</th></tr>
<tr><td class='num'>0</td><td>U1</td><td>gnssId</td><td>0=GPS,1=SBAS,2=GAL,3=BDS,5=QZSS,6=GLO,7=NavIC</td></tr>
<tr><td class='num'>1</td><td>U1</td><td>svId</td><td>satellite PRN</td></tr>
<tr><td class='num'>2</td><td>U1</td><td>cno</td><td>carrier-to-noise dB-Hz</td></tr>
<tr><td class='num'>8-12</td><td>X4</td><td>flags</td><td>bit 0-2 quality, bit 3 svUsed, bit 4-5 health</td></tr>
</table>
""")

    html.append("</body></html>")

    out_path = os.path.join(DESKTOP, "Here4_full_data_dump_all.html")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(html))
    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"\nWrote {out_path}  ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
