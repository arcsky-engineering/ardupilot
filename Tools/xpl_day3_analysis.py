#!/usr/bin/env python3
"""Analyze 2026-05-20 day-3 captures.

Critical experimental sequence (per user):
  E1  153514  Fresh new-old-stock unit, untouched, natural fire ~8:20.
  E2  154525  GPS_AUTO_CONFIG=0 on periph, then NAV-STATUS enabled via u-center.
              Bug fired right after.
  E3  155411  Same as E2 — repeat. Bug fired again right after enable.

GPS_AUTO_CONFIG=0 disables the GNSSPeriph driver's perpetual CFG-MSG cycle.
So if the bug still fires when only u-center's NAV-STATUS enable is sent,
that's a much cleaner pointer at the trigger mechanism.

This analysis focuses on:
  - Confirming canonical signature in E1 (natural fire baseline)
  - Inventory of CFG-MSG / ACK traffic in E2 and E3 — is it really suppressed?
  - When NAV-STATUS first appears in E2 and E3 (=when u-center enabled it)
  - When the bug fired relative to that first NAV-STATUS appearance
"""
import base64
import io
import os
import struct
from collections import Counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DESKTOP = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop"

CAPTURES = [
    ("E1", "ublox_nodeany_20260520-153514.ubx",
     "Fresh new-old-stock unit, untouched (natural fire baseline)",
     "Never altered, just powered on. Bug expected at ~8 min mark."),
    ("E2", "ublox_node121_20260520-154525.ubx",
     "GPS_AUTO_CONFIG=0, then NAV-STATUS enabled via u-center (attempt 1)",
     "Periph driver's CFG cycle disabled via GPS_AUTO_CONFIG=0. User then "
     "enabled NAV-STATUS through u-center. Bug fired right after."),
    ("E3", "ublox_node121_20260520-155411.ubx",
     "Same as E2 — repeat (attempt 2)",
     "Same procedure as E2. Bug fired again right after the NAV-STATUS enable."),
]

NAMES = {(0x01,0x07):"NAV-PVT",(0x01,0x04):"NAV-DOP",(0x01,0x20):"NAV-TIMEGPS",
         (0x01,0x22):"NAV-CLOCK",(0x01,0x35):"NAV-SAT",(0x01,0x43):"NAV-SIG",
         (0x01,0x36):"NAV-COV",(0x01,0x03):"NAV-STATUS",(0x01,0x61):"NAV-EOE",
         (0x05,0x00):"ACK-ACK",(0x05,0x01):"ACK-NAK",
         (0x06,0x00):"CFG-PRT",(0x06,0x01):"CFG-MSG",(0x06,0x04):"CFG-RST",
         (0x06,0x06):"CFG-DAT",(0x06,0x08):"CFG-RATE",(0x06,0x09):"CFG-CFG",
         (0x06,0x13):"CFG-ANT",(0x06,0x24):"CFG-NAV5",(0x06,0x31):"CFG-TP5",
         (0x06,0x3e):"CFG-GNSS",(0x06,0x8a):"CFG-VALSET",(0x06,0x8b):"CFG-VALGET",
         (0x0a,0x04):"MON-VER",(0x0a,0x09):"MON-HW",(0x0a,0x0b):"MON-IO",
         (0x0a,0x31):"MON-SPAN",(0x0d,0x03):"TIM-TM2"}


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
    pvt = []; cov = []; clock = []; sat = []; hw = []; navstatus = []
    ack_events = []  # (t, "ACK"|"NAK", target_cls, target_id)
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
        elif cls == 0x01 and mid == 0x22 and len(pl) >= 20:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            clkB = struct.unpack("<i", pl[4:8])[0]
            clkD = struct.unpack("<i", pl[8:12])[0]
            tAcc = struct.unpack("<I", pl[12:16])[0]
            fAcc = struct.unpack("<I", pl[16:20])[0]
            clock.append((t, clkB, clkD, tAcc, fAcc))
        elif cls == 0x01 and mid == 0x36 and len(pl) >= 64:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            vNN = struct.unpack("<f", pl[40:44])[0]
            vEE = struct.unpack("<f", pl[52:56])[0]
            vDD = struct.unpack("<f", pl[60:64])[0]
            cov.append((t, vNN, vEE, vDD))
        elif cls == 0x01 and mid == 0x35 and len(pl) >= 8:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            n = pl[5]
            used = tr = se = 0
            by_g = Counter()
            for i in range(n):
                o = 8 + i * 12
                if o + 12 > len(pl): break
                gnss = pl[o]
                flags = struct.unpack("<I", pl[o+8:o+12])[0]
                q = flags & 0x7
                u = (flags >> 3) & 0x1
                if q >= 4: tr += 1
                elif q in (1,2): se += 1
                if u:
                    used += 1
                    by_g[gnss] += 1
            sat.append((t, n, used, tr, se, dict(by_g)))
        elif cls == 0x01 and mid == 0x03 and len(pl) >= 16:
            # NAV-STATUS
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000 - t_start
            gpsFix = pl[4]
            flags = pl[5]
            fixStat = pl[6]
            flags2 = pl[7]
            ttff = struct.unpack("<I", pl[8:12])[0]
            msss = struct.unpack("<I", pl[12:16])[0]
            spoof_state = (flags2 >> 3) & 0x3
            navstatus.append((t, gpsFix, flags, fixStat, flags2, ttff, msss, spoof_state))
        elif cls == 0x0a and mid == 0x09 and len(pl) >= 60:
            t = off_to_t(off) - t_start
            n = struct.unpack("<H", pl[16:18])[0]
            agc = struct.unpack("<H", pl[18:20])[0]
            aS = pl[20]; aP = pl[21]; jam = pl[45]
            hw.append((t, n, agc, aS, aP, jam))
        elif cls == 0x05 and mid in (0x00, 0x01) and len(pl) >= 2:
            t = off_to_t(off) - t_start
            ack_events.append((t, "ACK" if mid == 0x00 else "NAK", pl[0], pl[1]))

    # Find event
    et = None
    window = []
    for entry in pvt:
        t = entry[0]; sv = entry[2]
        window = [(tt,n) for tt,n in window if t-tt <= 3.0]
        window.append((t, sv))
        rmax = max(n for _,n in window)
        if rmax >= 10 and sv + 10 <= rmax and et is None:
            et = t
    # Soft-fire fallback with healthy-fix guard
    soft = False
    if et is None:
        healthy_at = {}
        hw_window = []
        for i, entry in enumerate(pvt):
            t = entry[0]; hAcc = entry[4]; fix = entry[1]
            hw_window = [(tt, ha, fx) for tt, ha, fx in hw_window if t - tt <= 2.0]
            hw_window.append((t, hAcc, fix))
            healthy_at[i] = (len(hw_window) >= 3 and
                            all(ha < 5.0 and fx >= 3 for _, ha, fx in hw_window))
        prev = None
        biggest = (0, None)
        for i, entry in enumerate(pvt):
            if prev is not None:
                d = abs(entry[3] - prev)
                if d > biggest[0] and healthy_at.get(i, False):
                    biggest = (d, entry[0])
            prev = entry[3]
        if biggest[0] >= 200:
            et = biggest[1]
            soft = True

    agc_changes = []
    prev = None
    for r in hw:
        if prev is not None and r[2] != prev:
            agc_changes.append((r[0], prev, r[2]))
        prev = r[2]

    max_velcov = 0; max_velcov_t = None
    for r in cov:
        m = max(r[1], r[2], r[3])
        if m > max_velcov:
            max_velcov = m
            max_velcov_t = r[0]

    return dict(path=path, size=len(data), duration=t_end-t_start, inv=dict(inv),
                pvt=pvt, cov=cov, clock=clock, sat=sat, hw=hw, navstatus=navstatus,
                ack_events=ack_events, event_t=et, soft_fire=soft,
                agc_changes=agc_changes, max_velcov=max_velcov,
                max_velcov_t=max_velcov_t)


def b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=85, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


print("Parsing...")
data_by_tag = {}
for label, fname, _, _ in CAPTURES:
    p = os.path.join(DESKTOP, fname)
    print(f"  {label}: {fname}")
    data_by_tag[label] = parse(p)

# Quick text summary
print("\n=== SUMMARY ===")
for label, fname, title, desc in CAPTURES:
    d = data_by_tag[label]
    if d is None:
        print(f"{label}: missing")
        continue
    et_str = f"+{d['event_t']:.1f}s" if d['event_t'] is not None else "—"
    n_cfg_msg = d['inv'].get('CFG-MSG', 0)
    n_navstatus = d['inv'].get('NAV-STATUS', 0)
    n_ack = sum(1 for _,t,_,_ in d['ack_events'] if t == 'ACK')
    n_nak = sum(1 for _,t,_,_ in d['ack_events'] if t == 'NAK')
    first_navstatus_t = d['navstatus'][0][0] if d['navstatus'] else None
    print(f"\n{label}: {title}")
    print(f"  Duration: {d['duration']:.0f}s   Event: {et_str}")
    print(f"  CFG-MSG records: {n_cfg_msg}   ACK: {n_ack}   NAK: {n_nak}")
    print(f"  NAV-STATUS packets: {n_navstatus}")
    if first_navstatus_t is not None:
        print(f"  First NAV-STATUS @ +{first_navstatus_t:.1f}s")
        if d['event_t'] is not None:
            gap = d['event_t'] - first_navstatus_t
            print(f"  Event came {gap:+.2f}s after first NAV-STATUS")
    if d['agc_changes']:
        seq = " -> ".join([str(d['agc_changes'][0][1])] + [str(c[2]) for c in d['agc_changes']])
        print(f"  agcCnt transitions: {seq}")


# Event timing analysis
print("\n=== ACK/NAK timing in each capture ===")
for label, _, _, _ in CAPTURES:
    d = data_by_tag[label]
    if d is None: continue
    print(f"\n{label}: first 20 ACK/NAK events:")
    for t, kind, cls_t, id_t in d['ack_events'][:20]:
        target = NAMES.get((cls_t, id_t), f"UNK_{cls_t:02x}{id_t:02x}")
        print(f"  +{t:>7.2f}s  {kind} for {target}")


# Build HTML report
print("\nBuilding HTML report...")


def plot_overlay_sat_gspd():
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=False)
    colors = ["#1f4e8f", "#a02050", "#206030"]
    for (label, _, _, _), color in zip(CAPTURES, colors):
        d = data_by_tag[label]
        if d is None: continue
        ts = [r[0] for r in d['pvt']]
        svs = [r[2] for r in d['pvt']]
        gss = [r[3] for r in d['pvt']]
        axes[0].plot(ts, svs, lw=0.4, color=color, label=label, alpha=0.85)
        axes[1].plot(ts, gss, lw=0.4, color=color, label=label, alpha=0.85)
        if d['event_t'] is not None:
            axes[0].axvline(d['event_t'], color=color, lw=0.6, ls="--", alpha=0.4)
            axes[1].axvline(d['event_t'], color=color, lw=0.6, ls="--", alpha=0.4)
        if d['navstatus']:
            first_t = d['navstatus'][0][0]
            axes[0].axvline(first_t, color=color, lw=0.6, ls=":", alpha=0.6)
            axes[1].axvline(first_t, color=color, lw=0.6, ls=":", alpha=0.6)
    axes[0].set_ylabel("numSV")
    axes[0].set_ylim(0, 35)
    axes[0].set_xlabel("capture time (s)")
    axes[0].legend(loc="lower right", ncol=3, fontsize=10)
    axes[0].grid(alpha=0.3)
    axes[1].set_ylabel("gSpeed (mm/s)")
    axes[1].set_yscale("symlog", linthresh=100)
    axes[1].set_xlabel("capture time (s)")
    axes[1].legend(loc="upper right", ncol=3, fontsize=10)
    axes[1].grid(alpha=0.3)
    fig.suptitle("Day-3 captures overlaid. Dashed = event time. Dotted = first NAV-STATUS arrival.",
                 fontsize=12)
    fig.tight_layout()
    return b64(fig)


def plot_event_zooms():
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, (label, _, _, _) in zip(axes, CAPTURES):
        d = data_by_tag[label]
        if d is None or d['event_t'] is None:
            ax.set_title(f"{label}: no event")
            continue
        et = d['event_t']
        ts = [r[0] - et for r in d['pvt'] if et-3 <= r[0] <= et+3]
        svs = [r[2] for r in d['pvt'] if et-3 <= r[0] <= et+3]
        gss = [r[3] for r in d['pvt'] if et-3 <= r[0] <= et+3]
        ax.plot(ts, svs, marker="o", ms=3, lw=0.6, color="#1f4e8f", label="numSV")
        ax.set_ylim(0, 35)
        ax.set_ylabel("numSV", color="#1f4e8f")
        axb = ax.twinx()
        axb.plot(ts, gss, marker="x", ms=3, lw=0.5, color="#a02050", label="gSpd")
        axb.set_ylabel("gSpd (mm/s)", color="#a02050")
        ax.axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
        ax.set_xlabel("seconds rel. event")
        ax.set_title(f"{label}  event +{et:.0f}s")
        ax.grid(alpha=0.3)
    fig.suptitle("Event zooms (-3s to +3s)", fontsize=12)
    fig.tight_layout()
    return b64(fig)


def plot_agc():
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)
    for ax, (label, _, _, _) in zip(axes, CAPTURES):
        d = data_by_tag[label]
        if d is None or not d['hw']:
            ax.set_title(f"{label}: no MON-HW")
            continue
        ts = [r[0] for r in d['hw']]
        ag = [r[2] for r in d['hw']]
        ax.plot(ts, ag, marker="o", ms=3, lw=0.7, color="#106b3e")
        if d['event_t'] is not None:
            ax.axvline(d['event_t'], color="red", lw=0.8, ls="--", alpha=0.5)
        if d['navstatus']:
            ax.axvline(d['navstatus'][0][0], color="purple", lw=0.6, ls=":", alpha=0.6)
        ax.set_title(f"{label}")
        ax.set_xlabel("capture time (s)")
        ax.set_ylim(3400, 3950)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("MON-HW agcCnt")
    fig.suptitle("MON-HW agcCnt across the 3 captures (dotted = first NAV-STATUS)",
                 fontsize=12)
    fig.tight_layout()
    return b64(fig)


def plot_cfg_traffic():
    """Show CFG-MSG count rate per minute. Should be ~zero in E2/E3 if AUTO_CONFIG=0."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 4))
    for (label, _, _, _), color in zip(CAPTURES, ["#1f4e8f", "#a02050", "#206030"]):
        d = data_by_tag[label]
        if d is None: continue
        # bin ACK events per 30 sec
        ts = [t for t, kind, _, _ in d['ack_events']]
        if not ts: continue
        max_t = max(ts)
        bins = list(range(0, int(max_t) + 31, 30))
        counts = [sum(1 for t in ts if b <= t < b + 30) for b in bins[:-1]]
        ax.plot([(bins[i] + bins[i+1]) / 2 for i in range(len(counts))], counts,
                marker="o", lw=1.2, color=color, label=label, alpha=0.85)
    ax.set_xlabel("capture time (s)")
    ax.set_ylabel("ACK/NAK responses per 30s window")
    ax.set_title("Host CFG activity (ACK + NAK responses) per 30-second window")
    ax.legend(fontsize=10)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    return b64(fig)


pl_overlay = plot_overlay_sat_gspd()
pl_zoom = plot_event_zooms()
pl_agc = plot_agc()
pl_cfg = plot_cfg_traffic()

html = []
html.append("""<!doctype html>
<html><head><meta charset="utf-8">
<title>Here4 Day 3 — NAV-STATUS Trigger Test</title>
<style>
body { font-family: -apple-system, "Segoe UI", Arial, sans-serif; max-width: 1200px;
       margin: 24px auto; padding: 0 18px; color: #222; line-height: 1.55; }
h1 { border-bottom: 3px solid #1f4e8f; padding-bottom: 4px; color: #1f4e8f; }
h2 { color: #1f4e8f; margin-top: 32px; border-bottom: 1px solid #ccc; padding-bottom: 2px; }
h3 { color: #444; margin-top: 24px; }
table { border-collapse: collapse; margin: 10px 0; }
th, td { border: 1px solid #c0c0c0; padding: 5px 9px; text-align: left; font-size: 13px; vertical-align: top; }
th { background: #eaeef5; }
td.num, th.num { text-align: right; font-family: Consolas, monospace; }
img { max-width: 100%; height: auto; border: 1px solid #ccc; margin: 6px 0; }
.callout { background: #f0f4fc; border-left: 4px solid #1f4e8f; padding: 10px 14px; margin: 14px 0; }
.callout-warn { background: #fff5e8; border-left-color: #d88800; }
.callout-good { background: #ecf8ec; border-left-color: #2a8f2a; }
.tag-hard { background: #c83030; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; }
code { background: #f4f4f4; padding: 1px 4px; border-radius: 3px; font-size: 12.5px; font-family: Consolas, monospace; }
</style></head><body>""")

html.append("<h1>Here4 Day 3 — NAV-STATUS Trigger Test (2026-05-20)</h1>")
html.append("""
<p>Three captures on a fresh new-old-stock unit. The critical experimental design
on E2 and E3: <b>GPS_AUTO_CONFIG=0</b> on the periph disables the GNSSPeriph driver's
perpetual CFG-MSG cycle (per the firmware architecture review), so the only host
stimulus to the u-blox is u-center's NAV-STATUS enable command.</p>
""")

html.append("<h2>Sequence summary</h2>")
html.append("<table>")
html.append("<tr><th>Tag</th><th>Condition</th><th class='num'>Duration</th>"
            "<th class='num'>Event</th><th class='num'>NAV-STATUS first</th>"
            "<th class='num'>Gap</th><th>agcCnt transitions</th></tr>")
for label, fname, title, desc in CAPTURES:
    d = data_by_tag[label]
    if d is None: continue
    et_str = f"+{d['event_t']:.1f}s" if d['event_t'] is not None else "—"
    if d['navstatus']:
        ns_str = f"+{d['navstatus'][0][0]:.1f}s"
        if d['event_t'] is not None:
            gap = d['event_t'] - d['navstatus'][0][0]
            gap_str = f"{gap:+.2f}s"
        else:
            gap_str = "—"
    else:
        ns_str = "(not enabled)"
        gap_str = "—"
    agc = " → ".join([str(d['agc_changes'][0][1])] + [str(c[2]) for c in d['agc_changes']]) if d['agc_changes'] else "none"
    html.append(f"<tr><td><b>{label}</b></td><td>{title}<br><span style='color:#666;font-size:12px'>{desc}</span></td>"
                f"<td class='num'>{d['duration']:.0f}s</td>"
                f"<td class='num'>{et_str}</td>"
                f"<td class='num'>{ns_str}</td>"
                f"<td class='num'>{gap_str}</td>"
                f"<td>{agc}</td></tr>")
html.append("</table>")

html.append("<h2>Headline question — did NAV-STATUS enable trigger the bug?</h2>")

# Build the answer based on actual data
e2 = data_by_tag.get("E2")
e3 = data_by_tag.get("E3")

if e2 and e3:
    e2_gap = (e2['event_t'] - e2['navstatus'][0][0]) if (e2['event_t'] and e2['navstatus']) else None
    e3_gap = (e3['event_t'] - e3['navstatus'][0][0]) if (e3['event_t'] and e3['navstatus']) else None
    e2_ack_count = sum(1 for _,k,_,_ in e2['ack_events'] if k == 'ACK')
    e2_nak_count = sum(1 for _,k,_,_ in e2['ack_events'] if k == 'NAK')
    e3_ack_count = sum(1 for _,k,_,_ in e3['ack_events'] if k == 'ACK')
    e3_nak_count = sum(1 for _,k,_,_ in e3['ack_events'] if k == 'NAK')

    html.append('<div class="callout">')
    html.append("<p>Two independent repeats. In both:</p><ul>")
    if e2_gap is not None:
        html.append(f"<li>E2: event fired <b>{e2_gap:+.2f}s</b> relative to first NAV-STATUS arrival "
                    f"(ACKs in capture: {e2_ack_count}, NAKs: {e2_nak_count})</li>")
    if e3_gap is not None:
        html.append(f"<li>E3: event fired <b>{e3_gap:+.2f}s</b> relative to first NAV-STATUS arrival "
                    f"(ACKs in capture: {e3_ack_count}, NAKs: {e3_nak_count})</li>")
    html.append("</ul>")
    html.append("<p>If the ACK+NAK counts are very low (compare to ~190/min on prior captures), "
                "that confirms GPS_AUTO_CONFIG=0 worked — the periph really did stop polling. "
                "Then if both gaps are small and positive (event came shortly after NAV-STATUS "
                "enable), it's strong evidence the enable itself triggered the bug. If both gaps "
                "are large or negative, then NAV-STATUS isn't the trigger.</p>")
    html.append("</div>")

html.append("<h2>Plots</h2>")
html.append("<h3>All 3 captures overlaid (numSV + gSpeed, full timelines)</h3>")
html.append('<p style="color:#666;font-size:13px">Dashed vertical lines mark each capture\'s event. '
            'Dotted vertical lines mark first NAV-STATUS arrival.</p>')
html.append(f'<img src="data:image/png;base64,{pl_overlay}">')

html.append("<h3>Event zooms (±3 s)</h3>")
html.append(f'<img src="data:image/png;base64,{pl_zoom}">')

html.append("<h3>MON-HW agcCnt timelines</h3>")
html.append('<p style="color:#666;font-size:13px">Dashed red = event. Dotted purple = first NAV-STATUS arrival.</p>')
html.append(f'<img src="data:image/png;base64,{pl_agc}">')

html.append("<h3>Host CFG traffic (ACK + NAK responses per 30s)</h3>")
html.append('<p style="color:#666;font-size:13px">If GPS_AUTO_CONFIG=0 worked, '
            'E2 and E3 should show dramatically lower CFG response counts than E1 throughout '
            '(or zero, until u-center enables NAV-STATUS).</p>')
html.append(f'<img src="data:image/png;base64,{pl_cfg}">')

# Detailed per-capture data
html.append("<h2>Per-capture details</h2>")
for label, fname, title, desc in CAPTURES:
    d = data_by_tag[label]
    if d is None: continue
    html.append(f"<h3>{label}: {title}</h3>")
    html.append(f"<p><code>{fname}</code> &nbsp; size {d['size']:,} B &nbsp; duration {d['duration']:.0f}s</p>")
    html.append("<table>")
    html.append("<tr><th>Message</th><th class='num'>Count</th><th class='num'>Avg rate (Hz)</th></tr>")
    for n, c in sorted(d['inv'].items(), key=lambda x: -x[1])[:15]:
        rate = c / d['duration'] if d['duration'] > 0 else 0
        html.append(f"<tr><td>{n}</td><td class='num'>{c}</td><td class='num'>{rate:.3f}</td></tr>")
    html.append("</table>")

    # Event NAV-PVT detail if any
    if d['event_t'] is not None:
        et = d['event_t']
        html.append("<p><b>NAV-PVT around event:</b></p>")
        html.append("<table>")
        html.append("<tr><th class='num'>rel_t</th><th class='num'>sv</th><th class='num'>fix</th>"
                    "<th class='num'>gSpd</th><th class='num'>hAcc</th></tr>")
        for r in d['pvt']:
            t = r[0]; rel = t - et
            if -1 <= rel <= 1.5:
                marker = "←" if abs(rel) < 0.21 else ""
                html.append(f"<tr><td class='num'>{rel:+.2f}s</td>"
                            f"<td class='num'>{r[2]}</td><td class='num'>{r[1]}</td>"
                            f"<td class='num'>{r[3]}</td><td class='num'>{r[4]:.2f}</td>"
                            f"<td>{marker}</td></tr>")
        html.append("</table>")

    if d['ack_events']:
        html.append("<p><b>ACK/NAK events:</b></p>")
        html.append("<table>")
        html.append("<tr><th class='num'>t</th><th>kind</th><th>target</th></tr>")
        for t, kind, cls_t, id_t in d['ack_events'][:40]:
            target = NAMES.get((cls_t, id_t), f"UNK_{cls_t:02x}{id_t:02x}")
            html.append(f"<tr><td class='num'>+{t:.2f}s</td><td>{kind}</td><td>{target}</td></tr>")
        if len(d['ack_events']) > 40:
            html.append(f"<tr><td colspan='3'>... ({len(d['ack_events']) - 40} more)</td></tr>")
        html.append("</table>")
    else:
        html.append("<p><i>No ACK/NAK responses in capture (GPS_AUTO_CONFIG=0 worked completely?)</i></p>")

    if d['navstatus']:
        first = d['navstatus'][0]
        last = d['navstatus'][-1]
        html.append(f"<p><b>NAV-STATUS</b>: first @ +{first[0]:.2f}s, last @ +{last[0]:.2f}s, "
                    f"{len(d['navstatus'])} total messages</p>")
        # Show a few representative NAV-STATUS samples around the event
        if d['event_t'] is not None:
            et = d['event_t']
            html.append("<p>NAV-STATUS samples around event:</p>")
            html.append("<table>")
            html.append("<tr><th class='num'>rel_t</th><th class='num'>gpsFix</th>"
                        "<th class='num'>flags</th><th class='num'>fixStat</th>"
                        "<th class='num'>flags2</th><th class='num'>ttff(ms)</th>"
                        "<th class='num'>msss(ms)</th><th class='num'>spoof</th></tr>")
            for ns in d['navstatus']:
                rel = ns[0] - et
                if -2 <= rel <= 5:
                    marker = "←" if abs(rel) < 0.5 else ""
                    html.append(f"<tr><td class='num'>{rel:+.2f}s</td>"
                                f"<td class='num'>{ns[1]}</td>"
                                f"<td class='num'>0x{ns[2]:02x}</td>"
                                f"<td class='num'>0x{ns[3]:02x}</td>"
                                f"<td class='num'>0x{ns[4]:02x}</td>"
                                f"<td class='num'>{ns[5]}</td>"
                                f"<td class='num'>{ns[6]}</td>"
                                f"<td class='num'>{ns[7]}</td><td>{marker}</td></tr>")
            html.append("</table>")

html.append("</body></html>")

out = os.path.join(DESKTOP, "Here4_day3_analysis.html")
with open(out, "w", encoding="utf-8") as f:
    f.write("\n".join(html))
print(f"\nWrote {out}  ({os.path.getsize(out)/1024:.0f} KB)")
