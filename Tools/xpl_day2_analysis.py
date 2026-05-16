#!/usr/bin/env python3
"""Analyze 2026-05-14 day-2 captures from the old/unused Here4 unit.

Sequence (per user notes):
  D1  191122  fresh power-on, bug fired naturally at ~8:20
  D2  192155  after u-center cold-boot only, bug fired again
  D3  193327  after u-center config-reset + cold boot, no bug
  D4  194332  same condition as D3, no bug

The cold-boot-alone fail in D2 vs. clean D3/D4 after config-reset+cold-boot is
the most interesting finding — it tells us the *full* procedure matters, not
just the cold-start.
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
    ("D1", "ublox_node122_20260514-191122.ubx",
     "Fresh power-on (untouched, never cold-started)",
     "Old/unused hardware on carrier-board test stand. First capture after "
     "power-on. User expects natural bug fire ~8 min in."),
    ("D2", "ublox_node122_20260514-192155.ubx",
     "After u-center COLD BOOT only",
     "Cold-boot via u-center (UBX-CFG-RST). NO config reset. User reports bug "
     "fired again."),
    ("D3", "ublox_node122_20260514-193327.ubx",
     "After config-reset + cold boot",
     "u-center configuration reset to defaults, then cold boot. User reports "
     "no bug."),
    ("D4", "ublox_node122_20260514-194332.ubx",
     "Same as D3 (config-reset + cold boot still in effect)",
     "Continued session after D3. User reports no bug."),
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
    pvt = []
    cov = []
    clock = []
    sat = []
    hw = []
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
        name = {(0x01,0x07):"NAV-PVT",(0x01,0x04):"NAV-DOP",(0x01,0x22):"NAV-CLOCK",
                (0x01,0x35):"NAV-SAT",(0x01,0x43):"NAV-SIG",(0x01,0x36):"NAV-COV",
                (0x0a,0x09):"MON-HW",(0x05,0x00):"ACK-ACK",(0x05,0x01):"ACK-NAK",
                (0x06,0x01):"CFG-MSG"}.get((cls, mid))
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
            pNN = struct.unpack("<f", pl[16:20])[0]
            pEE = struct.unpack("<f", pl[28:32])[0]
            pDD = struct.unpack("<f", pl[36:40])[0]
            vNN = struct.unpack("<f", pl[40:44])[0]
            vEE = struct.unpack("<f", pl[52:56])[0]
            vDD = struct.unpack("<f", pl[60:64])[0]
            cov.append((t, pNN, pEE, pDD, vNN, vEE, vDD))
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
        elif cls == 0x0a and mid == 0x09 and len(pl) >= 60:
            t = off_to_t(off) - t_start
            n = struct.unpack("<H", pl[16:18])[0]
            agc = struct.unpack("<H", pl[18:20])[0]
            aS = pl[20]; aP = pl[21]; jam = pl[45]
            hw.append((t, n, agc, aS, aP, jam))

    # Find event (sat cliff)
    et = None
    window = []
    for entry in pvt:
        t = entry[0]; sv = entry[2]
        window = [(tt,n) for tt,n in window if t-tt <= 3.0]
        window.append((t, sv))
        rmax = max(n for _,n in window)
        if rmax >= 10 and sv + 10 <= rmax and et is None:
            et = t

    # Soft fire fallback: biggest gSpd jump
    soft = False
    if et is None:
        prev = None
        biggest = (0, None)
        for entry in pvt:
            t = entry[0]; g = entry[3]
            if prev is not None and abs(g - prev) > biggest[0]:
                biggest = (abs(g - prev), t)
            prev = g
        if biggest[0] >= 200:
            et = biggest[1]
            soft = True

    # agcCnt transitions
    agc_changes = []
    prev = None
    for r in hw:
        if prev is not None and r[2] != prev:
            agc_changes.append((r[0], prev, r[2]))
        prev = r[2]

    # Max vel-cov anywhere
    max_velcov = 0
    max_velcov_t = None
    for r in cov:
        m = max(r[4], r[5], r[6])
        if m > max_velcov:
            max_velcov = m
            max_velcov_t = r[0]

    # Max gSpd jump anywhere
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

    # Sat-cliff depth at event
    cliff_depth = 0
    if et is not None:
        sv_before = max((s for t,_,s,_,_,_,_ in pvt if et-3 <= t < et), default=0)
        sv_after = min((s for t,_,s,_,_,_,_ in pvt if et <= t <= et+1.5), default=sv_before)
        cliff_depth = sv_before - sv_after

    return dict(path=path, size=len(data), duration=t_end-t_start, inv=dict(inv),
                pvt=pvt, cov=cov, clock=clock, sat=sat, hw=hw,
                event_t=et, soft_fire=soft, agc_changes=agc_changes,
                max_velcov=max_velcov, max_velcov_t=max_velcov_t,
                max_jump=max_jump, max_jump_t=max_jump_t, cliff_depth=cliff_depth)


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

print("\n=== SEQUENCE SUMMARY ===")
print(f"{'tag':<5} {'duration':>9} {'event_t':>9} {'cliff':>6} {'gSpd_jump':>10} {'velcov_pk':>10} {'agcCnt_trans':>14}")
for label, fname, title, desc in CAPTURES:
    d = data_by_tag[label]
    if d is None:
        print(f"{label:<5} MISSING")
        continue
    et_str = f"+{d['event_t']:.1f}s" if d['event_t'] is not None else "—"
    cliff = d['cliff_depth'] if d['event_t'] is not None else 0
    agc = " -> ".join([str(d['agc_changes'][0][1])] + [str(c[2]) for c in d['agc_changes']]) if d['agc_changes'] else "—"
    print(f"{label:<5} {d['duration']:>8.0f}s {et_str:>9} {cliff:>6} {d['max_jump']:>10} {d['max_velcov']:>10.3f} {agc:>14}")

# Per-capture detailed event window (around event_t if exists, else center of capture)
for label, fname, title, desc in CAPTURES:
    d = data_by_tag[label]
    if d is None: continue
    print(f"\n--- {label}: {title} ---")
    if d['event_t'] is not None:
        et = d['event_t']
        print(f"  Event @ +{et:.2f}s ({'soft' if d['soft_fire'] else 'HARD'})  cliff={d['cliff_depth']}  jump={d['max_jump']}mm/s  velcov_peak={d['max_velcov']:.2f}")
        # Show -1s to +2s of NAV-PVT
        print(f"  NAV-PVT around event:")
        for entry in d['pvt']:
            t = entry[0]; rel = t - et
            if -1 <= rel <= 2:
                marker = " <-" if abs(rel) < 0.21 else ""
                print(f"    {rel:>+6.2f}s  sv={entry[2]:>2}  fix={entry[1]}  gSpd={entry[3]:>5}  hAcc={entry[4]:.2f}{marker}")
    else:
        # Clean: report any sub-threshold anomalies
        print(f"  No event detected.")
        print(f"  Max gSpd jump: {d['max_jump']} mm/s at +{d['max_jump_t']:.1f}s" if d['max_jump_t'] else "")
        if d['max_velcov'] > 0:
            print(f"  Max velcov: {d['max_velcov']:.3f} at +{d['max_velcov_t']:.1f}s")
        if d['agc_changes']:
            print(f"  agcCnt transitions:")
            for t, a, b in d['agc_changes']:
                print(f"    +{t:.1f}s  {a} -> {b}")
        else:
            print(f"  No agcCnt transitions (stayed constant)")


# Build HTML report
print("\nBuilding HTML report...")

# Plots
def plot_full_pvt_overlay():
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=False)
    colors = ["#c83030", "#d88800", "#2a8f2a", "#1f4e8f"]
    for (label, _, _, _), color in zip(CAPTURES, colors):
        d = data_by_tag[label]
        if d is None: continue
        ts = [r[0] for r in d['pvt']]
        svs = [r[2] for r in d['pvt']]
        gss = [r[3] for r in d['pvt']]
        hAcs = [r[4] for r in d['pvt']]
        axes[0].plot(ts, svs, lw=0.5, color=color, label=label, alpha=0.8)
        axes[1].plot(ts, gss, lw=0.5, color=color, label=label, alpha=0.8)
        axes[2].plot(ts, hAcs, lw=0.5, color=color, label=label, alpha=0.8)
        if d['event_t'] is not None:
            for ax in axes:
                ax.axvline(d['event_t'], color=color, lw=0.6, ls="--", alpha=0.4)
    axes[0].set_ylabel("numSV")
    axes[0].set_ylim(0, 35)
    axes[0].legend(loc="lower right", fontsize=9, ncol=4)
    axes[0].grid(alpha=0.3)
    axes[1].set_ylabel("gSpeed (mm/s)")
    axes[1].set_yscale("symlog", linthresh=100)
    axes[1].legend(loc="upper right", fontsize=9, ncol=4)
    axes[1].grid(alpha=0.3)
    axes[2].set_ylabel("hAcc (m)")
    axes[2].set_xlabel("capture time (s)")
    axes[2].legend(loc="upper right", fontsize=9, ncol=4)
    axes[2].grid(alpha=0.3)
    fig.suptitle("All 4 day-2 captures overlaid (dashed = event time per capture)",
                 fontsize=12)
    fig.tight_layout()
    return b64(fig)


def plot_agc_per_capture():
    fig, axes = plt.subplots(1, 4, figsize=(16, 4), sharey=True)
    for ax, (label, _, _, _) in zip(axes, CAPTURES):
        d = data_by_tag[label]
        if d is None or not d['hw']:
            ax.set_title(f"{label}: no MON-HW")
            continue
        ts = [r[0] for r in d['hw']]
        ag = [r[2] for r in d['hw']]
        ax.plot(ts, ag, marker="o", ms=4, lw=0.8, color="#106b3e")
        if d['event_t'] is not None:
            ax.axvline(d['event_t'], color="red", lw=0.8, ls="--", alpha=0.5)
        ax.set_title(f"{label}  ({len(ag)} samples)")
        ax.set_xlabel("capture time (s)")
        ax.set_ylim(3400, 3950)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("MON-HW agcCnt")
    fig.suptitle("MON-HW agcCnt across the 4 day-2 captures", fontsize=12)
    fig.tight_layout()
    return b64(fig)


def plot_velcov_per_capture():
    fig, axes = plt.subplots(1, 4, figsize=(16, 4))
    for ax, (label, _, _, _) in zip(axes, CAPTURES):
        d = data_by_tag[label]
        if d is None or not d['cov']:
            ax.set_title(f"{label}: no NAV-COV")
            continue
        ts = [r[0] for r in d['cov']]
        vmax = [max(r[4], r[5], r[6]) for r in d['cov']]
        ax.semilogy(ts, vmax, lw=0.6, color="#a04020")
        ax.axhline(0.3, color="orange", lw=0.5, ls=":", label="0.3 thresh")
        if d['event_t'] is not None:
            ax.axvline(d['event_t'], color="red", lw=0.8, ls="--", alpha=0.5)
        ax.set_title(label)
        ax.set_xlabel("capture time (s)")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8)
    axes[0].set_ylabel("max velCov (m²/s²)")
    fig.suptitle("NAV-COV peak velocity covariance across captures (log scale)",
                 fontsize=12)
    fig.tight_layout()
    return b64(fig)


def plot_event_zooms():
    """Side-by-side event zooms for D1, D2 (the fires) plus full timeline for D3, D4."""
    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    axes = axes.flatten()
    for ax, (label, _, _, _) in zip(axes, CAPTURES):
        d = data_by_tag[label]
        if d is None: continue
        if d['event_t'] is not None:
            et = d['event_t']
            ts = [r[0] - et for r in d['pvt'] if et-5 <= r[0] <= et+5]
            svs = [r[2] for r in d['pvt'] if et-5 <= r[0] <= et+5]
            gss = [r[3] for r in d['pvt'] if et-5 <= r[0] <= et+5]
            ax.plot(ts, svs, marker="o", ms=3, lw=0.6, color="#1f4e8f", label="numSV")
            ax.set_ylim(0, 35)
            ax.set_ylabel("numSV", color="#1f4e8f")
            axb = ax.twinx()
            axb.plot(ts, gss, marker="x", ms=3, lw=0.5, color="#a02050", label="gSpd")
            axb.set_ylabel("gSpd (mm/s)", color="#a02050")
            ax.axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
            ax.set_xlabel("seconds rel. event")
            ax.set_title(f"{label}  event +{et:.1f}s  cliff={d['cliff_depth']}  jump={d['max_jump']}")
            ax.grid(alpha=0.3)
        else:
            # full timeline
            ts = [r[0] for r in d['pvt']]
            svs = [r[2] for r in d['pvt']]
            ax.plot(ts, svs, lw=0.5, color="#1f4e8f")
            ax.set_ylim(0, 35)
            ax.set_xlabel("capture time (s)")
            ax.set_ylabel("numSV")
            ax.set_title(f"{label}  CLEAN  (max gSpd jump {d['max_jump']} mm/s)")
            ax.grid(alpha=0.3)
    fig.suptitle("D1/D2 event zooms, D3/D4 full timelines (clean)", fontsize=12)
    fig.tight_layout()
    return b64(fig)


pl_overlay = plot_full_pvt_overlay()
pl_agc = plot_agc_per_capture()
pl_velcov = plot_velcov_per_capture()
pl_zoom = plot_event_zooms()


# Generate HTML
html = []
html.append("""<!doctype html>
<html><head><meta charset="utf-8">
<title>Here4 Day 2 — Old Unit + Cold-Start Validation</title>
<style>
body { font-family: -apple-system, "Segoe UI", Arial, sans-serif; max-width: 1200px;
       margin: 24px auto; padding: 0 18px; color: #222; line-height: 1.5; }
h1 { border-bottom: 3px solid #1f4e8f; padding-bottom: 4px; color: #1f4e8f; }
h2 { color: #1f4e8f; margin-top: 32px; border-bottom: 1px solid #ccc; padding-bottom: 2px; }
table { border-collapse: collapse; margin: 10px 0; }
th, td { border: 1px solid #c0c0c0; padding: 5px 9px; text-align: left; font-size: 13px; }
th { background: #eaeef5; }
td.num, th.num { text-align: right; font-family: Consolas, monospace; }
tr.hard { background: #fff0f0; }
tr.clean { background: #f0fff0; }
img { max-width: 100%; height: auto; border: 1px solid #ccc; margin: 6px 0; }
.callout { background: #f0f4fc; border-left: 4px solid #1f4e8f; padding: 10px 14px; margin: 14px 0; }
.callout-good { background: #ecf8ec; border-left-color: #2a8f2a; }
.callout-warn { background: #fff5e8; border-left-color: #d88800; }
.tag-hard { background: #c83030; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; }
.tag-clean { background: #2a8f2a; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; }
code { background: #f4f4f4; padding: 1px 4px; border-radius: 3px; font-size: 12.5px; }
</style></head><body>""")

html.append("<h1>Here4 Day 2 Captures — Old Unit + Cold-Start Validation</h1>")
html.append("""
<p>Four captures collected today on an old/unused Here4 unit (DroneCAN node id 122)
that had never been used. The unit was placed on the carrier-board test stand and
captures taken in a specific sequence to test whether cold-boot alone, or
cold-boot plus configuration reset, suppresses the bug.</p>
""")

html.append("<h2>Sequence summary</h2>")
html.append("<table>")
html.append("<tr><th>Tag</th><th>Filename</th><th>Test condition</th>"
            "<th class='num'>Event</th><th>Result</th></tr>")
for label, fname, title, desc in CAPTURES:
    d = data_by_tag[label]
    if d is None:
        html.append(f"<tr><td>{label}</td><td><code>{fname}</code></td><td>{title}</td><td colspan='2'>missing</td></tr>")
        continue
    if d['event_t'] is not None:
        et_str = f"+{d['event_t']:.1f}s"
        tag = "<span class='tag-hard'>BUG FIRED</span>"
        klass = "hard"
    else:
        et_str = "—"
        tag = "<span class='tag-clean'>CLEAN</span>"
        klass = "clean"
    html.append(f"<tr class='{klass}'><td><b>{label}</b></td><td><code>{fname}</code></td>"
                f"<td>{title}<br><span style='color:#666;font-size:12px'>{desc}</span></td>"
                f"<td class='num'>{et_str}</td><td>{tag}</td></tr>")
html.append("</table>")

html.append("<h2>Headline finding</h2>")
html.append("""
<div class="callout callout-good">
<p>The sequence D1 → D2 → D3 → D4 is exactly what you'd want to see if
<b>cold-boot alone is NOT sufficient</b> but the <b>configuration-reset +
cold-boot procedure works.</b></p>
<ul>
<li><b>D1</b>: fresh hardware, bug fired naturally. Confirms the unit has the bug.</li>
<li><b>D2</b>: after cold-boot only, bug fired AGAIN. Cold-start alone does not fix it.</li>
<li><b>D3</b>: after config-reset + cold-boot, no bug in the capture window.</li>
<li><b>D4</b>: continued, no bug.</li>
</ul>
<p>This is the cleanest evidence we've collected that the workaround is the
<b>full procedure</b> (config reset + cold-boot), not just the cold-boot. It also
reframes Unit A's 2+ day clean streak — Unit A presumably had the full procedure
applied, not just cold-boot.</p>
</div>
""")

html.append("<h2>Per-capture event details</h2>")
html.append("<table>")
html.append("<tr><th>Tag</th><th class='num'>Duration</th><th class='num'>Event time</th>"
            "<th class='num'>Sat cliff</th><th class='num'>Max gSpd jump</th>"
            "<th class='num'>Max velCov</th><th>agcCnt transitions</th></tr>")
for label, _, _, _ in CAPTURES:
    d = data_by_tag[label]
    if d is None: continue
    et_str = f"+{d['event_t']:.1f}s" if d['event_t'] is not None else "—"
    cliff = d['cliff_depth'] if d['event_t'] is not None else 0
    agc = " → ".join([str(d['agc_changes'][0][1])] + [str(c[2]) for c in d['agc_changes']]) if d['agc_changes'] else "none"
    klass = "hard" if d['event_t'] is not None else "clean"
    html.append(f"<tr class='{klass}'><td><b>{label}</b></td>"
                f"<td class='num'>{d['duration']:.0f}s</td>"
                f"<td class='num'>{et_str}</td>"
                f"<td class='num'>{cliff}</td>"
                f"<td class='num'>{d['max_jump']}</td>"
                f"<td class='num'>{d['max_velcov']:.2f}</td>"
                f"<td>{agc}</td></tr>")
html.append("</table>")

html.append("<h2>Plots</h2>")
html.append("<h3>Event zooms for D1 & D2, full timelines for D3 & D4</h3>")
html.append(f'<img src="data:image/png;base64,{pl_zoom}">')

html.append("<h3>All 4 captures overlaid (numSV / gSpeed / hAcc)</h3>")
html.append(f'<img src="data:image/png;base64,{pl_overlay}">')

html.append("<h3>MON-HW agcCnt across the 4 captures</h3>")
html.append(f'<img src="data:image/png;base64,{pl_agc}">')

html.append("<h3>NAV-COV velocity covariance peak across the 4 captures (log scale)</h3>")
html.append(f'<img src="data:image/png;base64,{pl_velcov}">')

# Specific event narratives
html.append("<h2>D1 (fresh power-on, natural fire)</h2>")
d = data_by_tag["D1"]
if d and d['event_t']:
    html.append(f"<p>Natural bug fire at <b>+{d['event_t']:.1f}s</b> into the capture, matching the canonical ~8-minute timing for an untouched unit. Sat cliff of {d['cliff_depth']} sats, peak gSpd jump {d['max_jump']} mm/s, peak velCov {d['max_velcov']:.2f} m²/s². agcCnt transitions: {len(d['agc_changes'])}.</p>")
    if d['agc_changes']:
        html.append("<p>Transitions:</p><ul>")
        for t, a, b in d['agc_changes']:
            html.append(f"<li>+{t:.1f}s: {a} → {b}</li>")
        html.append("</ul>")

html.append("<h2>D2 (after cold-boot only, bug fired again)</h2>")
d = data_by_tag["D2"]
if d and d['event_t']:
    html.append(f"<p>Bug fired at <b>+{d['event_t']:.1f}s</b>. Sat cliff of {d['cliff_depth']} sats. This is the critical data point: <b>cold-boot via u-center alone did not suppress the bug</b>. Whatever cold-start does on its own is not enough — the alternate procedure (config reset + cold-boot) is what matters.</p>")
    if d['agc_changes']:
        html.append("<p>Transitions:</p><ul>")
        for t, a, b in d['agc_changes']:
            html.append(f"<li>+{t:.1f}s: {a} → {b}</li>")
        html.append("</ul>")

html.append("<h2>D3 + D4 (after config-reset + cold-boot, clean)</h2>")
d3 = data_by_tag["D3"]
d4 = data_by_tag["D4"]
if d3 and d4:
    html.append(f"<p>D3: {d3['duration']:.0f}s, no bug. Max gSpd jump in the capture: {d3['max_jump']} mm/s. Max velCov: {d3['max_velcov']:.2f}. agcCnt: {len(d3['agc_changes'])} transitions.</p>")
    html.append(f"<p>D4: {d4['duration']:.0f}s, no bug. Max gSpd jump: {d4['max_jump']} mm/s. Max velCov: {d4['max_velcov']:.2f}. agcCnt: {len(d4['agc_changes'])} transitions.</p>")
    total_clean = d3['duration'] + d4['duration']
    html.append(f"<p>Combined clean run after the full procedure: <b>{total_clean:.0f} seconds ({total_clean/60:.1f} minutes)</b>. The bug would normally have fired at ~8 minutes per the D1 baseline timing — so this is at least a full bug-window of clean operation.</p>")

html.append("<h2>What this means for the test plan</h2>")
html.append("""
<div class="callout callout-warn">
<p>The day-2 result has important implications for the <code>Here4_test_plan.md</code>:</p>
<ul>
<li><b>The "treatment" in Phase 2 must be the FULL procedure</b> — config reset + cold boot — not just cold boot. If you apply only cold-boot to Unit B, you'll likely see a recurrence and conclude the workaround doesn't work, when actually the procedure was incomplete.</li>
<li>The exact config-reset settings used in u-center matter. Worth documenting them precisely so they can be repeated on Units B, C, and to verify the procedure on Unit A.</li>
<li>D3 + D4 clean for ~17 minutes combined is encouraging but still short of the durability we'd want. Continue captures on this old/unused unit over the next several days to test durability.</li>
</ul>
</div>
""")

html.append("</body></html>")

out = os.path.join(DESKTOP, "Here4_day2_analysis.html")
with open(out, "w", encoding="utf-8") as f:
    f.write("\n".join(html))
print(f"\nWrote {out}  ({os.path.getsize(out)/1024:.0f} KB)")
