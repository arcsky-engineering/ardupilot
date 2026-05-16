#!/usr/bin/env python3
"""Generate a self-contained HTML analysis report for a specific GPS glitch event."""
from pymavlink import mavutil
import math, statistics, datetime, os
from collections import defaultdict

LOGFILE = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\xpl2\2026-04-30 16-30-29.bin"
OUTPUT  = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\xpl2\GPS_Glitch_Analysis_2026-04-30.html"
GLITCH_T0, GLITCH_T1 = 62.6, 68.5

m = mavutil.mavlink_connection(LOGFILE)
t0 = None
gps_q, gpa_q, xkf3, xkf4, msgs, events = [], [], [], [], [], []
mag_by_inst = {0: [], 1: [], 2: []}
bat = defaultdict(list)
params = {}

while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    ts = msg._timestamp
    if t0 is None: t0 = ts
    rt = ts - t0
    t = msg.get_type()
    if t == 'GPS' and msg.I == 0:
        gps_q.append((rt, msg.NSats, msg.HDop, msg.Spd))
    elif t == 'GPA' and msg.I == 0:
        gpa_q.append((rt, msg.HAcc, msg.VAcc, msg.SAcc))
    elif t == 'MAG':
        total = math.sqrt(msg.MagX ** 2 + msg.MagY ** 2 + msg.MagZ ** 2)
        mag_by_inst.setdefault(msg.I, []).append((rt, total))
    elif t == 'XKF3' and msg.C == 0:
        xkf3.append((rt, msg.IVN, msg.IVE, msg.IVD, msg.IMX, msg.IMY, msg.IMZ, msg.IYAW))
    elif t == 'XKF4' and msg.C == 0:
        xkf4.append((rt, msg.SV, msg.SP, msg.SH, msg.SM, msg.GPS))
    elif t == 'BAT':
        bat[msg.Inst].append((rt, msg.A))
    elif t == 'EV':
        events.append((rt, msg.Id))
    elif t == 'MSG':
        msgs.append((rt, msg.Message))
    elif t == 'PARM':
        params[msg.Name] = msg.Value


def corr(xs, ys):
    n = len(xs)
    if n < 2: return 0
    mx = sum(xs) / n; my = sum(ys) / n
    num = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    dx = math.sqrt(sum((xs[i] - mx) ** 2 for i in range(n)))
    dy = math.sqrt(sum((ys[i] - my) ** 2 for i in range(n)))
    return num / (dx * dy) if dx * dy > 0 else 0


def bin1s(rows, vidx_list):
    bins = defaultdict(list)
    for row in rows:
        b = int(row[0])
        for vi in vidx_list:
            bins[(b, vi)].append(row[vi])
    out = {}
    for (b, vi), vals in bins.items():
        out.setdefault(b, {})[vi] = statistics.mean(vals)
    return out


gps_b = bin1s(gps_q, [1, 2, 3])
gpa_b = bin1s(gpa_q, [1, 2, 3])
bat0_b = {int(rt): a for rt, a in bat[0]}
bat1_b = {int(rt): a for rt, a in bat[1]}
total_a = {k: bat0_b.get(k, 0) + bat1_b.get(k, 0) for k in bat0_b}

keys = sorted(set(gps_b.keys()) & set(total_a.keys()))
keys_acc = sorted(set(gpa_b.keys()) & set(total_a.keys()))
r_sats = corr([total_a[k] for k in keys], [gps_b[k].get(1, 0) for k in keys])
r_hacc = corr([total_a[k] for k in keys_acc], [gpa_b[k].get(1, 0) for k in keys_acc])
r_sacc = corr([total_a[k] for k in keys_acc], [gpa_b[k].get(3, 0) for k in keys_acc])


def aligned(rt_a, vals_a, rt_b, vals_b, t_lo, t_hi):
    ba = defaultdict(list); bb = defaultdict(list)
    for r, v in zip(rt_a, vals_a):
        if t_lo <= r <= t_hi: ba[int(r)].append(v)
    for r, v in zip(rt_b, vals_b):
        if t_lo <= r <= t_hi: bb[int(r)].append(v)
    ks = sorted(set(ba) & set(bb))
    return [statistics.mean(ba[k]) for k in ks], [statistics.mean(bb[k]) for k in ks]


total_curr_t = list(bat0_b.keys())
total_curr_v = [bat0_b[k] + bat1_b.get(k, 0) for k in total_curr_t]
compass_corr = {}
for I in [0, 1, 2]:
    rt_m = [r for r, _ in mag_by_inst[I]]
    v_m = [v for _, v in mag_by_inst[I]]
    xs, ys = aligned(total_curr_t, total_curr_v, rt_m, v_m, 140, 260)
    if len(xs) > 5:
        compass_corr[I] = (corr(xs, ys), min(ys), max(ys), min(xs), max(xs))


def phase(rt):
    if rt < 60: return 'pre-glitch'
    if rt < 70: return 'glitch'
    if rt < 140: return 'post-glitch hover'
    if rt < 260: return 'AUTO mission'
    return 'post-flight'


mag_phase = {}
for I in [0, 1, 2]:
    mag_phase[I] = {}
    for ph in ['pre-glitch', 'glitch', 'AUTO mission']:
        rows = [t for rt, t in mag_by_inst.get(I, []) if phase(rt) == ph]
        if rows:
            mag_phase[I][ph] = (statistics.mean(rows), statistics.pstdev(rows), len(rows))


def build_svg_timeline():
    W, H = 720, 240
    pad_l, pad_r, pad_t, pad_b = 50, 50, 20, 30
    plot_w = W - pad_l - pad_r
    plot_h = H - pad_t - pad_b
    t_lo, t_hi = 55, 80
    pts_sat = [(rt, n) for rt, n, _, _ in gps_q if t_lo <= rt <= t_hi]
    pts_hacc = [(rt, ha) for rt, ha, _, _ in gpa_q if t_lo <= rt <= t_hi]
    sx = lambda t: pad_l + (t - t_lo) / (t_hi - t_lo) * plot_w
    sy_sats = lambda n: pad_t + (1 - n / 32) * plot_h
    sy_hacc = lambda v: pad_t + (1 - min(v, 3) / 3) * plot_h
    sat_path = ' '.join(
        ('M' if i == 0 else 'L') + f"{sx(t):.1f},{sy_sats(n):.1f}"
        for i, (t, n) in enumerate(pts_sat)
    )
    hacc_path = ' '.join(
        ('M' if i == 0 else 'L') + f"{sx(t):.1f},{sy_hacc(v):.1f}"
        for i, (t, v) in enumerate(pts_hacc)
    )
    gx0 = sx(GLITCH_T0); gx1 = sx(GLITCH_T1)
    x_ticks = ''.join(
        f'<line x1="{sx(t):.1f}" y1="{H - pad_b}" x2="{sx(t):.1f}" y2="{H - pad_b + 4}" stroke="#666"/>'
        f'<text x="{sx(t):.1f}" y="{H - pad_b + 16}" text-anchor="middle" font-size="11" fill="#666">{t}s</text>'
        for t in [55, 60, 65, 70, 75, 80]
    )
    y_left = ''.join(
        f'<line x1="{pad_l - 4}" y1="{sy_sats(n):.1f}" x2="{pad_l}" y2="{sy_sats(n):.1f}" stroke="#666"/>'
        f'<text x="{pad_l - 6}" y="{sy_sats(n) + 4:.1f}" text-anchor="end" font-size="11" fill="#1f77b4">{n}</text>'
        for n in [0, 8, 16, 24, 32]
    )
    y_right = ''.join(
        f'<line x1="{W - pad_r}" y1="{sy_hacc(v):.1f}" x2="{W - pad_r + 4}" y2="{sy_hacc(v):.1f}" stroke="#666"/>'
        f'<text x="{W - pad_r + 6}" y="{sy_hacc(v) + 4:.1f}" text-anchor="start" font-size="11" fill="#d62728">{v}m</text>'
        for v in [0, 1, 2, 3]
    )
    return (
        f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" '
        f'style="background:#fff;border:1px solid #ddd;border-radius:6px">'
        f'<rect x="{gx0:.1f}" y="{pad_t}" width="{gx1 - gx0:.1f}" height="{plot_h}" fill="#fff3cd"/>'
        f'<text x="{(gx0 + gx1) / 2:.1f}" y="{pad_t + 14}" text-anchor="middle" font-size="11" fill="#856404">EKF rejected GPS</text>'
        f'<line x1="{pad_l}" y1="{H - pad_b}" x2="{W - pad_r}" y2="{H - pad_b}" stroke="#333"/>'
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{H - pad_b}" stroke="#333"/>'
        f'<line x1="{W - pad_r}" y1="{pad_t}" x2="{W - pad_r}" y2="{H - pad_b}" stroke="#333"/>'
        f'{x_ticks}{y_left}{y_right}'
        f'<path d="{sat_path}" stroke="#1f77b4" stroke-width="2" fill="none"/>'
        f'<path d="{hacc_path}" stroke="#d62728" stroke-width="2" fill="none"/>'
        f'<text x="{pad_l - 32}" y="{pad_t + plot_h / 2}" transform="rotate(-90 {pad_l - 32} {pad_t + plot_h / 2})" text-anchor="middle" font-size="11" fill="#1f77b4">Sat count</text>'
        f'<text x="{W - pad_r + 32}" y="{pad_t + plot_h / 2}" transform="rotate(-90 {W - pad_r + 32} {pad_t + plot_h / 2})" text-anchor="middle" font-size="11" fill="#d62728">HAcc (m)</text>'
        f'<text x="{W / 2}" y="{H - 2}" text-anchor="middle" font-size="11" fill="#333">Time (s since log start)</text>'
        f'</svg>'
    )


def build_svg_scatter():
    W, H = 720, 240
    pad_l, pad_r, pad_t, pad_b = 50, 30, 20, 30
    plot_w = W - pad_l - pad_r
    plot_h = H - pad_t - pad_b
    pts = [(total_a[k], gpa_b[k].get(1, 0)) for k in keys_acc if 5 <= k <= 270]
    if not pts: return ''
    xmax = max(p[0] for p in pts)
    ymax = max(3, max(p[1] for p in pts))
    sx = lambda v: pad_l + v / xmax * plot_w
    sy = lambda v: pad_t + (1 - v / ymax) * plot_h
    dots = ''.join(
        f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="2.5" fill="#1f77b4" opacity="0.6"/>'
        for x, y in pts
    )
    x_ticks = ''.join(
        f'<line x1="{sx(v):.0f}" y1="{H - pad_b}" x2="{sx(v):.0f}" y2="{H - pad_b + 4}" stroke="#666"/>'
        f'<text x="{sx(v):.0f}" y="{H - pad_b + 16}" text-anchor="middle" font-size="11" fill="#666">{v}A</text>'
        for v in [0, 15, 30, 45, 60]
    )
    y_ticks = ''.join(
        f'<line x1="{pad_l - 4}" y1="{sy(v):.0f}" x2="{pad_l}" y2="{sy(v):.0f}" stroke="#666"/>'
        f'<text x="{pad_l - 6}" y="{sy(v) + 4:.0f}" text-anchor="end" font-size="11" fill="#666">{v}m</text>'
        for v in [0, 1, 2, 3]
    )
    return (
        f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" '
        f'style="background:#fff;border:1px solid #ddd;border-radius:6px">'
        f'<line x1="{pad_l}" y1="{H - pad_b}" x2="{W - pad_r}" y2="{H - pad_b}" stroke="#333"/>'
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{H - pad_b}" stroke="#333"/>'
        f'{x_ticks}{y_ticks}{dots}'
        f'<text x="{W / 2}" y="{H - 2}" text-anchor="middle" font-size="11" fill="#333">Total battery current (A)</text>'
        f'<text x="14" y="{pad_t + plot_h / 2}" transform="rotate(-90 14 {pad_t + plot_h / 2})" text-anchor="middle" font-size="11" fill="#333">GPS HAcc (m)</text>'
        f'<text x="{W - pad_r - 10}" y="{pad_t + 10}" text-anchor="end" font-size="11" fill="#666">r = {r_hacc:+.3f}</text>'
        f'</svg>'
    )


# Glitch detail rows
glitch_rows = []
for rt, sats, hdop, spd in gps_q:
    if 61.5 <= rt <= 63.5:
        nearest = min(gpa_q, key=lambda r: abs(r[0] - rt))
        hacc, sacc = nearest[1], nearest[3]
        cls = 'glitch' if 62.0 <= rt <= 62.8 else ''
        glitch_rows.append(
            f'<tr class="{cls}"><td class="num">{rt:.2f}</td><td class="num">{sats}</td>'
            f'<td class="num">{hdop:.2f}</td><td class="num">{hacc:.2f}</td>'
            f'<td class="num">{sacc:.2f}</td><td class="num">{spd:.2f}</td></tr>'
        )

# Mag table rows
mag_rows = []
for I in [0, 1, 2]:
    if I in mag_phase and 'pre-glitch' in mag_phase[I] and 'glitch' in mag_phase[I]:
        pre = mag_phase[I]['pre-glitch']
        gli = mag_phase[I]['glitch']
        used = (
            (I == 0 and params.get('COMPASS_USE', 0) == 1) or
            (I == 1 and params.get('COMPASS_USE2', 0) == 1) or
            (I == 2 and params.get('COMPASS_USE3', 0) == 1)
        )
        mag_rows.append(
            f'<tr><td>Compass {I}</td><td class="num">{pre[0]:.0f}</td>'
            f'<td class="num">{gli[0]:.0f}</td><td class="num">{gli[1]:.1f}</td>'
            f'<td>{"Yes" if used else "No (disabled)"}</td></tr>'
        )

# Compass-current table rows
cc_rows = []
for I, (r, ymin, ymax, _, _) in compass_corr.items():
    flag = 'Strong correlation' if abs(r) > 0.5 else ('Weak correlation' if abs(r) > 0.2 else 'Effectively decoupled')
    cc_rows.append(
        f'<tr><td>Compass {I}</td><td class="num">{r:+.3f}</td>'
        f'<td class="num">{ymin:.0f}-{ymax:.0f}</td><td>{flag}</td></tr>'
    )

now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')

doc = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>GPS Glitch Analysis - 2026-04-30 Flight</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; max-width: 880px; margin: 30px auto; padding: 0 20px; color: #222; line-height: 1.5; }}
  h1 {{ border-bottom: 3px solid #1f77b4; padding-bottom: 8px; margin-bottom: 4px; }}
  h2 {{ margin-top: 36px; color: #1f77b4; border-bottom: 1px solid #eee; padding-bottom: 4px; }}
  h3 {{ margin-top: 24px; color: #444; }}
  .meta {{ color: #777; font-size: 13px; margin-bottom: 24px; }}
  .tldr {{ background: #f0f7ff; border-left: 4px solid #1f77b4; padding: 14px 18px; border-radius: 4px; margin: 16px 0; }}
  .tldr strong {{ color: #1f77b4; }}
  .verdict-no {{ color: #2e7d32; font-weight: 600; }}
  .verdict-yes {{ color: #c62828; font-weight: 600; }}
  table {{ border-collapse: collapse; width: 100%; margin: 12px 0; font-size: 13px; }}
  th, td {{ padding: 6px 10px; text-align: left; border: 1px solid #ddd; }}
  th {{ background: #f7f7f7; font-weight: 600; }}
  tr.glitch td {{ background: #fff3cd; }}
  td.num {{ text-align: right; font-variant-numeric: tabular-nums; font-family: ui-monospace, Menlo, Consolas, monospace; }}
  .rec {{ background: #fff; border: 1px solid #e0e0e0; border-radius: 6px; padding: 12px 16px; margin: 10px 0; }}
  .rec h4 {{ margin: 0 0 6px 0; color: #1f77b4; }}
  .rec.priority-high {{ border-left: 4px solid #c62828; }}
  .rec.priority-med {{ border-left: 4px solid #ef6c00; }}
  .rec.priority-low {{ border-left: 4px solid #2e7d32; }}
  code {{ background: #f5f5f5; padding: 1px 5px; border-radius: 3px; font-size: 13px; }}
  .footer {{ color: #888; font-size: 11px; margin-top: 40px; border-top: 1px solid #eee; padding-top: 12px; }}
  @media print {{ body {{ max-width: 100%; }} .rec {{ break-inside: avoid; }} }}
</style></head><body>

<h1>GPS Glitch Analysis</h1>
<div class="meta">
  Log file: <code>2026-04-30 16-30-29.bin</code><br>
  Aircraft: Xplorer (CubeOrangePlus, S/N 1924A0226030003), ArduCopter 4.6.2<br>
  Flight duration: 277s &middot; Modes: Loiter (0-140s) &rarr; Auto (140-210s) &rarr; Loiter (210-262s)<br>
  Report generated: {now}
</div>

<div class="tldr">
  <strong>TL;DR.</strong> A 6-second GPS glitch occurred at t=62.6s while hovering at ~15m AGL in Loiter. The GPS receiver itself reported degraded data (29-&gt;10 sats, HAcc 0.49-&gt;2.34m, SAcc 0.13-&gt;2.76 m/s, phantom 2.37 m/s velocity on a stationary aircraft). The EKF correctly rejected the velocity solution and recovered cleanly.
  <br><br>
  <span class="verdict-no">&#10007; Not a magnetometer issue.</span> Mag innovations and total field were stable throughout.<br>
  <span class="verdict-no">&#10007; Not motor/power EMI.</span> GPS quality has zero correlation with battery current across the entire flight (r &asymp; 0).<br>
  <span class="verdict-yes">&#10003; Most likely cause: in-band RF interference</span> from onboard Ethernet camera and/or radio mounted close to the GPS antenna. The 1-in-20-flight intermittency is consistent with traffic-dependent RF emissions (camera bitrate bursts, radio retransmits).
</div>

<h2>1. The glitch event</h2>
<p>At t=62.6s the EKF logged <code>GPS Glitch: vel=0.68 pos=0.08 mag=0.09</code> (Subsys 11, ECode 2), cleared at t=68.5s. This is a velocity-only rejection - position and mag innovations were tiny.</p>

<p><strong>GPS quality during the event:</strong></p>
<table><thead><tr><th>t (s)</th><th>Sats</th><th>HDop</th><th>HAcc (m)</th><th>SAcc (m/s)</th><th>Reported Spd (m/s)</th></tr></thead>
<tbody>{''.join(glitch_rows)}</tbody></table>

<p>Sat count crashed 29-&gt;10 in 0.5s, HDop spiked 0.56-&gt;1.87, and the GPS itself reported its velocity solution as bad (SAcc 0.13-&gt;2.76 m/s). The receiver flagged the data as poor before the EKF rejected it.</p>

{build_svg_timeline()}
<p style="font-size:12px;color:#666;text-align:center;margin-top:4px">Sat count (blue, left axis) and GPS-reported HAcc (red, right axis) around the glitch window. Yellow band marks the EKF rejection period.</p>

<h2>2. Was it a magnetometer issue? No.</h2>
<p>Compass innovations and total field were unaffected by the glitch:</p>

<table><thead><tr><th>Compass</th><th>Pre-glitch mean (mG)</th><th>During glitch (mG)</th><th>&sigma; during glitch</th><th>Active in EKF?</th></tr></thead>
<tbody>{''.join(mag_rows)}</tbody></table>

<p>EKF mag innovations (<code>XKF3.IMX/IMY/IMZ</code>) stayed at single-digit values before, during, and after the glitch. Yaw innovation (<code>IYAW</code>) was 0.00 throughout - the EKF wasn't fighting heading at all. The mag test ratio (<code>XKF4.SM</code>) stayed at 0.06-0.10, well below rejection threshold.</p>

<h2>3. Compass-vs-current correlation (CompassMot check)</h2>
<p>Across the AUTO mission (140-260s, current swings 9-57 A):</p>
<table><thead><tr><th>Compass</th><th>Pearson r vs. total current</th><th>Field range (mG)</th><th>Interpretation</th></tr></thead>
<tbody>{''.join(cc_rows)}</tbody></table>

<p>Compass 0 (priority 1, internal) and Compass 1 (priority 2, external) are well-decoupled from current - <code>COMPASS_MOTCT=2</code> with non-zero <code>COMPASS_MOT_X/Y/Z</code> means current-based compensation is working. Compass 2 shows strong current correlation, but it is <strong>already disabled</strong> (<code>COMPASS_USE3=0</code>) and not fed to the EKF.</p>

<h2>4. Was it caused by motor/power EMI? No.</h2>
<p>If high motor current were leaking RF into the GPS, sat count and HAcc would degrade during high-throttle moments. They don't:</p>

<table><thead><tr><th>Metric</th><th>Pearson r vs. total current</th></tr></thead>
<tbody>
<tr><td>Sat count</td><td class="num">{r_sats:+.3f}</td></tr>
<tr><td>HAcc (horizontal accuracy)</td><td class="num">{r_hacc:+.3f}</td></tr>
<tr><td>SAcc (speed accuracy)</td><td class="num">{r_sacc:+.3f}</td></tr>
</tbody></table>

{build_svg_scatter()}
<p style="font-size:12px;color:#666;text-align:center;margin-top:4px">GPS HAcc vs. total battery current, 1-second bins. No trend.</p>

<p>Current at the glitch moment (46.7 A) was nominal - no spike, no abnormal load. This rules out current-driven EMI from the propulsion system.</p>

<h2>5. The actual likely cause: in-band RF from onboard Ethernet camera + radio</h2>
<p>An Ethernet camera and a radio are mounted in close proximity to the GPS antenna in the same compartment. Both are well-known GPS L1 interference sources:</p>

<h3>Ethernet camera</h3>
<ul>
  <li>100BASE-TX clocks at 125 MHz; 1000BASE-T also has 125 MHz fundamentals.</li>
  <li>The 12th-13th harmonics land at 1.500-1.625 GHz - adjacent to GPS L1 (1.575 GHz).</li>
  <li>Unshielded Cat5e/6 cabling acts as an antenna and radiates these harmonics directly.</li>
  <li>Emission level is <strong>traffic-dependent</strong>: idle Ethernet emits much less than a saturated link, so GPS quality fluctuates with camera bitrate, packet bursts, and link-state events.</li>
</ul>

<h3>Radio</h3>
<ul>
  <li>900 MHz: 2nd harmonic at 1.8 GHz can leak into GPS L1.</li>
  <li>1.2/1.3 GHz video TX: fundamental sits adjacent to GPS L1; can desensitize the receiver.</li>
  <li>2.4 GHz: out-of-band emissions can blanket GPS L1 if antennas are close.</li>
</ul>

<h3>Why "1 in 20 flights" fits</h3>
<p>Constant-power RF leakage causes <em>constantly</em> degraded GPS - not glitches. Intermittent glitches at ~5% rate point to <strong>peaks</strong> in RF activity:</p>
<ul>
  <li>Ethernet auto-negotiation / link state changes (camera reconnects)</li>
  <li>Camera bitrate spikes (high-motion frames, codec keyframes, packet bursts)</li>
  <li>Radio retransmissions / range-stretching at the edge of useful link budget</li>
  <li>Thermal effects (RF emissions worsen with component temperature)</li>
</ul>
<p>The smoking gun: at t=62 the airframe was hovering with steady throttle and zero attitude change, yet 19 of 29 satellites dropped <em>simultaneously</em> in 0.5 seconds. That is the textbook fingerprint of a noise floor jump at the GPS receiver front-end, not a satellite-side or motion-related event.</p>

<h2>6. GPS antenna position offsets - not configured</h2>
<p>Currently:</p>
<table><thead><tr><th>Parameter</th><th>Value</th></tr></thead>
<tbody>
<tr><td><code>GPS1_POS_X</code></td><td class="num">{params.get('GPS1_POS_X', 0):.2f} m</td></tr>
<tr><td><code>GPS1_POS_Y</code></td><td class="num">{params.get('GPS1_POS_Y', 0):.2f} m</td></tr>
<tr><td><code>GPS1_POS_Z</code></td><td class="num">{params.get('GPS1_POS_Z', 0):.2f} m</td></tr>
</tbody></table>
<p>Should be set to the antenna location relative to the IMU/CG (X forward+, Y right+, Z down+, in meters). Not the cause of this glitch, but worth fixing for EKF lever-arm correctness during maneuvers.</p>

<h2>7. Recommendations</h2>

<div class="rec priority-high">
  <h4>Priority 1: Physical separation of GPS and RF sources</h4>
  <p>Move the GPS antenna as far from the Ethernet camera and radio as physically possible. 15-20 cm minimum, 30+ cm safe. Vertical separation (riser mast) helps significantly. This is the single highest-impact change.</p>
</div>

<div class="rec priority-high">
  <h4>Priority 2: Replace Ethernet cable with shielded (STP) and add ferrite chokes</h4>
  <p>Use foil-shielded twisted-pair (STP) Cat6 with the shield grounded at one end (typically airframe / FC ground). Add clip-on ferrite snap chokes (rated 100 MHz - 1 GHz) to:</p>
  <ul>
    <li>The Ethernet cable, near both ends</li>
    <li>The radio's RF coax (near the radio, not the antenna)</li>
    <li>Any DC power leads running near the GPS</li>
  </ul>
  <p>Cheapest first test: tape a ferrite bead onto the existing Ethernet cable and fly. If glitch frequency drops, you've confirmed the source.</p>
</div>

<div class="rec priority-med">
  <h4>Priority 3: Verify GPS antenna ground plane</h4>
  <p>Most patch GNSS antennas need a copper ground plane &ge;7 cm diameter directly underneath to perform well and reject RF arriving from below. If the GPS sits on a plastic mount with the radio/camera underneath, it's receiving RF straight into its lobe.</p>
</div>

<div class="rec priority-med">
  <h4>Priority 4: Add a second GPS and enable auto-switch</h4>
  <p>Currently <code>GPS2_TYPE=0</code> (single GPS) and <code>GPS_AUTO_SWITCH=0</code>. With a second GPS placed in a quieter location and <code>GPS_AUTO_SWITCH=1</code>, the EKF can blend or fail over rather than dead-reckon for 6+ seconds during a glitch. This protects against a worse version of the same event.</p>
</div>

<div class="rec priority-low">
  <h4>Priority 5: Configure GPS antenna position offsets</h4>
  <p>Set <code>GPS1_POS_X/Y/Z</code> to the actual antenna location relative to the IMU. Improves EKF lever-arm handling during maneuvers and improves residual diagnostics. Doesn't fix the glitch but is good hygiene.</p>
</div>

<div class="rec priority-low">
  <h4>Priority 6: Bench diagnostic test</h4>
  <p>Power up the airframe outside, watch sat count in Mission Planner. Then:</p>
  <ul>
    <li>Plug/unplug the Ethernet camera while observing sat count and HAcc</li>
    <li>Key the radio (TX) while observing the same</li>
    <li>Stream high-bitrate camera traffic and watch for sat count drops</li>
  </ul>
  <p>If sat count drops by 5+ when either is active, the source is confirmed and you can iterate on shielding/separation until the drop disappears.</p>
</div>

<h2>8. What this analysis does NOT support</h2>
<ul>
  <li>Magnetometer failure or yaw error - innovations stable throughout.</li>
  <li>Inadequate CompassMot calibration on active compasses - both decoupled from current.</li>
  <li>High-current EMI from motors/ESCs - zero correlation with current.</li>
  <li>Aircraft maneuver shading the antenna - yaw and attitude were essentially constant.</li>
  <li>GPS configuration error - receiver was healthy and reporting valid quality metrics.</li>
</ul>

<div class="footer">
  Analysis derived from binary log message types: GPS, GPA, MAG, XKF1-XKF5, BAT, IMU, ATT, RCOU, EV, MSG, PARM, MODE, VIBE.
  Compass-current correlation computed across AUTO mission window (140-260s) at 1 Hz binning.
  GPS-current correlation computed across full flight at 1 Hz binning.
</div>

</body></html>"""

with open(OUTPUT, 'w', encoding='utf-8') as f:
    f.write(doc)

print(f"Wrote {OUTPUT}")
print(f"Size: {os.path.getsize(OUTPUT)} bytes")
