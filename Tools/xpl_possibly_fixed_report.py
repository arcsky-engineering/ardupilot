#!/usr/bin/env python3
"""Plot sat count + GPS quality from a folder of ArduPilot bin logs into an HTML report."""
from pymavlink import mavutil
import os, glob, statistics, math, datetime
from collections import defaultdict

FOLDER = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\here4-possibly-good-fixed"
OUTPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\Here4_PossiblyFixed_Report.html"
DROP_THRESH = 10


def analyze(path):
    m = mavutil.mavlink_connection(path)
    t0 = None
    duration = 0
    gps = []  # (rt, sats, hdop, hacc, sacc)
    msgs = []
    errs = []
    arm_t = None
    disarm_t = None
    max_alt = 0.0
    params = {}
    xkf4_gps_bad = []
    ekf_innov_max = []
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None: break
        ts = msg._timestamp
        if t0 is None: t0 = ts
        rt = ts - t0
        duration = rt
        t = msg.get_type()
        if t == 'GPS' and msg.I == 0:
            gps.append((rt, msg.NSats, msg.HDop, 0, 0))
        elif t == 'GPA' and msg.I == 0 and gps:
            # attach hacc/sacc to most recent GPS sample
            gps[-1] = (gps[-1][0], gps[-1][1], gps[-1][2], msg.HAcc, msg.SAcc)
        elif t == 'MSG':
            msgs.append((rt, msg.Message))
        elif t == 'ERR':
            errs.append((rt, msg.Subsys, msg.ECode))
        elif t == 'XKF4' and msg.C == 0 and msg.GPS == 1:
            xkf4_gps_bad.append(rt)
        elif t == 'EV':
            if msg.Id == 10 and arm_t is None: arm_t = rt
            elif msg.Id == 11 and disarm_t is None: disarm_t = rt
        elif t == 'POS':
            if msg.RelHomeAlt > max_alt: max_alt = msg.RelHomeAlt
        elif t == 'PARM':
            params[msg.Name] = msg.Value

    sats_h = [s for _, s, _, _, _ in gps]
    sat_drops = []
    window = []
    for rt, s, hd, ha, sa in gps:
        window = [(tt, ss) for tt, ss in window if rt - tt <= 3.0]
        window.append((rt, s))
        recent_max = max(ss for _, ss in window) if window else 0
        if recent_max >= DROP_THRESH and s + DROP_THRESH <= recent_max:
            sat_drops.append((rt, recent_max, s, hd))
    drop_events = []
    if sat_drops:
        cur = [sat_drops[0]]
        for d in sat_drops[1:]:
            if d[0] - cur[-1][0] < 1.5: cur.append(d)
            else: drop_events.append(cur); cur = [d]
        drop_events.append(cur)
    # xkf4 windows
    xkf4_windows = []
    if xkf4_gps_bad:
        cur = [xkf4_gps_bad[0]]
        for t in xkf4_gps_bad[1:]:
            if t - cur[-1] < 2.0: cur.append(t)
            else: xkf4_windows.append((cur[0], cur[-1])); cur = [t]
        xkf4_windows.append((cur[0], cur[-1]))

    return {
        'duration': duration,
        'gps': gps,
        'msgs': msgs,
        'errs': errs,
        'drop_events': drop_events,
        'xkf4_windows': xkf4_windows,
        'arm_t': arm_t,
        'disarm_t': disarm_t,
        'max_alt': max_alt,
        'mean_sats': statistics.mean(sats_h) if sats_h else 0,
        'min_sats': min(sats_h) if sats_h else 0,
        'max_sats': max(sats_h) if sats_h else 0,
        'std_sats': statistics.pstdev(sats_h) if sats_h else 0,
        'gps_drv_options': params.get('GPS_DRV_OPTIONS', 'n/a'),
        'gps_gnss_mode': params.get('GPS1_GNSS_MODE', params.get('GPS_GNSS_MODE', 'n/a')),
    }


files = sorted(glob.glob(os.path.join(FOLDER, '*.bin')))
print(f"Scanning {len(files)} logs from {FOLDER}")
results = []
for f in files:
    name = os.path.basename(f).replace('.bin', '')
    print(f"  {name}: ", end='', flush=True)
    r = analyze(f)
    print(f"dur={r['duration']:.0f}s, sats mean={r['mean_sats']:.1f} min={r['min_sats']} max={r['max_sats']}, drops={len(r['drop_events'])}, errs={len(r['errs'])}")
    results.append((name, r))


def svg_sat_timeline(name, r):
    W, H = 760, 220
    pad_l, pad_r, pad_t, pad_b = 50, 50, 24, 30
    plot_w = W - pad_l - pad_r
    plot_h = H - pad_t - pad_b
    dur = r['duration']
    sx = lambda t: pad_l + (t / dur) * plot_w
    sy_sats = lambda n: pad_t + (1 - n / 35) * plot_h
    sy_hacc = lambda v: pad_t + (1 - min(v, 3.0) / 3.0) * plot_h

    # Decimate to ~2 per second
    sat_pts = []; hacc_pts = []
    last = -0.5
    for t, s, hd, ha, sa in r['gps']:
        if t - last >= 0.5:
            sat_pts.append((t, s))
            if ha > 0:
                hacc_pts.append((t, ha))
            last = t
    sat_path = ' '.join(('M' if i == 0 else 'L') + f"{sx(t):.1f},{sy_sats(s):.1f}" for i, (t, s) in enumerate(sat_pts))
    hacc_path = ' '.join(('M' if i == 0 else 'L') + f"{sx(t):.1f},{sy_hacc(v):.1f}" for i, (t, v) in enumerate(hacc_pts))

    # 4-min, 8-min reference lines
    refs = ''
    for t_min in [4, 8]:
        if t_min * 60 < dur:
            x = sx(t_min * 60)
            refs += f'<line x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{H - pad_b}" stroke="#bbb" stroke-dasharray="3 3" opacity="0.5"/>'
            refs += f'<text x="{x:.1f}" y="{H - pad_b + 22}" text-anchor="middle" font-size="9" fill="#888">{t_min}m</text>'

    # Drop event markers
    drops = ''
    for ev in r['drop_events']:
        t_start = ev[0][0]; mx = ev[0][1]; mn = min(d[2] for d in ev)
        x = sx(t_start)
        drops += f'<line x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{H - pad_b}" stroke="#c62828" stroke-width="2" opacity="0.7"/>'
        drops += f'<text x="{x:.1f}" y="{pad_t - 4}" text-anchor="middle" font-size="10" fill="#c62828">{t_start:.0f}s: {mx}&rarr;{mn}</text>'

    # Axes
    x_step = max(60, int(dur / 8))
    x_ticks = ''.join(
        f'<line x1="{sx(t):.0f}" y1="{H - pad_b}" x2="{sx(t):.0f}" y2="{H - pad_b + 4}" stroke="#666"/>'
        f'<text x="{sx(t):.0f}" y="{H - pad_b + 14}" text-anchor="middle" font-size="10" fill="#666">{t//60}m{t%60:02d}s</text>'
        for t in range(0, int(dur) + 1, x_step)
    )
    yL = ''.join(
        f'<line x1="{pad_l - 4}" y1="{sy_sats(n):.0f}" x2="{pad_l}" y2="{sy_sats(n):.0f}" stroke="#666"/>'
        f'<text x="{pad_l - 6}" y="{sy_sats(n) + 4:.0f}" text-anchor="end" font-size="10" fill="#1f77b4">{n}</text>'
        for n in [0, 10, 20, 30]
    )
    yR = ''.join(
        f'<line x1="{W - pad_r}" y1="{sy_hacc(v):.0f}" x2="{W - pad_r + 4}" y2="{sy_hacc(v):.0f}" stroke="#666"/>'
        f'<text x="{W - pad_r + 6}" y="{sy_hacc(v) + 4:.0f}" text-anchor="start" font-size="10" fill="#d62728">{v}m</text>'
        for v in [0, 1, 2, 3]
    )
    return (
        f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" style="background:#fff;border:1px solid #ddd;border-radius:6px;width:100%">'
        f'<line x1="{pad_l}" y1="{H - pad_b}" x2="{W - pad_r}" y2="{H - pad_b}" stroke="#333"/>'
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{H - pad_b}" stroke="#333"/>'
        f'<line x1="{W - pad_r}" y1="{pad_t}" x2="{W - pad_r}" y2="{H - pad_b}" stroke="#333"/>'
        f'{x_ticks}{yL}{yR}'
        f'{refs}'
        f'<path d="{sat_path}" stroke="#1f77b4" stroke-width="1.5" fill="none"/>'
        f'<path d="{hacc_path}" stroke="#d62728" stroke-width="1.5" fill="none" opacity="0.7"/>'
        f'{drops}'
        f'<text x="{pad_l - 32}" y="{H / 2}" transform="rotate(-90 {pad_l - 32} {H / 2})" text-anchor="middle" font-size="11" fill="#1f77b4">Sat count</text>'
        f'<text x="{W - pad_r + 32}" y="{H / 2}" transform="rotate(-90 {W - pad_r + 32} {H / 2})" text-anchor="middle" font-size="11" fill="#d62728">HAcc (m)</text>'
        f'</svg>'
    )


# Build HTML
now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
per_log = ''
for name, r in results:
    drv = r['gps_drv_options']
    if isinstance(drv, float): drv = f"{drv:.0f}"
    errs_html = ''
    if r['errs']:
        errs_html = '<p><strong>Errors logged:</strong></p><ul>'
        for rt, sub, ec in r['errs'][:10]:
            errs_html += f'<li>t={rt:.1f}s: Subsys={sub} ECode={ec}</li>'
        errs_html += '</ul>'
    msgs_html = ''
    interesting_msgs = [m for m in r['msgs'] if any(k in m[1] for k in ['EKF','GPS','Compass','Glitch','sat drop','error','ERROR'])]
    if interesting_msgs:
        msgs_html = '<p><strong>Interesting MSGs:</strong></p><ul>'
        for rt, txt in interesting_msgs[:20]:
            msgs_html += f'<li>t={rt:.1f}s: {txt}</li>'
        if len(interesting_msgs) > 20:
            msgs_html += f'<li>...{len(interesting_msgs) - 20} more</li>'
        msgs_html += '</ul>'

    per_log += f'''
<h2>{name}</h2>
<table>
<tr><th>Duration</th><td>{r['duration']/60:.1f} min ({r['duration']:.0f}s)</td></tr>
<tr><th>Mean sat count</th><td>{r['mean_sats']:.1f} (min {r['min_sats']}, max {r['max_sats']}, &sigma; {r['std_sats']:.1f})</td></tr>
<tr><th>Detected drop events (&ge;10 below 3s max)</th><td><strong>{len(r['drop_events'])}</strong></td></tr>
<tr><th>EKF GPS-bad windows</th><td>{len(r['xkf4_windows'])}</td></tr>
<tr><th>GPS_DRV_OPTIONS</th><td><strong>{drv}</strong></td></tr>
<tr><th>GPS_GNSS_MODE</th><td>{r['gps_gnss_mode']}</td></tr>
<tr><th>Max altitude</th><td>{r['max_alt']:.1f} m</td></tr>
</table>
{svg_sat_timeline(name, r)}
<p style="font-size:12px;color:#666">Sat count (blue, left axis) and HAcc (red, right axis) over time. Faint gray dashed lines at 4-min and 8-min for reference (the typical bug timings). Red vertical bars mark detected drop events.</p>
{msgs_html}
{errs_html}
'''

doc = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>Here4 Possibly-Fixed Unit Analysis</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; max-width: 900px; margin: 30px auto; padding: 0 20px; color: #222; line-height: 1.55; }}
  h1 {{ border-bottom: 3px solid #1f77b4; padding-bottom: 8px; margin-bottom: 4px; }}
  h2 {{ margin-top: 36px; color: #1f77b4; border-bottom: 1px solid #eee; padding-bottom: 4px; font-family: monospace; font-size: 1.1em; }}
  table {{ border-collapse: collapse; margin: 12px 0; font-size: 13px; }}
  th, td {{ padding: 6px 12px; text-align: left; border: 1px solid #ddd; }}
  th {{ background: #f7f7f7; font-weight: 600; width: 200px; }}
  .tldr {{ background: #e8f5e9; border-left: 4px solid #2e7d32; padding: 14px 18px; border-radius: 4px; margin: 16px 0; }}
  .tldr strong {{ color: #2e7d32; }}
  .meta {{ color: #777; font-size: 13px; margin-bottom: 24px; }}
</style></head><body>

<h1>Here4 "Possibly Fixed" Unit Analysis</h1>
<div class="meta">
  Folder: <code>here4-possibly-good-fixed</code> &middot; 3 logs scanned<br>
  Test setup: bare carrier board, USB power, outdoor, no u-center<br>
  Unit history: previously failed ~100%, was factory-reset in u-center + GNSS mode tinkering, now appears to stabilize<br>
  Report generated: {now}
</div>

{per_log}

</body></html>"""

with open(OUTPUT, 'w', encoding='utf-8') as f:
    f.write(doc)
print(f"\nWrote {OUTPUT} ({os.path.getsize(OUTPUT)} bytes)")
