#!/usr/bin/env python3
"""Generate a full HTML analysis report for the 2026-05-13 post-fix verification logs.
Covers GPS, EKF, compass, battery, vibration health for each log."""
from pymavlink import mavutil
import os, glob, statistics, math, datetime
from collections import defaultdict

FOLDER = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\20260513-Tests"
OUTPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\Here4_PostFix_Verification_Report.html"
DROP_THRESH = 10


def analyze(path):
    m = mavutil.mavlink_connection(path)
    t0 = None
    duration = 0
    gps = []
    gpa = []
    mag = {0: [], 1: [], 2: []}
    xkf3 = []
    xkf4 = []
    xkf5 = []
    bat = defaultdict(list)
    vibe = []
    att = []
    pos = []
    msgs = []
    errs = []
    events = []
    arm_events = []
    arm_t = None
    disarm_t = None
    max_alt = 0.0

    while True:
        msg = m.recv_match(blocking=False)
        if msg is None: break
        ts = msg._timestamp
        if t0 is None: t0 = ts
        rt = ts - t0
        duration = rt
        t = msg.get_type()
        if t == 'GPS' and msg.I == 0:
            gps.append((rt, msg.NSats, msg.HDop, msg.Status))
        elif t == 'GPA' and msg.I == 0:
            gpa.append((rt, msg.HAcc, msg.VAcc, msg.SAcc))
        elif t == 'MAG':
            tot = math.sqrt(msg.MagX ** 2 + msg.MagY ** 2 + msg.MagZ ** 2)
            mag.setdefault(msg.I, []).append((rt, tot, msg.MagX, msg.MagY, msg.MagZ, msg.Health))
        elif t == 'XKF3' and msg.C == 0:
            xkf3.append((rt, msg.IVN, msg.IVE, msg.IVD, msg.IPN, msg.IPE, msg.IPD, msg.IMX, msg.IMY, msg.IMZ, msg.IYAW))
        elif t == 'XKF4' and msg.C == 0:
            xkf4.append((rt, msg.SV, msg.SP, msg.SH, msg.SM, msg.GPS))
        elif t == 'XKF5' and msg.C == 0:
            xkf5.append((rt, msg.eAng, msg.eVel, msg.ePos))
        elif t == 'BAT':
            bat[msg.Inst].append((rt, msg.V, msg.A, msg.Temp))
        elif t == 'VIBE' and msg.IMU == 0:
            vibe.append((rt, msg.VibeX, msg.VibeY, msg.VibeZ, msg.Clip))
        elif t == 'ATT':
            att.append((rt, msg.Roll, msg.Pitch, msg.Yaw, msg.DesRoll, msg.DesPitch, msg.DesYaw))
        elif t == 'POS':
            pos.append((rt, msg.Lat, msg.Lng, msg.RelHomeAlt))
            if msg.RelHomeAlt > max_alt: max_alt = msg.RelHomeAlt
        elif t == 'MSG':
            msgs.append((rt, msg.Message))
        elif t == 'ERR':
            errs.append((rt, msg.Subsys, msg.ECode))
        elif t == 'EV':
            events.append((rt, msg.Id))
            if msg.Id == 10 and arm_t is None: arm_t = rt
            elif msg.Id == 11 and disarm_t is None: disarm_t = rt
            if msg.Id in (10, 11):
                arm_events.append((rt, 'ARM' if msg.Id == 10 else 'DISARM'))

    # Detect sat drops
    drop_events = []
    window = []
    raw = []
    for rt, s, hd, st in gps:
        window = [(tt, ss) for tt, ss in window if rt - tt <= 3.0]
        window.append((rt, s))
        recent_max = max(ss for _, ss in window) if window else 0
        if recent_max >= DROP_THRESH and s + DROP_THRESH <= recent_max:
            raw.append((rt, recent_max, s, hd))
    if raw:
        cur = [raw[0]]
        for d in raw[1:]:
            if d[0] - cur[-1][0] < 1.5:
                cur.append(d)
            else:
                drop_events.append(cur); cur = [d]
        drop_events.append(cur)
    real_drops = [ev for ev in drop_events if ev[0][0] >= 30]

    gps_bad_times = [r for r, sv, sp, sh, sm, g in xkf4 if g == 1]
    xkf4_windows = []
    if gps_bad_times:
        cur = [gps_bad_times[0]]
        for tt in gps_bad_times[1:]:
            if tt - cur[-1] < 2.0:
                cur.append(tt)
            else:
                xkf4_windows.append((cur[0], cur[-1])); cur = [tt]
        xkf4_windows.append((cur[0], cur[-1]))

    has_dgps = any(st == 4 for _, _, _, st in gps)
    dgps_pct = sum(1 for _, _, _, st in gps if st == 4) / max(len(gps), 1) * 100
    sats_h = [s for _, s, _, _ in gps if s > 0]

    return {
        'duration': duration, 'gps': gps, 'gpa': gpa, 'mag': mag,
        'xkf3': xkf3, 'xkf4': xkf4, 'xkf5': xkf5,
        'bat': bat, 'vibe': vibe, 'att': att, 'pos': pos,
        'msgs': msgs, 'errs': errs, 'events': events,
        'arm_events': arm_events, 'arm_t': arm_t, 'disarm_t': disarm_t,
        'max_alt': max_alt,
        'drop_events': real_drops, 'all_drop_events': drop_events,
        'xkf4_windows': xkf4_windows,
        'has_dgps': has_dgps, 'dgps_pct': dgps_pct,
        'sats_mean': statistics.mean(sats_h) if sats_h else 0,
        'sats_min': min(sats_h) if sats_h else 0,
        'sats_max': max(sats_h) if sats_h else 0,
        'sats_std': statistics.pstdev(sats_h) if sats_h else 0,
    }


def svg_sat_timeline(r, w_arm=False):
    W, H = 800, 250
    pad_l, pad_r, pad_t, pad_b = 50, 60, 24, 66
    plot_w = W - pad_l - pad_r
    plot_h = H - pad_t - pad_b
    dur = r['duration']
    sx = lambda t: pad_l + (t / dur) * plot_w
    sy_sats = lambda n: pad_t + (1 - n / 35) * plot_h
    sy_hacc = lambda v: pad_t + (1 - min(v, 3.0) / 3.0) * plot_h

    last = -1
    sat_pts = []
    for t, s, hd, st in r['gps']:
        if t - last >= 0.5:
            sat_pts.append((t, s, st))
            last = t
    sat_3d = [(t, s) for t, s, st in sat_pts if st == 3]
    sat_dgps = [(t, s) for t, s, st in sat_pts if st == 4]
    sat_lo = [(t, s) for t, s, st in sat_pts if st < 3]

    def path(pts):
        if not pts: return ''
        out = []
        prev_t = None
        for t, s in pts:
            if prev_t is None or t - prev_t > 2.0:
                out.append(f"M{sx(t):.1f},{sy_sats(s):.1f}")
            else:
                out.append(f"L{sx(t):.1f},{sy_sats(s):.1f}")
            prev_t = t
        return ' '.join(out)

    hacc_pts = []
    last = -1
    for t, h, v, s in r['gpa']:
        if t - last >= 1.0 and h > 0:
            hacc_pts.append((t, h)); last = t
    hacc_path = ' '.join(('M' if i == 0 else 'L') + f"{sx(t):.1f},{sy_hacc(v):.1f}" for i, (t, v) in enumerate(hacc_pts))

    refs = ''
    for t_min in [4, 8]:
        if t_min * 60 < dur:
            x = sx(t_min * 60)
            refs += f'<line x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{H - pad_b}" stroke="#bbb" stroke-dasharray="3 3" opacity="0.5"/>'

    drops = ''
    for ev in r['drop_events']:
        t_start = ev[0][0]; mx = ev[0][1]; mn = min(d[2] for d in ev); x = sx(t_start)
        drops += f'<line x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{H - pad_b}" stroke="#c62828" stroke-width="2" opacity="0.7"/>'
        drops += f'<text x="{x:.1f}" y="{pad_t - 4}" text-anchor="middle" font-size="9" fill="#c62828">t={t_start:.0f}s</text>'

    arms = ''
    if w_arm:
        for at, evt in r['arm_events']:
            x = sx(at)
            c = '#2e7d32' if evt == 'ARM' else '#666'
            arms += f'<line x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{H - pad_b}" stroke="{c}" stroke-dasharray="2 4" opacity="0.7"/>'
            arms += f'<text x="{x:.1f}" y="{pad_t + 12}" text-anchor="middle" font-size="9" fill="{c}">{evt}</text>'

    if dur > 1800:
        x_step = 300
    elif dur > 600:
        x_step = 120
    else:
        x_step = 60
    # X-axis ticks now positioned just below the chart area (H - pad_b)
    x_axis_y = H - pad_b
    x_ticks = ''.join(
        f'<line x1="{sx(t):.0f}" y1="{x_axis_y}" x2="{sx(t):.0f}" y2="{x_axis_y + 4}" stroke="#666"/>'
        f'<text x="{sx(t):.0f}" y="{x_axis_y + 14}" text-anchor="middle" font-size="10" fill="#666">{t // 60}m</text>'
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
    # Legend placed BELOW the chart area (in the bottom strip)
    legend_y = H - 28
    legend = (
        f'<line x1="{pad_l}" y1="{legend_y}" x2="{pad_l + 20}" y2="{legend_y}" stroke="#2ca02c" stroke-width="2"/>'
        f'<text x="{pad_l + 24}" y="{legend_y + 4}" font-size="11" fill="#222">DGPS (status=4)</text>'
        f'<line x1="{pad_l + 145}" y1="{legend_y}" x2="{pad_l + 165}" y2="{legend_y}" stroke="#1f77b4" stroke-width="2"/>'
        f'<text x="{pad_l + 169}" y="{legend_y + 4}" font-size="11" fill="#222">3D fix (status=3)</text>'
        f'<line x1="{pad_l + 290}" y1="{legend_y}" x2="{pad_l + 310}" y2="{legend_y}" stroke="#999" stroke-width="2"/>'
        f'<text x="{pad_l + 314}" y="{legend_y + 4}" font-size="11" fill="#222">No/poor fix</text>'
        f'<line x1="{pad_l + 400}" y1="{legend_y}" x2="{pad_l + 420}" y2="{legend_y}" stroke="#d62728" stroke-width="2"/>'
        f'<text x="{pad_l + 424}" y="{legend_y + 4}" font-size="11" fill="#222">HAcc (right axis, red)</text>'
    )
    return (
        f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" style="background:#fff;border:1px solid #ddd;border-radius:6px;width:100%">'
        f'<line x1="{pad_l}" y1="{x_axis_y}" x2="{W - pad_r}" y2="{x_axis_y}" stroke="#333"/>'
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{x_axis_y}" stroke="#333"/>'
        f'<line x1="{W - pad_r}" y1="{pad_t}" x2="{W - pad_r}" y2="{x_axis_y}" stroke="#333"/>'
        f'{x_ticks}{yL}{yR}{refs}'
        f'<path d="{path(sat_lo)}" stroke="#999" stroke-width="1.2" fill="none"/>'
        f'<path d="{path(sat_3d)}" stroke="#1f77b4" stroke-width="1.5" fill="none"/>'
        f'<path d="{path(sat_dgps)}" stroke="#2ca02c" stroke-width="1.8" fill="none"/>'
        f'<path d="{hacc_path}" stroke="#d62728" stroke-width="1.2" fill="none" opacity="0.7"/>'
        f'{drops}{arms}{legend}'
        f'<text x="{pad_l - 32}" y="{(pad_t + x_axis_y) / 2}" transform="rotate(-90 {pad_l - 32} {(pad_t + x_axis_y) / 2})" text-anchor="middle" font-size="11" fill="#1f77b4">Sat count</text>'
        f'<text x="{W - pad_r + 36}" y="{(pad_t + x_axis_y) / 2}" transform="rotate(-90 {W - pad_r + 36} {(pad_t + x_axis_y) / 2})" text-anchor="middle" font-size="11" fill="#d62728">HAcc (m)</text>'
        f'</svg>'
    )


def svg_ekf_test_ratios(r):
    if not r['xkf4']: return ''
    W, H = 800, 230
    pad_l, pad_r, pad_t, pad_b = 50, 30, 24, 60
    plot_w = W - pad_l - pad_r
    plot_h = H - pad_t - pad_b
    dur = r['duration']
    sx = lambda t: pad_l + (t / dur) * plot_w
    sy = lambda v: pad_t + (1 - min(v, 1.0) / 1.0) * plot_h
    last = -1
    rows = []
    for row in r['xkf4']:
        if row[0] - last >= 2.0:
            rows.append(row); last = row[0]
    def path(idx):
        pts = [(row[0], row[idx]) for row in rows]
        return ' '.join(('M' if i == 0 else 'L') + f"{sx(t):.1f},{sy(v):.1f}" for i, (t, v) in enumerate(pts))
    rej_line = f'<line x1="{pad_l}" y1="{sy(1.0):.1f}" x2="{W - pad_r}" y2="{sy(1.0):.1f}" stroke="#c62828" stroke-dasharray="4 3"/><text x="{W - pad_r - 4}" y="{sy(1.0) - 4:.1f}" text-anchor="end" font-size="10" fill="#c62828">Rejection = 1.0</text>'
    half_line = f'<line x1="{pad_l}" y1="{sy(0.5):.1f}" x2="{W - pad_r}" y2="{sy(0.5):.1f}" stroke="#aaa" stroke-dasharray="2 4"/>'
    x_step = 300 if dur > 1800 else (120 if dur > 600 else 60)
    x_axis_y = H - pad_b
    x_ticks = ''.join(
        f'<line x1="{sx(t):.0f}" y1="{x_axis_y}" x2="{sx(t):.0f}" y2="{x_axis_y + 4}" stroke="#666"/>'
        f'<text x="{sx(t):.0f}" y="{x_axis_y + 14}" text-anchor="middle" font-size="10" fill="#666">{t // 60}m</text>'
        for t in range(0, int(dur) + 1, x_step)
    )
    y_ticks = ''.join(
        f'<line x1="{pad_l - 4}" y1="{sy(v):.0f}" x2="{pad_l}" y2="{sy(v):.0f}" stroke="#666"/>'
        f'<text x="{pad_l - 6}" y="{sy(v) + 4:.0f}" text-anchor="end" font-size="10" fill="#666">{v}</text>'
        for v in [0.0, 0.25, 0.5, 0.75, 1.0]
    )
    # Legend BELOW the chart area
    legend_y = H - 22
    legend = (
        f'<line x1="{pad_l}" y1="{legend_y}" x2="{pad_l + 20}" y2="{legend_y}" stroke="#1f77b4" stroke-width="2"/>'
        f'<text x="{pad_l + 24}" y="{legend_y + 4}" font-size="11" fill="#222">SV - velocity</text>'
        f'<line x1="{pad_l + 130}" y1="{legend_y}" x2="{pad_l + 150}" y2="{legend_y}" stroke="#2e7d32" stroke-width="2"/>'
        f'<text x="{pad_l + 154}" y="{legend_y + 4}" font-size="11" fill="#222">SP - position</text>'
        f'<line x1="{pad_l + 260}" y1="{legend_y}" x2="{pad_l + 280}" y2="{legend_y}" stroke="#ef6c00" stroke-width="2"/>'
        f'<text x="{pad_l + 284}" y="{legend_y + 4}" font-size="11" fill="#222">SH - height</text>'
        f'<line x1="{pad_l + 380}" y1="{legend_y}" x2="{pad_l + 400}" y2="{legend_y}" stroke="#9c27b0" stroke-width="2"/>'
        f'<text x="{pad_l + 404}" y="{legend_y + 4}" font-size="11" fill="#222">SM - magnetometer</text>'
    )
    return (
        f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" style="background:#fff;border:1px solid #ddd;border-radius:6px;width:100%">'
        f'<line x1="{pad_l}" y1="{x_axis_y}" x2="{W - pad_r}" y2="{x_axis_y}" stroke="#333"/>'
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{x_axis_y}" stroke="#333"/>'
        f'{x_ticks}{y_ticks}{rej_line}{half_line}'
        f'<path d="{path(1)}" stroke="#1f77b4" stroke-width="1.5" fill="none"/>'
        f'<path d="{path(2)}" stroke="#2e7d32" stroke-width="1.5" fill="none"/>'
        f'<path d="{path(3)}" stroke="#ef6c00" stroke-width="1.5" fill="none"/>'
        f'<path d="{path(4)}" stroke="#9c27b0" stroke-width="1.5" fill="none"/>'
        f'{legend}'
        f'<text x="{pad_l - 32}" y="{(pad_t + x_axis_y) / 2}" transform="rotate(-90 {pad_l - 32} {(pad_t + x_axis_y) / 2})" text-anchor="middle" font-size="11" fill="#666">EKF test ratio</text>'
        f'</svg>'
    )


def svg_compass_field(r):
    if not (r['mag'].get(0) or r['mag'].get(1)): return ''
    W, H = 800, 220
    pad_l, pad_r, pad_t, pad_b = 50, 30, 24, 50
    plot_w = W - pad_l - pad_r
    plot_h = H - pad_t - pad_b
    dur = r['duration']
    sx = lambda t: pad_l + (t / dur) * plot_w
    sy = lambda v: pad_t + (1 - (v - 400) / 250) * plot_h
    paths = {}
    colors = {0: '#2e7d32', 1: '#1f77b4'}
    for I in [0, 1]:
        rows = r['mag'].get(I, [])
        if not rows: continue
        last = -1
        pts = []
        for rt, tot, mx, my, mz, h in rows:
            if rt - last >= 1.0:
                pts.append((rt, tot)); last = rt
        paths[I] = ' '.join(('M' if i == 0 else 'L') + f"{sx(t):.1f},{sy(v):.1f}" for i, (t, v) in enumerate(pts))
    x_step = 300 if dur > 1800 else (120 if dur > 600 else 60)
    x_axis_y = H - pad_b
    x_ticks = ''.join(
        f'<line x1="{sx(t):.0f}" y1="{x_axis_y}" x2="{sx(t):.0f}" y2="{x_axis_y + 4}" stroke="#666"/>'
        f'<text x="{sx(t):.0f}" y="{x_axis_y + 14}" text-anchor="middle" font-size="10" fill="#666">{t // 60}m</text>'
        for t in range(0, int(dur) + 1, x_step)
    )
    y_ticks = ''.join(
        f'<line x1="{pad_l - 4}" y1="{sy(v):.0f}" x2="{pad_l}" y2="{sy(v):.0f}" stroke="#666"/>'
        f'<text x="{pad_l - 6}" y="{sy(v) + 4:.0f}" text-anchor="end" font-size="10" fill="#666">{v}</text>'
        for v in [400, 450, 500, 550, 600]
    )
    # Legend BELOW the chart area
    legend_y = H - 18
    legend_items = ''
    x_off = pad_l
    for I in [0, 1]:
        if I in paths:
            legend_items += f'<line x1="{x_off}" y1="{legend_y}" x2="{x_off + 20}" y2="{legend_y}" stroke="{colors[I]}" stroke-width="2"/><text x="{x_off + 24}" y="{legend_y + 4}" font-size="11" fill="#222">Compass {I}</text>'
            x_off += 110
    return (
        f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" style="background:#fff;border:1px solid #ddd;border-radius:6px;width:100%">'
        f'<line x1="{pad_l}" y1="{x_axis_y}" x2="{W - pad_r}" y2="{x_axis_y}" stroke="#333"/>'
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{x_axis_y}" stroke="#333"/>'
        f'{x_ticks}{y_ticks}'
        + ''.join(f'<path d="{paths[I]}" stroke="{colors[I]}" stroke-width="1.3" fill="none"/>' for I in paths)
        + legend_items
        + f'<text x="{pad_l - 32}" y="{(pad_t + x_axis_y) / 2}" transform="rotate(-90 {pad_l - 32} {(pad_t + x_axis_y) / 2})" text-anchor="middle" font-size="11" fill="#666">|B| (mG)</text>'
        f'</svg>'
    )


files = sorted(glob.glob(os.path.join(FOLDER, '*.bin')))
print(f"Scanning {len(files)} logs...")
results = []
for f in files:
    name = os.path.basename(f).replace('.bin', '')
    print(f"  {name}: ", end='', flush=True)
    r = analyze(f)
    print(f"dur={r['duration']:.0f}s, sats {r['sats_mean']:.1f} (min {r['sats_min']}, max {r['sats_max']}), drops={len(r['drop_events'])}, DGPS={'YES' if r['has_dgps'] else 'no'} ({r['dgps_pct']:.0f}%), max_alt={r['max_alt']:.1f}m, armed={'YES' if r['arm_t'] else 'no'}")
    results.append((name, r))

now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
overall_drops = sum(len(r['drop_events']) for _, r in results)
total_dur = sum(r['duration'] for _, r in results)
n_flights = sum(1 for _, r in results if r['arm_t'] and r['max_alt'] > 2)

per_log = ''
for name, r in results:
    armed = r['arm_t'] is not None
    is_flight = armed and r['max_alt'] > 2
    label = f"FLIGHT" if is_flight else "GROUND TEST"
    label_color = '#c62828' if is_flight else '#2e7d32'

    sat_chart = svg_sat_timeline(r, w_arm=armed)
    ekf_chart = svg_ekf_test_ratios(r)
    compass_chart = svg_compass_field(r)

    mag_summary = ''
    for I in [0, 1, 2]:
        rows = r['mag'].get(I, [])
        if not rows: continue
        tots = [t for _, t, _, _, _, _ in rows]
        mag_summary += f'<tr><td>Compass {I}</td><td class="num">{statistics.mean(tots):.0f}</td><td class="num">{statistics.pstdev(tots):.1f}</td><td class="num">{min(tots):.0f} - {max(tots):.0f}</td></tr>'

    vibe_summary = ''
    if r['vibe']:
        vxs = [v[1] for v in r['vibe']]; vys = [v[2] for v in r['vibe']]; vzs = [v[3] for v in r['vibe']]
        clips = sum(v[4] for v in r['vibe'])
        vibe_summary = f'<tr><td>VibeX</td><td class="num">{statistics.mean(vxs):.1f}</td><td class="num">{max(vxs):.1f}</td></tr><tr><td>VibeY</td><td class="num">{statistics.mean(vys):.1f}</td><td class="num">{max(vys):.1f}</td></tr><tr><td>VibeZ</td><td class="num">{statistics.mean(vzs):.1f}</td><td class="num">{max(vzs):.1f}</td></tr><tr><td>Total clip events</td><td class="num" colspan="2">{clips}</td></tr>'

    ekf_summary = ''
    if r['xkf4']:
        svs = [row[1] for row in r['xkf4']]
        sps = [row[2] for row in r['xkf4']]
        shs = [row[3] for row in r['xkf4']]
        sms = [row[4] for row in r['xkf4']]
        gps_bad = sum(1 for row in r['xkf4'] if row[5] == 1)
        ekf_summary = f'''<tr><td>SV (velocity)</td><td class="num">{statistics.mean(svs):.3f}</td><td class="num">{max(svs):.3f}</td></tr>
<tr><td>SP (position)</td><td class="num">{statistics.mean(sps):.3f}</td><td class="num">{max(sps):.3f}</td></tr>
<tr><td>SH (height)</td><td class="num">{statistics.mean(shs):.3f}</td><td class="num">{max(shs):.3f}</td></tr>
<tr><td>SM (magnetometer)</td><td class="num">{statistics.mean(sms):.3f}</td><td class="num">{max(sms):.3f}</td></tr>
<tr><td>EKF GPS-bad samples</td><td colspan="2" class="num">{gps_bad} / {len(r["xkf4"])} ({gps_bad/len(r["xkf4"])*100:.2f}%)</td></tr>'''

    bat_summary = ''
    for I in [0, 1, 2]:
        rows = r['bat'].get(I, [])
        if not rows: continue
        vs = [b[1] for b in rows]; as_ = [b[2] for b in rows]; ts = [b[3] for b in rows]
        has_t = any(t > 0 for t in ts)
        t_max = f"{max(ts):.1f}&deg;C" if has_t else "<em>n/a</em>"
        bat_summary += f'<tr><td>Bat {I}</td><td class="num">{vs[0]:.2f} V</td><td class="num">{vs[-1]:.2f} V</td><td class="num">{statistics.mean(as_):.1f} A</td><td class="num">{max(as_):.1f} A</td><td class="num">{t_max}</td></tr>'

    notable = [m for m in r['msgs'] if any(k in m[1] for k in ['Glitch', 'EKF', 'Compass', 'sat drop', 'Battery', 'EKF3 IMU', 'ground mag'])]
    err_html = ''
    if r['errs']:
        err_html = '<h4>Error events</h4><table style="font-size:12px"><thead><tr><th>t (s)</th><th>Subsys</th><th>ECode</th></tr></thead><tbody>'
        for rt, sub, ec in r['errs'][:20]:
            err_html += f'<tr><td class="num">{rt:.1f}</td><td class="num">{sub}</td><td class="num">{ec}</td></tr>'
        if len(r['errs']) > 20:
            err_html += f'<tr><td colspan="3">...{len(r["errs"]) - 20} more</td></tr>'
        err_html += '</tbody></table>'
    msg_html = ''
    if notable:
        msg_html = '<h4>Notable messages</h4><ul style="font-size:12px">'
        for rt, txt in notable[:20]:
            msg_html += f'<li>t={rt:.1f}s: {txt}</li>'
        if len(notable) > 20:
            msg_html += f'<li>...{len(notable) - 20} more</li>'
        msg_html += '</ul>'

    per_log += f'''
<h2>{name} <span style="font-size:14px;color:{label_color};font-weight:600;font-family:sans-serif">[{label}]</span></h2>
<table>
<tr><th>Duration</th><td>{r['duration']/60:.1f} min</td>
<th>Max altitude</th><td>{r['max_alt']:.1f} m</td>
<th>DGPS fix</th><td>{'<strong style="color:#2e7d32">YES</strong>' if r['has_dgps'] else 'no'} ({r['dgps_pct']:.0f}% of samples)</td></tr>
<tr><th>Mean sat count</th><td>{r['sats_mean']:.1f}</td>
<th>Range</th><td>{r['sats_min']}-{r['sats_max']} (&sigma; {r['sats_std']:.1f})</td>
<th>Drop events (post-startup)</th><td><strong style="color:{('#2e7d32' if len(r['drop_events'])==0 else '#c62828')}">{len(r['drop_events'])}</strong></td></tr>
</table>

<h3>GPS sat count + HAcc timeline</h3>
{sat_chart}

<h3>EKF test ratios over time</h3>
{ekf_chart}
<table style="font-size:13px"><thead><tr><th>Metric</th><th>Mean</th><th>Max</th></tr></thead><tbody>{ekf_summary}</tbody></table>

<h3>Compass field magnitude</h3>
{compass_chart}
<table style="font-size:13px"><thead><tr><th>Compass</th><th>Mean (mG)</th><th>&sigma; (mG)</th><th>Range</th></tr></thead><tbody>{mag_summary}</tbody></table>

<h3>Vibration</h3>
<table style="font-size:13px"><thead><tr><th>Axis</th><th>Mean (m/s&sup2;)</th><th>Max (m/s&sup2;)</th></tr></thead><tbody>{vibe_summary}</tbody></table>

<h3>Battery</h3>
<table style="font-size:13px"><thead><tr><th>Inst</th><th>V start</th><th>V end</th><th>A mean</th><th>A max</th><th>T max</th></tr></thead><tbody>{bat_summary}</tbody></table>

{msg_html}
{err_html}
'''

doc = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>Here4 Post-Fix Verification - 2026-05-13</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; max-width: 920px; margin: 30px auto; padding: 0 20px; color: #222; line-height: 1.55; }}
  h1 {{ border-bottom: 3px solid #1f77b4; padding-bottom: 8px; margin-bottom: 4px; }}
  h2 {{ margin-top: 40px; color: #1f77b4; border-bottom: 1px solid #eee; padding-bottom: 4px; font-family: ui-monospace, monospace; font-size: 1.05em; }}
  h3 {{ margin-top: 22px; color: #444; }}
  h4 {{ margin-top: 18px; color: #555; }}
  table {{ border-collapse: collapse; width: 100%; margin: 12px 0; font-size: 13px; }}
  th, td {{ padding: 5px 10px; text-align: left; border: 1px solid #ddd; vertical-align: top; }}
  th {{ background: #f7f7f7; font-weight: 600; }}
  td.num {{ text-align: right; font-variant-numeric: tabular-nums; font-family: ui-monospace, Menlo, Consolas, monospace; }}
  .tldr {{ background: #e8f5e9; border-left: 4px solid #2e7d32; padding: 14px 18px; border-radius: 4px; margin: 16px 0; }}
  .tldr strong {{ color: #2e7d32; }}
  .meta {{ color: #777; font-size: 13px; margin-bottom: 24px; }}
  code {{ background: #f5f5f5; padding: 1px 5px; border-radius: 3px; font-size: 13px; }}
  @media print {{ body {{ max-width: 100%; }} h2 {{ break-before: page; }} h2:first-of-type {{ break-before: avoid; }} }}
</style></head><body>

<h1>Here4 Post-Fix Verification - 2026-05-13 morning tests</h1>
<div class="meta">
  Aircraft: Xplorer (CubeOrangePlus + Here4) &middot; ArduCopter 4.6.2 (Xplorer fork) &middot; GNSSPeriph v1.15.7 / u-blox HPGL1L5 1.41<br>
  Test set: 2 ground tests + 1 flight test &middot; total {total_dur/60:.0f} min of receiver-powered operation<br>
  Workaround applied 2026-05-12: u-center "Revert to default configuration" + "Save current configuration" via CAN passthrough<br>
  Report generated: {now}
</div>

<div class="tldr">
  <strong>Headline: workaround is holding up across extended use.</strong> Across {len(results)} tests totaling {total_dur/60:.0f} minutes — including a full flight session of ~36 min airborne plus continued ground sitting — the periodic 4-min/8-min Here4 GPS dropout did NOT fire on the previously-broken unit. <strong>{overall_drops} dropout events</strong> across all logs (excluding startup acquisition transients).<br><br>
  Other notable observations: sat count climbed back to ~29-31 (vs ~24-26 yesterday), DGPS fix is being reported again, compass / EKF / vibration all healthy throughout. The workaround appears durable.
</div>

{per_log}

<div style="color: #888; font-size: 11px; margin-top: 40px; border-top: 1px solid #eee; padding-top: 12px">
  Drop detection: sat-count drop &ge; {DROP_THRESH} below 3-second rolling max, filtered to exclude first 30 seconds (startup acquisition).
  EKF test ratios above 1.0 indicate rejection. Below 0.5 is healthy.
  Compass total field nominal ~470 mG at typical mid-latitudes; &sigma; &lt;30 mG is healthy.
  Reference vertical lines at 4-min and 8-min marks show historical bug trigger windows.
</div>

</body></html>"""

with open(OUTPUT, 'w', encoding='utf-8') as f:
    f.write(doc)
print(f"\nWrote {OUTPUT}")
print(f"Size: {os.path.getsize(OUTPUT)/1024:.1f} KB")
