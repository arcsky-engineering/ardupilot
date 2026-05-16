#!/usr/bin/env python3
"""Scan a directory of ArduPilot bin logs for the GPS-interference fingerprint:
- EKF GPS glitch events
- Sudden NSats drops (>=10 sats in <1s)
- EKF XKF4.GPS=1 flag set
Outputs per-log summary and an aggregate report."""
from pymavlink import mavutil
import os, sys, glob, time, statistics
from collections import defaultdict

FOLDER = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026 02 13 Logs to Date-20260511T185246Z-3-001\2026 02 13 Logs to Date"
DROP_THRESH = 10  # match what the code patch uses


def analyze_log(path):
    try:
        m = mavutil.mavlink_connection(path)
    except Exception as e:
        return {'error': str(e)}

    t0 = None
    duration = 0
    glitch_msgs = []     # MSG entries with "GPS Glitch" / "Glitch cleared"
    err_subsys11 = []    # ERR Subsys=11 events
    sat_history = []     # (rt, sats, hdop)
    gpa_history = []     # (rt, hacc, sacc)
    xkf4_gps_bad = []    # times when EKF flagged GPS bad
    armed_at = None
    disarmed_at = None
    max_alt = 0.0
    fw_version = None

    try:
        while True:
            msg = m.recv_match(blocking=False)
            if msg is None: break
            ts = msg._timestamp
            if t0 is None: t0 = ts
            rt = ts - t0
            duration = rt
            t = msg.get_type()

            if t == 'GPS' and msg.I == 0:
                sat_history.append((rt, msg.NSats, msg.HDop))
            elif t == 'GPA' and msg.I == 0:
                gpa_history.append((rt, msg.HAcc, msg.SAcc))
            elif t == 'MSG':
                txt = msg.Message
                if 'Glitch' in txt:
                    glitch_msgs.append((rt, txt))
                elif fw_version is None and 'ArduCopter' in txt:
                    fw_version = txt
            elif t == 'ERR':
                if msg.Subsys == 11:
                    err_subsys11.append((rt, msg.ECode))
            elif t == 'XKF4' and msg.C == 0:
                if msg.GPS == 1:
                    xkf4_gps_bad.append(rt)
            elif t == 'EV':
                if msg.Id == 10 and armed_at is None:
                    armed_at = rt
                elif msg.Id == 11 and disarmed_at is None:
                    disarmed_at = rt
            elif t == 'POS':
                if msg.RelHomeAlt > max_alt: max_alt = msg.RelHomeAlt
    except Exception as e:
        return {'error': f'parse error: {e}', 'duration': duration}

    # Detect sat drop events
    sat_drops = []
    if sat_history:
        window = []
        for rt, s, hd in sat_history:
            window = [(t, ss) for t, ss in window if rt - t <= 3.0]
            window.append((rt, s))
            recent_max = max(ss for _, ss in window) if window else 0
            if recent_max >= DROP_THRESH and s + DROP_THRESH <= recent_max:
                # log this dip
                sat_drops.append((rt, recent_max, s, hd))

    # Collapse consecutive drops into single events
    drop_events = []
    if sat_drops:
        cur = [sat_drops[0]]
        for d in sat_drops[1:]:
            if d[0] - cur[-1][0] < 1.5:
                cur.append(d)
            else:
                drop_events.append(cur); cur = [d]
        drop_events.append(cur)

    # Collapse XKF4 GPS-bad samples into windows
    xkf4_windows = []
    if xkf4_gps_bad:
        cur = [xkf4_gps_bad[0]]
        for t in xkf4_gps_bad[1:]:
            if t - cur[-1] < 2.0:
                cur.append(t)
            else:
                xkf4_windows.append((cur[0], cur[-1])); cur = [t]
        xkf4_windows.append((cur[0], cur[-1]))

    # Flight time: armed-to-disarmed
    flight_dur = None
    if armed_at is not None and disarmed_at is not None:
        flight_dur = disarmed_at - armed_at
    elif armed_at is not None:
        flight_dur = duration - armed_at

    # Was this an airborne flight? (max alt > 2m)
    airborne = max_alt > 2.0

    return {
        'duration': duration,
        'flight_dur': flight_dur,
        'airborne': airborne,
        'max_alt': max_alt,
        'fw': fw_version,
        'glitch_msgs': glitch_msgs,
        'err_subsys11': err_subsys11,
        'sat_drop_events': drop_events,
        'xkf4_gps_bad_windows': xkf4_windows,
        'mean_sats': statistics.mean([s for _, s, _ in sat_history]) if sat_history else 0,
        'sample_count': len(sat_history),
    }


def main():
    files = sorted(set(glob.glob(os.path.join(FOLDER, '*.BIN')) + glob.glob(os.path.join(FOLDER, '*.bin'))))
    if not files:
        print(f"No bin files in {FOLDER}")
        return

    print(f"Scanning {len(files)} logs from {FOLDER}\n")
    print(f"{'Log':<14} {'Dur':>6} {'Air':>4} {'Alt':>5} {'Sats':>5} {'GPSGlitch':>9} {'SatDrops':>8} {'EKFRejWin':>9} {'Worst drop':>11}")
    print('-' * 100)

    summary = []
    total_t = time.time()
    for f in files:
        t0 = time.time()
        r = analyze_log(f)
        elapsed = time.time() - t0
        name = os.path.basename(f).replace('.BIN','').replace('.bin','')
        if 'error' in r:
            print(f"{name:<14} ERROR: {r['error']}")
            continue
        worst_drop = "-"
        if r['sat_drop_events']:
            for ev in r['sat_drop_events']:
                mx = ev[0][1]
                mn = min(d[2] for d in ev)
                drop_size = mx - mn
                if worst_drop == "-" or drop_size > worst_drop[0]:
                    worst_drop = (drop_size, mx, mn)
            worst_drop_str = f"{worst_drop[1]}->{worst_drop[2]}"
        else:
            worst_drop_str = '-'
        air = 'YES' if r['airborne'] else 'no'
        print(f"{name:<14} {r['duration']:6.0f} {air:>4} {r['max_alt']:5.1f} {r['mean_sats']:5.1f} {len(r['glitch_msgs']):>9} {len(r['sat_drop_events']):>8} {len(r['xkf4_gps_bad_windows']):>9} {worst_drop_str:>11}  ({elapsed:.1f}s)")
        summary.append((name, r))

    # Aggregate
    print('\n=== AGGREGATE ===')
    total_logs = len(summary)
    airborne_logs = [s for s in summary if s[1]['airborne']]
    logs_with_glitch_msg = [s for s in summary if s[1]['glitch_msgs']]
    logs_with_drops = [s for s in summary if s[1]['sat_drop_events']]
    logs_with_xkf4 = [s for s in summary if s[1]['xkf4_gps_bad_windows']]
    logs_with_any = [s for s in summary if s[1]['glitch_msgs'] or s[1]['sat_drop_events'] or s[1]['xkf4_gps_bad_windows']]
    total_flight_dur = sum(s[1]['flight_dur'] or 0 for s in airborne_logs)
    total_glitches = sum(len(s[1]['glitch_msgs']) for s in summary)
    total_drops = sum(len(s[1]['sat_drop_events']) for s in summary)

    print(f"Total logs: {total_logs}")
    print(f"Airborne logs (alt > 2m): {len(airborne_logs)}")
    print(f"Total airborne flight time: {total_flight_dur/60:.1f} min")
    print(f"Logs with 'GPS Glitch' MSG: {len(logs_with_glitch_msg)}/{total_logs} ({len(logs_with_glitch_msg)/total_logs*100:.0f}%)")
    print(f"Logs with sat drop events (>={DROP_THRESH} sat drop): {len(logs_with_drops)}/{total_logs} ({len(logs_with_drops)/total_logs*100:.0f}%)")
    print(f"Logs with EKF GPS rejection windows: {len(logs_with_xkf4)}/{total_logs} ({len(logs_with_xkf4)/total_logs*100:.0f}%)")
    print(f"Logs with ANY interference fingerprint: {len(logs_with_any)}/{total_logs} ({len(logs_with_any)/total_logs*100:.0f}%)")
    print(f"Total glitch messages across all logs: {total_glitches}")
    print(f"Total sat drop events across all logs: {total_drops}")

    # Detail each affected log
    if logs_with_any:
        print('\n=== AFFECTED LOGS DETAIL ===')
        for name, r in logs_with_any:
            print(f"\n{name}: dur={r['duration']:.0f}s, alt_max={r['max_alt']:.1f}m, mean_sats={r['mean_sats']:.1f}")
            if r['glitch_msgs']:
                print(f"  Glitch messages ({len(r['glitch_msgs'])}):")
                for rt, txt in r['glitch_msgs'][:6]:
                    print(f"    t={rt:7.1f}s  {txt}")
                if len(r['glitch_msgs']) > 6:
                    print(f"    ...({len(r['glitch_msgs']) - 6} more)")
            if r['sat_drop_events']:
                print(f"  Sat drop events ({len(r['sat_drop_events'])}):")
                for ev in r['sat_drop_events'][:6]:
                    t_start = ev[0][0]
                    t_end = ev[-1][0]
                    mx = ev[0][1]
                    mn = min(d[2] for d in ev)
                    print(f"    t={t_start:7.1f}-{t_end:.1f}s  sats {mx} -> {mn}  ({len(ev)} samples)")
                if len(r['sat_drop_events']) > 6:
                    print(f"    ...({len(r['sat_drop_events']) - 6} more)")
            if r['xkf4_gps_bad_windows']:
                print(f"  EKF GPS-bad windows ({len(r['xkf4_gps_bad_windows'])}):")
                for w in r['xkf4_gps_bad_windows'][:6]:
                    print(f"    t={w[0]:7.1f}-{w[1]:.1f}s  ({w[1]-w[0]:.1f}s)")
                if len(r['xkf4_gps_bad_windows']) > 6:
                    print(f"    ...({len(r['xkf4_gps_bad_windows']) - 6} more)")

    # Per-flight-hour rate
    if total_flight_dur > 0:
        glitch_rate = total_glitches / (total_flight_dur / 3600)
        drop_rate = total_drops / (total_flight_dur / 3600)
        print(f"\nGlitch rate: {glitch_rate:.2f}/hour of flight")
        print(f"Sat drop event rate: {drop_rate:.2f}/hour of flight")

    print(f"\nTotal scan time: {time.time()-total_t:.1f}s")
    return summary


if __name__ == '__main__':
    main()
