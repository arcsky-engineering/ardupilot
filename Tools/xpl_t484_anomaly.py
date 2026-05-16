#!/usr/bin/env python3
"""Look at EVERYTHING happening around t=484s to find what else changes at that moment.
If a non-GPS subsystem also has an anomaly at the same time, that's the trigger."""
from pymavlink import mavutil
import os
from collections import defaultdict
import statistics

LOG = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026 02 13 Logs to Date-20260511T185246Z-3-001\2026 02 13 Logs to Date\00000318.BIN"
T_CENTER = 485
T_WINDOW = (T_CENTER - 30, T_CENTER + 10)

m = mavutil.mavlink_connection(LOG)
t0 = None
# Collect everything in window
all_msgs_in_window = []
type_first_seen = {}
type_counts = defaultdict(int)
# Track all data fields per type in/around the window for anomaly detection
type_field_traces = defaultdict(lambda: defaultdict(list))

ALL_TYPES_TO_TRACE = set()  # populate as we see them
INTERESTING_TYPES = {'GPS', 'GPA', 'MAG', 'IMU', 'XKF1', 'XKF3', 'XKF4', 'XKF5', 'XKFS', 'BAT', 'BARO', 'ESC', 'ESCX', 'VIBE', 'POS', 'RCIN', 'RCOU', 'RFND', 'SURF', 'BCL', 'CAN', 'CANS', 'MNT', 'MCN', 'PM', 'POWR'}

while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    ts = msg._timestamp
    if t0 is None: t0 = ts
    rt = ts - t0
    t = msg.get_type()
    type_counts[t] += 1
    if t not in type_first_seen: type_first_seen[t] = rt
    # In the window: capture for anomaly detection
    if T_WINDOW[0] <= rt <= T_WINDOW[1]:
        # special messages always interesting
        if t in ['MSG', 'EV', 'ERR', 'CMD']:
            all_msgs_in_window.append((rt, t, msg.to_dict()))
        # trace numerical fields for sudden changes
        if t in INTERESTING_TYPES:
            d = msg.to_dict()
            for k, v in d.items():
                if isinstance(v, (int, float)) and k not in ('TimeUS', 'mavpackettype'):
                    type_field_traces[t][k].append((rt, v))

print(f"Log: {os.path.basename(LOG)}")
print(f"Window analyzed: t={T_WINDOW[0]} to t={T_WINDOW[1]} (glitch at t~{T_CENTER})")
print(f"\nMSG/EV/ERR/CMD messages in window:")
for rt, t, d in all_msgs_in_window:
    if t == 'MSG':
        print(f"  t={rt:7.2f}s  MSG: {d.get('Message', '')}")
    elif t == 'EV':
        print(f"  t={rt:7.2f}s  EV: Id={d.get('Id', '')}")
    elif t == 'ERR':
        print(f"  t={rt:7.2f}s  ERR: Subsys={d.get('Subsys', '')} ECode={d.get('ECode', '')}")
    elif t == 'CMD':
        print(f"  t={rt:7.2f}s  CMD: {d}")

# Look at every traced field. For each, compare value just before glitch (t<484) to just after (t>484)
# Look for fields with sudden change at t=484
GLITCH_T = T_CENTER - 1  # 484
print(f"\n\nAnomaly scan: fields that change abruptly at t~{GLITCH_T} (>3 sigma from baseline)")
print(f"{'Type':<8} {'Field':<14} {'before mean':>12} {'before std':>10} {'just after':>11} {'sigma':>6}")
for t, fields in sorted(type_field_traces.items()):
    for k, pts in sorted(fields.items()):
        before = [v for r, v in pts if r < GLITCH_T - 1]
        right_after = [v for r, v in pts if GLITCH_T <= r <= GLITCH_T + 3]
        if len(before) < 5 or len(right_after) < 1: continue
        b_mean = statistics.mean(before)
        b_std = statistics.pstdev(before)
        if b_std < 1e-6: b_std = 1e-6
        for v in right_after:
            sigma = abs(v - b_mean) / b_std
            if sigma > 8:  # significantly off
                print(f"  {t:<8} {k:<14} {b_mean:>12.3f} {b_std:>10.3f} {v:>11.3f} {sigma:>6.1f}")
                break
