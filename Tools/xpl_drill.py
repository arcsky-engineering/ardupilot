from pymavlink import mavutil
import math, statistics
from collections import defaultdict

LOG = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026-05-09 11-36-54.bin"
m = mavutil.mavlink_connection(LOG)
t0 = None
xkf4_gps_events = []
mag0 = []
bat0 = []
bat1 = []
vibe_max = []
sats_low_window = []

while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    if t0 is None: t0 = msg._timestamp
    rt = msg._timestamp - t0
    if msg.get_type() == 'XKF4' and msg.C == 0 and msg.GPS == 1:
        xkf4_gps_events.append(rt)
    elif msg.get_type() == 'MAG' and msg.I == 0:
        total = math.sqrt(msg.MagX**2 + msg.MagY**2 + msg.MagZ**2)
        mag0.append((rt, total))
    elif msg.get_type() == 'BAT' and msg.Inst == 0:
        bat0.append((rt, msg.A, msg.V))
    elif msg.get_type() == 'BAT' and msg.Inst == 1:
        bat1.append((rt, msg.A, msg.V, msg.Temp))

# When was XKF4.GPS=1?
print(f"XKF4 GPS-bad samples: {len(xkf4_gps_events)}")
if xkf4_gps_events:
    # group consecutive
    groups = []
    g = [xkf4_gps_events[0]]
    for t in xkf4_gps_events[1:]:
        if t - g[-1] < 2.0:
            g.append(t)
        else:
            groups.append(g); g = [t]
    groups.append(g)
    for grp in groups:
        print(f"  GPS-bad window: {grp[0]:.1f}s to {grp[-1]:.1f}s ({len(grp)} samples)")

# Look at compass 0 vs current at high time resolution to understand the +0.879 correlation
# Bin both at 1s during hover
hover_lo, hover_hi = 30, 1280
bat0_a_bins = defaultdict(list)
bat1_a_bins = defaultdict(list)
for r, a, v in bat0:
    if hover_lo <= r <= hover_hi:
        bat0_a_bins[int(r)].append(a)
for r, a, v, t in bat1:
    if hover_lo <= r <= hover_hi:
        bat1_a_bins[int(r)].append(a)
mag0_bins = defaultdict(list)
for r, total in mag0:
    if hover_lo <= r <= hover_hi:
        mag0_bins[int(r)].append(total)

# Total current
total_curr = {}
for s in bat0_a_bins:
    total_curr[s] = statistics.mean(bat0_a_bins[s]) + (statistics.mean(bat1_a_bins[s]) if s in bat1_a_bins else 0)

# Look at the long-term trend: maybe it's actually a slow drift, not an instantaneous current effect
# Print mag0 vs total current every 60s
print(f"\n{'t(s)':>6} {'Bat0_A':>7} {'Bat1_A':>7} {'TotA':>6} {'mag0':>6} {'Bat1_T':>7} {'Bat0_V':>7}")
for s in range(30, 1280, 60):
    if s not in mag0_bins or s not in total_curr: continue
    b0 = statistics.mean(bat0_a_bins[s])
    b1 = statistics.mean(bat1_a_bins[s]) if s in bat1_a_bins else 0
    bt = total_curr[s]
    mag = statistics.mean(mag0_bins[s])
    bt1 = next((t for r,a,v,t in bat1 if int(r)==s), 0)
    bv0 = statistics.mean([v for r,a,v in bat0 if int(r)==s] or [0])
    print(f"{s:6d} {b0:7.1f} {b1:7.1f} {bt:6.1f} {mag:6.0f} {bt1:7.1f} {bv0:7.2f}")
