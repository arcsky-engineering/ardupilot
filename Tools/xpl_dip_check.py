from pymavlink import mavutil
LOG = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026-05-09 11-36-54.bin"
m = mavutil.mavlink_connection(LOG)
t0 = None
hits = []
while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    if t0 is None: t0 = msg._timestamp
    rt = msg._timestamp - t0
    if msg.get_type() == 'GPS' and msg.I == 0 and 30 <= rt <= 1280 and msg.NSats < 20:
        hits.append((rt, msg.NSats, msg.HDop, msg.Spd))
print(f"Samples with sats<20 during hover: {len(hits)}")
prev_t = -10
groups = []
g = []
for h in hits:
    if h[0] - prev_t > 1.0:
        if g: groups.append(g)
        g = []
    g.append(h)
    prev_t = h[0]
if g: groups.append(g)
print(f"Distinct events: {len(groups)}")
for grp in groups[:10]:
    print(f"  Event t={grp[0][0]:.1f}-{grp[-1][0]:.1f}s ({len(grp)} samples)  min_sats={min(g[1] for g in grp)}  max_HDop={max(g[2] for g in grp):.2f}")
