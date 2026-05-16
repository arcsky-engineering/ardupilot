from pymavlink import mavutil
from collections import defaultdict
LOG = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026-05-09 11-36-54.bin"
m = mavutil.mavlink_connection(LOG)
seen = defaultdict(int)
sample = {}
import statistics
data = defaultdict(list)
while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    if msg.get_type() == 'BAT':
        seen[msg.Inst] += 1
        if msg.Inst not in sample:
            sample[msg.Inst] = msg.to_dict()
        data[msg.Inst].append((msg.V, msg.A, msg.Temp))

print("BAT instances seen:")
for inst, cnt in sorted(seen.items()):
    vs = [v for v,a,t in data[inst]]
    ams = [a for v,a,t in data[inst]]
    ts = [t for v,a,t in data[inst]]
    print(f"  Inst={inst}: count={cnt}  V mean={statistics.mean(vs):.2f}  A mean={statistics.mean(ams):.2f}  Temp mean={statistics.mean(ts):.1f}  Temp max={max(ts):.1f}")
