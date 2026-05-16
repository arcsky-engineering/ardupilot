from pymavlink import mavutil
from collections import defaultdict
import statistics
LOG = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026-05-09 11-36-54.bin"
m = mavutil.mavlink_connection(LOG)
data = defaultdict(lambda: {'Temp':[], 'TChp':[], 'TCPU':[], 'TFET':[]})
while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    if msg.get_type() == 'BAT':
        d = data[msg.Inst]
        for k in ['Temp','TChp','TCPU','TFET']:
            d[k].append(getattr(msg, k))
for inst in sorted(data.keys()):
    print(f"=== Inst={inst} ===")
    for k in ['Temp','TChp','TCPU','TFET']:
        vs = data[inst][k]
        nz = [v for v in vs if v != 0]
        if nz:
            print(f"  {k:5s}: mean={statistics.mean(vs):.1f}  max={max(vs):.1f}  min={min(vs):.1f}  nonzero={len(nz)}/{len(vs)}")
        else:
            print(f"  {k:5s}: all zeros ({len(vs)} samples)")
