"""Quick triage of camera-trigger behavior in a dataflash log."""
import sys
import math
from collections import Counter
from pymavlink import mavutil

path = sys.argv[1]
m = mavutil.mavlink_connection(path, dialect="ardupilotmega")

params = {}
modes = []
cmds = []
cam_msgs = 0
trigger_msgs = 0
type_counts = Counter()
gps_samples = []
last_gps_t = 0
auto_segments = []
in_auto = False
auto_start = None
first_gps_print = True

n = 0
while True:
    msg = m.recv_match(blocking=False)
    if msg is None:
        break
    n += 1
    t = msg.get_type()
    type_counts[t] += 1

    if t == "PARM":
        if msg.Name.startswith("CAM") or msg.Name in ("WPNAV_SPEED","WPNAV_RADIUS"):
            params[msg.Name] = msg.Value
    elif t == "MODE":
        modes.append((msg.TimeUS, getattr(msg, "Mode", None), getattr(msg, "ModeNum", None)))
        is_auto = (getattr(msg, "ModeNum", -1) == 3)
        if is_auto and not in_auto:
            auto_start = msg.TimeUS
            in_auto = True
        elif not is_auto and in_auto:
            auto_segments.append((auto_start, msg.TimeUS))
            in_auto = False
    elif t == "CAM":
        cam_msgs += 1
    elif t == "TRIG":
        trigger_msgs += 1
    elif t == "CMD":
        cmds.append((msg.CNum, msg.CId, msg.Prm1, msg.Prm2, msg.Prm3, msg.Prm4, msg.Lat, msg.Lng, msg.Alt))
    elif t == "GPS":
        if first_gps_print:
            print(f"[debug first GPS] Lat={msg.Lat} Lng={msg.Lng} Alt={msg.Alt} Status={msg.Status}")
            first_gps_print = False
        if msg.TimeUS - last_gps_t > 200000:
            gps_samples.append((msg.TimeUS, msg.Lat, msg.Lng, msg.Alt, msg.Status))
            last_gps_t = msg.TimeUS

if in_auto and auto_start is not None:
    auto_segments.append((auto_start, modes[-1][0] if modes else auto_start))

print(f"=== {path} ===")
print(f"Total messages: {n}")
print()
print("--- Camera-relevant params ---")
for k in sorted(params):
    print(f"  {k} = {params[k]}")
print()
print(f"--- CAM log msgs: {cam_msgs}    TRIG log msgs: {trigger_msgs} ---")
print()
print("--- Mode timeline (first 30) ---")
mode_names = {0:"STAB",2:"ALT_HOLD",3:"AUTO",4:"GUIDED",5:"LOITER",6:"RTL",7:"CIRCLE",
              9:"LAND",16:"POSHOLD",17:"BRAKE"}
for tu, mn, mnum in modes[:30]:
    print(f"  t={tu/1e6:8.2f}s  mode={mn} ({mode_names.get(mnum, mnum)})")
print()
print(f"--- AUTO segments: {len(auto_segments)} ---")
for a, b in auto_segments:
    print(f"  {a/1e6:8.2f}s -> {b/1e6:8.2f}s   ({(b-a)/1e6:.1f}s)")
print()
print(f"--- CMD count: {len(cmds)} ---")
cmd_id_counts = Counter(c[1] for c in cmds)
print("  cmd id counts:")
for cid, cnt in sorted(cmd_id_counts.items(), key=lambda x: -x[1]):
    print(f"    {cid}: {cnt}")
print()
print("--- First 30 mission items in order ---")
cmds_ordered = sorted(cmds, key=lambda c: c[0])
for c in cmds_ordered[:30]:
    print(f"  CNum={c[0]:3d} CId={c[1]:4d}  P1={c[2]:.3f} P2={c[3]:.3f} P3={c[4]:.3f} P4={c[5]:.3f}  Lat={c[6]} Lng={c[7]} Alt={c[8]:.2f}")
print()

# Detect format of GPS lat/lng. If abs > 1000, it's int32 1e-7 deg; else float deg.
def in_auto_at(tu):
    for a, b in auto_segments:
        if a <= tu <= b:
            return True
    return False

if not gps_samples:
    print("No GPS samples")
else:
    sample = gps_samples[0]
    fmt_int = abs(sample[1]) > 1000
    print(f"--- GPS lat/lng format: {'int32 1e-7 deg' if fmt_int else 'float degrees'} ---")
    R = 6378137.0
    auto_pts = [(t,lat,lng,alt) for (t,lat,lng,alt,st) in gps_samples if in_auto_at(t) and st >= 3]
    print(f"  AUTO GPS samples: {len(auto_pts)}")
    if auto_pts:
        print(f"  first AUTO: lat={auto_pts[0][1]} lng={auto_pts[0][2]} alt={auto_pts[0][3]:.1f}")
        print(f"  last  AUTO: lat={auto_pts[-1][1]} lng={auto_pts[-1][2]} alt={auto_pts[-1][3]:.1f}")
    total_h = total_v = total_3d = 0.0
    max_alt = -1e9; min_alt = 1e9
    max_step_h = 0.0
    step_hist = []
    for i in range(1, len(auto_pts)):
        t0,la0,ln0,al0 = auto_pts[i-1]
        t1,la1,ln1,al1 = auto_pts[i]
        if t1 - t0 > 1_500_000:
            continue
        if fmt_int:
            la0d = la0 * 1e-7; la1d = la1 * 1e-7
            ln0d = ln0 * 1e-7; ln1d = ln1 * 1e-7
        else:
            la0d = la0; la1d = la1; ln0d = ln0; ln1d = ln1
        dlat = math.radians(la1d - la0d)
        dlng = math.radians(ln1d - ln0d) * math.cos(math.radians(la0d))
        dh = math.sqrt(dlat*dlat + dlng*dlng) * R
        dv = al1 - al0
        total_h += dh
        total_v += abs(dv)
        total_3d += math.sqrt(dh*dh + dv*dv)
        if dh > max_step_h: max_step_h = dh
        step_hist.append(dh)
        if al1 > max_alt: max_alt = al1
        if al1 < min_alt: min_alt = al1
    print(f"--- AUTO travel ({len(step_hist)} samples used) ---")
    print(f"  total horizontal travel: {total_h:.1f} m")
    print(f"  total |vertical| travel: {total_v:.1f} m")
    print(f"  total 3D travel:         {total_3d:.1f} m")
    print(f"  max single horizontal step (between 200ms samples): {max_step_h:.2f} m")
    print(f"  alt range: {min_alt:.1f} .. {max_alt:.1f} m  (span {max_alt-min_alt:.1f} m)")

print()
print("--- type counts (interesting) ---")
for t in ("CAM","TRIG","CMD","MISE","MSG","ERR","GPA","POS","ATT"):
    print(f"  {t}: {type_counts.get(t,0)}")

# Also pull MSG strings for any camera-related text
m2 = mavutil.mavlink_connection(path, dialect="ardupilotmega")
print()
print("--- MSG entries (first 60) ---")
nmsg = 0
while nmsg < 60:
    msg = m2.recv_match(type="MSG", blocking=False)
    if msg is None: break
    nmsg += 1
    print(f"  t={msg.TimeUS/1e6:8.2f}  {msg.Message}")
