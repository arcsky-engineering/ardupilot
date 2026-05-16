#!/usr/bin/env python3
"""Quick hover-test analysis: compass, EKF, GPS health summary."""
from pymavlink import mavutil
import math, statistics
from collections import defaultdict

LOG = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026-05-09 11-36-54.bin"
m = mavutil.mavlink_connection(LOG)

t0 = None
gps_q = []
gpa_q = []
mag = {0:[], 1:[], 2:[]}
xkf3 = {0:[], 1:[], 2:[]}
xkf4 = {0:[], 1:[], 2:[]}
xkf5 = {0:[], 1:[], 2:[]}
bat = defaultdict(list)
vibe = []
att = []
rcou = []
params = {}
pos = []

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
        total = math.sqrt(msg.MagX**2 + msg.MagY**2 + msg.MagZ**2)
        mag.setdefault(msg.I,[]).append((rt, total, msg.MagX, msg.MagY, msg.MagZ, msg.Health))
    elif t == 'XKF3':
        xkf3.setdefault(msg.C,[]).append((rt, msg.IVN, msg.IVE, msg.IVD, msg.IPN, msg.IPE, msg.IMX, msg.IMY, msg.IMZ, msg.IYAW))
    elif t == 'XKF4':
        xkf4.setdefault(msg.C,[]).append((rt, msg.SV, msg.SP, msg.SH, msg.SM, msg.SVT, msg.FS, msg.GPS, msg.PI))
    elif t == 'XKF5':
        xkf5.setdefault(msg.C,[]).append((rt, msg.HAGL, msg.Herr, msg.eAng, msg.eVel, msg.ePos))
    elif t == 'BAT':
        bat[msg.Inst].append((rt, msg.V, msg.A, msg.Temp))
    elif t == 'VIBE' and msg.IMU == 0:
        vibe.append((rt, msg.VibeX, msg.VibeY, msg.VibeZ, msg.Clip))
    elif t == 'ATT':
        att.append((rt, msg.Roll, msg.Pitch, msg.Yaw, msg.DesRoll, msg.DesPitch, msg.DesYaw))
    elif t == 'RCOU':
        rcou.append((rt, msg.C1, msg.C2, msg.C3, msg.C4))
    elif t == 'POS':
        pos.append((rt, msg.Lat, msg.Lng, msg.RelHomeAlt))
    elif t == 'PARM':
        params[msg.Name] = msg.Value

hover_lo, hover_hi = 30, 1280

def corr(xs, ys):
    n=len(xs)
    if n<2: return 0
    mx=sum(xs)/n; my=sum(ys)/n
    num=sum((xs[i]-mx)*(ys[i]-my) for i in range(n))
    dx=math.sqrt(sum((xs[i]-mx)**2 for i in range(n)))
    dy=math.sqrt(sum((ys[i]-my)**2 for i in range(n)))
    return num/(dx*dy) if dx*dy>0 else 0

def aligned(rt_a, vals_a, rt_b, vals_b, t_lo, t_hi):
    ba = defaultdict(list); bb = defaultdict(list)
    for r,v in zip(rt_a, vals_a):
        if t_lo<=r<=t_hi: ba[int(r)].append(v)
    for r,v in zip(rt_b, vals_b):
        if t_lo<=r<=t_hi: bb[int(r)].append(v)
    ks = sorted(set(ba) & set(bb))
    return [statistics.mean(ba[k]) for k in ks], [statistics.mean(bb[k]) for k in ks]

# Altitude
hover_alts = [a for r, lat, lng, a in pos if hover_lo <= r <= hover_hi]
print(f"Hover window: {hover_lo}s to {hover_hi}s ({(hover_hi-hover_lo)/60:.1f} min)")
print(f"Altitude AGL: mean={statistics.mean(hover_alts):.1f}m  min={min(hover_alts):.1f}m  max={max(hover_alts):.1f}m  sigma={statistics.pstdev(hover_alts):.2f}m")

# GPS quality
print("\n=== GPS quality during hover ===")
sats_h = [s for r, s, hd, sp in gps_q if hover_lo <= r <= hover_hi]
hdops_h = [hd for r, s, hd, sp in gps_q if hover_lo <= r <= hover_hi]
hacc_h = [h for r, h, v, s in gpa_q if hover_lo <= r <= hover_hi]
sacc_h = [s for r, h, v, s in gpa_q if hover_lo <= r <= hover_hi]
print(f"  Sats:  mean={statistics.mean(sats_h):.1f}  min={min(sats_h)}  max={max(sats_h)}  sigma={statistics.pstdev(sats_h):.1f}")
print(f"  HDop:  mean={statistics.mean(hdops_h):.2f}  min={min(hdops_h):.2f}  max={max(hdops_h):.2f}")
print(f"  HAcc:  mean={statistics.mean(hacc_h):.2f}m  min={min(hacc_h):.2f}m  max={max(hacc_h):.2f}m")
print(f"  SAcc:  mean={statistics.mean(sacc_h):.2f}m/s  max={max(sacc_h):.2f}m/s")

# Sat dropouts
sats_mean = statistics.mean(sats_h)
print(f"\n=== GPS dips: sat count below {sats_mean-5:.0f} (mean-5) or HAcc>1.0 ===")
gpa_lookup = {round(r,1): (h, v, s) for r, h, v, s in gpa_q}
dips = []
for r, sat, hd, sp in gps_q:
    if hover_lo <= r <= hover_hi:
        ha = gpa_lookup.get(round(r,1), (0,0,0))
        if sat < sats_mean - 5 or ha[0] > 1.0:
            dips.append((r, sat, hd, ha[0], ha[2]))
print(f"  Total dip samples: {len(dips)}")
if dips:
    print(f"  First 15:")
    for r, s, hd, ha, sa in dips[:15]:
        print(f"    t={r:7.1f}  sats={s}  HDop={hd:.2f}  HAcc={ha:.2f}  SAcc={sa:.2f}")

# Compass
print("\n=== Compass total field stability during hover ===")
for I in [0,1,2]:
    if not mag[I]: continue
    win = [t for r, t, _, _, _, h in mag[I] if hover_lo <= r <= hover_hi]
    if not win: continue
    mn = statistics.mean(win); sd = statistics.pstdev(win)
    used = (I==0 and params.get('COMPASS_USE',0)==1) or (I==1 and params.get('COMPASS_USE2',0)==1) or (I==2 and params.get('COMPASS_USE3',0)==1)
    healths = [h for r, t, _, _, _, h in mag[I] if hover_lo <= r <= hover_hi]
    bad = sum(1 for h in healths if h != 1)
    print(f"  Compass {I}: mean={mn:.0f} mG  sigma={sd:.1f} ({sd/mn*100:.1f}%)  range=[{min(win):.0f}..{max(win):.0f}]  Used={used}  unhealthy={bad}/{len(healths)}")

# EKF mag innovations
print("\n=== EKF mag innovations (core 0) during hover ===")
imxs, imys, imzs, iyaws = [], [], [], []
for r, ivn, ive, ivd, ipn, ipe, imx, imy, imz, iyaw in xkf3[0]:
    if hover_lo <= r <= hover_hi:
        imxs.append(imx); imys.append(imy); imzs.append(imz); iyaws.append(iyaw)
print(f"  IMX: mean={statistics.mean(imxs):+.2f}  sigma={statistics.pstdev(imxs):.1f}  max|x|={max(abs(x) for x in imxs)}")
print(f"  IMY: mean={statistics.mean(imys):+.2f}  sigma={statistics.pstdev(imys):.1f}  max|x|={max(abs(x) for x in imys)}")
print(f"  IMZ: mean={statistics.mean(imzs):+.2f}  sigma={statistics.pstdev(imzs):.1f}  max|x|={max(abs(x) for x in imzs)}")
print(f"  IYAW: mean={statistics.mean(iyaws):+.4f} rad  sigma={statistics.pstdev(iyaws):.4f}  max|x|={max(abs(x) for x in iyaws):.3f}")

# EKF velocity/pos innovations
print("\n=== EKF velocity/position innovations (core 0) during hover ===")
ivns,ives,ivds,ipns,ipes = [], [], [], [], []
for r, ivn, ive, ivd, ipn, ipe, imx, imy, imz, iyaw in xkf3[0]:
    if hover_lo <= r <= hover_hi:
        ivns.append(ivn); ives.append(ive); ivds.append(ivd); ipns.append(ipn); ipes.append(ipe)
print(f"  IVN: mean={statistics.mean(ivns):+.3f} m/s  sigma={statistics.pstdev(ivns):.3f}  max|x|={max(abs(x) for x in ivns):.2f}")
print(f"  IVE: mean={statistics.mean(ives):+.3f} m/s  sigma={statistics.pstdev(ives):.3f}  max|x|={max(abs(x) for x in ives):.2f}")
print(f"  IVD: mean={statistics.mean(ivds):+.3f} m/s  sigma={statistics.pstdev(ivds):.3f}  max|x|={max(abs(x) for x in ivds):.2f}")
print(f"  IPN: mean={statistics.mean(ipns):+.3f} m    sigma={statistics.pstdev(ipns):.3f}  max|x|={max(abs(x) for x in ipns):.2f}")
print(f"  IPE: mean={statistics.mean(ipes):+.3f} m    sigma={statistics.pstdev(ipes):.3f}  max|x|={max(abs(x) for x in ipes):.2f}")

# EKF test ratios
print("\n=== EKF test ratios (core 0) during hover (rejection threshold = 1.0) ===")
svs, sps, shs, sms = [], [], [], []
fs_count, gps_count = 0, 0
for r, sv, sp, sh, sm, svt, fs, gps_, pi in xkf4[0]:
    if hover_lo <= r <= hover_hi:
        svs.append(sv); sps.append(sp); shs.append(sh); sms.append(sm)
        if fs: fs_count += 1
        if gps_: gps_count += 1
print(f"  SV (vel):  mean={statistics.mean(svs):.3f}  max={max(svs):.3f}  count_above_0.5={sum(1 for x in svs if x>0.5)}")
print(f"  SP (pos):  mean={statistics.mean(sps):.3f}  max={max(sps):.3f}  count_above_0.5={sum(1 for x in sps if x>0.5)}")
print(f"  SH (hgt):  mean={statistics.mean(shs):.3f}  max={max(shs):.3f}  count_above_0.5={sum(1 for x in shs if x>0.5)}")
print(f"  SM (mag):  mean={statistics.mean(sms):.3f}  max={max(sms):.3f}  count_above_0.5={sum(1 for x in sms if x>0.5)}")
print(f"  FS flag set: {fs_count}/{len(svs)} samples")
print(f"  GPS bad flag set: {gps_count}/{len(svs)} samples ({gps_count/len(svs)*100:.1f}%)")

# Hover throttle
print("\n=== Motor outputs during hover ===")
m1s = [c1 for r, c1, c2, c3, c4 in rcou if hover_lo <= r <= hover_hi]
m2s = [c2 for r, c1, c2, c3, c4 in rcou if hover_lo <= r <= hover_hi]
m3s = [c3 for r, c1, c2, c3, c4 in rcou if hover_lo <= r <= hover_hi]
m4s = [c4 for r, c1, c2, c3, c4 in rcou if hover_lo <= r <= hover_hi]
mavgs = [statistics.mean(m1s), statistics.mean(m2s), statistics.mean(m3s), statistics.mean(m4s)]
print(f"  M1: mean={mavgs[0]:.0f}  M2: mean={mavgs[1]:.0f}  M3: mean={mavgs[2]:.0f}  M4: mean={mavgs[3]:.0f}")
print(f"  Hover avg: {sum(mavgs)/4:.0f} PWM  spread across motors: {max(mavgs)-min(mavgs):.0f}")

# Vibration
print("\n=== Vibration during hover (IMU 0) ===")
vxs = [vx for r, vx, vy, vz, c in vibe if hover_lo <= r <= hover_hi]
vys = [vy for r, vx, vy, vz, c in vibe if hover_lo <= r <= hover_hi]
vzs = [vz for r, vx, vy, vz, c in vibe if hover_lo <= r <= hover_hi]
clips = sum(c for r, vx, vy, vz, c in vibe if hover_lo <= r <= hover_hi)
print(f"  VibeX: mean={statistics.mean(vxs):.2f}  max={max(vxs):.2f}  m/s/s")
print(f"  VibeY: mean={statistics.mean(vys):.2f}  max={max(vys):.2f}")
print(f"  VibeZ: mean={statistics.mean(vzs):.2f}  max={max(vzs):.2f}")
print(f"  Clip events: {clips}")

# Battery
print("\n=== Battery during hover ===")
for I in [0,1]:
    if not bat[I]: continue
    vs = [v for r, v, a, t in bat[I] if hover_lo <= r <= hover_hi]
    asg = [a for r, v, a, t in bat[I] if hover_lo <= r <= hover_hi]
    ts = [t for r, v, a, t in bat[I] if hover_lo <= r <= hover_hi]
    print(f"  Bat{I}: V mean={statistics.mean(vs):.2f}V  V min={min(vs):.2f}V  A mean={statistics.mean(asg):.2f}A  A max={max(asg):.2f}A  Temp mean={statistics.mean(ts):.1f}C  max={max(ts):.1f}C")

# Compass-current correlation
print("\n=== Compass-current correlation during hover ===")
bat0_t = [r for r, v, a, t in bat[0]]
bat0_a = [a for r, v, a, t in bat[0]]
bat1_lookup = {int(r): a for r, v, a, t in bat[1]}
total_t = bat0_t
total_a = [bat0_a[i] + bat1_lookup.get(int(bat0_t[i]),0) for i in range(len(bat0_t))]

for I in [0,1,2]:
    if not mag[I]: continue
    rt_m = [r for r,_,_,_,_,_ in mag[I]]
    v_m  = [t for _,t,_,_,_,_ in mag[I]]
    xs, ys = aligned(total_t, total_a, rt_m, v_m, hover_lo, hover_hi)
    if len(xs) > 5:
        print(f"  Compass {I}: r={corr(xs,ys):+.3f}  current range={min(xs):.1f}-{max(xs):.1f}A  mag range={min(ys):.0f}-{max(ys):.0f}mG")

# GPS-current correlation
hacc_t = [r for r, h, v, s in gpa_q]
hacc_v = [h for r, h, v, s in gpa_q]
sat_t = [r for r, s, hd, sp in gps_q]
sat_v = [s for r, s, hd, sp in gps_q]
xs, ys = aligned(total_t, total_a, hacc_t, hacc_v, hover_lo, hover_hi)
if len(xs) > 5: print(f"\n  GPS HAcc vs current:  r={corr(xs, ys):+.3f}")
xs, ys = aligned(total_t, total_a, sat_t, sat_v, hover_lo, hover_hi)
if len(xs) > 5: print(f"  GPS sats  vs current:  r={corr(xs, ys):+.3f}")

# Position drift
print("\n=== Position hold quality (Loiter drift) ===")
hover_pos = [(r, lat, lng, a) for r, lat, lng, a in pos if hover_lo <= r <= hover_hi]
if hover_pos:
    lats = [p[1] for p in hover_pos]
    lngs = [p[2] for p in hover_pos]
    alts = [p[3] for p in hover_pos]
    lat_ref = sum(lats)/len(lats)
    lng_ref = sum(lngs)/len(lngs)
    m_per_deg_lat = 111320.0
    m_per_deg_lng = 111320.0 * math.cos(math.radians(lat_ref))
    n_offsets = [(l - lat_ref) * m_per_deg_lat for l in lats]
    e_offsets = [(l - lng_ref) * m_per_deg_lng for l in lngs]
    print(f"  N drift: range={max(n_offsets)-min(n_offsets):.2f}m  sigma={statistics.pstdev(n_offsets):.2f}m")
    print(f"  E drift: range={max(e_offsets)-min(e_offsets):.2f}m  sigma={statistics.pstdev(e_offsets):.2f}m")
    print(f"  Alt:     range={max(alts)-min(alts):.2f}m  sigma={statistics.pstdev(alts):.2f}m  mean={statistics.mean(alts):.2f}m")

# EKF SOLN errors
print("\n=== EKF solution error estimates (core 0) during hover ===")
eangs, evels, eposs = [], [], []
for r, hagl, herr, ea, ev, ep in xkf5[0]:
    if hover_lo <= r <= hover_hi:
        eangs.append(ea); evels.append(ev); eposs.append(ep)
if eangs:
    print(f"  eAng: mean={statistics.mean(eangs):.4f}  max={max(eangs):.4f}")
    print(f"  eVel: mean={statistics.mean(evels):.4f}  max={max(evels):.4f}")
    print(f"  ePos: mean={statistics.mean(eposs):.4f}  max={max(eposs):.4f}")

# Attitude tracking error
print("\n=== Attitude tracking quality (DesRoll-Roll, etc) ===")
roll_err = [(r-dr) for t, r, p, y, dr, dp, dy in att if hover_lo <= t <= hover_hi]
pitch_err = [(p-dp) for t, r, p, y, dr, dp, dy in att if hover_lo <= t <= hover_hi]
yaw_err = [(y-dy) for t, r, p, y, dr, dp, dy in att if hover_lo <= t <= hover_hi]
yaw_err = [((e + 540) % 360) - 180 for e in yaw_err]
print(f"  Roll error:  mean={statistics.mean(roll_err):+.2f}deg  sigma={statistics.pstdev(roll_err):.2f}  max|x|={max(abs(x) for x in roll_err):.2f}")
print(f"  Pitch error: mean={statistics.mean(pitch_err):+.2f}deg  sigma={statistics.pstdev(pitch_err):.2f}  max|x|={max(abs(x) for x in pitch_err):.2f}")
print(f"  Yaw error:   mean={statistics.mean(yaw_err):+.2f}deg  sigma={statistics.pstdev(yaw_err):.2f}  max|x|={max(abs(x) for x in yaw_err):.2f}")

# Compass setup
print("\n=== Compass config ===")
for k in ['COMPASS_USE','COMPASS_USE2','COMPASS_USE3','COMPASS_PRIO1_ID','COMPASS_PRIO2_ID','COMPASS_PRIO3_ID','COMPASS_DEV_ID','COMPASS_DEV_ID2','COMPASS_DEV_ID3','COMPASS_MOTCT','COMPASS_MOT_X','COMPASS_MOT_Y','COMPASS_MOT_Z','COMPASS_OFS_X','COMPASS_OFS_Y','COMPASS_OFS_Z','COMPASS_OFS2_X','COMPASS_OFS2_Y','COMPASS_OFS2_Z']:
    if k in params: print(f"  {k:20s} = {params[k]}")

# GPS config
print("\n=== GPS config ===")
for k in ['GPS1_TYPE','GPS2_TYPE','GPS_AUTO_SWITCH','GPS_PRIMARY','GPS1_POS_X','GPS1_POS_Y','GPS1_POS_Z']:
    if k in params: print(f"  {k:20s} = {params[k]}")
