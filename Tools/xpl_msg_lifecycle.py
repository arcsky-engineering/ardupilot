#!/usr/bin/env python3
"""Trace who's enabling and disabling messages over the full capture.

For each "new" message (NAV-SAT, NAV-SIG, NAV-CLOCK, NAV-COV):
  - Plot gaps in actual reception (the "GPS panel goes empty" effect)
  - List every host CFG-MSG command targeting it
  - Mark each as enable / disable / poll
  - Try to attribute who sent each command:
      * GNSSPeriph (predictable 50-sec cycle, 8-byte CFG-MSG, rate=0 on most messages)
      * u-center (off-cycle, 3-byte CFG-MSG often)
"""
import os
import struct
from collections import Counter

FILE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-185749.ubx"

MSG = {
    (0x01, 0x07): "NAV-PVT", (0x01, 0x04): "NAV-DOP", (0x01, 0x20): "NAV-TIMEGPS",
    (0x01, 0x22): "NAV-CLOCK", (0x01, 0x35): "NAV-SAT", (0x01, 0x43): "NAV-SIG",
    (0x01, 0x36): "NAV-COV", (0x01, 0x61): "NAV-EOE", (0x01, 0x03): "NAV-STATUS",
    (0x01, 0x14): "NAV-HPPOSLLH", (0x01, 0x02): "NAV-POSLLH", (0x01, 0x12): "NAV-VELNED",
    (0x02, 0x15): "RXM-RAWX", (0x02, 0x13): "RXM-SFRBX", (0x02, 0x31): "RXM-INTF",
    (0x05, 0x00): "ACK-ACK", (0x05, 0x01): "ACK-NAK",
    (0x06, 0x00): "CFG-PRT", (0x06, 0x01): "CFG-MSG", (0x06, 0x04): "CFG-RST",
    (0x06, 0x06): "CFG-DAT", (0x06, 0x08): "CFG-RATE", (0x06, 0x09): "CFG-CFG",
    (0x06, 0x13): "CFG-ANT", (0x06, 0x24): "CFG-NAV5", (0x06, 0x31): "CFG-TP5",
    (0x06, 0x3e): "CFG-GNSS",
    (0x06, 0x8a): "CFG-VALSET", (0x06, 0x8b): "CFG-VALGET", (0x06, 0x8c): "CFG-VALDEL",
    (0x0a, 0x04): "MON-VER", (0x0a, 0x09): "MON-HW", (0x0a, 0x0b): "MON-IO",
    (0x0a, 0x36): "MON-COMMS", (0x0a, 0x31): "MON-SPAN",
    (0x0d, 0x03): "TIM-TM2",
}

NEW_MSGS = [(0x01, 0x35, "NAV-SAT"), (0x01, 0x43, "NAV-SIG"),
            (0x01, 0x22, "NAV-CLOCK"), (0x01, 0x36, "NAV-COV")]


def iter_ubx(data):
    i = 0
    while i < len(data) - 7:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls = data[i + 2]; mid = data[i + 3]
            length = data[i + 4] | (data[i + 5] << 8)
            if i + 8 + length > len(data):
                i += 1; continue
            payload = data[i + 6:i + 6 + length]
            ck_a = ck_b = 0
            for b in data[i + 2:i + 6 + length]:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if data[i + 6 + length] == ck_a and data[i + 7 + length] == ck_b:
                yield i, cls, mid, payload
                i += 8 + length
                continue
        i += 1


with open(FILE, "rb") as f:
    data = f.read()

print(f"=== FILE: {os.path.basename(FILE)} ({len(data):,} bytes) ===\n")

records = []
for off, cls, mid, pl in iter_ubx(data):
    name = MSG.get((cls, mid), f"UNK_{cls:02x}{mid:02x}")
    records.append((off, name, pl))

# Inventory
print(f"=== INVENTORY ({len(records)} total) ===")
ctr = Counter(r[1] for r in records)
for n, c in ctr.most_common(20):
    print(f"  {n:<15} {c}")

# Index NAV-PVT for time
pvt_anchors = []
for off, name, pl in records:
    if name == "NAV-PVT" and len(pl) >= 92:
        iTOW = struct.unpack("<I", pl[0:4])[0]
        pvt_anchors.append((off, iTOW))

t_start = pvt_anchors[0][1] / 1000


def off_to_t(off):
    if off <= pvt_anchors[0][0]:
        return pvt_anchors[0][1] / 1000.0
    if off >= pvt_anchors[-1][0]:
        return pvt_anchors[-1][1] / 1000.0
    lo, hi = 0, len(pvt_anchors) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if pvt_anchors[mid][0] <= off:
            lo = mid
        else:
            hi = mid
    o0, t0 = pvt_anchors[lo]
    o1, t1 = pvt_anchors[hi]
    if o1 == o0: return t0 / 1000.0
    frac = (off - o0) / (o1 - o0)
    return (t0 + frac * (t1 - t0)) / 1000.0


# === For each new message, find ALL host CFG-MSG commands targeting it ===
def decode_cfg_msg(pl):
    if len(pl) == 2:
        return ("POLL", pl[0], pl[1], None)
    if len(pl) == 3:
        return ("SET-CURPORT", pl[0], pl[1], [pl[2]])
    if len(pl) == 8:
        return ("SET-ALLPORTS", pl[0], pl[1], [pl[2], pl[3], pl[4], pl[5], pl[6], pl[7]])
    return ("OTHER", None, None, None)


print(f"\n=== CFG-MSG commands targeting NEW messages (NAV-SAT/SIG/CLOCK/COV) ===")
print(f"  {'cap_time':>9}  {'form':>14}  {'target':<12}  {'rates'}")
target_cmds = {n[2]: [] for n in NEW_MSGS}
for off, name, pl in records:
    if name != "CFG-MSG":
        continue
    form, tgt_cls, tgt_id, rates = decode_cfg_msg(pl)
    if tgt_cls is None: continue
    target_name = MSG.get((tgt_cls, tgt_id), f"UNK_{tgt_cls:02x}{tgt_id:02x}")
    if target_name not in [n[2] for n in NEW_MSGS]:
        continue
    t = off_to_t(off) - t_start
    rates_str = str(rates) if rates is not None else "(poll)"
    print(f"  +{t:>7.2f}s  {form:>14}  {target_name:<12}  {rates_str}")
    target_cmds[target_name].append((t, form, rates))

# === Gaps in actual reception for each new message ===
print(f"\n=== Reception gaps for each NEW message (>2s without a message) ===")
for cls_b, mid_b, name in NEW_MSGS:
    timestamps = []
    for off, n, pl in records:
        if n != name: continue
        t = off_to_t(off) - t_start
        timestamps.append(t)
    if not timestamps:
        print(f"\n  {name}: NEVER APPEARED")
        continue
    print(f"\n  {name}: total {len(timestamps)} packets")
    print(f"    First @ +{timestamps[0]:.1f}s, last @ +{timestamps[-1]:.1f}s")
    gaps = []
    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i-1]
        if dt > 2.0:
            gaps.append((timestamps[i-1], timestamps[i], dt))
    if gaps:
        print(f"    {len(gaps)} gaps > 2s:")
        for start, end, dt in gaps[:20]:
            print(f"      gap @ +{start:>6.1f}s -> +{end:>6.1f}s  ({dt:>5.1f}s)")
        if len(gaps) > 20:
            print(f"      ... ({len(gaps) - 20} more gaps)")

# === The 50-second poll cycle pattern — check if it disables NEW messages ===
# Find every CFG-MSG that sets rate=0 for ANY message, plot the timeline.
print(f"\n=== Host commands that EXPLICITLY DISABLE any message (rate=0 on UART1) ===")
print(f"  {'time':>9}  {'target':<14}  {'form':>14}  rates")
disable_targets = Counter()
for off, name, pl in records:
    if name != "CFG-MSG":
        continue
    form, tgt_cls, tgt_id, rates = decode_cfg_msg(pl)
    if tgt_cls is None or rates is None: continue
    # Disable = all zeros, or rate=0 on UART1 (index 1 in 8-byte form, or single value in 3-byte form)
    is_disable = False
    if form == "SET-ALLPORTS":
        is_disable = rates[1] == 0  # rate on UART1 = 0
    elif form == "SET-CURPORT":
        is_disable = rates[0] == 0
    if not is_disable:
        continue
    target_name = MSG.get((tgt_cls, tgt_id), f"UNK_{tgt_cls:02x}{tgt_id:02x}")
    disable_targets[target_name] += 1
print(f"  Top disable targets (over whole capture):")
for tgt, cnt in disable_targets.most_common(15):
    print(f"    {tgt:<14}  x{cnt}")
