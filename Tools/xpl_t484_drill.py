#!/usr/bin/env python3
"""Drill into what's happening at t=484s in the affected logs."""
from pymavlink import mavutil
import os

LOGS = [
    r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026 02 13 Logs to Date-20260511T185246Z-3-001\2026 02 13 Logs to Date\00000318.BIN",
    r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\2026 02 13 Logs to Date-20260511T185246Z-3-001\2026 02 13 Logs to Date\00000321.BIN",
]

for LOG in LOGS:
    name = os.path.basename(LOG)
    print(f"\n========== {name} ==========")
    m = mavutil.mavlink_connection(LOG)
    t0 = None
    msgs = []
    events = []
    modes = []
    arm_t = None
    takeoff_t = None
    # Things to look at around t=484
    gps_around = []  # (rt, sats, hdop)
    bat_around = []  # (rt, inst, A)
    rcin_around = []  # (rt, chs)
    rcou_around = []
    cmd_around = []  # mission commands

    while True:
        msg = m.recv_match(blocking=False)
        if msg is None: break
        ts = msg._timestamp
        if t0 is None: t0 = ts
        rt = ts - t0
        t = msg.get_type()
        if t == 'MSG':
            msgs.append((rt, msg.Message))
        elif t == 'EV':
            events.append((rt, msg.Id))
            if msg.Id == 10: arm_t = rt
            elif msg.Id == 56: takeoff_t = rt
        elif t == 'MODE':
            modes.append((rt, msg.Mode))
        elif t == 'GPS' and msg.I == 0 and 478 <= rt <= 492:
            gps_around.append((rt, msg.NSats, msg.HDop, msg.Spd))
        elif t == 'BAT' and 478 <= rt <= 492:
            bat_around.append((rt, msg.Inst, msg.V, msg.A))
        elif t == 'CMD':
            cmd_around.append((rt, msg.CNum if hasattr(msg, 'CNum') else 0, msg.CId))
        elif t == 'RCIN' and 480 <= rt <= 488 and len(rcin_around) < 50:
            rcin_around.append((rt, msg.C1 if hasattr(msg,'C1') else 0, msg.C2 if hasattr(msg,'C2') else 0, msg.C3 if hasattr(msg,'C3') else 0, msg.C5 if hasattr(msg,'C5') else 0, msg.C7 if hasattr(msg,'C7') else 0))

    print(f"\nArmed at: t={arm_t}")
    print(f"Takeoff at: t={takeoff_t}")
    print(f"\nMode changes:")
    for rt, mod in modes:
        print(f"  t={rt:7.1f}s  Mode={mod}")

    print(f"\nMSGs between t=475 and t=495:")
    for rt, txt in msgs:
        if 475 <= rt <= 495:
            print(f"  t={rt:7.1f}s  {txt}")

    print(f"\nAll MSGs containing 'GPS' or 'Glitch' or 'EKF':")
    for rt, txt in msgs:
        if any(k in txt for k in ['GPS', 'Glitch', 'EKF', 'compass']):
            print(f"  t={rt:7.1f}s  {txt}")

    print(f"\nGPS samples 478-492s:")
    for rt, sats, hd, spd in gps_around[:30]:
        print(f"  t={rt:7.2f}s  sats={sats}  HDop={hd:.2f}  Spd={spd:.2f}")

    # Battery context
    print(f"\nBattery currents at t=480-486s:")
    last = -1
    for rt, inst, v, a in bat_around:
        if rt - last >= 0.5:
            print(f"  t={rt:7.2f}s  Inst{inst}: V={v:.2f}V  A={a:.2f}A")
            last = rt
