#!/usr/bin/env python3
"""Zoom into the major building-edge glitch windows from the 2026-05-15 flight.

For each event, show the second-by-second CTUN.Alt / CTUN.SAlt / SURF.TO
around the moment so we can see whether the drone actually followed the
terrain offset or got stuck.
"""
import sys
from pymavlink import mavutil

LOG = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/2026-05-15 11-43-30.bin"

# Events of interest (centred on the big >10m glitches)
EVENTS = [
    ("212.6s — first big drop over edge",      210, 230),
    ("221.4s — back over edge",                219, 235),
    ("238-250s — multiple oscillating jumps",  235, 255),
    ("312.1s — drop from 34->18m",             308, 320),
    ("331.8s — back from 18->34m",             328, 340),
    ("390-410s — descent + RFND OFF event",    390, 415),
]


def t_str(t_us):
    return f"{t_us / 1e6:8.2f}s"


def main():
    mlog = mavutil.mavlink_connection(LOG)
    ctun = []      # (t_us, sat, alt, dsat)
    surf = []      # (t_us, st, draw, dfilt, to)
    rfnd = []      # (t_us, dist, stat)
    psc = []       # (t_us, dvz, vz, pz, dpz)

    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        t = msg.get_type()
        if t == "CTUN":
            ctun.append((msg.TimeUS,
                         getattr(msg, "SAlt", None),
                         getattr(msg, "Alt", None),
                         getattr(msg, "DSAlt", None)))
        elif t == "SURF":
            if msg.I == 0:
                surf.append((msg.TimeUS, msg.St, msg.D, msg.FD, msg.TO))
        elif t == "RFND":
            inst = getattr(msg, "Instance", getattr(msg, "I", 0))
            if inst == 0:
                rfnd.append((msg.TimeUS, msg.Dist, msg.Stat))
        elif t == "PSC":
            psc.append((msg.TimeUS,
                        getattr(msg, "DVZ", None),
                        getattr(msg, "VZ", None),
                        getattr(msg, "PZ", None),
                        getattr(msg, "DPZ", None)))
        elif t == "PSCD":
            psc.append((msg.TimeUS,
                        getattr(msg, "DVD", None),
                        getattr(msg, "VD", None),
                        getattr(msg, "PD", None),
                        getattr(msg, "DPD", None)))

    for label, t0_s, t1_s in EVENTS:
        t0 = int(t0_s * 1e6)
        t1 = int(t1_s * 1e6)
        print()
        print("=" * 84)
        print(f"EVENT: {label}  ({t0_s}-{t1_s}s)")
        print("=" * 84)

        # CTUN at ~5Hz
        print("CTUN (5 Hz):  TimeUS    SAlt(rngdist)  Alt(above-home)  DSAlt(desired)")
        last = 0
        for t_us, sat, alt, dsat in ctun:
            if t_us < t0 or t_us > t1: continue
            if t_us - last < 200_000: continue
            s = f"{sat:7.2f}" if sat is not None else "  --   "
            a = f"{alt:7.2f}" if alt is not None else "  --   "
            d = f"{dsat:7.2f}" if dsat is not None else "  --   "
            print(f"  {t_str(t_us)}   {s}   {a}   {d}")
            last = t_us

        # SURF every transition
        print()
        print("SURF transitions:  St  Draw    Dfilt   TO")
        last_st = None
        last_to = None
        for t_us, st, draw, dfilt, to in surf:
            if t_us < t0 or t_us > t1: continue
            if st == last_st and abs((to or 0) - (last_to or 0)) < 0.5:
                continue
            flags = []
            if st & 0x01: flags.append("EN")
            if st & 0x02: flags.append("UNHEALTHY")
            if st & 0x04: flags.append("STALE")
            if st & 0x08: flags.append("GLITCH")
            f = ",".join(flags) if flags else "OFF"
            print(f"  {t_str(t_us)}  0x{st:02x} [{f:<25}]  Draw={draw:6.2f}  Dfilt={dfilt:6.2f}  TO={to:7.2f}")
            last_st, last_to = st, to

        # PSC at ~5Hz to see if vehicle tried to track the new terrain offset
        if psc:
            print()
            print("PSC (5 Hz):  DVZ(cmd vz)  VZ(actual)  PZ(actual pos)  DPZ(desired pos)")
            last = 0
            for t_us, dvz, vz, pz, dpz in psc:
                if t_us < t0 or t_us > t1: continue
                if t_us - last < 200_000: continue
                if any(v is None for v in (dvz, vz, pz, dpz)):
                    continue
                print(f"  {t_str(t_us)}  DVZ={dvz:+7.1f}  VZ={vz:+7.1f}  PZ={pz:+8.1f}  DPZ={dpz:+8.1f}  err={dpz-pz:+6.1f}")
                last = t_us


if __name__ == "__main__":
    main()
