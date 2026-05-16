#!/usr/bin/env python3
"""Rangefinder / surface-tracking flight review.

Built for the 2026-05-15 flight where the pilot reported surface tracking
not actually following terrain over a building edge, and the LAND_RNG_ALT
descent limit not being noticeable.

Pulls from the .bin:
  - RFND[I=0]: raw downward rangefinder distance and status
  - SURF[I=0]: surface distance state bitmask, raw/filtered distance, terrain offset
  - CTUN: SAlt (surface-tracking desired alt), Alt (above home), DSAlt (desired SAlt)
  - PSC: DVZ vs VZ (desired vs actual descent rate) for descent-limit assessment
  - MODE: flight mode timeline
  - NAMED_VALUE_INT[RFND_ST]: 4-state indicator (verifies firmware path)
  - PARM: snapshot of relevant rangefinder/landing params

Outputs:
  1) Timeline of flight modes
  2) RFND_ST transitions
  3) Glitch events from SURF.St bit 3 (start time, duration, distance jumps)
  4) Surface tracking "active" vs "standby" timeline from SURF.St bit 0+1+2
  5) Building-edge events: candidate moments where RFND distance jumped >2m
  6) Late-flight descent: PSC.DVZ vs VZ around fast descent attempt
"""
import sys
from pymavlink import mavutil

LOG = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/2026-05-15 11-43-30.bin"

# SURF.St bitmask (from AP_SurfaceDistance.cpp)
SURF_ENABLED   = 1 << 0
SURF_UNHEALTHY = 1 << 1
SURF_STALE     = 1 << 2
SURF_GLITCH    = 1 << 3

# Copter flight mode names (we only need the ones we'll see)
MODE_NAMES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT",
    14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
    24: "ZIGZAG", 27: "AUTO_RTL",
}


def t_str(t_us):
    """Convert TimeUS to seconds-since-boot string."""
    return f"{t_us / 1e6:8.2f}s"


def main():
    print(f"Opening {LOG}")
    mlog = mavutil.mavlink_connection(LOG)

    # Collect data
    rfnd = []           # (t_us, dist_m, status)
    surf = []           # (t_us, instance, st, dist_raw, dist_filt, terrain_offset)
    ctun = []           # (t_us, sat, alt, dsat)
    psc = []            # (t_us, dvz, vz)
    modes = []          # (t_us, mode_num)
    rfnd_st = []        # (t_us, value)
    rfnd_btn_en = None
    land_rng_alt = None
    land_rng_spd = None
    surftrak_mode = None
    rnd_min_cm = None
    rnd_max_cm = None
    poshold_used = False

    n = 0
    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        n += 1
        t = msg.get_type()

        if t == "RFND":
            # Only downward (Instance 0)
            inst = getattr(msg, "Instance", getattr(msg, "I", 0))
            if inst == 0:
                rfnd.append((msg.TimeUS, msg.Dist, msg.Stat))

        elif t == "SURF":
            surf.append((msg.TimeUS, msg.I, msg.St, msg.D, msg.FD, msg.TO))

        elif t == "CTUN":
            sat = getattr(msg, "SAlt", None)
            dsat = getattr(msg, "DSAlt", None)
            alt = getattr(msg, "Alt", None)
            ctun.append((msg.TimeUS, sat, alt, dsat))

        elif t == "PSC":
            # PSCx variants exist; the Z field names vary by build
            dvz = getattr(msg, "DVZ", None)
            vz  = getattr(msg, "VZ", None)
            if dvz is not None and vz is not None:
                psc.append((msg.TimeUS, dvz, vz))

        elif t == "PSCD":  # newer-style: PSCD has down-axis pos/vel/accel
            dvz = getattr(msg, "DVD", None)
            vz  = getattr(msg, "VD", None)
            if dvz is not None and vz is not None:
                psc.append((msg.TimeUS, dvz, vz))

        elif t == "MODE":
            modes.append((msg.TimeUS, msg.Mode))

        elif t == "PARM":
            name = msg.Name
            val = msg.Value
            if name == "RFND_BTN_EN":   rfnd_btn_en = val
            elif name == "LAND_RNG_ALT": land_rng_alt = val
            elif name == "LAND_RNG_SPD": land_rng_spd = val
            elif name == "SURFTRAK_MODE": surftrak_mode = val
            elif name == "RNGFND1_MIN_CM": rnd_min_cm = val
            elif name == "RNGFND1_MAX_CM": rnd_max_cm = val

        elif t == "NVAL":  # NAMED_VALUE_INT logged as NVAL? check both
            pass

    # In some firmware/dataflash builds, NAMED_VALUE_INT isn't logged at all
    # (it's an output-only MAVLink message). We'll note that if rfnd_st is empty.

    print()
    print("=" * 72)
    print("PARAMETERS (rangefinder + landing relevant)")
    print("=" * 72)
    print(f"  RFND_BTN_EN     = {rfnd_btn_en}")
    print(f"  LAND_RNG_ALT    = {land_rng_alt} cm")
    print(f"  LAND_RNG_SPD    = {land_rng_spd} cm/s")
    print(f"  SURFTRAK_MODE   = {surftrak_mode}  (0=NONE 1=GROUND 2=CEILING)")
    print(f"  RNGFND1_MIN_CM  = {rnd_min_cm}")
    print(f"  RNGFND1_MAX_CM  = {rnd_max_cm}")

    print()
    print("=" * 72)
    print(f"COUNTS:  RFND={len(rfnd)}  SURF={len(surf)}  CTUN={len(ctun)}  PSC={len(psc)}  MODE={len(modes)}")
    print("=" * 72)

    # MODE timeline
    print()
    print("FLIGHT MODE TIMELINE")
    print("-" * 72)
    for t_us, m in modes:
        name = MODE_NAMES.get(m, f"MODE({m})")
        print(f"  {t_str(t_us)}  {name}")

    # SURF status bitmask timeline — summarize transitions
    print()
    print("SURF.St TIMELINE (instance 0 only, transitions only)")
    print("-" * 72)
    last_st = None
    surf0 = [s for s in surf if s[1] == 0]
    transitions = 0
    for t_us, inst, st, draw, dfilt, to in surf0:
        if st != last_st:
            flags = []
            if st & SURF_ENABLED:   flags.append("EN")
            if st & SURF_UNHEALTHY: flags.append("UNHEALTHY")
            if st & SURF_STALE:     flags.append("STALE")
            if st & SURF_GLITCH:    flags.append("GLITCH")
            f = ",".join(flags) if flags else "OFF"
            print(f"  {t_str(t_us)}  St=0x{st:02x}  [{f}]  Draw={draw:.2f}m Dfilt={dfilt:.2f}m TerrOff={to:.2f}m")
            last_st = st
            transitions += 1
            if transitions > 200:
                print("  ...truncated (>200 transitions)...")
                break

    # GLITCH events: find contiguous runs of bit 3 set
    print()
    print("GLITCH EVENTS (SURF.St bit 3 set, instance 0)")
    print("-" * 72)
    in_glitch = False
    glitch_start = None
    glitch_start_dist = None
    glitch_events = []
    for t_us, inst, st, draw, dfilt, to in surf0:
        glitching = bool(st & SURF_GLITCH)
        if glitching and not in_glitch:
            in_glitch = True
            glitch_start = t_us
            glitch_start_dist = draw
        elif not glitching and in_glitch:
            in_glitch = False
            duration_ms = (t_us - glitch_start) / 1000
            jump = draw - glitch_start_dist
            glitch_events.append((glitch_start, duration_ms, glitch_start_dist, draw, jump))
    print(f"  Total: {len(glitch_events)} glitch episodes")
    for gs, dur, d0, d1, jump in glitch_events[:30]:
        print(f"  {t_str(gs)}  duration={dur:6.0f}ms  Dist: {d0:.2f} -> {d1:.2f} m  (jump {jump:+.2f}m)")
    if len(glitch_events) > 30:
        print(f"  ...{len(glitch_events)-30} more glitch events not shown...")

    # RFND raw distance — find large jumps between consecutive samples
    print()
    print("LARGE RFND JUMPS (>2m between consecutive samples, instance 0)")
    print("-" * 72)
    big_jumps = []
    last = None
    for t_us, dist, stat in rfnd:
        if last is not None:
            dt_ms = (t_us - last[0]) / 1000
            dd = dist - last[1]
            if abs(dd) >= 2.0 and dt_ms < 1000:  # ignore long gaps
                big_jumps.append((t_us, last[1], dist, dd, dt_ms, stat))
        last = (t_us, dist, stat)
    print(f"  Total: {len(big_jumps)} jumps >=2m")
    for t_us, d0, d1, dd, dt_ms, stat in big_jumps[:30]:
        print(f"  {t_str(t_us)}  {d0:.2f} -> {d1:.2f} m  (jump {dd:+.2f}m over {dt_ms:.0f}ms)  Stat={stat}")
    if len(big_jumps) > 30:
        print(f"  ...{len(big_jumps)-30} more not shown...")

    # RFND status timeline (transitions only)
    print()
    print("RFND.Stat TIMELINE (transitions only, instance 0)")
    print("-" * 72)
    last_stat = None
    for t_us, dist, stat in rfnd:
        if stat != last_stat:
            print(f"  {t_str(t_us)}  Stat={stat}  Dist={dist:.2f}m")
            last_stat = stat

    # CTUN: surface alt vs alt over time, downsample to ~1 Hz
    if ctun:
        print()
        print("CTUN SAMPLES (downsampled to ~1 Hz)")
        print("-" * 72)
        last_print_us = 0
        for t_us, sat, alt, dsat in ctun:
            if t_us - last_print_us >= 1_000_000:
                sat_str = f"{sat:7.2f}" if sat is not None else "  --   "
                dsat_str = f"{dsat:7.2f}" if dsat is not None else "  --   "
                alt_str = f"{alt:7.2f}" if alt is not None else "  --   "
                print(f"  {t_str(t_us)}  SAlt={sat_str}  DSAlt={dsat_str}  Alt={alt_str}")
                last_print_us = t_us

    # PSC: find descent windows (VZ < -100 cm/s) and report DVZ vs VZ
    if psc:
        print()
        print("FAST DESCENT WINDOWS (|VZ| > 100 cm/s)")
        print("-" * 72)
        # find contiguous descent windows
        in_descent = False
        win_start = None
        descents = []
        for t_us, dvz, vz in psc:
            if vz < -100 and not in_descent:
                in_descent = True
                win_start = t_us
            elif vz >= -50 and in_descent:
                in_descent = False
                descents.append((win_start, t_us))
        if in_descent:
            descents.append((win_start, psc[-1][0]))

        print(f"  Total: {len(descents)} descent windows")
        for ws, we in descents[-5:]:  # last 5 (most likely the LAND_RNG_ALT test)
            # Snapshot samples in window at ~0.5s intervals
            print(f"  Window {t_str(ws)} -> {t_str(we)}  ({(we-ws)/1e6:.1f}s)")
            last_sample = 0
            for t_us, dvz, vz in psc:
                if t_us < ws: continue
                if t_us > we: break
                if t_us - last_sample >= 500_000:
                    # find concurrent RFND distance
                    rng_d = None
                    for rt, rd, rs in rfnd:
                        if rt > t_us: break
                        rng_d = rd
                    rng_str = f"rng={rng_d:.2f}m" if rng_d is not None else "rng=--"
                    print(f"    {t_str(t_us)}  DVZ={dvz:+7.1f}  VZ={vz:+7.1f}  gap={dvz-vz:+6.1f}  {rng_str}")
                    last_sample = t_us

    print()
    print("DONE")


if __name__ == "__main__":
    main()
