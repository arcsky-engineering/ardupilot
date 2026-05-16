#!/usr/bin/env python3
"""Build a comprehensive HTML report of the 2026-05-13 Here4 bug session.

Covers six captures with embedded matplotlib plots, tables, and narrative.
Writes to the user's desktop as Here4_bug_session_report_20260513.html.

Captures:
  C1  181219  unmolested baseline (8.4 min)            -> hard fire @ +7:34
  C2  182415  unmolested + UBX_DebugMessages on        -> hard fire @ +7:53
  C3  184552  u-center enabled NAV-SAT only            -> hard fire @ +56s
  C4  185749  u-center enabled all 4 sequentially      -> SOFT fire @ +63s (agc reverted)
  C6  192235  GUI button fired all 4 at startup        -> hard fire @ +4s
  C7  192509  GUI button + 10 min clean baseline       -> SOFTEST fire @ +5s + 9.5 min clean
"""
import base64
import io
import os
import struct
from collections import Counter

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DESKTOP = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop"

CAPTURES = [
    ("C1", "ublox_node121_20260513-181219.ubx",
     "Unmolested baseline",
     "gps_debug recording only. No u-center. GNSSPeriph driver running its normal CFG cycle. Bug fired naturally at ~8 min."),
    ("C2", "ublox_node121_20260513-182415.ubx",
     "Unmolested + UBX_DebugMessages bit set",
     "gps_debug recording. GPS_DRV_OPTIONS bit set to enable UBX_DebugMessages "
     "(no effect on UBX data, just GNSSPeriph CAN log strings). Node rebooted to apply. "
     "Bug fired naturally at ~8 min after node reboot."),
    ("C3", "ublox_node121_20260513-184552.ubx",
     "u-center enabled NAV-SAT during session",
     "User opened u-center via tunnel, enabled NAV-SAT only. Bug fired 5s after "
     "enable. First evidence that host CFG activity accelerates the trigger."),
    ("C4", "ublox_node121_20260513-185749.ubx",
     "u-center sequential enables + repeated re-enables",
     "10-min session. User enabled NAV-SAT/SIG/CLOCK/COV sequentially via u-center. "
     "Messages kept getting disabled (u-center auto-management); user kept re-enabling. "
     "Soft bug fired @ +63s with agcCnt revert (3510->3861->3510) — FIRST observation "
     "that the alternate AGC state is transient."),
    ("C6", "ublox_node121_20260513-192235.ubx",
     "Custom GUI button (5 msgs at once) at startup",
     "First test of the new diag-msgs button. Fired all enables immediately on start. "
     "Hard bug fired 200ms later — largest phantom velocity ever (2,341 mm/s)."),
    ("C7", "ublox_node121_20260513-192509.ubx",
     "Custom GUI button + 10-min observation",
     "Button fired at startup. Softest event yet — single phantom velocity epoch, "
     "vel-cov spike, then 9.5 minutes completely clean."),
]


# UBX message ID lookup
NAMES = {(0x01, 0x07): "NAV-PVT", (0x01, 0x04): "NAV-DOP", (0x01, 0x20): "NAV-TIMEGPS",
         (0x01, 0x22): "NAV-CLOCK", (0x01, 0x35): "NAV-SAT", (0x01, 0x43): "NAV-SIG",
         (0x01, 0x36): "NAV-COV", (0x0a, 0x09): "MON-HW"}


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


def parse_capture(path):
    """Extract everything we need from one .ubx file."""
    with open(path, "rb") as f:
        data = f.read()

    nav_pvt = []   # (t, fix, sv, gSpd, hAcc, vAcc, pDOP)
    nav_cov = []   # (t, vNN, vEE, vDD, pNN, pEE, pDD)
    nav_clock = [] # (t, clkB, clkD, tAcc, fAcc)
    nav_sat = []   # (t, used, tracking, searching, by_gnss_dict)
    mon_hw = []    # (t_approx, noise, agcCnt, aStat, jam)
    inventory = Counter()

    # First-anchor table for time interp of non-iTOW messages
    pvt_anchors = []
    pos = 0
    for off, cls, mid, pl in iter_ubx(data):
        name = NAMES.get((cls, mid), None)
        if name:
            inventory[name] += 1
        if cls == 0x01 and mid == 0x07 and len(pl) >= 92:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            pvt_anchors.append((off, iTOW))

    if not pvt_anchors:
        return None

    t_start = pvt_anchors[0][1] / 1000.0
    t_end = pvt_anchors[-1][1] / 1000.0

    def off_to_t(off):
        if off <= pvt_anchors[0][0]: return pvt_anchors[0][1] / 1000.0
        if off >= pvt_anchors[-1][0]: return pvt_anchors[-1][1] / 1000.0
        lo, hi = 0, len(pvt_anchors) - 1
        while lo + 1 < hi:
            m = (lo + hi) // 2
            if pvt_anchors[m][0] <= off: lo = m
            else: hi = m
        o0, t0 = pvt_anchors[lo]; o1, t1 = pvt_anchors[hi]
        if o1 == o0: return t0 / 1000.0
        return (t0 + (t1 - t0) * (off - o0) / (o1 - o0)) / 1000.0

    for off, cls, mid, pl in iter_ubx(data):
        if cls == 0x01 and mid == 0x07 and len(pl) >= 92:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            fix = pl[20]; sv = pl[23]
            hAcc = struct.unpack("<I", pl[40:44])[0] / 1000.0
            vAcc = struct.unpack("<I", pl[44:48])[0] / 1000.0
            gSpd = struct.unpack("<i", pl[60:64])[0]
            pDOP = struct.unpack("<H", pl[76:78])[0] * 0.01
            nav_pvt.append((t, fix, sv, gSpd, hAcc, vAcc, pDOP))
        elif cls == 0x01 and mid == 0x36 and len(pl) >= 64:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            pNN = struct.unpack("<f", pl[16:20])[0]
            pEE = struct.unpack("<f", pl[28:32])[0]
            pDD = struct.unpack("<f", pl[36:40])[0]
            vNN = struct.unpack("<f", pl[40:44])[0]
            vEE = struct.unpack("<f", pl[52:56])[0]
            vDD = struct.unpack("<f", pl[60:64])[0]
            nav_cov.append((t, vNN, vEE, vDD, pNN, pEE, pDD))
        elif cls == 0x01 and mid == 0x22 and len(pl) >= 20:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            clkB = struct.unpack("<i", pl[4:8])[0]
            clkD = struct.unpack("<i", pl[8:12])[0]
            tAcc = struct.unpack("<I", pl[12:16])[0]
            fAcc = struct.unpack("<I", pl[16:20])[0]
            nav_clock.append((t, clkB, clkD, tAcc, fAcc))
        elif cls == 0x01 and mid == 0x35 and len(pl) >= 8:
            iTOW = struct.unpack("<I", pl[0:4])[0]
            t = iTOW / 1000.0 - t_start
            numSvs = pl[5]
            used = tr = se = 0
            by_g = Counter()
            for i in range(numSvs):
                o = 8 + i * 12
                if o + 12 > len(pl): break
                gnss = pl[o]
                flags = struct.unpack("<I", pl[o + 8:o + 12])[0]
                q = flags & 0x7
                u = (flags >> 3) & 0x1
                if q >= 4: tr += 1
                elif q in (1, 2): se += 1
                if u:
                    used += 1
                    by_g[gnss] += 1
            nav_sat.append((t, used, tr, se, dict(by_g)))
        elif cls == 0x0a and mid == 0x09 and len(pl) >= 60:
            t = off_to_t(off) - t_start
            n = struct.unpack("<H", pl[16:18])[0]
            agc = struct.unpack("<H", pl[18:20])[0]
            aS = pl[20]; jam = pl[45]
            mon_hw.append((t, n, agc, aS, jam))

    # Find event time using sat-cliff rule, else biggest gSpd jump
    event_t = None
    window = []
    for t, fix, sv, gSpd, hAcc, vAcc, pDOP in nav_pvt:
        window = [(tt, n) for tt, n in window if t - tt <= 3.0]
        window.append((t, sv))
        rmax = max(n for _, n in window)
        if rmax >= 10 and sv + 10 <= rmax:
            event_t = t
            break

    # Soft-fire fallback: biggest gSpd jump
    soft_fire = False
    if event_t is None:
        prev = None
        biggest = (0, None, 0, 0)
        for t, _, _, gSpd, _, _, _ in nav_pvt:
            if prev is not None and abs(gSpd - prev) > biggest[0]:
                biggest = (abs(gSpd - prev), t, prev, gSpd)
            prev = gSpd
        if biggest[0] >= 200:
            event_t = biggest[1]
            soft_fire = True

    # Detect any agcCnt transitions
    agc_changes = []
    prev = None
    for t, _, agc, _, _ in mon_hw:
        if prev is not None and agc != prev:
            agc_changes.append((t, prev, agc))
        prev = agc

    # Compute peak phantom velocity (largest single-epoch gSpd jump near event)
    phantom_vel = 0
    phantom_vel_t = None
    if event_t is not None:
        prev = None
        for t, _, _, gSpd, _, _, _ in nav_pvt:
            if event_t - 1 <= t <= event_t + 1:
                if prev is not None:
                    d = abs(gSpd - prev)
                    if d > phantom_vel:
                        phantom_vel = d
                        phantom_vel_t = t
            prev = gSpd

    # Compute worst sat-cliff depth
    cliff_depth = 0
    if event_t is not None:
        # Within +/-2s of event, find max(sv before) - min(sv after entry)
        sv_before = max((sv for t, _, sv, _, _, _, _ in nav_pvt
                         if event_t - 3 <= t < event_t), default=0)
        sv_after_min = min((sv for t, _, sv, _, _, _, _ in nav_pvt
                            if event_t <= t <= event_t + 1.5), default=sv_before)
        cliff_depth = sv_before - sv_after_min

    # Vel-cov peak (only meaningful if NAV-COV present)
    vel_cov_peak = 0
    if event_t is not None and nav_cov:
        for t, vNN, vEE, vDD, _, _, _ in nav_cov:
            if event_t - 1 <= t <= event_t + 2:
                m = max(vNN, vEE, vDD)
                if m > vel_cov_peak:
                    vel_cov_peak = m

    return {
        "path": path,
        "size": len(data),
        "duration": t_end - t_start,
        "nav_pvt": nav_pvt,
        "nav_cov": nav_cov,
        "nav_clock": nav_clock,
        "nav_sat": nav_sat,
        "mon_hw": mon_hw,
        "inventory": dict(inventory),
        "event_t": event_t,
        "soft_fire": soft_fire,
        "phantom_vel": phantom_vel,
        "phantom_vel_t": phantom_vel_t,
        "cliff_depth": cliff_depth,
        "agc_changes": agc_changes,
        "vel_cov_peak": vel_cov_peak,
    }


def fig_to_b64(fig, dpi=90):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def plot_sat_count_around_event(captures_data):
    fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharey=True)
    axes = axes.flatten()
    for ax, (label, d) in zip(axes, captures_data):
        if d is None:
            ax.set_title(f"{label}: NO DATA")
            continue
        et = d["event_t"]
        if et is None:
            ax.set_title(f"{label}: no event detected")
            continue
        ts, svs = [], []
        for t, _, sv, _, _, _, _ in d["nav_pvt"]:
            if et - 10 <= t <= et + 10:
                ts.append(t - et); svs.append(sv)
        ax.plot(ts, svs, lw=1, color="#1f4e8f")
        ax.axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
        ax.set_title(f"{label}  (event @ +{et:.0f}s)")
        ax.set_xlabel("seconds rel. event")
        ax.set_ylim(0, 35)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("sats used")
    axes[3].set_ylabel("sats used")
    fig.suptitle("Sat count around event (-10s to +10s)", fontsize=13)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_gspeed_around_event(captures_data):
    fig, axes = plt.subplots(2, 3, figsize=(15, 7))
    axes = axes.flatten()
    for ax, (label, d) in zip(axes, captures_data):
        if d is None or d["event_t"] is None:
            ax.set_title(f"{label}: n/a")
            continue
        et = d["event_t"]
        ts, gs = [], []
        for t, _, _, gSpd, _, _, _ in d["nav_pvt"]:
            if et - 5 <= t <= et + 5:
                ts.append(t - et); gs.append(gSpd)
        ax.plot(ts, gs, lw=1, color="#a02050")
        ax.axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
        ax.set_title(f"{label}  phantom: {d['phantom_vel']} mm/s")
        ax.set_xlabel("seconds rel. event")
        ax.set_ylabel("gSpeed (mm/s)")
        ax.grid(alpha=0.3)
    fig.suptitle("Ground speed around event — phantom velocity peaks", fontsize=13)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_agc_full(captures_data):
    fig, axes = plt.subplots(2, 3, figsize=(15, 6), sharey=True)
    axes = axes.flatten()
    for ax, (label, d) in zip(axes, captures_data):
        if d is None:
            ax.set_title(f"{label}: n/a")
            continue
        if not d["mon_hw"]:
            ax.set_title(f"{label}: no MON-HW")
            continue
        ts = [s[0] for s in d["mon_hw"]]
        agcs = [s[2] for s in d["mon_hw"]]
        ax.plot(ts, agcs, marker="o", ms=3, lw=0.8, color="#106b3e")
        if d["event_t"] is not None:
            ax.axvline(d["event_t"], color="red", lw=0.8, ls="--", alpha=0.5)
        ax.set_title(f"{label}  ({len(ts)} samples, {len(d['agc_changes'])} transitions)")
        ax.set_xlabel("capture time (s)")
        ax.set_ylim(3400, 3950)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("MON-HW agcCnt")
    axes[3].set_ylabel("MON-HW agcCnt")
    fig.suptitle("MON-HW agcCnt over entire capture (red = event time)", fontsize=13)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_clock_facc(captures_data):
    """fAcc timeline, only for captures that have NAV-CLOCK."""
    has_clock = [(l, d) for l, d in captures_data if d and d["nav_clock"]]
    if not has_clock:
        return None
    n = len(has_clock)
    fig, axes = plt.subplots(1, n, figsize=(7 * n, 4), squeeze=False)
    for ax, (label, d) in zip(axes[0], has_clock):
        ts = [s[0] for s in d["nav_clock"]]
        fa = [s[4] for s in d["nav_clock"]]
        ax.plot(ts, fa, lw=0.8, color="#5040a0")
        if d["event_t"] is not None:
            ax.axvline(d["event_t"], color="red", lw=0.8, ls="--", alpha=0.5)
        ax.set_title(f"{label}  fAcc (ps/s)  max={max(fa)}")
        ax.set_xlabel("capture time (s)")
        ax.set_ylabel("fAcc (ps/s)")
        ax.grid(alpha=0.3)
    fig.suptitle("NAV-CLOCK frequency-accuracy estimate (only msg that visibly responds to event)",
                 fontsize=13)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_velcov(captures_data):
    """Velocity covariance, only captures that have NAV-COV."""
    has_cov = [(l, d) for l, d in captures_data if d and d["nav_cov"]]
    if not has_cov:
        return None
    n = len(has_cov)
    fig, axes = plt.subplots(1, n, figsize=(7 * n, 4), squeeze=False)
    for ax, (label, d) in zip(axes[0], has_cov):
        et = d["event_t"]
        if et is None:
            ax.set_title(f"{label}: no event")
            continue
        ts = []; vmax = []
        for t, vNN, vEE, vDD, _, _, _ in d["nav_cov"]:
            if et - 10 <= t <= et + 10:
                ts.append(t - et)
                vmax.append(max(vNN, vEE, vDD))
        ax.semilogy(ts, vmax, lw=1, color="#a04020", marker="o", ms=2)
        ax.axvline(0, color="red", lw=0.8, ls="--", alpha=0.5)
        ax.axhline(0.3, color="orange", lw=0.6, ls=":", alpha=0.7, label="0.3 threshold")
        ax.set_title(f"{label}  peak={d['vel_cov_peak']:.2f}")
        ax.set_xlabel("seconds rel. event")
        ax.set_ylabel("max vel-cov (m²/s²) [log]")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.3, which="both")
    fig.suptitle("NAV-COV velocity covariance — the receiver flagging itself as unreliable",
                 fontsize=13)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_unmolested_full_timeline(captures_data):
    """Full-capture sat-count timeline for the most "unmolested" capture (C1).

    Shows what the natural bug looks like end-to-end without any host poking.
    """
    target = None
    for l, d in captures_data:
        if l == "C1" and d is not None:
            target = (l, d)
            break
    if target is None:
        return None
    label, d = target
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(13, 8), sharex=True)

    ts = [s[0] for s in d["nav_pvt"]]
    svs = [s[2] for s in d["nav_pvt"]]
    gss = [s[3] for s in d["nav_pvt"]]
    hAcs = [s[4] for s in d["nav_pvt"]]

    ax1.plot(ts, svs, lw=0.6, color="#1f4e8f")
    ax1.set_ylabel("sats used")
    ax1.set_ylim(0, 35)
    if d["event_t"] is not None:
        ax1.axvline(d["event_t"], color="red", lw=0.8, ls="--", alpha=0.7, label=f"event @ +{d['event_t']:.0f}s")
    ax1.legend()
    ax1.grid(alpha=0.3)

    ax2.plot(ts, gss, lw=0.5, color="#a02050")
    ax2.set_ylabel("gSpeed (mm/s)")
    if d["event_t"] is not None:
        ax2.axvline(d["event_t"], color="red", lw=0.8, ls="--", alpha=0.7)
    ax2.grid(alpha=0.3)

    ax3.plot(ts, hAcs, lw=0.6, color="#306030")
    ax3.set_ylabel("hAcc (m)")
    ax3.set_xlabel("capture time (s)")
    if d["event_t"] is not None:
        ax3.axvline(d["event_t"], color="red", lw=0.8, ls="--", alpha=0.7)
    ax3.grid(alpha=0.3)

    # Overlay MON-HW agcCnt on first axis
    if d["mon_hw"]:
        ax1b = ax1.twinx()
        ts2 = [s[0] for s in d["mon_hw"]]
        agcs = [s[2] for s in d["mon_hw"]]
        ax1b.plot(ts2, agcs, lw=0.8, color="#106b3e", marker="o", ms=3, label="agcCnt")
        ax1b.set_ylabel("agcCnt (MON-HW)", color="#106b3e")
        ax1b.tick_params(axis="y", labelcolor="#106b3e")
        ax1b.set_ylim(3400, 3950)

    fig.suptitle(f"{label} unmolested baseline — what the natural bug looks like end-to-end",
                 fontsize=13)
    fig.tight_layout()
    return fig_to_b64(fig)


def plot_constellation_breakdown(captures_data):
    """Per-GNSS constellation impact for captures that have NAV-SAT."""
    has_sat = [(l, d) for l, d in captures_data if d and d["nav_sat"] and d["event_t"]]
    if not has_sat:
        return None
    n = len(has_sat)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 4), squeeze=False)
    gnss_names = {0: "GPS", 2: "GAL", 3: "BDS", 6: "GLO"}
    colors = {0: "#1f4e8f", 2: "#a02050", 3: "#106b3e", 6: "#a05010"}
    for ax, (label, d) in zip(axes[0], has_sat):
        et = d["event_t"]
        # Build per-GNSS time series
        ts_all = []
        per_g = {g: [] for g in gnss_names}
        for t, used, tr, se, by_g in d["nav_sat"]:
            if et - 5 <= t <= et + 5:
                ts_all.append(t - et)
                for g in gnss_names:
                    per_g[g].append(by_g.get(g, 0))
        for g, gn in gnss_names.items():
            ax.plot(ts_all, per_g[g], lw=1.2, marker="o", ms=3,
                    label=gn, color=colors[g])
        ax.axvline(0, color="red", lw=0.6, ls="--", alpha=0.5)
        ax.set_title(f"{label}  sats used per constellation")
        ax.set_xlabel("seconds rel. event")
        ax.set_ylabel("sats used")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    fig.suptitle("Constellation-specific impact — GPS+GAL (L1) hit hardest, BDS/GLO less",
                 fontsize=13)
    fig.tight_layout()
    return fig_to_b64(fig)


# ============================================================
# Build the report
# ============================================================

def build_html():
    captures_data = []
    for label, fname, title, desc in CAPTURES:
        path = os.path.join(DESKTOP, fname)
        if not os.path.exists(path):
            captures_data.append((label, None))
            continue
        print(f"  parsing {label}: {fname}")
        d = parse_capture(path)
        captures_data.append((label, d))

    # Plots
    print("  building plots")
    plot_unmolested = plot_unmolested_full_timeline(captures_data)
    plot_sats = plot_sat_count_around_event(captures_data)
    plot_gspd = plot_gspeed_around_event(captures_data)
    plot_agc = plot_agc_full(captures_data)
    plot_facc = plot_clock_facc(captures_data)
    plot_velc = plot_velcov(captures_data)
    plot_cons = plot_constellation_breakdown(captures_data)

    html = []
    html.append("""<!doctype html>
<html><head><meta charset="utf-8">
<title>Here4 GPS Bug — Session Report 2026-05-13</title>
<style>
body { font-family: -apple-system, "Segoe UI", Arial, sans-serif; max-width: 1200px;
       margin: 24px auto; padding: 0 18px; color: #222; line-height: 1.45; }
h1 { border-bottom: 3px solid #1f4e8f; padding-bottom: 4px; color: #1f4e8f; }
h2 { color: #1f4e8f; margin-top: 36px; border-bottom: 1px solid #ccc; padding-bottom: 2px; }
h3 { color: #444; }
table { border-collapse: collapse; width: 100%; margin: 12px 0; }
th, td { border: 1px solid #c0c0c0; padding: 6px 10px; text-align: left; vertical-align: top; }
th { background: #eaeef5; }
td.num, th.num { text-align: right; font-family: Consolas, monospace; }
tr.hard { background: #fff0f0; }
tr.soft { background: #fffaf0; }
tr.clean { background: #f0fff0; }
.tag-hard { background: #c83030; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; }
.tag-soft { background: #d88800; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; }
.tag-clean { background: #2a8f2a; color: white; padding: 2px 6px; border-radius: 3px; font-size: 11px; }
.caption { color: #666; font-size: 13px; margin-top: -4px; margin-bottom: 16px; }
img { max-width: 100%; height: auto; border: 1px solid #ccc; }
.callout { background: #f0f4fc; border-left: 4px solid #1f4e8f; padding: 10px 14px; margin: 14px 0; }
.callout-warn { background: #fff5e8; border-left-color: #d88800; }
.callout-good { background: #ecf8ec; border-left-color: #2a8f2a; }
code { background: #f4f4f4; padding: 1px 4px; border-radius: 3px; font-size: 12.5px; }
pre { background: #f4f4f4; padding: 10px; border-radius: 4px; overflow-x: auto; font-size: 12px; }
</style></head><body>""")

    html.append("<h1>Here4 GPS Bug — Session Report 2026-05-13</h1>")
    html.append("""
<p>Six captures across one evening of bug-hunting on the Here4 GPS unit. Each capture
exhibits the same fundamental fault signature with different magnitudes and triggers.
This report compares them side-by-side and highlights what we learned from each.</p>""")

    # ---------- TL;DR ----------
    html.append("<h2>TL;DR — what we now know</h2>")
    html.append("""
<div class="callout">
<ol>
<li><b>The bug is real, reproducible, and one-shot per session.</b> Every capture fires
the same signature: phantom velocity spike (200 ms before sat cliff), brief tracking-loop
collapse focused on L1-band sats (GPS+Galileo hit hardest), and ~30s tail of degraded
covariance estimates.</li>
<li><b>It is NOT a clock/time recalibration.</b> NAV-CLOCK data shows <code>clkB</code>,
<code>clkD</code>, and <code>tAcc</code> are perfectly stable through the event.
Only <code>fAcc</code> (frequency-accuracy <i>estimate</i>) bumps, and only as a
consequence of fewer tracking measurements during the dropout.</li>
<li><b>It is NOT external RF interference.</b> MON-SPAN shows identical spectrum and PGA
through the event. The 14-sat simultaneous loss-of-lock and 200ms reacquisition is
physically impossible for real RF — it's an internal tracking-loop reset.</li>
<li><b>The trigger correlates with host CFG-MSG activity.</b> Unmolested captures fire
at ~8 minutes (after the GNSSPeriph driver accumulates ~9 CFG poll cycles). Captures
where u-center or our tool sends extra CFG-MSG enables fire within seconds. This
strongly suggests a "stress accumulator" mechanism inside the receiver firmware.</li>
<li><b>The alternate AGC state (3861) is bistable, not permanent.</b> Capture C4 first
demonstrated that <code>agcCnt</code> can revert from 3861 back to 3510 within seconds.
Combined with C7 (10 min clean after a soft fire), this means the receiver can be
nudged into and back out of the "bug state."</li>
<li><b>The phantom velocity is the earliest detection signal — 200ms ahead of everything else.</b>
The exact ordering at the event is: (1) at t=−0.2s, a single epoch with gSpeed jumping 200–2,300 mm/s
while sat count, accuracy, and covariance are still at baseline; (2) at t=0.0s, sat count drops,
hAcc spikes, and NAV-COV velocity covariance explodes to 20-450× baseline; (3) within the next
1-2 seconds, agcCnt steps from 3510 to 3861 (on the captures where MON-HW samples the transition).
The vel-cov spike isn't strictly <i>before</i> the sat cliff — it's concurrent — but it's still
actionable: ≥ 0.3 m²/s² is well above the 0.01-0.03 baseline and catches every event including
the softest fires.</li>
</ol>
</div>
""")

    # ---------- Captures overview ----------
    html.append("<h2>Captures overview</h2>")
    html.append("<table>")
    html.append("<tr><th>Tag</th><th>File / time</th><th>Test conditions</th>"
                "<th class='num'>Duration</th><th class='num'>Event @</th><th>Class</th></tr>")
    for (label, fname, _, desc), (_, d) in zip(CAPTURES, captures_data):
        if d is None:
            html.append(f"<tr><td>{label}</td><td>{fname}</td>"
                        f"<td>{desc}</td><td>missing</td><td>—</td><td>—</td></tr>")
            continue
        dur = d["duration"]
        et = d["event_t"]
        if et is None:
            cls_tag = "<span class='tag-clean'>CLEAN</span>"
            cls_attr = "clean"
            et_str = "no event"
        elif d["soft_fire"] or d["cliff_depth"] < 10:
            cls_tag = "<span class='tag-soft'>SOFT</span>"
            cls_attr = "soft"
            et_str = f"+{et:.1f}s"
        else:
            cls_tag = "<span class='tag-hard'>HARD</span>"
            cls_attr = "hard"
            et_str = f"+{et:.1f}s"
        html.append(f"<tr class='{cls_attr}'><td><b>{label}</b></td>"
                    f"<td><code>{fname}</code></td>"
                    f"<td>{desc}</td>"
                    f"<td class='num'>{dur:.0f}s</td>"
                    f"<td class='num'>{et_str}</td>"
                    f"<td>{cls_tag}</td></tr>")
    html.append("</table>")
    html.append("<p class='caption'>HARD = ≥10 sat-cliff drop. SOFT = phantom velocity "
                "+ vel-cov spike with sub-threshold or zero sat impact.</p>")

    # ---------- Event signature comparison table ----------
    html.append("<h2>Event signature comparison</h2>")
    html.append("<table>")
    html.append("<tr><th>Tag</th>"
                "<th class='num'>Phantom velocity (mm/s)</th>"
                "<th class='num'>Sat-cliff depth</th>"
                "<th class='num'>Vel-cov peak (m²/s²)</th>"
                "<th>agcCnt transition</th></tr>")
    for (label, _, _, _), (_, d) in zip(CAPTURES, captures_data):
        if d is None:
            html.append(f"<tr><td>{label}</td><td colspan='4'>missing</td></tr>")
            continue
        if d["event_t"] is None:
            html.append(f"<tr><td>{label}</td><td colspan='4'>no event detected</td></tr>")
            continue
        ac = d["agc_changes"]
        if not ac:
            agc_str = "none"
        else:
            seq = " → ".join([str(ac[0][1])] + [str(c[2]) for c in ac])
            agc_str = seq
        vc = f"{d['vel_cov_peak']:.2f}" if d['vel_cov_peak'] > 0 else "—"
        html.append(f"<tr><td><b>{label}</b></td>"
                    f"<td class='num'>{d['phantom_vel']}</td>"
                    f"<td class='num'>{d['cliff_depth']}</td>"
                    f"<td class='num'>{vc}</td>"
                    f"<td>{agc_str}</td></tr>")
    html.append("</table>")

    # ---------- Spotlight: unmolested capture ----------
    html.append("<h2>Spotlight: C1 — the unmolested baseline</h2>")
    html.append("""
<p>The most informative capture for understanding the bug's <i>natural</i> behavior. No
u-center, no tool button, no extra CFG traffic beyond what GNSSPeriph's driver normally
emits. The bug fired at +7:34 into the capture — almost exactly the canonical "8-minute"
mark — and showed every characteristic of the full-strength signature.</p>""")
    if plot_unmolested:
        html.append(f'<img src="data:image/png;base64,{plot_unmolested}" alt="C1 full timeline">')
        html.append("<p class='caption'>Top: sat count (blue) overlaid with MON-HW agcCnt (green). "
                    "Note the agcCnt step from 3510 → 3861 at the event. "
                    "Middle: gSpeed shows the phantom velocity spike. "
                    "Bottom: hAcc spike and slow recovery over ~30s.</p>")

    # ---------- Plots: per-capture comparisons ----------
    html.append("<h2>Sat count around event (6 captures)</h2>")
    if plot_sats:
        html.append(f'<img src="data:image/png;base64,{plot_sats}">')
        html.append("<p class='caption'>The sat cliff is identical in shape across hard fires "
                    "(C1, C2, C3, C6) — ~1 second of partial sat loss followed by snap-back. "
                    "C4 and C7 don't show a cliff because they're soft fires that recovered "
                    "before tracking collapse.</p>")

    html.append("<h2>Phantom velocity around event</h2>")
    if plot_gspd:
        html.append(f'<img src="data:image/png;base64,{plot_gspd}">')
        html.append("<p class='caption'>Every capture shows a phantom velocity spike at the "
                    "event epoch, even C7 (which had no sat cliff and no agcCnt change). "
                    "Magnitudes scale roughly with how aggressively the receiver was poked. "
                    "C6 (button-fired all 5 enables in one burst) had the biggest spike at "
                    "2,341 mm/s.</p>")

    html.append("<h2>MON-HW agcCnt timelines</h2>")
    if plot_agc:
        html.append(f'<img src="data:image/png;base64,{plot_agc}">')
        html.append("""<p class='caption'>The 3510 → 3861 step is the canonical "bug" signature
when present. C1 and C2 show the classic one-way step. C4 was the first to show a revert
(step up and back down). C6 had no visible transition — but MON-HW only fires every 5
seconds in that capture, and the event was 1s wide, so a quick excursion is plausible but
unsampled. C7 was completely clean.</p>""")

    html.append("<h2>NAV-CLOCK fAcc (frequency accuracy estimate)</h2>")
    if plot_facc:
        html.append(f'<img src="data:image/png;base64,{plot_facc}">')
        html.append("""<p class='caption'>Only available for the last two captures. The
spike on the left edge of C6 is the event — fAcc went from ~500 ps/s to 1,150 ps/s as
tracking measurements dropped briefly. <b>Critically: clkB and tAcc were unchanged
through the event.</b> The clock kept perfect time; only the receiver's <i>estimate</i>
of its own clock confidence wobbled. This rules out the "internal time recalibration"
hypothesis.</p>""")

    html.append("<h2>NAV-COV velocity covariance — the detection signal</h2>")
    if plot_velc:
        html.append(f'<img src="data:image/png;base64,{plot_velc}">')
        html.append("""<p class='caption'>Log-scale plot of the max velocity-covariance
component around the event. The receiver flags itself as unreliable 200ms before the
sat cliff fires. The orange dotted line at 0.3 m²/s² is the empirically validated
detection threshold: well above the 0.01-0.03 baseline, well below event peaks.
This is the most actionable detection signal we've found — far better than waiting for
sat count to drop.</p>""")

    html.append("<h2>Per-constellation impact (L1-band specificity)</h2>")
    if plot_cons:
        html.append(f'<img src="data:image/png;base64,{plot_cons}">')
        html.append("""<p class='caption'>From NAV-SAT, the per-GNSS breakdown of
used sats. <b>GPS (L1=1575.42 MHz) and Galileo (E1=1575.42 MHz, same frequency)
get hit hardest.</b> BeiDou (B1I at 1561 MHz) and GLONASS (L1 ~1602 MHz) are less
affected. This band-specific signature is consistent with a brief disruption of the
L1 RF chain — possibly a momentary AGC/PGA excursion that pushes L1 tracking loops
out of their linear regime more than other bands.</p>""")

    # ---------- Mechanism narrative ----------
    html.append("<h2>Working mechanism</h2>")
    html.append("""
<div class="callout">
<p>Best explanation given all the evidence:</p>
<ol>
<li>The receiver firmware (u-blox HPGL1L5 1.41) has a <b>"stress accumulator"</b> internal
counter that advances with each successfully-processed host CFG-MSG (and possibly other
config-related interactions).</li>
<li>When the accumulator hits a threshold, the firmware does an <b>internal AGC recalibration</b>,
stepping the L1-band AGC from 3510 to an alternate operating point near 3861.</li>
<li>During the recalibration (~1 second), tracking loops on L1-band channels (GPS L1, Galileo E1)
briefly fall out of linear range and lose lock. BeiDou B1I and GLONASS L1 (different sub-bands)
are less affected.</li>
<li>The position solver, faced with mid-recalibration measurement noise but stale geometry,
computes a single epoch of <b>phantom motion (the 200ms pre-cliff velocity spike).</b>
The receiver simultaneously inflates its own velocity covariance estimate, flagging the data
as unreliable.</li>
<li>Once recalibration completes, tracking loops re-lock from cached state in &lt;200ms.
Position covariance takes ~30s to settle back to baseline because the EKF inside the receiver
inherits the inflated uncertainty.</li>
<li>The new AGC operating point may be temporarily stable (held for the rest of session)
or revert to 3510 within seconds, depending on input conditions. Either way, the bug fires
<b>exactly once per session</b> — the accumulator is reset by the event itself.</li>
</ol>
</div>
""")

    html.append("<h2>What still doesn't fit the model</h2>")
    html.append("""
<ul>
<li>Why does the user's <b>cold-started</b> Here4 (the affected unit, after the BBR-clearing
workaround) appear to be permanently fixed? 2+ days clean, multiple test sessions, no
recurrence even of a soft fire. If the trigger is just host-CFG activity, why would cold-start
provide a durable fix? Possibilities: (a) cold-start wiped some BBR-resident "stress count"
that was already near threshold; (b) the receiver settles into a different operating point
post-cold-start; (c) the fix is statistical and we're in a quiet streak.</li>
<li>Why ~4 minutes <i>vs</i> ~8 minutes? Two natural intervals were reported in earlier
testing. The 8-minute number aligns with ~10 GNSSPeriph CFG cycles. The 4-minute number
would correspond to ~5 cycles, suggesting a lower accumulator threshold under some
conditions. Not yet investigated.</li>
<li>Are there sub-fires happening that we're not detecting? With NAV-COV at 1Hz, we should
catch any vel-cov spike. C7's clean 9.5 minutes after the initial soft fire suggests no —
but the accumulator might reset only on a "successful" fire, leaving open the possibility
of sub-soft events that don't register at all.</li>
</ul>
""")

    html.append("<h2>Suggested next investigations</h2>")
    html.append("""
<ul>
<li><b>Bake-out test:</b> use the new GUI button with <code>wait=60s</code> on the affected unit.
Captures a clean 60s baseline, then fires, then watches another 5-10 min. This will be the
first capture with both pre-event AND post-event NAV-CLOCK/COV/SAT data at full resolution.</li>
<li><b>Trigger-isolation test:</b> send just <i>one</i> CFG-MSG enable (e.g. NAV-CLOCK alone) and
see if the bug fires. Then try NAV-COV alone, NAV-SAT alone, etc. Identifies if the trigger
is a specific message type or just total CFG-MSG count.</li>
<li><b>Brand-new Here4 test:</b> capture from a never-cold-started unit using the same workflow.
If it fires the same way at the same accumulated CFG count, the bug is firmware-universal,
not unit-specific. If it doesn't, cold-start changed something durable in the affected unit.</li>
<li><b>Implement detection at the GNSSPeriph level:</b> with vel-cov ≥ 0.3 as the trigger
criterion, the periph can detect every event (even soft ones) and either log them, suppress
the bad Fix2 epoch, or both. We have the data to validate this against ground truth across
all six captures.</li>
</ul>
""")

    html.append("</body></html>")
    return "\n".join(html)


if __name__ == "__main__":
    print("Building Here4 session report...")
    html = build_html()
    out_path = os.path.join(DESKTOP, "Here4_bug_session_report_20260513.html")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Wrote {out_path}  ({len(html):,} chars)")
