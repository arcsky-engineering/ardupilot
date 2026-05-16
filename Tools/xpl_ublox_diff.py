#!/usr/bin/env python3
"""Diff two u-blox F9P CFG-VALGET dumps. Highlights every key where the two
receivers have different values. Designed to compare a 'working' Here4 unit
against a 'broken' Here4 unit to find what's different in their u-blox config."""
import re

WORKING = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\receiver-config-good.txt"
BROKEN = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\receiver-config.txt"

KEY_NAMES = {
    # CFG-SIGNAL master + L1
    0x10310001: "CFG-SIGNAL-GPS_ENA",
    0x10310003: "CFG-SIGNAL-GPS_L1CA_ENA",
    0x10310004: "CFG-SIGNAL-GPS_L2C_ENA",
    0x10310032: "CFG-SIGNAL-GPS_L5_ENA",
    0x10310005: "CFG-SIGNAL-GAL_ENA",
    0x10310007: "CFG-SIGNAL-GAL_E1_ENA",
    0x1031000A: "CFG-SIGNAL-GAL_E5B_ENA",
    0x10310033: "CFG-SIGNAL-GAL_E5A_ENA",
    0x10310009: "CFG-SIGNAL-BDS_ENA",
    0x1031000D: "CFG-SIGNAL-BDS_B1_ENA",
    0x1031000E: "CFG-SIGNAL-BDS_B2_ENA",
    0x1031000F: "CFG-SIGNAL-BDS_B1C_ENA",
    0x10310028: "CFG-SIGNAL-BDS_B2A_ENA",
    0x1031001F: "CFG-SIGNAL-QZSS_ENA",
    0x10310012: "CFG-SIGNAL-QZSS_L1CA_ENA",
    0x10310014: "CFG-SIGNAL-QZSS_L1S_ENA",
    0x10310015: "CFG-SIGNAL-QZSS_L2C_ENA",
    0x10310016: "CFG-SIGNAL-QZSS_L5_ENA",
    0x10310025: "CFG-SIGNAL-GLO_ENA",
    0x10310018: "CFG-SIGNAL-GLO_L1_ENA",
    0x1031001A: "CFG-SIGNAL-GLO_L2_ENA",
    0x10310020: "CFG-SIGNAL-SBAS_ENA",
    0x10310046: "CFG-SIGNAL-SBAS_L1CA_ENA",
    0x10310026: "CFG-SIGNAL-NAVIC_ENA",
    0x10310027: "CFG-SIGNAL-NAVIC_L5_ENA",
    # Some guesses for unknowns observed
    0x10310017: "CFG-SIGNAL-?_0x17",
    0x1031001D: "CFG-SIGNAL-?_0x1D",
    0x10310021: "CFG-SIGNAL-?_0x21",
    0x10310022: "CFG-SIGNAL-?_0x22",
    0x10310024: "CFG-SIGNAL-?_0x24",
    # CFG-SIGNAL health overrides
    0x10320001: "CFG-SIGNAL-GPS_L1CA_HEALTH_OVRD",
    0x10320003: "CFG-SIGNAL-GPS_L2C_HEALTH_OVRD",
    0x10320004: "CFG-SIGNAL-GPS_L5_HEALTH_OVRD",
    # NAV
    0x20110011: "CFG-NAVSPG-FIXMODE",
    0x20110021: "CFG-NAVSPG-DYNMODEL",
    0x201100A1: "CFG-NAVSPG-INFIL_MINCNO",
    0x201100A2: "CFG-NAVSPG-INFIL_MINELEV",
    0x201100A3: "CFG-NAVSPG-INFIL_NCNOTHRS",
    0x201100AA: "CFG-NAVSPG-INFIL_CNOTHRS",
    0x401100C4: "CFG-NAVSPG-CONSTR_ALT",
    0x20140011: "CFG-NAVHPG-DGNSSMODE",
    # Rate
    0x30210001: "CFG-RATE-MEAS",
    0x30210002: "CFG-RATE-NAV",
    0x20210003: "CFG-RATE-TIMEREF",
    # Antenna
    0x10A3000C: "CFG-HW-ANT_CFG_VOLTCTRL",
    0x10A30010: "CFG-HW-ANT_CFG_OPEN",
    0x10A30011: "CFG-HW-ANT_CFG_SHORT",
    0x10A30013: "CFG-HW-ANT_ON_SHORTDET",
    # USB
    0x10650001: "CFG-USB-ENABLED",
    # NMEA
    0x20930031: "CFG-NMEA-PROTVER",
    0x20930002: "CFG-NMEA-MAXSVS",
    # INFMSG
    0x20920001: "CFG-INFMSG-UBX_I2C",
    0x20920002: "CFG-INFMSG-UBX_UART1",
    0x20920003: "CFG-INFMSG-UBX_UART2",
    0x20920004: "CFG-INFMSG-UBX_USB",
    0x20920005: "CFG-INFMSG-UBX_SPI",
    0x20920006: "CFG-INFMSG-NMEA_I2C",
    0x20920007: "CFG-INFMSG-NMEA_UART1",
    0x20920008: "CFG-INFMSG-NMEA_UART2",
    0x20920009: "CFG-INFMSG-NMEA_USB",
    0x2092000A: "CFG-INFMSG-NMEA_SPI",
}


def parse_hex(s):
    return bytes.fromhex(s.replace(' ', '').replace('\n', ''))


def value_size(size_class):
    return {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 8}.get(size_class, None)


def decode_dump(path):
    with open(path, 'r') as f:
        lines = [l.strip() for l in f if l.strip()]
    entries = {}
    mon_ver_info = None
    for line in lines:
        if 'MON-VER' in line:
            m = re.search(r'MON-VER\s*-\s*([0-9A-F ]+)', line, re.IGNORECASE)
            if m:
                data = parse_hex(m.group(1))
                sw_ver = data[4:34].split(b'\0')[0].decode('ascii', errors='replace')
                hw_ver = data[34:44].split(b'\0')[0].decode('ascii', errors='replace')
                exts = []
                offset = 44
                while offset + 30 <= len(data):
                    ext = data[offset:offset + 30].split(b'\0')[0].decode('ascii', errors='replace')
                    if ext.strip(): exts.append(ext)
                    offset += 30
                mon_ver_info = (sw_ver, hw_ver, exts)
            continue
        m = re.search(r'CFG-VALGET\s*-\s*([0-9A-F ]+)', line, re.IGNORECASE)
        if not m: continue
        data = parse_hex(m.group(1))
        if len(data) < 4 or data[0] != 0x06 or data[1] != 0x8B: continue
        length = int.from_bytes(data[2:4], 'little')
        payload = data[4:4 + length]
        if len(payload) < 4: continue
        offset = 4  # skip version+layer+position
        while offset < len(payload):
            if offset + 4 > len(payload): break
            key = int.from_bytes(payload[offset:offset + 4], 'little')
            offset += 4
            size_class = (key >> 28) & 0xF
            vsize = value_size(size_class)
            if vsize is None or offset + vsize > len(payload): break
            if vsize <= 4:
                val = int.from_bytes(payload[offset:offset + vsize], 'little')
            else:
                val = payload[offset:offset + vsize]
            offset += vsize
            entries[key] = (val, size_class)
    return entries, mon_ver_info


working, work_info = decode_dump(WORKING)
broken, brok_info = decode_dump(BROKEN)

print("=" * 78)
print("WORKING UNIT (receiver-config-old.txt)")
print("=" * 78)
if work_info:
    print(f"  SW: {work_info[0]}")
    for ext in work_info[2]: print(f"  Extension: {ext}")
print(f"  Total config entries: {len(working)}")

print("\n" + "=" * 78)
print("BROKEN UNIT (receiver-config.txt)")
print("=" * 78)
if brok_info:
    print(f"  SW: {brok_info[0]}")
    for ext in brok_info[2]: print(f"  Extension: {ext}")
print(f"  Total config entries: {len(broken)}")

# Diff
all_keys = sorted(set(working.keys()) | set(broken.keys()))
diffs = []
for key in all_keys:
    w = working.get(key)
    b = broken.get(key)
    if w != b:
        diffs.append((key, w, b))

print("\n" + "=" * 78)
print(f"DIFFERENCES: {len(diffs)} keys differ between working and broken")
print("=" * 78)
print(f"{'Key':>12}  {'Working':>10}  {'Broken':>10}  Name")
print("-" * 78)

# Sort diffs into known-name and unknown buckets
known_diffs = [(k, w, b) for k, w, b in diffs if k in KEY_NAMES]
unknown_diffs = [(k, w, b) for k, w, b in diffs if k not in KEY_NAMES]
# Within known, prioritize SIGNAL group then NAVSPG then others
signal_diffs = [(k, w, b) for k, w, b in known_diffs if (k >> 16) & 0xFF in (0x31, 0x32)]
nav_diffs = [(k, w, b) for k, w, b in known_diffs if (k >> 16) & 0xFF == 0x11]
other_known_diffs = [(k, w, b) for k, w, b in known_diffs if (k, w, b) not in signal_diffs and (k, w, b) not in nav_diffs]

def show_val(v):
    if v is None: return "(absent)"
    val, sz = v
    return str(val)

if signal_diffs:
    print("\n*** CFG-SIGNAL differences (most relevant for L5 hypothesis) ***")
    for k, w, b in signal_diffs:
        print(f"  0x{k:08X}  {show_val(w):>10}  {show_val(b):>10}  {KEY_NAMES[k]}")

if nav_diffs:
    print("\n*** CFG-NAVSPG differences ***")
    for k, w, b in nav_diffs:
        print(f"  0x{k:08X}  {show_val(w):>10}  {show_val(b):>10}  {KEY_NAMES[k]}")

if other_known_diffs:
    print("\n*** Other known-name differences ***")
    for k, w, b in other_known_diffs:
        print(f"  0x{k:08X}  {show_val(w):>10}  {show_val(b):>10}  {KEY_NAMES[k]}")

print(f"\n*** Unknown-name differences: {len(unknown_diffs)} ***")
# Group by top byte / group ID
unk_by_group = {}
for k, w, b in unknown_diffs:
    group = (k >> 16) & 0xFF
    unk_by_group.setdefault(group, []).append((k, w, b))
for group, items in sorted(unk_by_group.items()):
    print(f"  Group 0x{group:02X}: {len(items)} keys differ")
    for k, w, b in items[:5]:
        print(f"    0x{k:08X}  {show_val(w):>10}  {show_val(b):>10}")
    if len(items) > 5:
        print(f"    ...{len(items) - 5} more in this group")

# Summary
print(f"\nTotal: {len(diffs)} differences ({len(known_diffs)} named, {len(unknown_diffs)} unknown)")
