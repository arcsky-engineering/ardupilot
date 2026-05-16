#!/usr/bin/env python3
"""Convert a raw u-center CFG-VALGET hex dump (receiver-config.txt format) into
a human-readable key=value text file suitable for forum sharing and diffing.

Usage: edit INPUT and OUTPUT paths below, then run."""
import re, sys, os

# Default: convert the user's broken-unit dump
INPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\test-fix.txt"
OUTPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\test-fix-readable.txt"

# u-blox CFG-* key name table for HPGL1L5 1.40+ firmware.
# Covers the most relevant groups. Unknown keys are output as 0xKKKKKKKK.
# Key encoding: top nibble = storage size (1=bool, 2=u8, 3=u16, 4=u32, 5=u64/str, 6=double)
# Next byte = group ID; lower 16 bits = item ID.
KEY_NAMES = {
    # CFG-SIGNAL (constellation/signal enables)
    0x10310001: "CFG-SIGNAL-GPS_ENA",
    0x10310003: "CFG-SIGNAL-GPS_L1CA_ENA",
    0x10310004: "CFG-SIGNAL-GPS_L5_ENA",
    0x10310005: "CFG-SIGNAL-SBAS_L1CA_ENA",
    0x10310007: "CFG-SIGNAL-GAL_E1_ENA",
    0x10310009: "CFG-SIGNAL-GAL_E5A_ENA",
    0x1031000A: "CFG-SIGNAL-GAL_E5B_ENA",
    0x1031000B: "CFG-SIGNAL-GAL_E6_ENA",
    0x1031000D: "CFG-SIGNAL-BDS_B1_ENA",
    0x1031000E: "CFG-SIGNAL-BDS_B2_ENA",
    0x1031000F: "CFG-SIGNAL-BDS_B1C_ENA",
    0x10310012: "CFG-SIGNAL-QZSS_L1CA_ENA",
    0x10310014: "CFG-SIGNAL-QZSS_L1S_ENA",
    0x10310015: "CFG-SIGNAL-QZSS_L1CB_ENA",
    0x10310017: "CFG-SIGNAL-QZSS_L5_ENA",
    0x10310018: "CFG-SIGNAL-GLO_L1_ENA",
    0x1031001A: "CFG-SIGNAL-GLO_L2_ENA",
    0x1031001D: "CFG-SIGNAL-NAVIC_L5_ENA",
    0x1031001F: "CFG-SIGNAL-GPS_ENA",  # Sometimes used here
    0x10310020: "CFG-SIGNAL-SBAS_ENA",
    0x10310021: "CFG-SIGNAL-GAL_ENA",
    0x10310022: "CFG-SIGNAL-BDS_ENA",
    0x10310024: "CFG-SIGNAL-QZSS_ENA",
    0x10310025: "CFG-SIGNAL-GLO_ENA",
    0x10310026: "CFG-SIGNAL-NAVIC_ENA",
    0x10310027: "CFG-SIGNAL-NAVIC_L5_ENA",
    0x10310028: "CFG-SIGNAL-BDS_B2A_ENA",
    0x10310046: "CFG-SIGNAL-SBAS_L1CA_ENA",
    # CFG-SIGNAL health overrides (group 0x32)
    0x10320001: "CFG-SIGNAL-GPS_L5_HEALTH_OVRD",
    # CFG-NAV2
    0x10170001: "CFG-NAV2-OUT_ENABLED",
    0x10170002: "CFG-NAV2-SBAS_USE_INTEGRITY",
    # CFG-NAVSPG
    0x20110011: "CFG-NAVSPG-FIXMODE",
    0x20110021: "CFG-NAVSPG-DYNMODEL",
    0x20110017: "CFG-NAVSPG-USRDAT",
    0x201100A1: "CFG-NAVSPG-INFIL_MINCNO",
    0x201100A2: "CFG-NAVSPG-INFIL_MINELEV",
    0x201100A3: "CFG-NAVSPG-INFIL_NCNOTHRS",
    0x201100A4: "CFG-NAVSPG-INFIL_CNOTHRS",
    0x201100AA: "CFG-NAVSPG-INFIL_CNOTHRS2",
    0x201100C4: "CFG-NAVSPG-CONSTR_ALT",
    # CFG-NAVHPG
    0x20140011: "CFG-NAVHPG-DGNSSMODE",
    # CFG-RATE
    0x30210001: "CFG-RATE-MEAS",
    0x30210002: "CFG-RATE-NAV",
    0x20210003: "CFG-RATE-TIMEREF",
    # CFG-SBAS group (0x36)
    0x10360002: "CFG-SBAS-USE_RANGING",
    0x10360003: "CFG-SBAS-USE_DIFFCORR",
    0x10360004: "CFG-SBAS-USE_INTEGRITY",
    0x10360005: "CFG-SBAS-USE_TESTMODE",
    0x10360007: "CFG-SBAS-USE_IONOONLY",
    0x20360008: "CFG-SBAS-ACCEPT_NOT_IN_PRNMASK",
    0x50360009: "CFG-SBAS-PRNSCANMASK",
    # CFG-ITFM (anti-jamming)
    0x10410001: "CFG-ITFM-BBTHRESHOLD",
    0x10410002: "CFG-ITFM-CWTHRESHOLD",
    0x10410013: "CFG-ITFM-ENABLE",
    0x10410014: "CFG-ITFM-ANT_SETTING",
    0x10410015: "CFG-ITFM-ENABLE_AUX",
    # CFG-INFMSG
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
    # CFG-NMEA
    0x20930031: "CFG-NMEA-PROTVER",
    0x20930002: "CFG-NMEA-MAXSVS",
    # CFG-UART1
    0x40520001: "CFG-UART1-BAUDRATE",
    0x10520002: "CFG-UART1-STOPBITS",
    0x10520003: "CFG-UART1-DATABITS",
    0x10520004: "CFG-UART1-PARITY",
    # CFG-UART2
    0x40530001: "CFG-UART2-BAUDRATE",
    # CFG-USB
    0x10650001: "CFG-USB-ENABLED",
    # CFG-HW
    0x10A3000C: "CFG-HW-ANT_CFG_VOLTCTRL",
    0x10A30010: "CFG-HW-ANT_CFG_OPEN",
    0x10A30011: "CFG-HW-ANT_CFG_SHORT",
    0x10A30013: "CFG-HW-ANT_ON_SHORTDET",
}

GROUP_NAMES = {
    0x01: "CFG-?", 0x03: "CFG-?", 0x04: "CFG-?", 0x05: "CFG-?",
    0x0B: "CFG-?", 0x11: "CFG-NAVSPG", 0x14: "CFG-NAVHPG", 0x17: "CFG-NAV2",
    0x21: "CFG-RATE", 0x22: "CFG-INFMSG", 0x24: "CFG-?", 0x25: "CFG-?",
    0x31: "CFG-SIGNAL", 0x32: "CFG-SIGNAL", 0x33: "CFG-?", 0x34: "CFG-?",
    0x35: "CFG-?", 0x36: "CFG-SBAS", 0x37: "CFG-?", 0x38: "CFG-?",
    0x51: "CFG-MSGOUT", 0x52: "CFG-UART1", 0x53: "CFG-UART2",
    0x64: "CFG-?", 0x65: "CFG-USB", 0x71: "CFG-?", 0x72: "CFG-?",
    0x73: "CFG-?", 0x74: "CFG-?", 0x75: "CFG-?", 0x76: "CFG-?",
    0x77: "CFG-?", 0x78: "CFG-?", 0x79: "CFG-?", 0x7A: "CFG-?",
    0x81: "CFG-?", 0x82: "CFG-?", 0x83: "CFG-?", 0x84: "CFG-?", 0x85: "CFG-?",
    0x91: "CFG-MSGOUT", 0x92: "CFG-INFMSG", 0x93: "CFG-NMEA",
    0xA1: "CFG-?", 0xA2: "CFG-?", 0xA3: "CFG-HW", 0xA4: "CFG-?",
    0xA5: "CFG-?", 0xA7: "CFG-?", 0xC5: "CFG-?", 0xC6: "CFG-?", 0xC7: "CFG-?",
    0xDE: "CFG-?", 0xF6: "CFG-?",
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
        offset = 4
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
                val = int.from_bytes(payload[offset:offset + vsize], 'little')
            offset += vsize
            entries[key] = (val, size_class)
    return entries, mon_ver_info


entries, mon_ver = decode_dump(INPUT)

with open(OUTPUT, 'w') as f:
    f.write("# u-blox configuration dump (converted from u-center receiver-config.txt raw hex format)\n")
    f.write(f"# Source: {os.path.basename(INPUT)}\n")
    f.write(f"# Total entries: {len(entries)}\n\n")
    if mon_ver:
        f.write("# === MON-VER (firmware info) ===\n")
        f.write(f"# SW: {mon_ver[0]}\n")
        f.write(f"# HW: {mon_ver[1]}\n")
        for ext in mon_ver[2]:
            f.write(f"# Ext: {ext}\n")
        f.write("\n")

    # Group entries by their group ID
    by_group = {}
    for key, (val, size) in entries.items():
        group_id = (key >> 16) & 0xFF
        by_group.setdefault(group_id, []).append((key, val, size))

    for group_id in sorted(by_group.keys()):
        group_name = GROUP_NAMES.get(group_id, f"GROUP_0x{group_id:02X}")
        rows = sorted(by_group[group_id])
        f.write(f"# === Group 0x{group_id:02X} ({group_name}) - {len(rows)} keys ===\n")
        for key, val, size in rows:
            name = KEY_NAMES.get(key, f"0x{key:08X}")
            if size == 1:
                val_str = "1 (true)" if val else "0 (false)"
            elif size == 5:
                val_str = f"0x{val:016X}"
            else:
                val_str = str(val)
            f.write(f"{name:<45} = {val_str}\n")
        f.write("\n")

print(f"Wrote {OUTPUT}")
print(f"  Total entries: {len(entries)}")
print(f"  Named keys: {sum(1 for k in entries if k in KEY_NAMES)}")
print(f"  Unnamed keys: {sum(1 for k in entries if k not in KEY_NAMES)} (shown as 0xKKKKKKKK)")
