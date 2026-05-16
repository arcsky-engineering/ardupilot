#!/usr/bin/env python3
"""Decode a u-blox F9P CFG-VALGET dump from u-center.

Parses the hex byte stream, extracts each configuration key + value, and
maps known keys to human-readable names. Outputs the most diagnostically
useful CFG-SIGNAL-* entries highlighted (especially anything related to L5
and L2 bands, since L5 health-flag behavior is a leading hypothesis for the
periodic dropout symptom on Here4)."""
import re

INPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\receiver-config.txt"

# Known CFG-* key IDs (32-bit). From u-blox HPG 1.40 interface description.
# Format: 0xSSGGIIII where SS = storage size class, GG = group ID, IIII = item ID
# Storage: 0x10=bit/bool, 0x20=u8, 0x30=u16/i16, 0x40=u32/i32, 0x50=string/u64
KEY_NAMES = {
    # CFG-SIGNAL group (constellation/signal enables) - group 0x31
    0x10310001: "CFG-SIGNAL-GPS_ENA",          # Master GPS enable
    0x10310003: "CFG-SIGNAL-GPS_L1CA_ENA",
    0x10310004: "CFG-SIGNAL-GPS_L2C_ENA",
    0x10310032: "CFG-SIGNAL-GPS_L5_ENA",       # <<< L5 SUSPECT
    0x10310005: "CFG-SIGNAL-GAL_ENA",
    0x10310007: "CFG-SIGNAL-GAL_E1_ENA",
    0x1031000A: "CFG-SIGNAL-GAL_E5B_ENA",
    0x10310033: "CFG-SIGNAL-GAL_E5A_ENA",      # <<< Galileo L5/E5a (also L5 band)
    0x10310009: "CFG-SIGNAL-BDS_ENA",
    0x1031000D: "CFG-SIGNAL-BDS_B1_ENA",
    0x1031000E: "CFG-SIGNAL-BDS_B2_ENA",
    0x1031000F: "CFG-SIGNAL-BDS_B1C_ENA",
    0x10310028: "CFG-SIGNAL-BDS_B2A_ENA",
    0x1031001F: "CFG-SIGNAL-QZSS_ENA",
    0x10310012: "CFG-SIGNAL-QZSS_L1CA_ENA",
    0x10310014: "CFG-SIGNAL-QZSS_L1S_ENA",
    0x10310015: "CFG-SIGNAL-QZSS_L2C_ENA",
    0x10310016: "CFG-SIGNAL-QZSS_L5_ENA",      # <<< QZSS L5 too
    0x10310025: "CFG-SIGNAL-GLO_ENA",
    0x10310018: "CFG-SIGNAL-GLO_L1_ENA",
    0x1031001A: "CFG-SIGNAL-GLO_L2_ENA",
    0x10310020: "CFG-SIGNAL-SBAS_ENA",
    0x10310046: "CFG-SIGNAL-SBAS_L1CA_ENA",
    0x10310026: "CFG-SIGNAL-NAVIC_ENA",
    0x10310027: "CFG-SIGNAL-NAVIC_L5_ENA",     # <<< NavIC L5 (L5 band again)

    # CFG-SIGNAL health overrides (group 0x32)
    0x10320001: "CFG-SIGNAL-GPS_L1CA_HEALTH_OVRD",
    0x10320003: "CFG-SIGNAL-GPS_L2C_HEALTH_OVRD",
    0x10320004: "CFG-SIGNAL-GPS_L5_HEALTH_OVRD",  # <<< L5 health override

    # CFG-NAVSPG group (navigation)
    0x20110011: "CFG-NAVSPG-FIXMODE",
    0x20110021: "CFG-NAVSPG-DYNMODEL",
    0x20110017: "CFG-NAVSPG-USRDAT",
    0x201100A1: "CFG-NAVSPG-INFIL_MINCNO",
    0x301100A4: "CFG-NAVSPG-INFIL_NCNOTHRS",
    0x201100AA: "CFG-NAVSPG-INFIL_CNOTHRS",
    0x201100A2: "CFG-NAVSPG-INFIL_MINELEV",
    0x201100A3: "CFG-NAVSPG-INFIL_NCNOTHRS",
    0x401100C4: "CFG-NAVSPG-CONSTR_ALT",

    # CFG-RATE
    0x30210001: "CFG-RATE-MEAS",
    0x30210002: "CFG-RATE-NAV",
    0x20210003: "CFG-RATE-TIMEREF",

    # CFG-ITFM (jamming detection)
    0x10410001: "CFG-ITFM-BBTHRESHOLD",
    0x10410002: "CFG-ITFM-CWTHRESHOLD",

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

    # CFG-PM (power management)
    0x20d00002: "CFG-PM-WAITTIMEFIX",
    0x40d00003: "CFG-PM-UPDATEEPH",

    # CFG-UART1
    0x40520001: "CFG-UART1-BAUDRATE",

    # CFG-USB
    0x10650001: "CFG-USB-ENABLED",

    # CFG-NMEA
    0x20930031: "CFG-NMEA-PROTVER",
    0x20930002: "CFG-NMEA-MAXSVS",

    # CFG-MSGOUT-NMEA (USB messages)
    0x209100bc: "CFG-MSGOUT-NMEA_ID_DTM_USB",

    # CFG-HW (anti-jamming receiver)
    0x10a3000c: "CFG-HW-ANT_CFG_VOLTCTRL",
    0x10a30010: "CFG-HW-ANT_CFG_OPEN",
    0x10a30011: "CFG-HW-ANT_CFG_SHORT",
    0x10a30013: "CFG-HW-ANT_ON_SHORTDET",

    # CFG-TP (time pulse)
    0x4005000a: "CFG-TP-PERIOD_TP1",
    0x40050030: "CFG-TP-LEN_TP1",

    # CFG-SFCORE / CFG-SFIMU (sensor fusion - not used here)

    # CFG-RTCM, CFG-MSGOUT etc - too many to list

    # CFG-NAVHPG (high precision)
    0x20140011: "CFG-NAVHPG-DGNSSMODE",

    # CFG-MOT (motion)
    0x40250011: "CFG-MOT-GNSSSPEED_THRS",
    0x30250038: "CFG-MOT-GNSSDIST_THRS",
}


def parse_hex(s):
    """Parse a hex string like '06 8B 44 01' into bytes."""
    return bytes.fromhex(s.replace(' ', '').replace('\n', ''))


def parse_key(data, offset):
    """Read 4-byte little-endian key starting at offset.
    Returns (key_value, size_class)."""
    if offset + 4 > len(data):
        return None, None
    key = int.from_bytes(data[offset:offset + 4], 'little')
    size_class = (key >> 28) & 0xF
    return key, size_class


def value_size(size_class):
    """Bytes consumed by value for a given storage size class."""
    return {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 8}.get(size_class, None)


def decode_cfg_valget(payload):
    """Decode the content of a UBX-CFG-VALGET response payload."""
    # First 4 bytes: version (1) + layer (1) + position (2)
    version = payload[0]
    layer = payload[1]
    position = int.from_bytes(payload[2:4], 'little')
    entries = []
    offset = 4
    while offset < len(payload):
        key, size_class = parse_key(payload, offset)
        if key is None: break
        offset += 4
        vsize = value_size(size_class)
        if vsize is None:
            break
        if offset + vsize > len(payload):
            break
        if vsize == 1:
            val = payload[offset]
        elif vsize == 2:
            val = int.from_bytes(payload[offset:offset + 2], 'little')
        elif vsize == 4:
            val = int.from_bytes(payload[offset:offset + 4], 'little')
        elif vsize == 8:
            val = int.from_bytes(payload[offset:offset + 8], 'little')
        else:
            val = payload[offset:offset + vsize]
        offset += vsize
        entries.append((key, val, size_class))
    return version, layer, position, entries


# Parse the file
with open(INPUT, 'r') as f:
    content = f.read()

lines = [l.strip() for l in content.split('\n') if l.strip()]

# First line: MON-VER (firmware info)
print("=" * 70)
print("RECEIVER INFO (from MON-VER)")
print("=" * 70)
mon_ver_match = re.search(r'MON-VER\s*-\s*([0-9A-F ]+)', lines[0], re.IGNORECASE)
if mon_ver_match:
    bytes_str = mon_ver_match.group(1)
    data = parse_hex(bytes_str)
    # Format: msgClass(1) msgId(1) length(2) sw(30 ASCII) hw(10 ASCII) extensions(30 ASCII each)
    sw_ver = data[4:34].split(b'\0')[0].decode('ascii', errors='replace')
    hw_ver = data[34:44].split(b'\0')[0].decode('ascii', errors='replace')
    print(f"SW version: {sw_ver}")
    print(f"HW version: {hw_ver}")
    # Extensions (30 bytes each)
    offset = 44
    while offset + 30 <= len(data):
        ext = data[offset:offset + 30].split(b'\0')[0].decode('ascii', errors='replace')
        if ext.strip():
            print(f"Extension: {ext}")
        offset += 30

# Combine all CFG-VALGET lines into entries
print("\n" + "=" * 70)
print("ALL CONFIGURATION ENTRIES (KEY = VALUE, with name if known)")
print("=" * 70)
all_entries = []
for line in lines[1:]:
    m = re.search(r'CFG-VALGET\s*-\s*([0-9A-F ]+)', line, re.IGNORECASE)
    if not m: continue
    bytes_str = m.group(1)
    data = parse_hex(bytes_str)
    # Skip header (class 06, id 8B, length 2 bytes)
    if len(data) < 4: continue
    if data[0] != 0x06 or data[1] != 0x8B: continue
    length = int.from_bytes(data[2:4], 'little')
    payload = data[4:4 + length]
    version, layer, position, entries = decode_cfg_valget(payload)
    for key, val, size in entries:
        all_entries.append((key, val, size))

# Dedup (same key may appear in multiple responses)
seen = {}
for key, val, size in all_entries:
    seen[key] = (val, size)
all_entries = sorted(seen.items())

# Look for key groups of interest
SIGNAL_KEYS = [(k, v) for k, (v, s) in all_entries if (k >> 16) & 0xFF in (0x31, 0x32)]
SIGNAL_KEYS.sort()

print("\n--- CFG-SIGNAL-* AND CFG-SIGNAL-*_HEALTH_OVRD (constellation/signal enables) ---")
print(f"{'Key':>12} {'Value':>6} {'Name':<40}")
for key, val in SIGNAL_KEYS:
    name = KEY_NAMES.get(key, '?')
    flag = ''
    if 'L5' in name:
        flag = ' <<< L5 BAND'
    elif 'E5' in name:
        flag = ' <<< L5 BAND (Galileo)'
    elif 'L2' in name:
        flag = ' <<< L2 BAND'
    print(f"0x{key:08X} {val:>6} {name:<40}{flag}")

# Show all known interesting keys with values
print("\n--- ALL KNOWN KEYS (highlighted by name) ---")
known = [(k, v, s) for k, (v, s) in all_entries if k in KEY_NAMES]
for key, val, size in sorted(known):
    name = KEY_NAMES[key]
    if size == 1:
        val_str = "ENABLED" if val else "disabled"
        val_disp = f"{val} ({val_str})"
    else:
        val_disp = str(val)
    print(f"  0x{key:08X}  {KEY_NAMES[key]:<40} = {val_disp}")

# Unknown keys count
unknown = [k for k, (v, s) in all_entries if k not in KEY_NAMES]
print(f"\n--- UNKNOWN KEYS (not in our lookup): {len(unknown)} ---")
print(f"Total decoded entries: {len(all_entries)}")
print(f"Known names matched:   {len(known)}")
