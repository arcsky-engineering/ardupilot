#!/usr/bin/env python3
"""Decode a raw u-center CFG-VALGET hex dump using the full u-blox HPGL1L5 1.40
interface description key table (extracted from the official PDF).

Produces a fully-named human-readable text file with name, type, value, and description."""
import re, sys, os, importlib.util

INPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\test-fix.txt"
OUTPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\test-fix-fully-decoded.txt"

# Load the extracted key table
KEY_NAMES_PATH = os.path.join(os.path.dirname(__file__), "ublox_key_names.py")
spec = importlib.util.spec_from_file_location("ublox_key_names", KEY_NAMES_PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
KEY_NAMES = dict(mod.KEY_NAMES)

# Augment with known keys from the GPS L5 app note (UBX-21038688) and other
# sources that aren't documented in the main interface description PDF.
KEY_NAMES.update({
    0x10320001: ("CFG-SIGNAL-GPS_L5_HEALTH_OVRD", "L", "Override GPS L5 health flag - if set, L5 SVs broadcasting unhealthy are still used in navigation solution. Required for pre-operational L5 (per UBX-21038688)."),
})

# Type → byte-size table
TYPE_SIZE = {
    'L': 1, 'U1': 1, 'I1': 1, 'X1': 1, 'E1': 1,
    'U2': 2, 'I2': 2, 'X2': 2, 'E2': 2,
    'U4': 4, 'I4': 4, 'X4': 4, 'E4': 4, 'R4': 4,
    'U8': 8, 'I8': 8, 'X8': 8, 'R8': 8,
}


def parse_hex(s):
    return bytes.fromhex(s.replace(' ', '').replace('\n', ''))


def value_size_from_storage(size_class):
    """Storage size class from top nibble of key ID."""
    return {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 8}.get(size_class, None)


def format_value(val, type_str, size):
    """Pretty-print a value based on its u-blox type."""
    if type_str == 'L':
        return f"{val} ({'true' if val else 'false'})"
    if type_str in ('I1',):
        if val >= 0x80: val -= 0x100
        return str(val)
    if type_str in ('I2',):
        if val >= 0x8000: val -= 0x10000
        return str(val)
    if type_str in ('I4',):
        if val >= 0x80000000: val -= 0x100000000
        return str(val)
    if type_str in ('X1',):
        return f"0x{val:02X}"
    if type_str in ('X2',):
        return f"0x{val:04X}"
    if type_str in ('X4',):
        return f"0x{val:08X}"
    if type_str in ('X8',):
        return f"0x{val:016X}"
    if type_str in ('U8', 'I8'):
        return f"{val} (0x{val:016X})"
    if type_str == 'R4':
        import struct
        try:
            f = struct.unpack('<f', val.to_bytes(4, 'little'))[0]
            return f"{f:g}"
        except Exception:
            return str(val)
    if type_str == 'R8':
        import struct
        try:
            f = struct.unpack('<d', val.to_bytes(8, 'little'))[0]
            return f"{f:g}"
        except Exception:
            return str(val)
    return str(val)


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
            vsize = value_size_from_storage(size_class)
            if vsize is None or offset + vsize > len(payload): break
            val = int.from_bytes(payload[offset:offset + vsize], 'little')
            offset += vsize
            entries[key] = (val, size_class, vsize)
    return entries, mon_ver_info


def group_name(group_id):
    """Map u-blox CFG group ID (second byte of key, bits 16-23) to group name."""
    # Built from KEY_NAMES — for each group, find the most common prefix
    from collections import Counter
    if not hasattr(group_name, '_cache'):
        cache = {}
        for k, (n, _, _) in KEY_NAMES.items():
            g = (k >> 16) & 0xFF
            # Extract the second segment, e.g. CFG-SIGNAL-... → SIGNAL
            parts = n.split('-')
            if len(parts) >= 2:
                cache.setdefault(g, Counter())[parts[1]] += 1
        # Most common name per group
        group_name._cache = {g: c.most_common(1)[0][0] for g, c in cache.items()}
    return group_name._cache.get(group_id, '?')


entries, mon_ver = decode_dump(INPUT)
print(f"Decoded {len(entries)} entries")

with open(OUTPUT, 'w', encoding='utf-8') as f:
    f.write("# u-blox HPG L1L5 1.40 configuration dump (full decode)\n")
    f.write(f"# Source: {os.path.basename(INPUT)}\n")
    f.write(f"# Total entries: {len(entries)}\n")
    f.write(f"# Named keys: {sum(1 for k in entries if k in KEY_NAMES)} / {len(entries)} ({100*sum(1 for k in entries if k in KEY_NAMES)/len(entries):.1f}%)\n\n")

    if mon_ver:
        f.write("# === MON-VER (firmware info) ===\n")
        f.write(f"# SW: {mon_ver[0]}\n")
        f.write(f"# HW: {mon_ver[1]}\n")
        for ext in mon_ver[2]:
            f.write(f"# Ext: {ext}\n")
        f.write("\n")

    # Group entries
    by_group = {}
    for key, (val, size_cls, vsize) in entries.items():
        group_id = (key >> 16) & 0xFF
        by_group.setdefault(group_id, []).append((key, val, size_cls, vsize))

    # Identify "all-default-value" groups (groups where every entry is 0 or default)
    # so we can summarize them rather than printing every entry
    name_lookup = KEY_NAMES

    for group_id in sorted(by_group.keys()):
        gname = group_name(group_id)
        rows = sorted(by_group[group_id])
        named_count = sum(1 for k, *_ in rows if k in name_lookup)
        f.write(f"\n# ============================================================\n")
        f.write(f"# === CFG-{gname} (group 0x{group_id:02X}) - {len(rows)} keys ({named_count} named) ===\n")
        f.write(f"# ============================================================\n\n")
        for key, val, size_cls, vsize in rows:
            if key in name_lookup:
                name, type_str, desc = name_lookup[key]
                val_str = format_value(val, type_str, vsize)
                # Wrap long descriptions
                desc_short = desc if len(desc) < 120 else desc[:117] + "..."
                f.write(f"{name:<60} = {val_str:<16}  # {type_str:<3}\n")
                if desc_short.strip():
                    f.write(f"{'':<60}     {desc_short}\n")
            else:
                # Fallback: still show the value, just with hex key
                val_str = str(val) if size_cls < 5 else f"0x{val:016X}"
                f.write(f"0x{key:08X}{'':<52} = {val_str:<16}  # (unknown, size_class={size_cls})\n")
        f.write("\n")

print(f"\nWrote {OUTPUT}")
print(f"Named: {sum(1 for k in entries if k in KEY_NAMES)} / {len(entries)} ({100*sum(1 for k in entries if k in KEY_NAMES)/len(entries):.1f}%)")
print(f"Unnamed: {sum(1 for k in entries if k not in KEY_NAMES)}")
