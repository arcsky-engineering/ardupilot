#!/usr/bin/env python3
"""Decode and compare the two u-center CFG dumps from the forum.

Files:
  Here4ShippedU-BloxModulParameter.txt    — as-shipped factory state
  Here4WorkaroundU-BloxModulParameter.txt — after the "workaround" was applied

Output: every parameter that differs, with key name, description, and values.
"""
import os
import re

DIR = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/Files"
SHIPPED = os.path.join(DIR, "Here4ShippedU-BloxModulParameter.txt")
WORKAROUND = os.path.join(DIR, "Here4WorkaroundU-BloxModulParameter.txt")

# Import the key database we extracted from the u-blox spec PDF
import sys
sys.path.insert(0, r"C:/GitHub/ardupilot-xplorer/Tools")
from ublox_key_names import KEY_NAMES as KEYS

# u-blox VALGET key-type encoding (high nibble of first byte of key id):
#   0x10  L (boolean, 1B)
#   0x20  U1 (1B)
#   0x30  U2 (2B)
#   0x40  U4 (4B)
#   0x50  U8 (8B)
TYPE_LEN = {0x10: 1, 0x20: 1, 0x30: 2, 0x40: 4, 0x50: 8}


def parse_cfg_valget_response(payload):
    """Yield (key_id, value_bytes) for each entry in a CFG-VALGET response payload.

    Payload layout (after CLS=06 ID=8B LEN_LO LEN_HI are stripped):
      version(1) layers(1) position(2) [ key(4) value(variable) ]*
    """
    # First 8 bytes are: ver(1) layers(1) position(2) reserved... actually
    # u-blox spec for CFG-VALGET response:
    #   version(1) layers(1) position(2)  -- 4 bytes
    # Then list of (keyId(4-byte LE), value(size-by-type)) pairs
    i = 4
    while i < len(payload):
        if i + 4 > len(payload): break
        key = (payload[i] | (payload[i+1] << 8) | (payload[i+2] << 16) | (payload[i+3] << 24))
        i += 4
        # type byte is the high byte of the key
        type_code = payload[i-1]  # actually the 4th byte of the key
        # Wait, the type indicator is the SECOND byte from the top in u-blox key encoding:
        # key = (type_code << 28) | (group_id << 16) | item_id
        # actually the layout is different. Let me use the standard parsing:
        # bits 31..28 = size class (1=L, 2=U1/X1/E1, 3=U2/X2, 4=U4/X4/I4/R4, 5=U8/R8)
        # So size code is (key >> 28) & 0xF
        size_code = (key >> 28) & 0xF
        size_map = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}
        n_bytes = size_map.get(size_code, 1)
        if i + n_bytes > len(payload): break
        val = payload[i:i + n_bytes]
        i += n_bytes
        yield key, bytes(val)


def parse_file(path):
    """Returns dict mapping key_id -> value_bytes."""
    all_kv = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line.startswith("CFG-VALGET -"):
                continue
            # Split tokens: "CFG-VALGET", "-", then hex bytes
            tokens = line.split()
            hex_bytes = []
            for t in tokens:
                if re.fullmatch(r"[0-9A-Fa-f]{2}", t):
                    hex_bytes.append(int(t, 16))
            if len(hex_bytes) < 8: continue
            # First two bytes are CLS=06, ID=8B
            # Next two are length LE
            # Then payload
            cls = hex_bytes[0]; mid = hex_bytes[1]
            if not (cls == 0x06 and mid == 0x8B): continue
            payload = bytes(hex_bytes[4:])
            for key, val in parse_cfg_valget_response(payload):
                all_kv[key] = val
    return all_kv


def value_str(val_bytes):
    if len(val_bytes) == 1:
        return f"0x{val_bytes[0]:02x} ({val_bytes[0]})"
    elif len(val_bytes) == 2:
        v = int.from_bytes(val_bytes, "little")
        return f"0x{v:04x} ({v})"
    elif len(val_bytes) == 4:
        v = int.from_bytes(val_bytes, "little")
        return f"0x{v:08x} ({v})"
    elif len(val_bytes) == 8:
        v = int.from_bytes(val_bytes, "little")
        return f"0x{v:016x} ({v})"
    return val_bytes.hex()


print("=" * 80)
print("u-blox CFG parameter comparison: shipped Here4 vs. workaround state")
print("=" * 80)

shipped = parse_file(SHIPPED)
workaround = parse_file(WORKAROUND)

print(f"\nShipped state: {len(shipped)} parameters")
print(f"Workaround state: {len(workaround)} parameters")

shipped_keys = set(shipped.keys())
workaround_keys = set(workaround.keys())
common = shipped_keys & workaround_keys
only_in_shipped = shipped_keys - workaround_keys
only_in_workaround = workaround_keys - shipped_keys

print(f"Parameters present only in shipped: {len(only_in_shipped)}")
print(f"Parameters present only in workaround: {len(only_in_workaround)}")
print(f"Parameters common: {len(common)}")

# Find value differences
diffs = []
for key in sorted(common):
    if shipped[key] != workaround[key]:
        diffs.append(key)

print(f"\n{'='*80}")
print(f"PARAMETERS THAT DIFFER ({len(diffs)} total):")
print(f"{'='*80}")
if not diffs:
    print("  (no value differences in common parameters)")

for key in diffs:
    name, dtype, desc = KEYS.get(key, (f"UNKNOWN_KEY_0x{key:08x}", "?", "(not in DB)"))
    print(f"\n  Key: 0x{key:08x}  {name}")
    print(f"  Type: {dtype}")
    print(f"  Description: {desc}")
    print(f"  Shipped value:    {value_str(shipped[key])}")
    print(f"  Workaround value: {value_str(workaround[key])}")

# Parameters only present in one or the other (would be very unusual for CFG-VALGET dumps)
if only_in_shipped:
    print(f"\n=== Only in shipped (not workaround) ===")
    for key in sorted(only_in_shipped)[:20]:
        name = KEYS.get(key, (f"UNKNOWN_0x{key:08x}",))[0]
        print(f"  0x{key:08x}  {name}  = {value_str(shipped[key])}")
if only_in_workaround:
    print(f"\n=== Only in workaround (not shipped) ===")
    for key in sorted(only_in_workaround)[:20]:
        name = KEYS.get(key, (f"UNKNOWN_0x{key:08x}",))[0]
        print(f"  0x{key:08x}  {name}  = {value_str(workaround[key])}")
