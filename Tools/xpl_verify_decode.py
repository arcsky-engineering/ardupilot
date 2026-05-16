#!/usr/bin/env python3
"""Verify UBX decoding against u-blox HPG L1L5 1.40 spec by:
  1. Extracting one real NAV-PVT and one MON-HW message from a capture
  2. Showing the raw bytes
  3. Decoding each field by hand using spec-defined offsets
  4. Verifying the Fletcher-16 checksum independently
"""
import struct

CAPTURE = r"C:/Users/js.LAPTOP-OK5IHI7D/Desktop/ublox_node121_20260513-181219.ubx"


def find_first_frame(data, target_cls, target_id):
    """Find the first valid UBX frame matching class/id. Returns (start_offset, full_frame)."""
    i = 0
    while i < len(data) - 7:
        if data[i] == 0xB5 and data[i + 1] == 0x62:
            cls = data[i + 2]; mid = data[i + 3]
            length = data[i + 4] | (data[i + 5] << 8)
            if i + 8 + length > len(data):
                i += 1; continue
            # checksum check
            ck_a = ck_b = 0
            for b in data[i + 2:i + 6 + length]:
                ck_a = (ck_a + b) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if data[i + 6 + length] == ck_a and data[i + 7 + length] == ck_b:
                if cls == target_cls and mid == target_id:
                    return i, data[i:i + 8 + length]
                i += 8 + length
                continue
        i += 1
    return None, None


with open(CAPTURE, "rb") as f:
    data = f.read()


def hexdump(label, b):
    print(f"\n{label} (total {len(b)} bytes):")
    for off in range(0, len(b), 16):
        chunk = b[off:off + 16]
        hex_part = " ".join(f"{c:02x}" for c in chunk)
        print(f"  {off:04x}:  {hex_part}")


def verify_checksum(frame):
    """Recompute Fletcher-16 and compare to frame's claimed checksum."""
    body = frame[2:-2]  # exclude sync (2) and checksum (2)
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    expected_a, expected_b = frame[-2], frame[-1]
    return ck_a, ck_b, expected_a, expected_b


print("=" * 70)
print("REFERENCE: u-blox F9 HPG L1L5 1.40 Interface Description (UBX-23006991)")
print("=" * 70)

# ============================================================
# Part 1: NAV-PVT decode
# ============================================================
print("\n\n#### MESSAGE 1: UBX-NAV-PVT (class=0x01, id=0x07) ####")
print("Per spec Section 'UBX-NAV-PVT': 92-byte payload, navigation PVT solution.")

off, frame = find_first_frame(data, 0x01, 0x07)
print(f"\nFound at file offset 0x{off:x} (frame is offset 0..{len(frame)-1}, total {len(frame)} bytes)")
hexdump("Raw frame bytes", frame)

# Decompose the envelope
print("\n--- Envelope decode ---")
print(f"  bytes [0:2]  sync chars:    {frame[0]:#04x} {frame[1]:#04x}  (expected 0xb5 0x62, ASCII 'µb')")
print(f"  byte  [2]    message class: {frame[2]:#04x}  (NAV = 0x01)")
print(f"  byte  [3]    message id:    {frame[3]:#04x}  (PVT = 0x07)")
length = frame[4] | (frame[5] << 8)
print(f"  bytes [4:6]  payload length: {length} (little-endian uint16)")
print(f"  bytes [6:{6+length}]  payload ({length} bytes)")
print(f"  bytes [{6+length}:{6+length+2}]  checksum: {frame[-2]:#04x} {frame[-1]:#04x}")

# Verify checksum
ck_a, ck_b, exp_a, exp_b = verify_checksum(frame)
print(f"\n--- Fletcher-16 checksum verification ---")
print(f"  Computed over bytes [2..{6+length-1}] (class + id + length + payload):")
print(f"  computed ck_a={ck_a:#04x}, ck_b={ck_b:#04x}")
print(f"  expected ck_a={exp_a:#04x}, ck_b={exp_b:#04x}")
print(f"  {'PASS' if (ck_a == exp_a and ck_b == exp_b) else 'FAIL'}")

# Field-by-field decode
pl = frame[6:6+length]
print("\n--- NAV-PVT payload decode (offsets per spec) ---")

iTOW = struct.unpack("<I", pl[0:4])[0]
year = struct.unpack("<H", pl[4:6])[0]
month = pl[6]; day = pl[7]; hour = pl[8]; minute = pl[9]; sec = pl[10]
valid = pl[11]
tAcc = struct.unpack("<I", pl[12:16])[0]
nano = struct.unpack("<i", pl[16:20])[0]
fixType = pl[20]
flags = pl[21]; flags2 = pl[22]
numSV = pl[23]
# Skip lon/lat/heights to avoid exposing user location
hAcc = struct.unpack("<I", pl[40:44])[0]
vAcc = struct.unpack("<I", pl[44:48])[0]
velN = struct.unpack("<i", pl[48:52])[0]
velE = struct.unpack("<i", pl[52:56])[0]
velD = struct.unpack("<i", pl[56:60])[0]
gSpeed = struct.unpack("<i", pl[60:64])[0]
headMot = struct.unpack("<i", pl[64:68])[0]
sAcc = struct.unpack("<I", pl[68:72])[0]
headAcc = struct.unpack("<I", pl[72:76])[0]
pDOP_raw = struct.unpack("<H", pl[76:78])[0]
flags3 = struct.unpack("<H", pl[78:80])[0]
# bytes 80-84 reserved
headVeh = struct.unpack("<i", pl[84:88])[0]
magDec = struct.unpack("<h", pl[88:90])[0]
magAcc = struct.unpack("<H", pl[90:92])[0]

print(f"  pl[ 0: 4]  iTOW             (u4): {iTOW} ms             (GPS time of week)")
print(f"  pl[ 4: 6]  year             (u2): {year}")
print(f"  pl[ 6: 7]  month            (u1): {month}")
print(f"  pl[ 7: 8]  day              (u1): {day}")
print(f"  pl[ 8: 9]  hour UTC         (u1): {hour}")
print(f"  pl[ 9:10]  min UTC          (u1): {minute}")
print(f"  pl[10:11]  sec UTC          (u1): {sec}")
print(f"  pl[11:12]  valid bitfield   (u1): 0x{valid:02x}    bit0=validDate, bit1=validTime, bit2=fullyResolved, bit3=validMag")
print(f"  pl[12:16]  tAcc             (u4): {tAcc} ns")
print(f"  pl[16:20]  nano             (i4): {nano} ns           (sub-second fraction)")
print(f"  pl[20:21]  fixType          (u1): {fixType}              (0=no fix, 2=2D, 3=3D, 4=GNSS+DR, 5=time only)")
print(f"  pl[21:22]  flags            (u1): 0x{flags:02x}")
print(f"  pl[22:23]  flags2           (u1): 0x{flags2:02x}")
print(f"  pl[23:24]  numSV            (u1): {numSV}             (# of satellites used in nav solution)")
print(f"  pl[24:28]  lon (1e-7 deg)   (i4): [REDACTED]    (user location)")
print(f"  pl[28:32]  lat (1e-7 deg)   (i4): [REDACTED]    (user location)")
print(f"  pl[32:36]  height (mm)      (i4): [REDACTED]    (user location)")
print(f"  pl[36:40]  hMSL (mm)        (i4): [REDACTED]    (user location)")
print(f"  pl[40:44]  hAcc (mm)        (u4): {hAcc} mm          (= {hAcc/1000:.3f} m)  *** USED BY ANALYSIS ***")
print(f"  pl[44:48]  vAcc (mm)        (u4): {vAcc} mm          (= {vAcc/1000:.3f} m)  *** USED BY ANALYSIS ***")
print(f"  pl[48:52]  velN (mm/s)      (i4): {velN} mm/s")
print(f"  pl[52:56]  velE (mm/s)      (i4): {velE} mm/s")
print(f"  pl[56:60]  velD (mm/s)      (i4): {velD} mm/s")
print(f"  pl[60:64]  gSpeed (mm/s)    (i4): {gSpeed} mm/s         *** USED BY ANALYSIS (phantom velocity detector) ***")
print(f"  pl[64:68]  headMot (1e-5 deg)(i4): {headMot}")
print(f"  pl[68:72]  sAcc (mm/s)      (u4): {sAcc}")
print(f"  pl[72:76]  headAcc (1e-5 deg)(u4): {headAcc}")
print(f"  pl[76:78]  pDOP * 100       (u2): {pDOP_raw}            (= {pDOP_raw*0.01:.2f})")
print(f"  pl[78:80]  flags3           (u2): 0x{flags3:04x}")
print(f"  pl[80:84]  reserved         (4B)")
print(f"  pl[84:88]  headVeh (1e-5 deg)(i4): {headVeh}")
print(f"  pl[88:90]  magDec (1e-2 deg)(i2): {magDec}")
print(f"  pl[90:92]  magAcc (1e-2 deg)(u2): {magAcc}")

print(f"\n  Wall-clock timestamp: {year:04d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{sec:02d} UTC")

# ============================================================
# Part 2: MON-HW decode
# ============================================================
print("\n\n#### MESSAGE 2: UBX-MON-HW (class=0x0A, id=0x09) ####")
print("Per spec Section 'UBX-MON-HW': 60-byte payload, hardware status.")

off, frame = find_first_frame(data, 0x0a, 0x09)
print(f"\nFound at file offset 0x{off:x}")
hexdump("Raw frame bytes", frame)

ck_a, ck_b, exp_a, exp_b = verify_checksum(frame)
print(f"\nChecksum: computed {ck_a:#04x} {ck_b:#04x}, expected {exp_a:#04x} {exp_b:#04x}  "
      f"-> {'PASS' if (ck_a == exp_a and ck_b == exp_b) else 'FAIL'}")

pl = frame[6:6 + (frame[4] | (frame[5] << 8))]
print("\n--- MON-HW payload decode (offsets per spec) ---")

pinSel = struct.unpack("<I", pl[0:4])[0]
pinBank = struct.unpack("<I", pl[4:8])[0]
pinDir = struct.unpack("<I", pl[8:12])[0]
pinVal = struct.unpack("<I", pl[12:16])[0]
noisePerMS = struct.unpack("<H", pl[16:18])[0]
agcCnt = struct.unpack("<H", pl[18:20])[0]
aStatus = pl[20]
aPower = pl[21]
flags = pl[22]
# reserved at 23
usedMask = struct.unpack("<I", pl[24:28])[0]
# VP at 28..(28+17) = pin virtual mapping, 17 bytes
jamInd = pl[45]

print(f"  pl[ 0: 4]  pinSel           (X4): 0x{pinSel:08x}    pin-select bitmask")
print(f"  pl[ 4: 8]  pinBank          (X4): 0x{pinBank:08x}")
print(f"  pl[ 8:12]  pinDir           (X4): 0x{pinDir:08x}")
print(f"  pl[12:16]  pinVal           (X4): 0x{pinVal:08x}")
print(f"  pl[16:18]  noisePerMS       (u2): {noisePerMS}            (broadband noise floor)")
print(f"  pl[18:20]  agcCnt           (u2): {agcCnt}           (AGC monitor, 0..8191, scales to 0..100%)  *** USED ***")
print(f"  pl[20:21]  aStatus          (u1): {aStatus}              (0=INIT 1=DONTKNOW 2=OK 3=SHORT 4=OPEN)  *** USED ***")
print(f"  pl[21:22]  aPower           (u1): {aPower}              (0=off 1=on 2=don't-know)")
print(f"  pl[22:23]  flags            (X1): 0x{flags:02x}")
print(f"  pl[23:24]  reserved         (1B)")
print(f"  pl[24:28]  usedMask         (X4): 0x{usedMask:08x}")
print(f"  pl[28:45]  VP (virtual pin mapping, 17 bytes)")
print(f"  pl[45:46]  jamInd           (u1): {jamInd}             (CW jamming indicator, 0..255)  *** USED ***")
print(f"  pl[46:48]  reserved         (2B)")
print(f"  pl[48:52]  pinIrq           (X4)")
print(f"  pl[52:56]  pullH            (X4)")
print(f"  pl[56:60]  pullL            (X4)")

print("\n" + "=" * 70)
print("All offsets above match the u-blox HPG L1L5 1.40 Interface Description")
print("PDF, sections 'UBX-NAV-PVT' and 'UBX-MON-HW'. Fletcher-16 checksum is")
print("specified in section '32.4 UBX checksum'.")
print("=" * 70)
