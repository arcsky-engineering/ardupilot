"""One-off parser to extract CFG-* configuration items from the u-blox F9 HPG L1L5 1.40
Interface Description (UBX-23006991).

Uses the non-layout pdftotext output. Tokenizes the body of section 6.9 into a
stream of NAME / KEY / HEADER / TABLE events, then pairs names with keys using
a layout-detection heuristic that handles both:

  * bulk layout: NAME NAME NAME ... KEY KEY KEY ... (the column reading order
    pdftotext produces when names occupy one column on the page)
  * per-row layout: NAME, KEY, NAME, KEY, ... (when each name and its key are
    on the same row in the source PDF)

The trick is that description text often mentions OTHER CFG- names. We filter
those out by only enqueuing names that appear:
  - before the very first KEY in the chunk (bulk preamble), OR
  - immediately before a KEY with no intervening NAME-mention (per-row case).
"""
from __future__ import annotations
import re
from pathlib import Path

RAW = Path(r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\ublox_f9_raw.txt")
OUT = Path(__file__).with_name("ublox_key_names.py")

text = RAW.read_text(encoding="cp1252")

# Pre-process: fix names broken across lines.
# Case A: `CFG-MSGOUT-UBX_NAV_ORB_ UART1` -> `CFG-MSGOUT-UBX_NAV_ORB_UART1`
#         (suffix appears on the same logical line, just with a space inside the name).
text = re.sub(
    r"(CFG-[A-Z][A-Z0-9_]*(?:-[A-Z0-9_]+)+)_\s+([A-Z][A-Z0-9_]*)",
    r"\1_\2",
    text,
)
# Case B: the suffix appears AFTER the key value (multi-paragraph), e.g.:
#     CFG-MSGOUT-UBX_NAV_POSECEF_ 0x20910024 U1
#     -
#     I2C
# This needs to be glued to: CFG-MSGOUT-UBX_NAV_POSECEF_I2C 0x20910024 U1
# We splice the suffix back into the name and drop it from its original spot.
def _splice_suffix(m: re.Match) -> str:
    name = m.group("name")
    key = m.group("key")
    suffix = m.group("suffix")
    return f"{name}{suffix} {key}\n\n-"

text = re.sub(
    r"(?P<name>CFG-[A-Z][A-Z0-9_]*(?:-[A-Z0-9_]+)+_)\s+(?P<key>0x[0-9a-fA-F]{8}\s+(?:L|[UIXE][1248]|R[48])(?:\s+-)?)\s*\n\s*-\s*\n\s*(?P<suffix>I2C|SPI|UART1|UART2|USB|UP_ENA|TP1|TP2)\b",
    _splice_suffix,
    text,
)


BODY_START = 567000
CFG_REF_END = text.find("Configuration defaults", BODY_START)
if CFG_REF_END < 0:
    CFG_REF_END = len(text)

DEFAULTS_START = CFG_REF_END
DEFAULTS_END = text.find("Related documents", DEFAULTS_START)
if DEFAULTS_END < 0:
    DEFAULTS_END = len(text)

NAME_RE = re.compile(r"\b(CFG-[A-Z][A-Z0-9_]*(?:-[A-Z0-9_]+)+)\b")
KEY_RE = re.compile(r"\b(0x[0-9a-fA-F]{8})\s+(L|U[1248]|I[1248]|X[1248]|E[124]|R[48])\b")
HEADER_RE = re.compile(r"\b(Configuration item|Key ID|Type Scale|Unit|Description|Default value)\b")
TABLE_RE = re.compile(r"\bTable \d+: CFG-")

TYPE_SIZE = {
    "L": 1,
    "U1": 2, "I1": 2, "X1": 2,
    "U2": 3, "I2": 3, "X2": 3, "E1": 2, "E2": 3,
    "U4": 4, "I4": 4, "X4": 4, "R4": 4, "E4": 4,
    "U8": 5, "I8": 5, "X8": 5, "R8": 5,
}


_PROSE_BEFORE_RE = re.compile(r"[A-Za-z](?:\s+|[.,:;!?])$")
_PROSE_AFTER_RE = re.compile(r"^\s+(?:is|are|must|will|cannot|=|set|enabled|disabled|together|to|in|and|or|via|with|for|of|by|on|when|if|the|each|its|when|while|takes|takes|determines|see|configures)\b", re.IGNORECASE)


def _is_real_entry_name(t: str, name_start: int, name_end: int) -> bool:
    """Return True if the NAME at (name_start, name_end) in text `t` looks like
    a table-row entry rather than a prose reference within description text.

    Heuristics:
      * If preceded by "\\n" (newline): real (start of new line in table)
      * If preceded by "Configuration item " (header): real (bulk listing)
      * If preceded by another CFG-NAME + space: real (bulk listing continuation)
      * If preceded by ANY word character (letter/digit): prose
      * If followed by " is", " are", " must", "=", etc.: prose
      * Otherwise: real (conservative — assume it's a table entry)
    """
    # Look at 60 chars before and 40 after for context.
    before_window = t[max(0, name_start - 60):name_start]
    after_window = t[name_end:name_end + 40]

    # 1) Preceded by newline OR "Configuration item " OR another CFG-NAME + spaces only?
    #    Check the last non-space chars before name_start
    stripped = before_window.rstrip(" ")
    if stripped.endswith("\n") or stripped == "":
        return True
    if stripped.endswith("Configuration item"):
        return True
    # Preceded by another CFG-NAME directly (possibly with comma):
    m = re.search(r"CFG-[A-Z][A-Z0-9_-]+(?:,)?$", stripped)
    if m:
        return True

    # 2) Followed by clear table-entry markers: " 0xHEX TYPE" or newline
    if re.match(r"^\s+0x[0-9a-fA-F]{8}\s+(?:L|[UIXE][1248]|R[48])\b", after_window):
        return True

    # 3) Followed by a "prose continuation" word? Then prose.
    if _PROSE_AFTER_RE.match(after_window):
        return False

    # 4) Preceded by a word/dot (within a sentence)?
    if _PROSE_BEFORE_RE.search(before_window):
        return False

    # Default: treat as real (less restrictive).
    return True


def tokenize(start: int, end: int) -> list[tuple[int, str, object]]:
    body = text[start:end]
    events: list[tuple[int, str, object]] = []
    for m in NAME_RE.finditer(body):
        real = _is_real_entry_name(body, m.start(), m.end())
        kind = "NAME" if real else "NAME_INLINE"
        events.append((m.start() + start, kind, m.group(1)))
    for m in KEY_RE.finditer(body):
        events.append((m.start() + start, "KEY", (m.group(1), m.group(2))))
    for m in HEADER_RE.finditer(body):
        events.append((m.start() + start, "HEADER", m.group(1)))
    for m in TABLE_RE.finditer(body):
        events.append((m.start() + start, "TABLE", None))
    events.sort(key=lambda ev: ev[0])
    return events


def split_chunks(events: list[tuple[int, str, object]]) -> list[list[tuple[int, str, object]]]:
    """Split events into chunks bounded by 'Configuration item' headers or TABLE markers.

    Filters out NAME_INLINE events (prose references to other CFG names that
    are not table-row entries).
    """
    chunks: list[list[tuple[int, str, object]]] = []
    cur: list[tuple[int, str, object]] = []
    for ev in events:
        _, etype, eval = ev
        if etype == "NAME_INLINE":
            continue
        if etype == "HEADER" and eval == "Configuration item":
            if cur:
                chunks.append(cur)
            cur = [ev]
            continue
        if etype == "TABLE":
            if cur:
                chunks.append(cur)
            cur = []
            continue
        cur.append(ev)
    if cur:
        chunks.append(cur)
    return chunks


def pair_chunk(chunk: list[tuple[int, str, object]]) -> list[tuple[str, str, str, int]]:
    """Pair names and keys in a chunk. Returns list of (name, key_hex, type, key_pos).

    Algorithm: maintain a FIFO queue of pending NAMEs. Each KEY pops the front
    of the queue. This handles both layouts uniformly:
      * Per-row (NAME, KEY, NAME, KEY, ...) - queue holds 1 name at a time.
      * Bulk (NAME, NAME, NAME, ..., KEY, KEY, KEY, ...) - queue fills up then
        drains in order.
      * Mixed (NAME, NAME, KEY, KEY interspersed) - works because we always
        enqueue/dequeue in source order.

    Note that NAME_INLINE events (prose name references inside description text)
    are filtered out by split_chunks before reaching here.
    """
    pairs: list[tuple[str, str, str, int]] = []
    name_queue: list[str] = []
    for pos, etype, eval in chunk:
        if etype == "NAME":
            name_queue.append(eval)  # type: ignore
        elif etype == "KEY":
            key_hex, typ = eval  # type: ignore
            if name_queue:
                chosen = name_queue.pop(0)
                pairs.append((chosen, key_hex, typ, pos))
        # ignore headers
    return pairs


def extract_description(chunk_end_pos: int, next_chunk_start_pos: int) -> str:
    """Extract description text between end of this chunk's last key and next chunk."""
    # Not used in the simplified pipeline; we'll fetch descriptions per-key below.
    return ""


def descriptions_for_pairs(pairs: list[tuple[str, str, str, int]]) -> dict[str, str]:
    """Given pair list with byte positions, extract a one-line description for each.

    Heuristic: description text follows shortly after the KEY token. We grab text
    from key end-position up to the next NAME, KEY, or TABLE marker.
    """
    out: dict[str, str] = {}
    for i, (name, key_hex, typ, pos) in enumerate(pairs):
        # find end of the key+type string in the source text
        # The KEY match group included `0xHEX TYPE`, so end is `pos + len("0xHEX TYPE")`
        # but we need to compute it from the actual text snippet at pos.
        snippet = text[pos:pos + 200]
        m = KEY_RE.match(snippet)
        if not m:
            continue
        end = pos + m.end()
        # description text spans from `end` until next NAME/KEY/TABLE
        # next pair pos is pairs[i+1][3] if it exists
        next_pos = pairs[i + 1][3] if i + 1 < len(pairs) else end + 600
        desc_text = text[end:next_pos]
        # Truncate at table-end / page-break markers
        for marker in [r"\bTable \d+: CFG-", r"UBX-\d+ - R\d+", r"Page \d+ of \d+", r"\bConfiguration defaults\b", r"\bConfiguration item\b"]:
            mm = re.search(marker, desc_text)
            if mm:
                desc_text = desc_text[: mm.start()]
        # also stop at next CFG- name appearance (next entry's row)
        mm = NAME_RE.search(desc_text)
        if mm:
            # but allow CFG- references inside the description; only stop if the next CFG- is followed by a KEY shortly after
            # We already have pairs[i+1] position which is `next_pos`; we already cut there.
            pass
        # collapse whitespace
        d = re.sub(r"\s+", " ", desc_text).strip()
        # Strip "Unit", "Description" header words at start
        d = re.sub(r"^(?:Unit|Description|-)+\s*", "", d).strip()
        # remove a leading scale token like "1e-7" or "0.001" or "-"
        d = re.sub(r"^(?:-|\d+(?:\.\d+)?(?:e-?\d+)?)\s+", "", d)
        # also leading single-letter unit + space: e.g. "s ", "m ", "deg "
        # (Not stripping these — they're part of the description)
        out[key_hex] = d
    return out


# --- Main extraction --------------------------------------------------------

all_entries: dict[int, tuple[str, str, str]] = {}


def add(name: str, key_hex: str, typ: str, desc: str) -> None:
    key_int = int(key_hex, 16)
    sz = (key_int >> 28) & 0xF
    if TYPE_SIZE.get(typ) != sz:
        return
    # check group byte consistency: the third byte of the key encodes the group
    # Each CFG group has a unique group byte. Sanity-skip if the name's group prefix
    # doesn't match the key's group byte (this catches a stray description-name pairing
    # that somehow slipped through).
    if key_int in all_entries:
        old_name, old_typ, old_desc = all_entries[key_int]
        if old_name == name and not old_desc and desc:
            all_entries[key_int] = (name, typ, desc.strip())
        return
    all_entries[key_int] = (name, typ, desc.strip())


events = tokenize(BODY_START, CFG_REF_END)
chunks = split_chunks(events)

for chunk in chunks:
    pairs = pair_chunk(chunk)
    descs = descriptions_for_pairs(pairs)
    for name, key_hex, typ, pos in pairs:
        add(name, key_hex, typ, descs.get(key_hex, ""))

print(f"After reference section: {len(all_entries)} entries")

# Process Defaults appendix for backstopping
events2 = tokenize(DEFAULTS_START, DEFAULTS_END)
chunks2 = split_chunks(events2)
for chunk in chunks2:
    pairs = pair_chunk(chunk)
    for name, key_hex, typ, pos in pairs:
        add(name, key_hex, typ, "")  # no descriptions in defaults

print(f"After defaults appendix: {len(all_entries)} entries")

# Sanity-check group-byte consistency: each CFG-GROUP prefix should map to a
# unique (bytes 2-3) value in its key IDs. Report any violations.
GROUP_BYTES: dict[str, set[int]] = {}
for k, (name, typ, _) in all_entries.items():
    grp = name.split("-")[1]
    gb = (k >> 16) & 0xFFFF
    GROUP_BYTES.setdefault(grp, set()).add(gb)
inconsistent = [(grp, gbs) for grp, gbs in GROUP_BYTES.items() if len(gbs) > 1]
if inconsistent:
    print("WARNING - group byte inconsistencies (likely parse errors):")
    for grp, gbs in inconsistent:
        bad = ", ".join(f"0x{x:04x}" for x in sorted(gbs))
        print(f"  {grp}: bytes seen = {bad}")
        # show offending entries
        for k, (name, typ, _) in sorted(all_entries.items()):
            if name.split("-")[1] == grp:
                print(f"    0x{k:08x} {name} {typ}")
else:
    print("Group byte consistency: OK")

# Sanity spot-check
sanity = {
    0x1031001f: "CFG-SIGNAL-GPS_ENA",
    0x10310001: "CFG-SIGNAL-GPS_L1CA_ENA",
    0x10310004: "CFG-SIGNAL-GPS_L5_ENA",
    0x10340014: "CFG-BDS-USE_GEO_PRN",
    0x30210001: "CFG-RATE-MEAS",
    0x30210002: "CFG-RATE-NAV",
    0x20210003: "CFG-RATE-TIMEREF",
    0x209100bb: "CFG-MSGOUT-NMEA_ID_GGA_UART1",
    0x20910007: "CFG-MSGOUT-UBX_NAV_PVT_UART1",
    0x40050005: "CFG-TP-LEN_LOCK_TP1",
    0x10050007: "CFG-TP-TP1_ENA",
    0x10050008: "CFG-TP-SYNC_GNSS_TP1",
    0x1005000a: "CFG-TP-ALIGN_TO_TOW_TP1",
    0x1005000b: "CFG-TP-POL_TP1",
    0x40520001: "CFG-UART1-BAUDRATE",
}
print("Sanity:")
for k, expected in sanity.items():
    got = all_entries.get(k)
    status = "OK" if got and got[0] == expected else "FAIL"
    actual_name = got[0] if got else "<missing>"
    print(f"  {status} 0x{k:08x}: got={actual_name:38s} expected={expected}")

# Sort output
items_sorted = sorted(all_entries.items(), key=lambda kv: kv[0])

header_text = (
    "# Generated from u-blox F9 HPG L1L5 1.40 Interface Description (UBX-23006991).\n"
    "# Source: section 6.9 Configuration reference + Configuration defaults appendix.\n"
    "# Auto-extracted by Tools/_ublox_parse_pdf.py - do not edit by hand.\n"
    "#\n"
    "# Format: key_id (int) -> (name, type, description)\n"
    "#   type codes: L=bool, U1/U2/U4/U8=unsigned int, I1/I2/I4/I8=signed int,\n"
    "#               X1/X2/X4/X8=bitmask, E1/E2/E4=enum, R4/R8=float\n"
    "\n"
    "KEY_NAMES = {\n"
)

NEXT_ROW_RE = re.compile(
    r"\s+CFG-[A-Z][A-Z0-9_-]+\s+0x[0-9a-fA-F]{8}\s+(?:L|[UIXE][1248]|R[48])\b"
)
BARE_KEY_RE = re.compile(r"\s+0x[0-9a-fA-F]{8}\s+(?:L|[UIXE][1248]|R[48])\b")

def _clean_description(d: str) -> str:
    d = re.sub(r"\s+", " ", d).strip()
    # Strip leading "Unit", "Description", "-" header words and unit codes
    d = re.sub(r"^(?:Unit|Description|-|\s)+", "", d).strip()
    # Strip leading scale token like "1e-7", "0.001", "0.01"
    d = re.sub(r"^\d+(?:\.\d+)?(?:e-?\d+)?\s+", "", d)
    # Cut at next row's `CFG-X-Y 0xKEY TYPE` or bare `0xKEY TYPE`
    m = NEXT_ROW_RE.search(d)
    if m:
        d = d[: m.start()].strip()
    m = BARE_KEY_RE.search(d)
    if m:
        d = d[: m.start()].strip()
    # Cut at trailing CFG-NAME (a stray reference to next row's name) IF it
    # appears at the very end after a space (whole-word boundary).
    m = re.search(r"\s+CFG-[A-Z][A-Z0-9_-]+\s*$", d)
    if m:
        d = d[: m.start()].strip()
    # If the whole description is just a CFG-NAME (leaked next-row name), drop it.
    if re.fullmatch(r"CFG-[A-Z][A-Z0-9_-]+", d):
        d = ""
    return d


def _synth_msgout_description(name: str) -> str:
    """Synthesise a description for CFG-MSGOUT-... entries which follow a
    predictable pattern: `CFG-MSGOUT-<protocol>_<MSG>_<PORT>`."""
    if not name.startswith("CFG-MSGOUT-"):
        return ""
    tail = name[len("CFG-MSGOUT-"):]
    # tail looks like UBX_NAV_PVT_USB or NMEA_ID_GGA_UART1 or RTCM_3X_TYPE1005_I2C
    parts = tail.rsplit("_", 1)
    if len(parts) != 2:
        return ""
    msg, port = parts
    if port not in ("I2C", "SPI", "UART1", "UART2", "USB"):
        return ""
    # Re-shape the msg portion. UBX_NAV_PVT -> UBX-NAV-PVT. NMEA_ID_GGA -> NMEA-ID-GGA.
    msg_pretty = msg.replace("_", "-")
    return f"Output rate of the {msg_pretty} message on port {port}"


with OUT.open("w", encoding="utf-8") as f:
    f.write(header_text)
    for k, (name, typ, desc) in items_sorted:
        d = _clean_description(desc)
        # For CFG-MSGOUT-..., synthesise if missing/too short
        if name.startswith("CFG-MSGOUT-") and (not d or len(d) < 20):
            synth = _synth_msgout_description(name)
            if synth:
                d = synth
        if len(d) > 160:
            d = d[:157] + "..."
        d_esc = d.replace("\\", "\\\\").replace('"', '\\"')
        f.write(f'    0x{k:08x}: ("{name}", "{typ}", "{d_esc}"),\n')
    f.write("}\n")

print(f"\nWrote {OUT}")
print(f"Total: {len(all_entries)} entries")

from collections import Counter
counts = Counter(v[0].split("-")[1] for v in all_entries.values())
print("Per-group counts:")
for grp, c in sorted(counts.items()):
    print(f"  {grp}: {c}")
