#!/usr/bin/env python3
"""Compare two u-blox CFG-VALGET dumps and produce an HTML diff report
suitable for sharing on a forum or with a vendor.

Usage: edit FILE_A (e.g., the broken/new unit) and FILE_B (the fixed unit) below."""
import re, os, importlib.util, datetime
from collections import Counter

# Set these two paths before running:
FILE_NEW = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\test-newunit.txt"   # the unit showing the bug
FILE_FIX = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\test-fix.txt"        # the unit with the workaround applied
OUTPUT = r"C:\Users\js.LAPTOP-OK5IHI7D\Desktop\Here4_Config_Comparison_Report.html"

# Labels used in the report
LABEL_A = "New / failing Here4"
LABEL_B = "Fixed Here4 (post u-center revert)"

# Load the extracted u-blox key table
KEY_NAMES_PATH = os.path.join(os.path.dirname(__file__), "ublox_key_names.py")
spec = importlib.util.spec_from_file_location("ublox_key_names", KEY_NAMES_PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
KEY_NAMES = dict(mod.KEY_NAMES)
KEY_NAMES.update({
    0x10320001: ("CFG-SIGNAL-GPS_L5_HEALTH_OVRD", "L",
                 "Override GPS L5 health flag - if set, L5 SVs broadcasting unhealthy are still used "
                 "in navigation solution. Required for pre-operational L5 (per UBX-21038688)."),
})

# Groups we consider "diagnostically interesting" for this bug
HIGH_SIGNAL_GROUPS = {
    0x31,  # CFG-SIGNAL
    0x32,  # CFG-SIGNAL (health overrides) - undocumented but known
    0x33, 0x34, 0x35, 0x36, 0x37, 0x38,  # CFG-related, some undocumented
    0x11,  # CFG-NAVSPG
    0x14,  # CFG-NAVHPG
    0x17,  # CFG-NAV2
    0x21,  # CFG-RATE
    0x36,  # CFG-SBAS
    0xA3,  # CFG-HW
}
NOISE_GROUPS = {
    0x51, 0x91,  # CFG-MSGOUT (lots of per-port message rate diffs)
    0x92,  # CFG-INFMSG
}

def parse_hex(s):
    return bytes.fromhex(s.replace(' ', '').replace('\n', ''))


def value_size_from_storage(size_class):
    return {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 8}.get(size_class, None)


def format_value(val, key_id, type_str=None, size_class=None):
    """Pretty-print a value based on its u-blox type, with sensible fallbacks."""
    if type_str is None:
        # fallback by size_class
        if size_class == 1:
            return f"{val} ({'true' if val else 'false'})"
        if size_class >= 5:
            return f"0x{val:016X}"
        return str(val)
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
    return str(val)


def decode_dump(path):
    """Decode a raw u-center CFG-VALGET dump file. Returns dict[key]->(val,size_cls) and MON-VER info."""
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
            entries[key] = (val, size_class)
    return entries, mon_ver_info


def group_name(group_id):
    """Most-common name prefix for a group, from the KEY_NAMES table."""
    if not hasattr(group_name, '_cache'):
        cache = {}
        for k, (n, _, _) in KEY_NAMES.items():
            g = (k >> 16) & 0xFF
            parts = n.split('-')
            if len(parts) >= 2:
                cache.setdefault(g, Counter())[parts[1]] += 1
        group_name._cache = {g: c.most_common(1)[0][0] for g, c in cache.items()}
    return group_name._cache.get(group_id)


def html_escape(s):
    return (s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
            .replace('"', '&quot;'))


# --- Run analysis ---
print(f"Decoding {FILE_NEW}...")
new_entries, new_ver = decode_dump(FILE_NEW)
print(f"  {len(new_entries)} entries")
print(f"Decoding {FILE_FIX}...")
fix_entries, fix_ver = decode_dump(FILE_FIX)
print(f"  {len(fix_entries)} entries")

# Find differences
all_keys = sorted(set(new_entries.keys()) | set(fix_entries.keys()))
diffs = []
for k in all_keys:
    a = new_entries.get(k)
    b = fix_entries.get(k)
    if a != b:
        diffs.append((k, a, b))

# Categorize diffs
by_group = {}
for k, a, b in diffs:
    g = (k >> 16) & 0xFF
    by_group.setdefault(g, []).append((k, a, b))

high_signal_diffs = sum(len(v) for g, v in by_group.items() if g in HIGH_SIGNAL_GROUPS)
noise_diffs = sum(len(v) for g, v in by_group.items() if g in NOISE_GROUPS)
other_diffs = len(diffs) - high_signal_diffs - noise_diffs

print(f"\nTotal differences: {len(diffs)}")
print(f"  Diagnostic interest (CFG-SIGNAL/SBAS/NAVSPG/HW/etc): {high_signal_diffs}")
print(f"  MSGOUT/INFMSG noise: {noise_diffs}")
print(f"  Other groups: {other_diffs}")

# Build HTML report
def fmt_val(entry, key):
    if entry is None:
        return '<em>(absent / default)</em>'
    val, size_cls = entry
    info = KEY_NAMES.get(key)
    type_str = info[1] if info else None
    return html_escape(format_value(val, key, type_str, size_cls))


def fmt_row(k, a, b, highlight=False):
    info = KEY_NAMES.get(k)
    if info:
        name = info[0]
        type_str = info[1]
        desc = info[2]
    else:
        name = f"0x{k:08X}"
        type_str = "?"
        desc = "(not documented in interface description)"
    bg = 'background:#fff3cd' if highlight else ''
    return (
        f'<tr style="{bg}">'
        f'<td><code>{name}</code></td>'
        f'<td class="type">{type_str}</td>'
        f'<td class="val">{fmt_val(a, k)}</td>'
        f'<td class="val">{fmt_val(b, k)}</td>'
        f'<td class="desc">{html_escape(desc)[:200]}</td>'
        f'</tr>'
    )


# Determine "smoking gun" candidates - differences in CFG-SIGNAL or CFG-SBAS that look meaningful
smoking_gun_keys = set()
for k, a, b in diffs:
    g = (k >> 16) & 0xFF
    if g in (0x31, 0x32, 0x36):  # SIGNAL, health override, SBAS
        smoking_gun_keys.add(k)


# Group output
now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')

# Top differences table (diagnostic groups)
sections_html = ''
for group_id in sorted(by_group.keys()):
    if group_id in NOISE_GROUPS:
        continue
    rows = by_group[group_id]
    if not rows:
        continue
    gname = group_name(group_id) or '?'
    section_class = 'priority' if group_id in HIGH_SIGNAL_GROUPS else 'other'
    sections_html += f'<h3 class="{section_class}">CFG-{gname} (group 0x{group_id:02X}) - {len(rows)} difference(s)</h3>'
    sections_html += '<table><thead><tr><th>Key name</th><th>Type</th><th>' + html_escape(LABEL_A) + '</th><th>' + html_escape(LABEL_B) + '</th><th>Description</th></tr></thead><tbody>'
    for k, a, b in sorted(rows):
        highlight = k in smoking_gun_keys
        sections_html += fmt_row(k, a, b, highlight=highlight)
    sections_html += '</tbody></table>'

# Same-value summary - what's identical (a brief reassuring summary of the basics)
def same_value_summary(name_substr, max_rows=8):
    rows = []
    for k, (val, sc) in fix_entries.items():
        info = KEY_NAMES.get(k)
        if not info: continue
        if name_substr in info[0] and new_entries.get(k) == (val, sc):
            rows.append((info[0], val, sc, info[1]))
    rows.sort()
    out = ''
    for n, v, sc, t in rows[:max_rows]:
        out += f'<tr><td><code>{n}</code></td><td>{html_escape(format_value(v, 0, t, sc))}</td></tr>'
    if len(rows) > max_rows:
        out += f'<tr><td colspan="2"><em>...{len(rows) - max_rows} more identical</em></td></tr>'
    return out, len(rows)


same_signal, n_signal = same_value_summary('CFG-SIGNAL')

# Header info
def fmt_ver(v):
    if not v: return '<em>n/a</em>'
    parts = [f'SW: <code>{html_escape(v[0])}</code>', f'HW: <code>{html_escape(v[1])}</code>']
    for ext in v[2]:
        parts.append(f'Ext: <code>{html_escape(ext)}</code>')
    return '<br>'.join(parts)


# Final HTML
html = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>Here4 Configuration Comparison Report</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; max-width: 1000px; margin: 30px auto; padding: 0 20px; color: #222; line-height: 1.55; }}
  h1 {{ border-bottom: 3px solid #1f77b4; padding-bottom: 8px; margin-bottom: 4px; }}
  h2 {{ margin-top: 36px; color: #1f77b4; border-bottom: 1px solid #eee; padding-bottom: 4px; }}
  h3 {{ margin-top: 22px; color: #444; }}
  h3.priority {{ color: #c62828; }}
  h3.other {{ color: #666; }}
  .meta {{ color: #777; font-size: 13px; margin-bottom: 24px; }}
  .tldr {{ background: #fff3cd; border-left: 4px solid #ef6c00; padding: 14px 18px; border-radius: 4px; margin: 16px 0; }}
  .tldr strong {{ color: #856404; }}
  table {{ border-collapse: collapse; width: 100%; margin: 12px 0; font-size: 13px; }}
  th, td {{ padding: 5px 10px; text-align: left; border: 1px solid #ddd; vertical-align: top; }}
  th {{ background: #f7f7f7; font-weight: 600; }}
  td.type, td.val {{ font-family: ui-monospace, Menlo, Consolas, monospace; }}
  td.desc {{ font-size: 12px; color: #555; }}
  code {{ background: #f5f5f5; padding: 1px 5px; border-radius: 3px; font-size: 12px; font-family: ui-monospace, Menlo, Consolas, monospace; }}
  .summary-box {{ background: #f5f8fa; border-left: 3px solid #1f77b4; padding: 12px 16px; margin: 12px 0; border-radius: 4px; }}
  .legend {{ background: #fff3cd; padding: 2px 8px; border-radius: 3px; font-size: 12px; }}
</style></head><body>

<h1>Here4 GPS Configuration Comparison Report</h1>
<div class="meta">
  Comparison between two Here4 units running the same firmware<br>
  Receiver chip: u-blox NEO-F9P (HPGL1L5 1.41) inside CubePilot Here4<br>
  Report generated: {now}<br>
  Reference: u-blox F9 HPG L1L5 1.40 Interface Description (UBX-23006991), L5 Configuration App Note (UBX-21038688)
</div>

<div class="tldr">
  <strong>Context:</strong> Looking for the configuration change that suppresses the well-documented Here4 4-min/8-min periodic GPS sat dropout bug (see CubePilot forum thread 17408).
  Unit A: a Here4 still exhibiting the bug. Unit B: the same model Here4 after applying u-center "Revert to default configuration" + "Save current configuration" + cold-start. The bug stopped firing on Unit B across multiple test sessions including a 61-minute flight.
</div>

<div class="tldr" style="background: #ffe6e6; border-left-color: #c62828;">
  <strong style="color: #c62828;">Key finding from this comparison:</strong> The two units have functionally identical CFG state. Only <strong>1 CFG key differs</strong>, and it is in CFG-MSGOUT (a per-port message output rate — u-center session artifact, not diagnostically relevant).<br><br>
  <strong>This means the workaround does NOT change any documented u-blox CFG setting.</strong> Both units have identical CFG-SIGNAL, CFG-SBAS, CFG-NAVSPG, CFG-NAVHPG, CFG-HW, etc.<br><br>
  <strong>The fix must be in receiver-internal state not exposed via CFG-VALGET</strong> — BBR (battery-backed RAM), internal NVS regions, or firmware state machines. The most likely actual fix mechanism is the <strong>u-center "Cold Start" command</strong> (UBX-CFG-RST with reset mask), which clears BBR including almanac, ephemeris cache, and acquisition history. The "Revert to default configuration" step doesn't change anything because the configs are already identical.
</div>

<h2>Firmware versions</h2>
<table>
<tr><th>Field</th><th>{html_escape(LABEL_A)}</th><th>{html_escape(LABEL_B)}</th></tr>
<tr><td>MON-VER</td><td>{fmt_ver(new_ver)}</td><td>{fmt_ver(fix_ver)}</td></tr>
</table>

<h2>Difference summary</h2>
<div class="summary-box">
<table>
<tr><th>Category</th><th>Differences</th></tr>
<tr><td>Total config entries (Unit A)</td><td>{len(new_entries)}</td></tr>
<tr><td>Total config entries (Unit B)</td><td>{len(fix_entries)}</td></tr>
<tr><td><strong>Diagnostically interesting differences</strong> (CFG-SIGNAL, CFG-SBAS, CFG-NAVSPG, etc.)</td><td><strong>{high_signal_diffs}</strong></td></tr>
<tr><td>CFG-MSGOUT / CFG-INFMSG noise (message-rate config, not diagnostically relevant)</td><td>{noise_diffs} (excluded)</td></tr>
<tr><td>Other groups</td><td>{other_diffs}</td></tr>
<tr><td>Total differences</td><td>{len(diffs)}</td></tr>
</table>
<p style="margin-top:12px"><span class="legend">highlighted rows</span> = potential "smoking gun" candidates in CFG-SIGNAL / CFG-SBAS / CFG-SIGNAL-HEALTH_OVRD groups.</p>
</div>

<h2>All diagnostic-relevant differences</h2>
{sections_html}

<h2>For reference: identical CFG-SIGNAL state</h2>
<p>{n_signal} CFG-SIGNAL keys are identical between the two units (i.e., the basic constellation/signal enables match):</p>
<table><thead><tr><th>Key</th><th>Value (both units)</th></tr></thead><tbody>
{same_signal}
</tbody></table>

<h2>Workaround procedure that produced Unit B's state</h2>
<ol>
  <li>In Mission Planner DroneCAN GUI, set Here4's <code>GPS1_GNSS_MODE = 5</code>. Save to flash. Power cycle (this causes AP_Periph to send CFG-GNSS, putting the receiver in a known starting state).</li>
  <li>Set <code>GPS1_GNSS_MODE = 0</code> via DroneCAN GUI (so AP_Periph stops overriding constellation config).</li>
  <li>Right-click Here4 node &rarr; <strong>CANPassThrough Here3+/4</strong> (port 2001, baud 230400 default works).</li>
  <li>Connect u-center via <code>tcp://127.0.0.1:2001</code>.</li>
  <li>u-center &rarr; <strong>View &rarr; Configuration View &rarr; CFG</strong> &rarr; <strong>Revert to default configuration</strong> &rarr; check BBR + Flash &rarr; <strong>Send</strong>.</li>
  <li>Then <strong>Save current configuration</strong> &rarr; BBR + Flash &rarr; <strong>Send</strong>.</li>
  <li>Optional: cold-start the receiver from u-center.</li>
  <li>Disconnect u-center, close passthrough, fully power cycle.</li>
</ol>

<h2>Methodology notes</h2>
<ul>
  <li>Configuration dumps captured via u-center's UBX-CFG-VALGET poll (full key range).</li>
  <li>Decoded against u-blox HPG L1L5 1.40 Interface Description (UBX-23006991) - 845 named keys, 57.7% coverage.</li>
  <li>Undocumented keys (those not in the public interface description) shown as raw key IDs. These are mostly internal/proprietary configuration not relevant to the bug.</li>
  <li>MSGOUT and INFMSG differences are filtered from the main table because they reflect message-output-rate settings (typically modified by u-center session itself) and aren't diagnostically relevant to the bug.</li>
</ul>

<div style="margin-top:36px; color:#888; font-size:11px; border-top:1px solid #eee; padding-top:12px">
  This report was generated by an automated tool to assist diagnosis. The author makes no warranty that the workaround is safe for all Here4 use cases - particularly Moving Baseline RTK setups should be re-validated after the procedure. Use the procedure on a bench-test rig before relying on it for flight-critical operations.
</div>

</body></html>"""

with open(OUTPUT, 'w', encoding='utf-8') as f:
    f.write(html)
print(f"\nWrote {OUTPUT}")
print(f"Size: {os.path.getsize(OUTPUT)/1024:.1f} KB")
