#!/usr/bin/env python3
"""
Generate the customer-facing Xplorer_ArduCopter_Release_Notes.txt from
doc/XPLORER-FIRMWARE-CHANGELOG.md.

The changelog is the single source of truth. This script only reformats it into
the plain-text layout techs and customers already read, and stamps in the hard
build facts (version, commit, tag, boards, sha256) from a MANIFEST.txt so the
prose can never disagree with the binary it ships beside.

  python Tools/xplorer/gen_release_notes.py --out notes.txt
  python Tools/xplorer/gen_release_notes.py --out notes.txt --manifest release/xplorer-v1.0.1/MANIFEST.txt
  python Tools/xplorer/gen_release_notes.py --check --version 1.0.0

--check parses the changelog, verifies an entry exists for --version, and writes
nothing. Used as a release gate by build_release.sh.
"""

import argparse
import os
import re
import sys
import textwrap

WIDTH = 80
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CHANGELOG = os.path.join(REPO, 'doc', 'XPLORER-FIRMWARE-CHANGELOG.md')

# '## v1.0.1 - 2026-08-13'  or  '## v1.0.1 - Build 2 - 2026-08-13'
# (em dash in the source, hyphen tolerated). Unlike X55 the Xplorer has no legacy
# build numbering, so 'Build N' is optional and ordering falls back to the semver
# version -- keeping exactly one number to increment per release.
HEADING = re.compile(
    r'^##\s+'
    r'v(?P<version>[0-9]+\.[0-9]+\.[0-9]+)'
    r'(?:\s*[-—]\s*Build\s+(?P<build>\d+))?'
    r'\s*[-—]\s*'
    r'(?P<date>\S+)\s*$'
)


def vkey(v):
    """'1.2.10' -> (1, 2, 10) for ordering."""
    return tuple(int(x) for x in v.split('.'))

MONTHS = ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
          'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']


def pretty_date(iso):
    """2026-08-11 -> Aug 11, 2026. Passes anything unparseable straight through."""
    m = re.match(r'^(\d{4})-(\d{2})-(\d{2})$', iso)
    if not m:
        return iso
    y, mo, d = int(m.group(1)), int(m.group(2)), int(m.group(3))
    if not 1 <= mo <= 12:
        return iso
    return '%s %d, %d' % (MONTHS[mo - 1], d, y)


def demarkdown(s):
    """Strip the inline markdown that would read as noise in a .txt."""
    s = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', s)   # links -> label
    s = s.replace('`', '')
    s = re.sub(r'\*\*(.+?)\*\*', r'\1', s)
    s = s.replace('—', '--').replace('–', '-')
    s = s.replace('→', '-->')
    return s


def parse(path):
    """Split the changelog into entries. Preamble (before the first ## ) is dropped."""
    with open(path, encoding='utf-8') as f:
        lines = f.read().splitlines()

    entries, cur = [], None
    for line in lines:
        m = HEADING.match(line)
        if m:
            cur = {
                'version': m.group('version'),
                'build': int(m.group('build')) if m.group('build') else None,
                'date': m.group('date'),
                'title': None,
                'commit': None,
                'body': [],
            }
            entries.append(cur)
            continue
        if cur is None:
            continue
        if line.strip() == '---':
            continue
        if line.startswith('### ') and cur['title'] is None:
            cur['title'] = demarkdown(line[4:]).strip()
            continue
        m = re.match(r'^\*\*Commit:\*\*\s*(.+)$', line)
        if m and cur['commit'] is None:
            cur['commit'] = demarkdown(m.group(1)).strip()
            continue
        cur['body'].append(line)
    return entries


def render_body(body):
    """Reflow the markdown body to WIDTH, preserving list shape and blank lines.

    Markdown wraps a long list item by indenting its continuation lines. Those
    have to be folded back into the item and re-wrapped, or the .txt ends up with
    continuations sitting at column 0 and the list shape is lost.
    """
    out = []
    # pending block: (kind, meta, [text parts]) where kind is
    #   'para'  meta unused          flush at column 0
    #   'item'  meta = bullet depth  '- ' at depth, continuations at depth+2
    #   'cont'  meta = indent        continuation of an outer item after a
    #                                nested list closed it
    #   'label' meta = label text    'Files: ...' with a 7-space hanging indent,
    #                                matching the original notes' style
    block = None
    # depths of the list items open in the current run, so an indented
    # continuation can be re-attached to the right level
    list_depths = []
    # suppress one blank line, so 'Changes:' sits directly above its first item
    eat_blank = False

    def flush():
        nonlocal block
        if block is None:
            return
        kind, meta, parts = block
        block = None
        text = demarkdown(' '.join(p.strip() for p in parts if p.strip()))
        if not text:
            return
        if kind == 'item':
            out.extend(textwrap.wrap(
                text, WIDTH,
                initial_indent=' ' * meta + '- ',
                subsequent_indent=' ' * (meta + 2),
                break_long_words=False, break_on_hyphens=False))
        elif kind == 'cont':
            out.extend(textwrap.wrap(
                text, WIDTH,
                initial_indent=' ' * meta, subsequent_indent=' ' * meta,
                break_long_words=False, break_on_hyphens=False))
        elif kind == 'label':
            out.extend(textwrap.wrap(
                '%s: %s' % (meta, text), WIDTH,
                subsequent_indent=' ' * 7,
                break_long_words=False, break_on_hyphens=False))
        else:
            out.extend(textwrap.wrap(
                text, WIDTH,
                break_long_words=False, break_on_hyphens=False) or [''])

    for raw in body:
        line = raw.rstrip()
        indent = len(raw) - len(raw.lstrip())

        if not line.strip():
            flush()
            list_depths = []
            if eat_blank:
                eat_blank = False
                continue
            if out and out[-1] != '':
                out.append('')
            continue

        # section labels: '**Changes:**' -> 'Changes:'
        m = re.match(r'^\*\*([A-Za-z][^*]*?):\*\*\s*(.*)$', line)
        if m and indent == 0:
            flush()
            list_depths = []
            if out and out[-1] != '':
                out.append('')
            label, rest = m.group(1), demarkdown(m.group(2)).strip()
            if rest:
                block = ('label', label, [rest])
            else:
                out.append('%s:' % label)
                eat_blank = True
            continue
        eat_blank = False

        # list item, at any nesting depth
        m = re.match(r'^(\s*)[-*]\s+(.*)$', line)
        if m:
            flush()
            depth = len(m.group(1))
            if depth not in list_depths:
                list_depths.append(depth)
            block = ('item', depth, [m.group(2)])
            continue

        if indent > 0:
            # continuation of the item we are inside
            if block is not None and block[0] == 'item' and indent > block[1]:
                block[2].append(line)
                continue
            # more of a continuation block already open
            if block is not None and block[0] == 'cont':
                block[2].append(line)
                continue
            # continuation of an outer item that a nested list closed: attach to
            # the innermost still-open level shallower than this line
            outer = [d for d in list_depths if d < indent]
            if outer:
                flush()
                block = ('cont', max(outer) + 2, [line])
                continue
            # no list context: verbatim (aligned tables, code)
            if indent >= 4:
                flush()
                out.append(demarkdown(line))
                continue

        if block is not None and block[0] in ('para', 'label'):
            block[2].append(line)
        else:
            flush()
            block = ('para', None, [line])

    flush()
    while out and out[-1] == '':
        out.pop()
    return out


def read_manifest(path):
    """Pull the build facts out of a MANIFEST.txt written by build_release.sh."""
    facts, artifacts, boards = {}, [], []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.rstrip()
            m = re.match(r'^(\S[^:]*?)\s*:\s*(.*)$', line)
            if m and not line.startswith('  '):
                key, val = m.group(1).strip(), m.group(2).strip()
                # 'board' repeats once per board -- collect, don't overwrite
                if key == 'board':
                    boards.append(val)
                else:
                    facts[key] = val
                continue
            m = re.match(r'^\s+(\S+)\s+sha256=(\S+)$', line)
            if m:
                artifacts.append((m.group(1), m.group(2)))
    facts['_artifacts'] = artifacts
    facts['_boards'] = boards
    return facts


def render(entries, manifest=None):
    L = []
    bar = '=' * WIDTH
    sep = '-' * WIDTH

    latest = entries[0] if entries else None

    L.append(bar)
    L.append('  Xplorer ArduCopter Custom Firmware - Release Notes')
    L.append('  Based on ArduCopter 4.6.2 Stable (branch: xplorer/4.6.2-custom)')
    L.append('  Maintained by: JS / Arcsky')
    L.append(bar)
    L.append('')
    L.append('GENERATED FILE - do not edit.')
    L.append('Source: doc/XPLORER-FIRMWARE-CHANGELOG.md in the ardupilot repo.')
    L.append('Regenerate with Tools/xplorer/build_release.sh.')
    L.append('')

    if manifest:
        L.append(sep)
        L.append('THIS RELEASE')
        L.append(sep)
        for key, label in (('version string', 'Version string'),
                           ('upstream base', 'Upstream base'),
                           ('git tag', 'Git tag'),
                           ('git commit', 'Git commit'),
                           ('branch', 'Branch'),
                           ('tree state', 'Tree state')):
            if manifest.get(key):
                L.append('  %-16s %s' % (label + ':', manifest[key]))
        for i, board in enumerate(manifest.get('_boards', [])):
            L.append('  %-16s %s' % ('Boards:' if i == 0 else '', board))
        if manifest.get('_artifacts'):
            L.append('')
            L.append('  Files (sha256):')
            for name, sha in manifest['_artifacts']:
                L.append('    %s' % name)
                L.append('      %s' % sha)
        if manifest.get('tree state', 'clean') != 'clean':
            L.append('')
            L.append('  *** WARNING: built from a DIRTY working tree. The commit')
            L.append('  *** above does NOT identify the code in these binaries.')
            L.append('  *** Engineering build only - do not ship.')
        L.append('')
        L.append('Note: earlier builds predate firmware versioning and report only')
        L.append('"ArduCopter V4.6.2". Identify those by the git hash in the banner.')
        L.append('')

    for e in entries:
        L.append('')
        L.append(sep)
        if e['build'] is not None:
            L.append('Xplorer v%s  (BUILD %d) - %s' % (e['version'], e['build'],
                                                       pretty_date(e['date'])))
        else:
            L.append('Xplorer v%s - %s' % (e['version'], pretty_date(e['date'])))
        if e['commit']:
            L.append('Commit: %s' % e['commit'])
        L.append(sep)
        if e['title']:
            L.append('*** %s ***' % e['title'].upper())
            L.append('')
        L.extend(render_body(e['body']))
        L.append('')

    return '\n'.join(L).rstrip() + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--changelog', default=CHANGELOG)
    ap.add_argument('--out', help='output .txt path')
    ap.add_argument('--manifest', help='MANIFEST.txt to stamp build facts from')
    ap.add_argument('--check', action='store_true',
                    help='validate only; write nothing')
    ap.add_argument('--version', help='require an entry for this version')
    args = ap.parse_args()

    if not os.path.exists(args.changelog):
        sys.exit('ERROR: %s not found' % args.changelog)

    entries = parse(args.changelog)
    if not entries:
        sys.exit('ERROR: no release entries parsed from %s.\n'
                 'Headings must look like "## v1.0.1 - 2026-08-13"'
                 % args.changelog)

    versions = [e['version'] for e in entries]
    keys = [vkey(v) for v in versions]
    if keys != sorted(keys, reverse=True):
        sys.exit('ERROR: entries in %s are not in descending version order: %s\n'
                 'Newest release goes at the top.' % (args.changelog, versions))
    dupes = {v for v in versions if versions.count(v) > 1}
    if dupes:
        sys.exit('ERROR: duplicate versions in %s: %s'
                 % (args.changelog, sorted(dupes)))
    builds = [e['build'] for e in entries if e['build'] is not None]
    bdupes = {b for b in builds if builds.count(b) > 1}
    if bdupes:
        sys.exit('ERROR: duplicate build numbers in %s: %s'
                 % (args.changelog, sorted(bdupes)))

    if args.version:
        match = [e for e in entries if e['version'] == args.version]
        if not match:
            sys.exit('ERROR: no changelog entry for v%s in %s.\n'
                     'Add one before releasing -- the notes are how anyone else\n'
                     'finds out what changed. Newest entry is: %s'
                     % (args.version, args.changelog,
                        'v%s' % entries[0]['version']))
        if match[0] is not entries[0]:
            sys.exit('ERROR: v%s is not the newest entry in %s. Newest entries '
                     'go at the top.' % (args.version, args.changelog))

    if args.check:
        print('OK: %d release entries parsed%s'
              % (len(entries), ', v%s present' % args.version if args.version else ''))
        return

    if not args.out:
        sys.exit('ERROR: --out is required unless --check is given')

    manifest = read_manifest(args.manifest) if args.manifest else None
    text = render(entries, manifest)
    with open(args.out, 'w', encoding='utf-8', newline='\r\n') as f:
        f.write(text)
    print('wrote %s (%d entries, %d lines)'
          % (args.out, len(entries), text.count('\n')))


if __name__ == '__main__':
    main()
