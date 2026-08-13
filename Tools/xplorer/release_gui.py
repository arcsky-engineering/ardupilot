#!/usr/bin/env python3
"""
Arcsky firmware build & release console. Drives BOTH products.

A GUI wrapper over the release process so it can be run without remembering the
inner workings. It does not reimplement anything: every action shells out to the
same scripts a developer would run by hand, and the log pane shows the real
command and its real output.

    python Tools/xplorer/release_gui.py

Packaged as ArcskyRelease.exe by Tools/xplorer/build_gui_exe.ps1.

PRODUCTS
  Everything product-specific is data in PROFILES below - version include,
  changelog, boards, configure options, signing, whether parameter-disposition
  tooling and an unlocked DEV target exist. The active profile is chosen by which
  version include the repository contains, so pointing Settings at the other
  clone switches products with no further configuration.

  Deliberately ONE tool rather than two: two would drift, and the next fix would
  land in only one of them.

WHY IT SHELLS INTO CYGWIN
  ./waf is configured under Cygwin on this machine (see .lock-waf_cygwin_build),
  so builds must run there or waf re-configures and paths break. Git runs
  natively on Windows. build_release.sh runs under Cygwin too, and forces
  core.autocrlf=true internally so its dirty-tree check agrees with Windows git.
"""

import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, filedialog

APP = 'Arcsky Firmware Release Console'   # title is set per-product at runtime
CFG_NAME = 'arcsky_release_gui.json'

# ---------------------------------------------------------------- profiles ---
# Everything product-specific lives here as DATA. The console logic is shared, so
# a fix lands for both products at once - which is the whole point of one tool
# rather than two that drift apart.
#
# The active profile is chosen by which version include the repo contains, so
# pointing Settings at the other clone switches products with no configuration.

PROFILES = [
    {
        'key': 'xplorer',
        'name': 'Xplorer',
        'fw_prefix': 'Xplorer',      # AP_CUSTOM_FIRMWARE_STRING "Xplorer v1.0.1"
        'version_inc': 'libraries/AP_HAL_ChibiOS/hwdef/include/xplorer_version.inc',
        'changelog': 'doc/XPLORER-FIRMWARE-CHANGELOG.md',
        'release_sh': 'Tools/xplorer/build_release.sh',
        'param_docs': 'Tools/xplorer/gen_param_docs.py',
        'tag_prefix': 'xplorer-fw-v',
        'artifact_prefix': 'xplorer',
        'boards': ['CubeOrangePlus-ODID', 'CubeOrangePlus-ODID-DEV',
                   'CubeOrangePlus'],
        'dev_board': 'CubeOrangePlus-ODID-DEV',
        'private_key': 'Arcsky_private_key.dat',
        'configure_opts': '--signed-fw --private-key Arcsky_private_key.dat',
        'changelog_needs_build': False,
        'notes': 'ODID is compiled in via a hwdef define; firmware is signed.',
    },
    {
        'key': 'x55',
        'name': 'X55',
        'fw_prefix': 'X55',
        'version_inc': 'libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc',
        'changelog': 'doc/X55-FIRMWARE-CHANGELOG.md',
        'release_sh': 'Tools/x55/build_release.sh',
        'param_docs': None,          # no parameter disposition tooling on X55
        'tag_prefix': 'x55-fw-v',
        'artifact_prefix': 'x55',
        'boards': ['CubeOrange', 'CubeOrangePlus'],
        'dev_board': None,           # no unlocked engineering target on X55
        'private_key': None,         # not signed
        'configure_opts': '--enable-opendroneid',
        'changelog_needs_build': True,   # X55 headings require "Build N"
        'notes': 'ODID via the --enable-opendroneid build flag, not a hwdef '
                 'define. Firmware is not signed. Changelog headings require a '
                 'Build number.',
    },
]

# Active profile. Populated by detect_profile(); never read before then.
P = PROFILES[0]


def detect_profile(repo):
    """Pick the profile whose version include exists in this repo."""
    if repo:
        for prof in PROFILES:
            if os.path.isfile(os.path.join(repo, prof['version_inc'])):
                return prof
    return None

DEFAULT_BASH = r'C:\cygwin64\bin\bash.exe'

OK, WARN, FAIL, INFO = 'OK', 'WARN', 'FAIL', 'INFO'
COLORS = {OK: '#1a7f37', WARN: '#9a6700', FAIL: '#b3261e', INFO: '#57606a'}
GLYPH = {OK: 'OK', WARN: '!', FAIL: 'X', INFO: '-'}


# ---------------------------------------------------------------- helpers ----

def find_repo(start=None):
    """Walk up looking for this repo. Works frozen (exe next to repo) or not."""
    cands = []
    if start:
        cands.append(start)
    if getattr(sys, 'frozen', False):
        cands.append(os.path.dirname(sys.executable))
    else:
        cands.append(os.path.dirname(os.path.abspath(__file__)))
    cands.append(os.getcwd())
    for c in cands:
        d = os.path.abspath(c)
        for _ in range(6):
            # an ArduPilot tree carrying one of our product version includes
            if os.path.isfile(os.path.join(d, 'ArduCopter', 'version.h')) and \
               any(os.path.isfile(os.path.join(d, pr['version_inc']))
                   for pr in PROFILES):
                return d
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
    return None


def cfg_path():
    base = os.environ.get('LOCALAPPDATA') or os.path.expanduser('~')
    return os.path.join(base, CFG_NAME)


def load_cfg():
    try:
        with open(cfg_path(), encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return {}


def save_cfg(cfg):
    try:
        with open(cfg_path(), 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


def to_cygwin(path):
    p = os.path.abspath(path).replace('\\', '/')
    if len(p) > 1 and p[1] == ':':
        return '/cygdrive/%s%s' % (p[0].lower(), p[2:])
    return p


FROZEN = getattr(sys, 'frozen', False)

# NEVER use sys.executable to run a .py helper. Under PyInstaller sys.executable
# is THIS EXE, so doing so relaunches the GUI, which refreshes its status, which
# relaunches again -- an exponential fork bomb that fires on startup. Resolve a
# real interpreter instead, and treat "no interpreter" as a reportable condition
# rather than something to guess at.
_PY_CACHE = []


def python_exe(cfg=None):
    """Path to a real Python interpreter, or None if none can be found."""
    if _PY_CACHE:
        return _PY_CACHE[0]
    cand = []
    if cfg and cfg.get('python'):
        cand.append(cfg['python'])
    if not FROZEN:
        cand.append(sys.executable)
    for name in ('python', 'python3'):
        w = shutil.which(name)
        if w:
            cand.append(w)
    launcher = shutil.which('py')
    for c in cand:
        if not c or not os.path.isfile(c):
            continue
        if FROZEN and os.path.samefile(c, sys.executable):
            continue          # that is us; never recurse
        try:
            p = subprocess.run([c, '-c', 'print(1)'], capture_output=True,
                               text=True, timeout=20,
                               creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            if p.returncode == 0:
                _PY_CACHE.append(c)
                return c
        except Exception:
            continue
    if launcher:
        _PY_CACHE.append(launcher)
        return launcher
    return None


def guard_reentry():
    """Refuse to start the GUI if we were invoked as if we were an interpreter.

    Belt and braces against the fork bomb above: even if some call site regresses
    to sys.executable, the child exits instead of opening another window.
    """
    if not FROZEN:
        return
    for a in sys.argv[1:]:
        if a.endswith('.py') or a in ('-c', '-m'):
            sys.stderr.write(
                'XplorerRelease.exe was invoked with interpreter-style arguments '
                '(%r).\nThis is not a Python interpreter; refusing to start.\n'
                % (sys.argv[1:],))
            sys.exit(2)


def single_instance():
    """Windows named mutex. Returns handle, or None if another instance holds it."""
    try:
        import ctypes
        from ctypes import wintypes
        k32 = ctypes.WinDLL('kernel32', use_last_error=True)
        k32.CreateMutexW.restype = wintypes.HANDLE
        k32.CreateMutexW.argtypes = [wintypes.LPCVOID, wintypes.BOOL, wintypes.LPCWSTR]
        h = k32.CreateMutexW(None, True, 'Global\\ArcskyXplorerReleaseConsole')
        if ctypes.get_last_error() == 183:      # ERROR_ALREADY_EXISTS
            return None
        return h
    except Exception:
        return True                              # non-Windows / no ctypes: allow


def run(args, cwd, timeout=60):
    """Run a command, return (rc, output). Never raises."""
    try:
        p = subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                           timeout=timeout,
                           creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        return p.returncode, (p.stdout or '') + (p.stderr or '')
    except Exception as e:
        return 1, str(e)


def git(repo, *a, timeout=60):
    return run(['git', '-c', 'core.autocrlf=true'] + list(a), repo, timeout)


# ---------------------------------------------------------------- status -----

def read_version(repo):
    path = os.path.join(repo, P['version_inc'])
    try:
        src = open(path, encoding='utf-8').read()
    except OSError:
        return None
    m = re.search(r'^define\s+AP_CUSTOM_FIRMWARE_STRING\s+"%s v([^"]+)"'
                  % re.escape(P['fw_prefix']),
                  src, re.M)
    return m.group(1) if m else None


def write_version(repo, new):
    path = os.path.join(repo, P['version_inc'])
    src = open(path, encoding='utf-8').read()
    out = re.sub(r'(^define\s+AP_CUSTOM_FIRMWARE_STRING\s+")%s v[^"]+(")'
                 % re.escape(P['fw_prefix']),
                 r'\g<1>%s v%s\g<2>' % (P['fw_prefix'], new),
                 src, count=1, flags=re.M)
    if out == src:
        raise RuntimeError('could not rewrite the version line in %s' % P['version_inc'])
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(out)


def gather_status(repo, bash, py=None):
    """Returns (rows, ready, blockers). rows = [(label, state, detail, hint)]"""
    rows = []
    blockers = []

    def add(label, state, detail, hint=''):
        rows.append((label, state, detail, hint))
        if state == FAIL:
            blockers.append('%s: %s' % (label, hint or detail))

    # --- version ---
    ver = read_version(repo)
    if ver:
        add('Firmware version', OK, '%s v%s' % (P['fw_prefix'], ver),
            'from %s' % P['version_inc'])
    else:
        add('Firmware version', FAIL, 'could not parse',
            'check %s' % P['version_inc'])
        return rows, False, blockers

    # --- branch / head ---
    _, branch = git(repo, 'rev-parse', '--abbrev-ref', 'HEAD')
    _, head = git(repo, 'rev-parse', '--short=8', 'HEAD')
    branch, head = branch.strip(), head.strip()
    add('Branch / commit', INFO, '%s @ %s' % (branch, head))

    # --- tree state ---
    _, porcelain = git(repo, 'status', '--porcelain')
    dirty = [l for l in porcelain.splitlines() if l.strip()]
    if dirty:
        add('Working tree', FAIL, '%d uncommitted file(s)' % len(dirty),
            'commit before releasing, or use an engineering build')
    else:
        add('Working tree', OK, 'clean')

    # --- tag ---
    tag = P['tag_prefix'] + ver
    _, tags = git(repo, 'tag', '--points-at', 'HEAD')
    if tag in tags.split():
        add('Release tag', OK, '%s on HEAD' % tag)
    else:
        _, exists = git(repo, 'tag', '--list', tag)
        if exists.strip():
            add('Release tag', FAIL, '%s exists but not on HEAD' % tag,
                'the tag points at a different commit')
        else:
            add('Release tag', FAIL, '%s missing' % tag,
                'create the tag once the changelog is written')

    # --- changelog ---
    cl = os.path.join(repo, P['changelog'])
    if not os.path.isfile(cl):
        add('Changelog entry', FAIL, 'file missing', P['changelog'])
    else:
        body = open(cl, encoding='utf-8').read()
        if re.search(r'^##\s+v%s\b' % re.escape(ver), body, re.M):
            add('Changelog entry', OK, 'v%s documented' % ver)
        else:
            add('Changelog entry', FAIL, 'no entry for v%s' % ver,
                'add a "## v%s - YYYY-MM-DD" section' % ver)

    # --- signing key (only products that sign) ---
    if P['private_key']:
        if os.path.isfile(os.path.join(repo, P['private_key'])):
            add('Signing key', OK, P['private_key'])
        else:
            add('Signing key', FAIL, '%s not found' % P['private_key'],
                'copy it from Google Drive; it is gitignored')

    # --- python interpreter (needed to run the helper scripts) ---
    if py:
        add('Python', OK, py)
    else:
        add('Python', FAIL, 'no interpreter found',
            'install Python or set its path in Settings')

    # --- parameter docs (only products that generate them) ---
    if not P['param_docs']:
        pass
    elif not py:
        add('Parameter docs', INFO, 'not checked', 'needs a Python interpreter')
    else:
        rc, out = run([py, P['param_docs'], '--check'], repo, timeout=300)
        if rc == 0:
            add('Parameter docs', OK, 'up to date')
        else:
            probs = [l.strip('- ').strip() for l in out.splitlines()
                     if l.strip().startswith('-')]
            add('Parameter docs', WARN, '%d issue(s)' % len(probs),
                probs[0] if probs else 'run gen_param_docs.py')

    # --- cygwin ---
    if os.path.isfile(bash):
        add('Cygwin bash', OK, bash)

        # Build prerequisites, checked against the interpreter the waf shebang
        # actually resolves to (`env python3`), NOT `python`. Cygwin here has
        # python -> 3.7 and python3 -> 3.9, and ArduPilot's own Windows installer
        # pip-installs into 3.7, so a fresh setup leaves 3.9 missing empy and
        # pexpect. That produced "you need to install empy..." mid-build, and the
        # usual workaround (PYTHONPATH pointing at 3.7's site-packages) exposes
        # every 3.7 package to 3.9. Checking here catches it before a build.
        probe = ('env python3 -V; for m in em pexpect serial pymavlink; do '
                 'env python3 -c "import $m" 2>/dev/null || echo MISSING:$m; done')
        rc, out = run([bash, '-lc', probe], repo, timeout=120)
        lines = [l.strip() for l in out.splitlines() if l.strip()]
        pyver = next((l.split()[-1] for l in lines if l.startswith('Python ')), '?')
        if rc != 0 or pyver == '?':
            add('Build prereqs', WARN, 'could not check',
                'in Cygwin run: python3 -m pip install empy==3.3.4 pexpect pyserial')
        else:
            miss = [l.split(':', 1)[1] for l in lines if l.startswith('MISSING:')]
            if miss:
                add('Build prereqs', FAIL,
                    'python3 %s missing: %s' % (pyver, ', '.join(miss)),
                    'in Cygwin run: python3 -m pip install %s'
                    % ' '.join('empy==3.3.4' if m == 'em' else
                               'pyserial' if m == 'serial' else m
                               for m in miss))
            else:
                add('Build prereqs', OK, 'python3 %s has empy, pexpect, pyserial'
                    % pyver)
    else:
        add('Cygwin bash', FAIL, 'not found at %s' % bash,
            'builds run under Cygwin; set the path in Settings')

    # --- built boards ---
    for b in P['boards']:
        h = os.path.join(repo, 'build', b, 'hwdef.h')
        apj = os.path.join(repo, 'build', b, 'bin', 'arducopter.apj')
        if not os.path.isfile(h):
            add('Build: %s' % b, INFO, 'not built')
            continue
        src = open(h, encoding='utf-8', errors='ignore').read()
        m = re.search(r'#define\s+AP_CUSTOM_FIRMWARE_STRING\s+"%s v([^"]+)"'
                      % re.escape(P['fw_prefix']), src)
        bv = m.group(1) if m else '?'
        unlocked = '#define XPLORER_DEV_UNLOCK_ENABLED 1' in src
        signed = '#define AP_SIGNED_FIRMWARE 1' in src
        bits = ['v%s' % bv]
        if unlocked:
            bits.append('PARAMS UNLOCKED')
        if P['private_key'] and not signed:
            bits.append('UNSIGNED')
        if not os.path.isfile(apj):
            bits.append('no apj')
        detail = ', '.join(bits)
        # Only meaningful where an unlocked target exists. On products without
        # one, ANY unlock compiled in is wrong.
        want_unlock = (P['dev_board'] is not None and b == P['dev_board'])
        if bv != ver:
            add('Build: %s' % b, WARN, detail,
                'stale - built at v%s, source is v%s' % (bv, ver))
        elif unlocked != want_unlock:
            add('Build: %s' % b, FAIL, detail,
                'DEV unlock state is wrong for this board')
        elif P['private_key'] and not signed:
            add('Build: %s' % b, FAIL, detail, 'would not boot on a production Cube')
        else:
            add('Build: %s' % b, OK, detail)

    ready = not blockers
    return rows, ready, blockers


TESTBUILD_DIR = 'testbuilds'
SECTIONS = ['Added', 'Fixed', 'Changed', 'Known issues']


def stamp_test_build(repo, board):
    """Copy a just-built apj/bin to testbuilds/ under a self-describing name.

    Point is that a test build you hand to someone, or find on your desk a week
    later, identifies itself: version, board, DEV-ness, when, which commit, and
    whether the tree was dirty at the time. The release process is not involved.
    """
    import datetime
    import shutil as sh
    src_dir = os.path.join(repo, 'build', board, 'bin')
    apj = os.path.join(src_dir, 'arducopter.apj')
    if not os.path.isfile(apj):
        return None, 'no arducopter.apj in %s' % src_dir

    ver = read_version(repo) or 'unknown'
    hw = os.path.join(repo, 'build', board, 'hwdef.h')
    dev = False
    if os.path.isfile(hw):
        dev = '#define XPLORER_DEV_UNLOCK_ENABLED 1' in \
              open(hw, encoding='utf-8', errors='ignore').read()

    _, head = git(repo, 'rev-parse', '--short=8', 'HEAD')
    _, porcelain = git(repo, 'status', '--porcelain')
    dirty = bool([l for l in porcelain.splitlines() if l.strip()])

    # mtime of the artifact, not "now" -- so re-stamping an older build is honest
    ts = datetime.datetime.fromtimestamp(os.path.getmtime(apj))
    token = board.replace('CubeOrangePlus-', '')
    # the DEV board's token already says DEV; do not say it twice
    dev_tag = '-DEV' if dev and 'DEV' not in token.upper() else ''

    name = '%s-v%s%s-%s-%s-%s%s' % (
        P['artifact_prefix'],
        ver,
        dev_tag,
        token,
        ts.strftime('%Y%m%d-%H%M'),
        head.strip() or 'nogit',
        '-dirty' if dirty else '')

    out = os.path.join(repo, TESTBUILD_DIR)
    os.makedirs(out, exist_ok=True)
    made = []
    for ext in ('apj', 'bin'):
        s = os.path.join(src_dir, 'arducopter.%s' % ext)
        if os.path.isfile(s):
            d = os.path.join(out, '%s.%s' % (name, ext))
            sh.copy2(s, d)
            made.append(os.path.basename(d))
    return made, None


def next_build_number(repo):
    """Highest existing 'Build N' in the changelog, plus one. 1 if none."""
    path = os.path.join(repo, P['changelog'])
    try:
        src = open(path, encoding='utf-8').read()
    except OSError:
        return 1
    nums = [int(n) for n in re.findall(r'^##\s+.*?Build\s+(\d+)', src, re.M)]
    return (max(nums) + 1) if nums else 1


def build_changelog_md(version, date, headline, blocks, build=None):
    """Render one changelog section from the structured form.

    X55 changelog headings carry a Build number (its gen_release_notes.py
    requires one); Xplorer's do not. Driven by the profile so one form serves
    both.
    """
    if build:
        L = ['## v%s — Build %s — %s' % (version, build, date), '']
    else:
        L = ['## v%s — %s' % (version, date), '']
    if headline:
        L += ['### %s' % headline.strip().upper(), '']
    for name in SECTIONS:
        items = [l.strip() for l in blocks.get(name, '').splitlines() if l.strip()]
        if not items:
            continue
        L.append('### %s' % name)
        for it in items:
            it = it.lstrip('-*').strip()
            # wrap continuation lines to match the file's style
            L.append('- %s' % it)
        L.append('')
    return '\n'.join(L).rstrip() + '\n'


def insert_changelog_entry(repo, version, md, replace=False):
    """Insert (or replace) a version section, keeping newest-first order."""
    path = os.path.join(repo, P['changelog'])
    src = open(path, encoding='utf-8').read()
    heads = list(re.finditer(r'^##\s+v(\d+\.\d+\.\d+)\b.*$', src, re.M))

    existing = next((m for m in heads if m.group(1) == version), None)
    if existing and not replace:
        return False, 'v%s already has a section' % version

    if existing:
        # replace from this heading up to the next '---' + heading, or EOF
        nxt = next((m for m in heads if m.start() > existing.start()), None)
        if nxt:
            tail = src[nxt.start():]
            sep = src.rfind('\n---\n', existing.start(), nxt.start())
            end = sep + 1 if sep != -1 else nxt.start()
            new = src[:existing.start()] + md + '\n' + src[end:]
        else:
            new = src[:existing.start()] + md
    elif heads:
        at = heads[0].start()
        new = src[:at] + md + '\n---\n\n' + src[at:]
    else:
        new = src.rstrip() + '\n\n---\n\n' + md
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(new)
    return True, None


# ------------------------------------------------------------------- GUI -----

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP)
        self.geometry('1060x760')
        self.minsize(900, 620)

        self.cfg = load_cfg()
        self.repo = self.cfg.get('repo') or find_repo()
        self.bash = self.cfg.get('bash', DEFAULT_BASH)
        self.py = python_exe(self.cfg)
        self.q = queue.Queue()
        self.busy = False
        self.rows = []
        self._use_profile(self.repo)

        self._build_ui()
        self._use_profile(self.repo)      # now that the widgets exist
        if not self.repo:
            self.after(200, self.pick_repo)
        else:
            self.after(200, self.refresh)
        self.after(80, self._drain)

    def _use_profile(self, repo):
        """Switch the active product profile to match `repo`.

        Called at startup and whenever the repository changes, so pointing at the
        other clone switches products with no further configuration.
        """
        global P
        prof = detect_profile(repo)
        if prof is None:
            return False
        changed = prof is not P
        P = prof
        self.title('%s Firmware Release Console' % P['name'])
        # Adapt controls that only make sense for some products.
        if getattr(self, 'b_reldev', None) is not None:
            self.b_reldev.config(
                text='Build engineering (DEV)' if P['dev_board']
                else 'Build engineering (relaxed gates)')
        if getattr(self, 'b_docs', None) is not None:
            self.b_docs.config(
                state='normal' if P['param_docs'] else 'disabled')
        if getattr(self, 'board', None) is not None:
            self.board.set(P['boards'][0])
        if getattr(self, 'board_combo', None) is not None:
            self.board_combo.config(values=P['boards'])
        if changed:
            try:
                self.emit('\nproduct: %s   (%s)\n' % (P['name'], P['notes']), 'ok')
            except Exception:
                pass          # emit() needs the queue; harmless during __init__
        return True

    # -- layout --
    def _build_ui(self):
        top = ttk.Frame(self, padding=(10, 8))
        top.pack(fill='x')
        ttk.Label(top, text=APP, font=('Segoe UI', 13, 'bold')).pack(side='left')
        self.repo_lbl = ttk.Label(top, text='', foreground='#57606a')
        self.repo_lbl.pack(side='left', padx=12)
        ttk.Button(top, text='Settings', command=self.settings).pack(side='right')
        ttk.Button(top, text='Guide / Help',
                   command=lambda: HelpWindow(self)).pack(side='right', padx=6)
        ttk.Button(top, text='Refresh', command=self.refresh).pack(side='right', padx=6)

        self.banner = tk.Label(self, text='Checking...', anchor='w',
                               font=('Segoe UI', 10, 'bold'),
                               padx=10, pady=6, bg='#eaeef2')
        self.banner.pack(fill='x')

        body = ttk.Frame(self, padding=(10, 6))
        body.pack(fill='both', expand=True)

        # status table
        sf = ttk.LabelFrame(body, text='Status', padding=8)
        sf.pack(fill='x')
        self.status_host = ttk.Frame(sf)
        self.status_host.pack(fill='x')

        # actions
        af = ttk.LabelFrame(body, text='Actions', padding=8)
        af.pack(fill='x', pady=(8, 0))

        r1 = ttk.Frame(af); r1.pack(fill='x')
        ttk.Label(r1, text='Board:').pack(side='left')
        self.board = tk.StringVar(value=P['boards'][0])
        self.board_combo = ttk.Combobox(r1, textvariable=self.board,
                                        values=P['boards'], width=26,
                                        state='readonly')
        self.board_combo.pack(side='left', padx=6)
        self.b_build = ttk.Button(r1, text='Build firmware (test)',
                                  command=self.do_build)
        self.b_build.pack(side='left', padx=4)
        self.b_docs = ttk.Button(r1, text='Regenerate param docs',
                                 command=self.do_param_docs)
        self.b_docs.pack(side='left', padx=4)
        self.b_open_build = ttk.Button(r1, text='Open build folder',
                                       command=self.open_build)
        self.b_open_build.pack(side='left', padx=4)
        ttk.Button(r1, text='Open test builds',
                   command=self.open_testbuilds).pack(side='left', padx=4)

        r2 = ttk.Frame(af); r2.pack(fill='x', pady=(8, 0))
        ttk.Label(r2, text='Version:').pack(side='left')
        self.ver_var = tk.StringVar()
        ttk.Entry(r2, textvariable=self.ver_var, width=12).pack(side='left', padx=6)
        self.b_bump = ttk.Button(r2, text='Set version', command=self.do_bump)
        self.b_bump.pack(side='left', padx=4)
        self.b_cl = ttk.Button(r2, text='Changelog entry...',
                               command=self.do_changelog_form)
        self.b_cl.pack(side='left', padx=4)
        ttk.Button(r2, text='Open .md', command=self.open_changelog
                   ).pack(side='left', padx=2)
        self.b_commit = ttk.Button(r2, text='Stage all + commit', command=self.do_commit)
        self.b_commit.pack(side='left', padx=4)
        self.b_tag = ttk.Button(r2, text='Create release tag', command=self.do_tag)
        self.b_tag.pack(side='left', padx=4)
        self.b_movetag = ttk.Button(r2, text='Move tag to HEAD',
                                    command=self.do_move_tag)
        self.b_movetag.pack(side='left', padx=4)

        r3 = ttk.Frame(af); r3.pack(fill='x', pady=(8, 0))
        self.b_verify = ttk.Button(r3, text='Verify release (dry run)',
                                   command=lambda: self.do_release(check=True))
        self.b_verify.pack(side='left', padx=4)
        self.b_rel = ttk.Button(r3, text='BUILD RELEASE', command=self.do_release)
        self.b_rel.pack(side='left', padx=4)
        self.b_reldev = ttk.Button(r3, text='Build engineering',
                                   command=lambda: self.do_release(dev=True))
        self.b_reldev.pack(side='left', padx=4)
        self.b_open_rel = ttk.Button(r3, text='Open release folder',
                                     command=self.open_release)
        self.b_open_rel.pack(side='left', padx=4)

        lf = ttk.LabelFrame(body, text='Log', padding=6)
        lf.pack(fill='both', expand=True, pady=(8, 0))
        self.log = scrolledtext.ScrolledText(lf, height=16, wrap='none',
                                            font=('Consolas', 9))
        self.log.pack(fill='both', expand=True)
        for tag, col in (('cmd', '#0b6bcb'), ('err', '#b3261e'),
                         ('ok', '#1a7f37'), ('warn', '#9a6700')):
            self.log.tag_config(tag, foreground=col)

        self.status_bar = ttk.Label(self, text='Ready', anchor='w',
                                    relief='sunken', padding=(8, 3))
        self.status_bar.pack(fill='x', side='bottom')

        self._action_buttons = [self.b_build, self.b_docs, self.b_bump, self.b_cl,
                                self.b_commit, self.b_tag, self.b_movetag,
                                self.b_verify,
                                self.b_rel, self.b_reldev]

    # -- log plumbing --
    def emit(self, text, tag=None):
        self.q.put(('log', text, tag))

    def _drain(self):
        try:
            while True:
                kind, a, b = self.q.get_nowait()
                if kind == 'log':
                    self.log.insert('end', a, b or '')
                    self.log.see('end')
                elif kind == 'status':
                    self.status_bar.config(text=a)
                elif kind == 'done':
                    self._set_busy(False)
                    if a:
                        self.refresh()
                elif kind == 'rows':
                    self._render(a, b)
        except queue.Empty:
            pass
        self.after(80, self._drain)

    def _set_busy(self, busy):
        self.busy = busy
        state = 'disabled' if busy else 'normal'
        for b in self._action_buttons:
            b.config(state=state)

    # -- running commands --
    def _spawn(self, title, argv, cygwin_cmd=None, refresh_after=True,
               on_success=None):
        if self.busy:
            messagebox.showinfo(APP, 'Another task is still running.')
            return
        self._set_busy(True)
        self.q.put(('status', title, None))
        self.emit('\n%s\n' % ('=' * 78))
        self.emit('%s\n' % title, 'cmd')
        self.emit('%s\n' % ('=' * 78))

        def worker():
            try:
                if cygwin_cmd:
                    argv2 = [self.bash, '-lc',
                             'cd %s && %s' % (to_cygwin(self.repo), cygwin_cmd)]
                    self.emit('$ %s\n' % cygwin_cmd, 'cmd')
                else:
                    argv2 = argv
                    self.emit('$ %s\n' % ' '.join(argv2), 'cmd')
                p = subprocess.Popen(
                    argv2, cwd=self.repo, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, text=True, bufsize=1,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                for line in p.stdout:
                    low = line.lower()
                    tag = None
                    if 'error' in low or 'fail' in low:
                        tag = 'err'
                    elif 'warning' in low:
                        tag = 'warn'
                    self.emit(line, tag)
                rc = p.wait()
                if rc == 0:
                    self.emit('\n-- finished OK --\n', 'ok')
                    if on_success is not None:
                        try:
                            on_success()
                        except Exception as e:
                            self.emit('post-step failed: %s\n' % e, 'err')
                else:
                    self.emit('\n-- FAILED (exit %d) --\n' % rc, 'err')
            except Exception as e:
                self.emit('\nlauncher error: %s\n' % e, 'err')
            finally:
                self.q.put(('status', 'Ready', None))
                self.q.put(('done', refresh_after, None))

        threading.Thread(target=worker, daemon=True).start()

    # -- status --
    def refresh(self):
        if not self.repo:
            return
        self.repo_lbl.config(text='%s  |  %s' % (P['name'], self.repo))
        self.q.put(('status', 'Checking status...', None))

        def worker():
            rows, ready, blockers = gather_status(self.repo, self.bash, self.py)
            self.q.put(('rows', rows, (ready, blockers)))
            self.q.put(('status', 'Ready', None))
        threading.Thread(target=worker, daemon=True).start()

    def _render(self, rows, extra):
        ready, blockers = extra
        for w in self.status_host.winfo_children():
            w.destroy()
        for i, (label, state, detail, hint) in enumerate(rows):
            tk.Label(self.status_host, text=GLYPH[state], width=3,
                     fg=COLORS[state], font=('Segoe UI', 9, 'bold')
                     ).grid(row=i, column=0, sticky='w')
            tk.Label(self.status_host, text=label, width=22, anchor='w'
                     ).grid(row=i, column=1, sticky='w')
            tk.Label(self.status_host, text=detail, anchor='w',
                     fg=COLORS[state]).grid(row=i, column=2, sticky='w')
            if hint:
                tk.Label(self.status_host, text=hint, anchor='w',
                         fg='#57606a').grid(row=i, column=3, sticky='w', padx=(12, 0))
        self.status_host.columnconfigure(3, weight=1)

        ver = read_version(self.repo) or ''
        if not self.ver_var.get():
            self.ver_var.set(ver)
        if ready:
            self.banner.config(
                text='READY to build release %s v%s' % (P['name'], ver),
                bg='#d6f5dd', fg='#1a7f37')
        else:
            self.banner.config(
                text='NOT READY  -  %s' % ';  '.join(blockers[:3]),
                bg='#fde8e6', fg='#b3261e')

    # -- actions --
    def do_build(self):
        b = self.board.get()
        cmd = ('./waf configure --board %s %s && ./waf copter'
               % (b, P['configure_opts']))
        self._spawn('Build firmware: %s' % b, None, cygwin_cmd=cmd,
                    on_success=lambda: self._stamp(b))

    def _stamp(self, board):
        made, err = stamp_test_build(self.repo, board)
        if err:
            self.emit('could not stamp a named copy: %s\n' % err, 'warn')
            return
        self.emit('\nnamed copy in %s/:\n' % TESTBUILD_DIR, 'ok')
        for n in made:
            self.emit('  %s\n' % n, 'ok')

    def do_changelog_form(self):
        ver = read_version(self.repo)
        d = ChangelogDialog(self, self.repo, ver)
        if d.saved:
            self.emit('\nchangelog updated for v%s\n' % ver, 'ok')
            self.refresh()

    def open_testbuilds(self):
        self._open(os.path.join(self.repo, TESTBUILD_DIR))

    def do_param_docs(self):
        if not self.py:
            messagebox.showerror(
                APP, 'No Python interpreter found.\n\n'
                     'This exe cannot run the helper scripts itself. Install '
                     'Python, or set its path under Settings.')
            return
        self._spawn('Regenerate parameter documentation',
                    [self.py, P['param_docs']])

    def do_bump(self):
        new = self.ver_var.get().strip()
        if not re.match(r'^\d+\.\d+\.\d+$', new):
            messagebox.showerror(APP, 'Version must look like 1.0.2')
            return
        cur = read_version(self.repo)
        if new == cur:
            messagebox.showinfo(APP, 'Version is already %s' % cur)
            return
        if not messagebox.askyesno(
                APP, 'Set the firmware version from %s to %s?\n\n'
                     'This edits %s. You will still need a changelog entry, a '
                     'commit and a tag.' % (cur, new, P['version_inc'])):
            return
        try:
            write_version(self.repo, new)
        except Exception as e:
            messagebox.showerror(APP, str(e))
            return
        self.emit('\nversion: %s -> %s (%s)\n' % (cur, new, P['version_inc']), 'ok')
        self.refresh()

    def open_changelog(self):
        self._open(os.path.join(self.repo, P['changelog']))

    def open_build(self):
        self._open(os.path.join(self.repo, 'build', self.board.get(), 'bin'))

    def open_release(self):
        ver = read_version(self.repo)
        d = os.path.join(self.repo, 'release')
        cand = os.path.join(d, '%s-v%s' % (P['artifact_prefix'], ver))
        self._open(cand if os.path.isdir(cand) else d)

    def _open(self, path):
        if not os.path.exists(path):
            messagebox.showinfo(APP, 'Not created yet:\n%s' % path)
            return
        os.startfile(path)  # noqa: S606 - Windows-only tool

    def do_commit(self):
        _, porcelain = git(self.repo, 'status', '--porcelain')
        files = [l for l in porcelain.splitlines() if l.strip()]
        if not files:
            messagebox.showinfo(APP, 'Nothing to commit - tree is clean.')
            return
        ver = read_version(self.repo)
        msg = SimplePrompt(self, 'Commit message',
                           'Staging %d file(s).\n\n%s' %
                           (len(files), '\n'.join(files[:15])),
                           'Xplorer firmware v%s' % ver).result
        if not msg:
            return
        rc, out = git(self.repo, 'add', '-A')
        self.emit('\n$ git add -A\n%s' % out, 'cmd')
        rc, out = run(['git', 'commit', '-m', msg], self.repo)
        self.emit('$ git commit -m %r\n%s\n' % (msg, out),
                  'ok' if rc == 0 else 'err')
        self.refresh()

    def do_tag(self):
        ver = read_version(self.repo)
        tag = P['tag_prefix'] + ver
        _, porcelain = git(self.repo, 'status', '--porcelain')
        if [l for l in porcelain.splitlines() if l.strip()]:
            if not messagebox.askyesno(
                    APP, 'The tree is dirty. Tagging now records a commit that '
                         'does not contain your uncommitted changes.\n\nTag anyway?'):
                return
        if not messagebox.askyesno(APP, 'Create annotated tag %s on HEAD?' % tag):
            return
        rc, out = run(['git', 'tag', '-a', tag, '-m',
                       'Xplorer firmware v%s' % ver], self.repo)
        self.emit('\n$ git tag -a %s\n%s\n' % (tag, out or '(created)'),
                  'ok' if rc == 0 else 'err')
        self.refresh()

    def _tag_on_remote(self, tag):
        """(is_on_remote, checked_ok). Network call; offline is not 'absent'."""
        self.q.put(('status', 'Checking whether %s is on origin...' % tag, None))
        rc, out = run(['git', 'ls-remote', '--tags', 'origin',
                       'refs/tags/%s' % tag], self.repo, timeout=45)
        self.q.put(('status', 'Ready', None))
        if rc != 0:
            return False, False
        return ('refs/tags/%s' % tag) in out, True

    def do_move_tag(self):
        """Re-point the release tag at HEAD. Refuses if it is already published."""
        ver = read_version(self.repo)
        tag = 'xplorer-fw-v%s' % ver

        _, exists = run(['git', 'tag', '--list', tag], self.repo)
        if not exists.strip():
            messagebox.showinfo(
                APP, '%s does not exist yet.\n\nUse "Create release tag".' % tag)
            return

        _, at_head = run(['git', 'tag', '--points-at', 'HEAD'], self.repo)
        if tag in at_head.split():
            messagebox.showinfo(APP, '%s is already on HEAD. Nothing to do.' % tag)
            return

        on_remote, checked = self._tag_on_remote(tag)
        if on_remote:
            messagebox.showerror(
                APP,
                'REFUSING to move %s — it is already pushed to origin.\n\n'
                'Moving a published tag means anyone who already fetched it has '
                'different code under the same name, and a binary in the field '
                'can no longer be resolved to source.\n\n'
                'Bump to the next patch version instead, document it, and tag '
                'that.' % tag)
            self.emit('\nrefused to move %s: already on origin\n' % tag, 'err')
            return
        if not checked:
            if not messagebox.askyesno(
                    APP,
                    'Could not reach origin to confirm whether %s has been '
                    'pushed.\n\nOnly continue if you are sure it has NOT been '
                    'pushed. Moving a published tag is not recoverable.\n\n'
                    'Continue?' % tag):
                return

        old = run(['git', 'rev-parse', '--short=8', '%s^{commit}' % tag],
                  self.repo)[1].strip()
        head = run(['git', 'rev-parse', '--short=8', 'HEAD'], self.repo)[1].strip()
        if not messagebox.askyesno(
                APP, 'Move %s from %s to %s?\n\nThe tag is local only, so this is '
                     'safe.' % (tag, old, head)):
            return

        rc, out = run(['git', 'tag', '-d', tag], self.repo)
        self.emit('\n$ git tag -d %s\n%s' % (tag, out), 'cmd')
        if rc != 0:
            self.emit('failed to delete the old tag; aborting\n', 'err')
            self.refresh()
            return
        rc, out = run(['git', 'tag', '-a', tag, '-m',
                       'Xplorer firmware v%s' % ver], self.repo)
        self.emit('$ git tag -a %s\n%s\n' % (tag, out or '(created on %s)' % head),
                  'ok' if rc == 0 else 'err')
        self.refresh()

    def do_release(self, check=False, dev=False):
        args = []
        if check:
            args.append('--check')
        if dev:
            args.append('--dev')
        title = ('Verify release (dry run)' if check else
                 'Build ENGINEERING release' if dev else 'BUILD RELEASE')
        if not check and not dev:
            ver = read_version(self.repo)
            if not messagebox.askyesno(
                    APP, 'Build and stage release v%s?\n\n'
                         'This runs the full gated release build.' % ver):
                return
        self._spawn(title, None,
                    cygwin_cmd='bash %s %s' % (P['release_sh'], ' '.join(args)))

    # -- settings --
    def pick_repo(self):
        d = filedialog.askdirectory(title='Select the ardupilot-xplorer repo')
        if not d:
            messagebox.showerror(APP, 'No repository selected; exiting.')
            self.destroy()
            return
        if not os.path.isfile(os.path.join(d, 'ArduCopter', 'version.h')):
            messagebox.showerror(APP, 'That does not look like an ArduPilot repo.')
            return
        if detect_profile(d) is None:
            messagebox.showerror(
                APP, 'No product version include found in that repo.\n\n'
                     'Expected one of:\n  ' +
                '\n  '.join(pr['version_inc'] for pr in PROFILES))
            return
        self.repo = d
        self._use_profile(d)
        self.cfg['repo'] = d
        save_cfg(self.cfg)
        self.refresh()

    def settings(self):
        win = tk.Toplevel(self)
        win.title('Settings')
        win.transient(self)
        win.resizable(False, False)
        f = ttk.Frame(win, padding=12)
        f.pack(fill='both', expand=True)

        ttk.Label(f, text='Repository:').grid(row=0, column=0, sticky='w')
        rv = tk.StringVar(value=self.repo or '')
        ttk.Entry(f, textvariable=rv, width=62).grid(row=0, column=1, padx=6)
        ttk.Button(f, text='...', width=3,
                   command=lambda: rv.set(filedialog.askdirectory() or rv.get())
                   ).grid(row=0, column=2)

        ttk.Label(f, text='Cygwin bash:').grid(row=1, column=0, sticky='w', pady=(8, 0))
        bv = tk.StringVar(value=self.bash)
        ttk.Entry(f, textvariable=bv, width=62).grid(row=1, column=1, padx=6, pady=(8, 0))
        ttk.Button(f, text='...', width=3,
                   command=lambda: bv.set(filedialog.askopenfilename(
                       filetypes=[('bash.exe', 'bash.exe')]) or bv.get())
                   ).grid(row=1, column=2, pady=(8, 0))

        ttk.Label(f, text='Python:').grid(row=2, column=0, sticky='w', pady=(8, 0))
        pv = tk.StringVar(value=self.py or '')
        ttk.Entry(f, textvariable=pv, width=62).grid(row=2, column=1, padx=6, pady=(8, 0))
        ttk.Button(f, text='...', width=3,
                   command=lambda: pv.set(filedialog.askopenfilename(
                       filetypes=[('python.exe', 'python*.exe')]) or pv.get())
                   ).grid(row=2, column=2, pady=(8, 0))

        ttk.Label(f, text='waf runs under Cygwin on this machine, so builds must '
                          'use that bash. Python is needed to run the helper\n'
                          'scripts -- the packaged exe is not itself an '
                          'interpreter.', foreground='#57606a', justify='left'
                  ).grid(row=3, column=0, columnspan=3, sticky='w', pady=(10, 0))

        def save():
            self.repo = rv.get() or self.repo
            self._use_profile(self.repo)
            self.bash = bv.get() or self.bash
            if pv.get() and pv.get() != (self.py or ''):
                self.cfg['python'] = pv.get()
                _PY_CACHE.clear()
                self.py = python_exe(self.cfg)
            self.cfg.update({'repo': self.repo, 'bash': self.bash})
            save_cfg(self.cfg)
            win.destroy()
            self.refresh()

        bb = ttk.Frame(f)
        bb.grid(row=4, column=0, columnspan=3, sticky='e', pady=(14, 0))
        ttk.Button(bb, text='Cancel', command=win.destroy).pack(side='right', padx=4)
        ttk.Button(bb, text='Save', command=save).pack(side='right')


GUIDE = """\
XPLORER FIRMWARE — HOW THIS ALL WORKS

  This console drives the whole firmware process. It never reimplements
  anything: every button runs the same command a developer would type, and the
  Log pane shows that command plus its live output. If this tool is ever broken,
  everything still works by hand — see Tools/xplorer/README.md.

────────────────────────────────────────────────────────────────────────────
THE NORMAL DAY: change code, build, test
────────────────────────────────────────────────────────────────────────────

  1. Edit code in the repo however you like (VS Code, etc).
  2. Pick a Board, press "Build firmware (test)".
       Production airframes      -> CubeOrangePlus-ODID
       Unlocked engineering build -> CubeOrangePlus-ODID-DEV
       Virgin Cube bootstrap      -> CubeOrangePlus
  3. When it finishes, a self-describing copy is placed in testbuilds/, e.g.
       xplorer-v1.0.1-DEV-ODID-DEV-20260813-1445-d5934508-dirty.apj
     version, board, DEV flag, timestamp, commit, and whether the tree was
     dirty. Press "Open test builds" to get at it. Flash that with Mission
     Planner / QGC.
  4. Repeat. No versioning, tagging or changelog needed for test builds.

────────────────────────────────────────────────────────────────────────────
WHEN YOU ARE READY TO RELEASE
────────────────────────────────────────────────────────────────────────────

  1. "Set version"        — bump the number (e.g. 1.0.1 -> 1.0.2). This edits
                            ONE file, and it reaches the boot banner, the
                            MAVLink AUTOPILOT_VERSION, and the dataflash log.
  2. "Changelog entry..." — fill in the form. It writes the markdown for you;
                            you never have to hand-edit the .md.
  3. "Stage all + commit" — commits everything with a message.
  4. "Create release tag" — makes xplorer-fw-v<version> on that commit.
  5. "Verify release"     — dry run. Tells you if anything is still wrong.
  6. "BUILD RELEASE"      — builds, verifies, and stages everything into
                            release/xplorer-v<version>/ with checksums and
                            generated customer release notes.

  The status panel is green READY only when all of that is in order. If it is
  red, the reason is printed next to the failing row.

────────────────────────────────────────────────────────────────────────────
WHAT THE STATUS ROWS MEAN
────────────────────────────────────────────────────────────────────────────

  Firmware version  From hwdef/include/xplorer_version.inc — the single source
                    of truth. Nothing else needs editing.
  Working tree      Uncommitted files. A release from a dirty tree records a
                    commit that does not contain the code you built, so the
                    release build refuses. Test builds do not care.
  Release tag       Every release carries a tag, so a banner seen in the field
                    resolves back to exact source.
  Changelog entry   The customer release notes are GENERATED from the changelog,
                    so no entry means no release note.
  Signing key       Arcsky_private_key.dat must be in the repo root. Fielded
                    bootloaders only run signed firmware. The key is NOT in git
                    — copy it from Google Drive.
  Python            Needed to run the helper scripts. The exe is not itself a
                    Python interpreter.
  Build prereqs     Cygwin has python 3.7 AND python3 3.9. waf uses 3.9, but
                    ArduPilot's installer puts packages in 3.7. If something is
                    missing this row gives you the exact pip command. Do NOT
                    "fix" it with PYTHONPATH.
  Build: <board>    Whether that board is built, at which version, signed, and
                    whether the DEV parameter unlock is compiled in. "stale"
                    means the binary predates the current version.

────────────────────────────────────────────────────────────────────────────
PRODUCTION vs DEV BUILD — IMPORTANT
────────────────────────────────────────────────────────────────────────────

  The DEV build ignores the @READONLY locks on PSC/ATC/EK3/notch/FFT/MOT and
  bypasses the parameter clamp table, so tuning can be done over MAVLink.

  It shares APJ_BOARD_ID 10163 with production, because fielded bootloaders
  only accept that ID — a unique ID would make DEV firmware impossible to
  upload. So the separation is PROCEDURAL:

    * DEV firmware announces itself at WARNING level on every boot:
        "DEV BUILD - PARAMS UNLOCKED - NOT FOR FLIGHT OPS"
    * DEV artifacts have -DEV in the filename.
    * A signed DEV apj WILL load onto any Xplorer. Never publish one to the
      release/update channel.

────────────────────────────────────────────────────────────────────────────
FILES WORTH KNOWING
────────────────────────────────────────────────────────────────────────────

  libraries/AP_HAL_ChibiOS/hwdef/include/xplorer_version.inc
      The version. One line. Everything else follows from it.
  doc/<PRODUCT>-FIRMWARE-CHANGELOG.md
      Source of the customer release notes. Use the form, not an editor.
  libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-ODID/defaults.parm
      Shipped parameter defaults and @READONLY locks.
  libraries/AP_Param/xplorer_dev_unlock.h
      Which parameter groups the DEV build unlocks.
  libraries/GCS_MAVLink/GCS_Param_Clamps.h
      Hard min/max bounds enforced on parameter writes.
  doc/params/xplorer-params.html
      Searchable reference for every parameter. Generated — ship it to support.
  Tools/xplorer/README.md
      The long-form version of this guide.

────────────────────────────────────────────────────────────────────────────
IF SOMETHING GOES WRONG
────────────────────────────────────────────────────────────────────────────

  Build says "you need to install empy"
      Cygwin python version mix-up. See the Build prereqs row for the exact
      command. Never use PYTHONPATH to work around it.
  Build fails right after configure
      Another board may have been configured last. Just press Build again for
      the board you want; configure runs each time.
  "not ready" and you do not know why
      Read the red rows. Each one states what to do.
  You edited release_gui.py
      Close this console, then run:
        powershell -ExecutionPolicy Bypass -File Tools\\xplorer\\build_gui_exe.ps1
      (a running exe holds a file lock and the copy will silently fail)
"""


class HelpWindow(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title('%s — Guide' % APP)
        self.geometry('900x680')
        f = ttk.Frame(self, padding=8)
        f.pack(fill='both', expand=True)
        t = scrolledtext.ScrolledText(f, wrap='word', font=('Consolas', 9))
        t.pack(fill='both', expand=True)
        t.insert('1.0', GUIDE)
        t.config(state='disabled')
        ttk.Button(f, text='Close', command=self.destroy).pack(anchor='e', pady=(8, 0))


class ChangelogDialog(tk.Toplevel):
    """Structured form -> markdown. So the .md never needs hand-editing."""

    def __init__(self, parent, repo, version):
        super().__init__(parent)
        self.repo = repo
        self.saved = False
        self.title('New changelog entry')
        self.geometry('880x720')
        self.transient(parent)

        import datetime
        f = ttk.Frame(self, padding=12)
        f.pack(fill='both', expand=True)

        hdr = ttk.Frame(f)
        hdr.pack(fill='x')
        ttk.Label(hdr, text='Version:').grid(row=0, column=0, sticky='w')
        self.v_ver = tk.StringVar(value=version or '')
        ttk.Entry(hdr, textvariable=self.v_ver, width=12).grid(row=0, column=1, padx=(4, 16))
        ttk.Label(hdr, text='Date:').grid(row=0, column=2, sticky='w')
        self.v_date = tk.StringVar(value=datetime.date.today().isoformat())
        ttk.Entry(hdr, textvariable=self.v_date, width=14).grid(row=0, column=3, padx=(4, 16))
        self.v_build = tk.StringVar()
        if P['changelog_needs_build']:
            ttk.Label(hdr, text='Build:').grid(row=1, column=0, sticky='w',
                                               pady=(6, 0))
            self.v_build.set(str(next_build_number(repo)))
            ttk.Entry(hdr, textvariable=self.v_build, width=8).grid(
                row=1, column=1, sticky='w', padx=(4, 16), pady=(6, 0))
        ttk.Label(hdr, text='Headline:').grid(row=0, column=4, sticky='w')
        self.v_head = tk.StringVar()
        ttk.Entry(hdr, textvariable=self.v_head, width=44).grid(row=0, column=5, padx=4)

        ttk.Label(f, foreground='#57606a', justify='left',
                  text='One change per line. Plain text — no markdown, no dashes '
                       'needed. Write for the person\nflying or supporting the '
                       'aircraft, not for a developer.'
                  ).pack(anchor='w', pady=(10, 4))

        self.boxes = {}
        for name in SECTIONS:
            lf = ttk.LabelFrame(f, text=name, padding=6)
            lf.pack(fill='both', expand=True, pady=3)
            t = tk.Text(lf, height=4, wrap='word', font=('Segoe UI', 9))
            t.pack(fill='both', expand=True)
            self.boxes[name] = t

        bb = ttk.Frame(f)
        bb.pack(fill='x', pady=(10, 0))
        ttk.Button(bb, text='Cancel', command=self.destroy).pack(side='right', padx=4)
        ttk.Button(bb, text='Save to changelog', command=self._save).pack(side='right')
        ttk.Button(bb, text='Preview', command=self._preview).pack(side='left')

        self.grab_set()
        parent.wait_window(self)

    def _md(self):
        return build_changelog_md(
            self.v_ver.get().strip(), self.v_date.get().strip(),
            self.v_head.get().strip(),
            {n: t.get('1.0', 'end') for n, t in self.boxes.items()},
            build=self.v_build.get().strip() or None)

    def _preview(self):
        w = tk.Toplevel(self)
        w.title('Preview')
        w.geometry('760x520')
        t = scrolledtext.ScrolledText(w, wrap='word', font=('Consolas', 9))
        t.pack(fill='both', expand=True)
        t.insert('1.0', self._md())
        t.config(state='disabled')

    def _save(self):
        ver = self.v_ver.get().strip()
        if not re.match(r'^\d+\.\d+\.\d+$', ver):
            messagebox.showerror(APP, 'Version must look like 1.0.2', parent=self)
            return
        if not re.match(r'^\d{4}-\d{2}-\d{2}$', self.v_date.get().strip()):
            messagebox.showerror(APP, 'Date must be YYYY-MM-DD', parent=self)
            return
        if not any(t.get('1.0', 'end').strip() for t in self.boxes.values()):
            messagebox.showerror(APP, 'Add at least one change.', parent=self)
            return
        md = self._md()
        ok, err = insert_changelog_entry(self.repo, ver, md)
        if not ok:
            if not messagebox.askyesno(
                    APP, '%s.\n\nReplace the existing section?' % err, parent=self):
                return
            ok, err = insert_changelog_entry(self.repo, ver, md, replace=True)
            if not ok:
                messagebox.showerror(APP, err, parent=self)
                return
        self.saved = True
        self.destroy()


class SimplePrompt(tk.Toplevel):
    """Multi-line-ish prompt returning .result (None if cancelled)."""

    def __init__(self, parent, title, info, initial=''):
        super().__init__(parent)
        self.result = None
        self.title(title)
        self.transient(parent)
        self.resizable(False, False)
        f = ttk.Frame(self, padding=12)
        f.pack(fill='both', expand=True)
        tk.Label(f, text=info, justify='left', anchor='w',
                 font=('Consolas', 8), fg='#57606a').pack(anchor='w')
        self.var = tk.StringVar(value=initial)
        e = ttk.Entry(f, textvariable=self.var, width=70)
        e.pack(fill='x', pady=(10, 0))
        e.focus_set()
        e.icursor('end')
        bb = ttk.Frame(f)
        bb.pack(anchor='e', pady=(12, 0))
        ttk.Button(bb, text='Cancel', command=self.destroy).pack(side='right', padx=4)
        ttk.Button(bb, text='OK', command=self._ok).pack(side='right')
        self.bind('<Return>', lambda _e: self._ok())
        self.bind('<Escape>', lambda _e: self.destroy())
        self.grab_set()
        parent.wait_window(self)

    def _ok(self):
        self.result = self.var.get().strip() or None
        self.destroy()


def main():
    guard_reentry()
    lock = single_instance()
    if lock is None:
        # Deliberately NOT a modal messagebox: that blocks until dismissed, so a
        # duplicate launch would sit there as a live process. Show a small notice
        # that closes itself, guaranteeing the duplicate always goes away.
        try:
            root = tk.Tk()
            root.title(APP)
            root.attributes('-topmost', True)
            root.resizable(False, False)
            tk.Label(root, padx=24, pady=18, justify='left',
                     font=('Segoe UI', 10),
                     text='The release console is already running.\n\n'
                          'Look for its window in the taskbar.\n'
                          '(this notice closes itself)').pack()
            root.after(4000, root.destroy)
            root.mainloop()
        except Exception:
            sys.stderr.write('already running\n')
        return 0
    app = App()
    app.mainloop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
