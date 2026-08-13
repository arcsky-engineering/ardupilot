# X55 firmware versioning

The X55 identifies itself as **`X55 v<X.Y.Z>`**, not `ArduCopter V4.4.4`, via
ArduPilot's `AP_CUSTOM_FIRMWARE_STRING` vendor hook.

This file covers the versioning and release tooling. For the whole workflow —
setup, making a change, testing, releasing — see
[`doc/X55-DEV-WORKFLOW.md`](../../doc/X55-DEV-WORKFLOW.md).

## Cutting a release

```bash
# 1. bump the version
vi libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc

# 2. describe what changed
vi doc/X55-FIRMWARE-CHANGELOG.md

# 3. commit and tag
git commit -am "X55 firmware v1.0.1"
git tag -a x55-fw-v1.0.1 -m "X55 firmware v1.0.1"

# 4. build both boards and stage traceable artifacts
Tools/x55/build_release.sh
```

Output lands in `release/x55-v1.0.1/` (gitignored):

| File | What it is |
|---|---|
| `x55-v1.0.1-<board>-<hash>.apj` | flash this (Mission Planner / QGC) |
| `x55-v1.0.1-<board>-<hash>.bin` / `.abin` | direct-flash / flash-from-bootloader |
| `MANIFEST.txt` | version, baseline, full commit, tag, tree state, configure opts, sha256 per artifact |
| `X55_ArduCopter_Release_Notes.txt` | **generated** customer-facing notes |

**Ship the manifest and the notes alongside the binaries.** They are what lets a
unit's firmware be identified six months from now.

Copy them into the Dropbox release folder **by hand** — the script deliberately
never writes outside the repo, so a release build can't sync outward before
you've flown it.

## Release notes

`doc/X55-FIRMWARE-CHANGELOG.md` is the single source of truth.
`gen_release_notes.py` reformats it into the plain-text layout techs already
read, and stamps in the build facts from `MANIFEST.txt`:

```
doc/X55-FIRMWARE-CHANGELOG.md   ← you write here, in the same commit as the code
         │
         │  gen_release_notes.py  (run automatically by build_release.sh)
         ▼
release/x55-v1.0.1/X55_ArduCopter_Release_Notes.txt   ← generated, never edited
```

**Never hand-edit the `.txt`.** It is overwritten on every release build. Edit
the changelog and rebuild.

Writing an entry — one `##` heading per release, an optional `###` headline, then
prose with `**Changes:**` / `**Compatibility:**` / `**Files:**` sections:

```markdown
## v1.0.1 — Build 16 — 2026-09-02

### SHORT HEADLINE

**Commit:** `abc12345`

**Changes:**

- what changed

**Compatibility:**

- what an operator or tech has to know before flashing this

**Files:** Foo.cpp, Bar.h
```

The `Build N` numbers continue the historical sequence so techs who know
"Build 13" can still navigate. Newest entry goes at the top; the generator
enforces descending order and rejects duplicate build numbers.

The build **fails** if there is no changelog entry for the version being
released. That is deliberate: `77e5940f` (the DB300 Remote ID work, 135 lines
across three files) shipped with no write-up anywhere, and nothing caught it.

You can regenerate or preview the notes without building:

```bash
python Tools/x55/gen_release_notes.py --out /tmp/notes.txt   # no build facts
python Tools/x55/gen_release_notes.py --check --version 1.0.1
```

Other modes:

```bash
Tools/x55/build_release.sh --check                  # verify only, no build
Tools/x55/build_release.sh --dev                    # engineering build (dirty tree OK, no tag)
Tools/x55/build_release.sh --board CubeOrangePlus   # one board
```

## Why the script refuses things

**Dirty tree.** [`Tools/ardupilotwaf/boards.py`](../ardupilotwaf/boards.py) sets
`GIT_VERSION` from a bare `git_head_hash(short=True)` with no dirty marker. A
binary built from an uncommitted tree therefore advertises a hash whose code it
is not, which is the fastest way to lose traceability on a fielded unit. Use
`--dev` when you genuinely want a scratch build; it warns and stamps the manifest
`DIRTY`.

**Missing tag.** Without a tag, `X55 v1.0.1` is a claim rather than a pointer.
The tag is what makes a banner resolvable back to source.

**No changelog entry for this version.** See Release notes above. Undocumented
firmware is how DB300 support reached a release with nothing written up.

**Version string absent from `hwdef.h`.** After each board builds, the script
greps the generated `build/<board>/hwdef.h` to confirm the string actually
compiled in. This catches a new board hwdef that forgot
`include ../include/x55_version.inc` — which would otherwise ship silently
branded as stock ArduCopter.

## What must not change

Do **not** edit `ArduCopter/version.h`. `THISFIRMWARE` and `FIRMWARE_VERSION`
have to stay at the upstream 4.4.4 values:

- ground stations use them to select which parameter metadata to load
- the autotest scripts parse the `FIRMWARE_VERSION` line
- `AP_CheckFirmware` / bootloader compatibility assumes upstream semantics

[`AP_FWVersionDefine.h`](../../libraries/AP_Common/AP_FWVersionDefine.h) swaps
`AP_CUSTOM_FIRMWARE_STRING` into `fw_string` and `fw_short_string` while keeping
`THISFIRMWARE` in `fw_string_original`, and
[`LoggerMessageWriter.cpp`](../../libraries/AP_Logger/LoggerMessageWriter.cpp)
logs both. That is why the log line carries `[ArduCopter V4.4.4]` in brackets:
the upstream baseline stays recoverable from any flight log.

## Adding a board

Add these two lines to its `hwdef.dat` and nothing else — the version is
inherited, so it can never drift from the other boards:

```
include ../include/x55_version.inc
```

Plus the USB branding, if the board does not already inherit it from
`CubeOrange/hwdef.dat`:

```
undef USB_STRING_MANUFACTURER
undef USB_STRING_PRODUCT
USB_STRING_MANUFACTURER "Arcsky"
USB_STRING_PRODUCT "X55"
```

Then add the board to `BOARDS` in `build_release.sh`.

## Boards

| hwdef | APJ_BOARD_ID | Notes |
|---|---|---|
| `CubeOrange` | 140 | Production |
| `CubeOrangePlus` | 1063 | Production |
| `CubeOrange-Arcsky` | 10140 | Not built by `build_release.sh`; inherits branding via `CubeOrange/hwdef.dat` |
| `CubeOrangePlus-Arcsky` | 1063 | Not built by `build_release.sh`; branded explicitly |

The two `-Arcsky` hwdefs have never been built in this tree (nothing under
`build/` came from them) — production uses the stock board names with
`--enable-opendroneid`. If they are dead, deleting them removes a second place a
conflicting version string could be introduced.

Rebranding the USB strings does not change `APJ_BOARD_ID`, so existing `.apj`
files still flash and the bootloader is unaffected. What changes is the device
name Windows/Mission Planner shows for the COM port: `Arcsky X55`.
