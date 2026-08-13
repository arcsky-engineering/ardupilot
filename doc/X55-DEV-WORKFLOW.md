# X55 firmware: how to make a change and ship it

Start-to-finish process for modifying X55 ArduCopter firmware. If you have never
built this before, do the one-time setup first.

Related: [`Tools/x55/README.md`](../Tools/x55/README.md) (versioning and release
tooling), [`doc/X55-FIRMWARE-CHANGELOG.md`](X55-FIRMWARE-CHANGELOG.md) (release
notes source), [`ARCSKY-README.md`](../ARCSKY-README.md) (fork and branch layout).

---

## 0. One-time setup

**Clone.** This repo holds firmware for more than one product on different
branches. Keep X55 in its own clone so you never have to switch branches:

```
C:\GitHub\ardupilot-x55\ardupilot\   → x55/4.4.4-custom      (this one)
C:\GitHub\ardupilot-xplorer\         → xplorer/4.6.2-custom
```

Remotes: `origin` = `arcsky-engineering/ardupilot`, `upstream` = `ArduPilot/ardupilot`.

**Toolchain.** Builds run under **Cygwin**, not cmd, PowerShell, or Git Bash.
The waf configuration in this tree is Cygwin-specific (`.lock-waf_cygwin_build`,
`/cygdrive/c/...` paths). You need:

- Cygwin64 with `python`, `git`, `make`
- GNU Arm Embedded Toolchain 10-2020-q4-major on `PATH`
- `pip install -r Tools/ardupilotwaf/../../requirements.txt` per ArduPilot docs

ArduPilot's own build docs: https://ardupilot.org/dev/docs/building-the-code.html

**Fix Cygwin's git line endings — do this before your first commit.**

```bash
git config --global core.autocrlf true      # run this IN the Cygwin terminal
```

Windows Git sets `core.autocrlf=true` in its own system config and checks the
tree out as CRLF. Cygwin ships a *separate* git that never reads that config, so
without this it reports **all ~4500 text files as modified**. If you `git add -A`
in that state you will commit a CRLF rewrite of the entire repo. Verify with
`git status` in Cygwin — it should be clean, not thousands of files.

---

## 1. Branch

More than one person touches this now, so work on a branch rather than committing
straight to the product branch:

```bash
git checkout x55/4.4.4-custom
git pull
git checkout -b x55/<short-description>
```

## 2. Make the change

Where things live:

| What | Where |
|---|---|
| X55-specific flight logic | `ArduCopter/UserCode.cpp` |
| Custom parameters | `ArduCopter/Parameters.cpp`, `Parameters.h` |
| Hybrid Module / generator | `ArduCopter/UserCode.cpp` |
| Remote ID | `libraries/AP_OpenDroneID/` |
| Per-board defaults, read-only params | `libraries/AP_HAL_ChibiOS/hwdef/<board>/defaults.parm` |
| Board config, feature enables, branding | `libraries/AP_HAL_ChibiOS/hwdef/<board>/hwdef.dat` |

**Do not touch `ArduCopter/version.h`.** `THISFIRMWARE` and `FIRMWARE_VERSION`
must stay at the upstream 4.4.4 values — ground stations use them to pick
parameter metadata and autotest parses that line. The X55 version string lives in
`libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc`.

## 3. Build and test

```bash
./waf configure --board CubeOrangePlus --enable-opendroneid
./waf copter
```

Output: `build/CubeOrangePlus/bin/arducopter.apj`. `--enable-opendroneid` is
required — Remote ID is not compiled in without it.

SITL first where the change can be tested there:

```bash
./waf configure --board sitl && ./waf copter
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

Then bench-test on hardware. Flash via Mission Planner → Setup → Install
Firmware → **Load custom firmware**. Confirm the boot banner, then fly it.

Anything touching the Hybrid Module, Remote ID, battery failsafe, or flight modes
needs hardware testing — SITL will not exercise the rectifier link or a Remote ID
module.

## 4. Commit and open a PR

```bash
git add <specific files>
git commit -m "Clear description of what changed and why"
git push -u origin x55/<short-description>
gh pr create --base x55/4.4.4-custom
```

Do not bump the version yet. Several changes can batch into one release.

## 5. Release

Once the change is merged into `x55/4.4.4-custom` and tested, cut a release. Full
detail in [`Tools/x55/README.md`](../Tools/x55/README.md); the short version:

```bash
git checkout x55/4.4.4-custom && git pull

# 1. bump the version
vi libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc      # X55 v1.0.1

# 2. write the release notes entry (new ## block at the TOP)
vi doc/X55-FIRMWARE-CHANGELOG.md

# 3. commit and tag
git commit -am "X55 firmware v1.0.1"
git tag -a x55-fw-v1.0.1 -m "X55 firmware v1.0.1"

# 4. build both boards (Cygwin terminal)
./Tools/x55/build_release.sh

# 5. push
git push origin x55/4.4.4-custom && git push origin x55-fw-v1.0.1
```

**Version numbers:** patch for fixes, minor for new behaviour or new parameters,
major when a release needs a deliberate parameter migration or is not
field-upgradable in place.

`build_release.sh` will stop you if the tree is dirty, if there is no
`x55-fw-v<version>` tag on HEAD, if the changelog has no entry for the version,
or if a board did not compile the version string in. Those are the guardrails —
read the error, don't work around it. `--dev` bypasses the tree and tag checks
for a throwaway engineering build and stamps the manifest `DIRTY`.

## 6. Distribute

`release/x55-v1.0.1/` now holds the `.apj`/`.bin`/`.abin` per board, plus
`MANIFEST.txt` and a generated `X55_ArduCopter_Release_Notes.txt`.

Copy the `.apj` files and the `.txt` **by hand** into:

```
Dropbox\Firmware-Releases\X55\Cube Flight Controller\Firmware\
```

Move the previous release into `Firmware\ARCHIVE\`. Keep the manifest with the
release — it is what identifies a fielded unit later.

The script never writes outside the repo on purpose, so a release build cannot
sync to anyone before you have flown it.

---

## Rules

1. **Never ship a build from a dirty tree.** `GIT_VERSION` is a bare
   `rev-parse --short=8` with no dirty marker, so the hash in the banner will not
   match the code. This is how the fleet became hard to identify in the first
   place.
2. **Never edit `ArduCopter/version.h`.**
3. **Never hand-edit `X55_ArduCopter_Release_Notes.txt`.** It is generated and
   overwritten. Edit `doc/X55-FIRMWARE-CHANGELOG.md`.
4. **Every release gets a tag.** An untagged version string is a claim, not a
   pointer.
5. **Write the changelog entry in the same commit as the code change** where you
   can. Its purpose is that the next person can tell what a fielded unit does.
6. **Do not retag a version you have already shipped binaries from.** Bump the
   patch version instead.
7. **Adding a board?** Add `include ../include/x55_version.inc` to its
   `hwdef.dat` and add it to `BOARDS` in `build_release.sh`, or it ships branded
   as stock ArduCopter. The build's post-compile check catches this.

## Identifying what a unit is running

Boot banner and dataflash `MSG`:

```
X55 v1.0.0 (ece37646) [ArduCopter V4.4.4]
 │           │          └─ upstream ArduPilot baseline
 │           └─ git commit (8 chars, matches the release filename)
 └─ Arcsky firmware version → git tag x55-fw-v1.0.0
```

Anything reporting only `ArduCopter V4.4.4` with no `X55 v…` predates v1.0.0
(2026-08-11) and can only be identified by its git hash, where one was recorded
at all.

## Known gaps

- **Parameter files are not versioned with the firmware.** The `.param` files in
  the Dropbox `Parameters\` folder are dated, not tied to a firmware version, so
  nothing records which parameter set belongs with which release. Worth closing.
- **No unit database.** There is no authoritative record of which aircraft is
  running which firmware, and no way to ask a unit to self-report.
- **Pre-v1.0.0 releases are not reliably identifiable**, per above.
- **Upstream is not tracked.** This branch is pinned to ArduCopter 4.4.4; moving
  to a newer baseline is a deliberate porting exercise, not a merge.
