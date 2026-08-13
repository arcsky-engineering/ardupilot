#!/bin/bash
#
# Build a traceable Arcsky Xplorer firmware release.
#
# The point of this script is that a fielded binary's version string can always
# be resolved back to exact source. It refuses to produce a "release" build that
# nothing could later be reconstructed from, and it writes a manifest recording
# what went into each artifact.
#
# Usage:
#   Tools/xplorer/build_release.sh                  # verify, build, stage artifacts
#   Tools/xplorer/build_release.sh --check          # verify only; no build
#   Tools/xplorer/build_release.sh --board CubeOrangePlus
#   Tools/xplorer/build_release.sh --dev            # engineering build of the
#                                                   # unlocked DEV target
#
# Run from a Cygwin shell -- the same environment ./waf is configured in.

set -e

cd "$(dirname "$0")/../.."

VERSION_INC=libraries/AP_HAL_ChibiOS/hwdef/include/xplorer_version.inc
CHANGELOG=doc/XPLORER-FIRMWARE-CHANGELOG.md
NOTES_GEN=Tools/xplorer/gen_release_notes.py
NOTES_TXT=Xplorer_ArduCopter_Release_Notes.txt
PARAM_DOCS=Tools/xplorer/gen_param_docs.py

# CubeOrangePlus-ODID is the product. CubeOrangePlus (board id 1063) is the
# bootstrap build used to get ODID firmware onto a virgin Cube -- add it with
# --board when you need it.
BOARDS="CubeOrangePlus-ODID"
DEV_BOARD="CubeOrangePlus-ODID-DEV"

PRIVATE_KEY="Arcsky_private_key.dat"
PYTHON="${PYTHON:-python}"

CHECK_ONLY=0
ALLOW_DIRTY=0
CHECK_TAG=1
DEV_BUILD=0

while [ $# -gt 0 ]; do
    case "$1" in
        --check)        CHECK_ONLY=1 ;;
        --dev)          DEV_BUILD=1; ALLOW_DIRTY=1; CHECK_TAG=0; BOARDS="$DEV_BOARD" ;;
        --allow-dirty)  ALLOW_DIRTY=1 ;;
        --no-tag-check) CHECK_TAG=0 ;;
        --board)        shift; BOARDS="$1" ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

fail() { echo "ERROR: $*" >&2; exit 1; }
warn() { echo "WARNING: $*" >&2; }

# ---------------------------------------------------------------- version ----

[ -f "$VERSION_INC" ] || fail "$VERSION_INC not found"

VERSION=$(sed -n 's/^define AP_CUSTOM_FIRMWARE_STRING "Xplorer v\([^"]*\)".*/\1/p' "$VERSION_INC")
[ -n "$VERSION" ] || fail "could not parse a version out of $VERSION_INC"

FWSTRING="Xplorer v$VERSION"
TAG="xplorer-fw-v$VERSION"
# --short=8 to match GIT_VERSION exactly (Tools/ardupilotwaf/git_submodule.py),
# so the hash in the filename is the same hash the boot banner reports.
HASH=$(git rev-parse --short=8 HEAD)
UPSTREAM=$(sed -n 's/^#define THISFIRMWARE "\(.*\)"/\1/p' ArduCopter/version.h)

echo "Xplorer firmware release"
echo "  version string : $FWSTRING"
echo "  upstream base  : $UPSTREAM"
echo "  commit         : $HASH"
echo "  boards         : $BOARDS"
[ "$DEV_BUILD" = "1" ] && echo "  MODE           : ENGINEERING (params unlocked, not for flight ops)"
echo

[ -f "$PRIVATE_KEY" ] || fail "$PRIVATE_KEY not found in the repo root.
Fielded bootloaders are built with AP_SIGNED_FIRMWARE, so an unsigned build will
not boot. Copy the key from Google Drive (it is gitignored, deliberately)."

# ---------------------------------------------------------------- gate: tree --
# GIT_VERSION is a bare short hash (Tools/ardupilotwaf/boards.py) with no dirty
# marker, so a binary built from an uncommitted tree reports a hash whose code it
# is not. That is the single easiest way to lose traceability, so block it.
#
# The '-c core.autocrlf=true' is load-bearing, not decoration. This tree is
# checked out by Windows Git, whose system config sets core.autocrlf=true, so
# every text file is CRLF on disk. Cygwin ships its own git which never reads
# that config, defaults to autocrlf=false, and therefore reports all ~4500 text
# files as modified -- and this script is normally run from Cygwin, because that
# is where waf is configured. Forcing the setting for this one call makes the
# answer identical from either git, and it propagates into submodules too.

DIRTY=$(git -c core.autocrlf=true status --porcelain)
if [ -n "$DIRTY" ]; then
    if [ "$ALLOW_DIRTY" = "1" ]; then
        warn "working tree is dirty; ($HASH) in the version string will NOT match the built code"
    else
        echo "$DIRTY" >&2
        fail "working tree is dirty. Commit first, or pass --dev for an engineering build."
    fi
fi

# ---------------------------------------------------------------- gate: tag ---

if [ "$CHECK_TAG" = "1" ]; then
    if ! git tag --points-at HEAD | grep -qFx "$TAG"; then
        fail "no tag '$TAG' on HEAD. Bump $VERSION_INC, update $CHANGELOG, commit, then:
    git tag -a $TAG -m \"Xplorer firmware v$VERSION\"
Or pass --dev for an engineering build."
    fi
fi

# ------------------------------------------------------------ gate: changelog -
# A hard failure, not a warning. The release notes are generated from this file,
# so a missing entry means a missing release note, not just a missing changelog
# line.

[ -f "$CHANGELOG" ] || fail "$CHANGELOG is missing -- it is the source of the release notes"

if [ -f "$NOTES_GEN" ]; then
    if [ "$CHECK_TAG" = "1" ]; then
        $PYTHON "$NOTES_GEN" --changelog "$CHANGELOG" --check --version "$VERSION" \
            || fail "changelog gate failed (see above). Pass --dev to bypass for an engineering build."
    else
        $PYTHON "$NOTES_GEN" --changelog "$CHANGELOG" --check \
            || fail "$CHANGELOG does not parse (see above)"
    fi
else
    # Minimal gate if the notes generator has not been ported yet.
    grep -qF "## Xplorer v$VERSION" "$CHANGELOG" \
        || fail "$CHANGELOG has no '## Xplorer v$VERSION' section. Document the release first."
fi

# ------------------------------------------------------- gate: parameter docs -
# The generated parameter reference ships to support, so a stale one is a support
# problem. This also catches a param added to firmware but never documented.

if [ -f "$PARAM_DOCS" ] && [ "$CHECK_TAG" = "1" ]; then
    $PYTHON "$PARAM_DOCS" --check \
        || fail "parameter docs are stale or drifting (see above). Run: $PYTHON $PARAM_DOCS"
fi

if [ "$CHECK_ONLY" = "1" ]; then
    echo "OK: $FWSTRING is consistent and releasable from $HASH"
    exit 0
fi

# ----------------------------------------------------------------- build ------

if [ "$DEV_BUILD" = "1" ]; then
    RELDIR="release/xplorer-v$VERSION-DEV"
else
    RELDIR="release/xplorer-v$VERSION"
fi
# Start from an empty staging directory. Re-running after a new commit otherwise
# leaves the previous run's artifacts alongside the new ones while MANIFEST.txt
# lists only the latest -- filenames differ by an 8-char hash, so it is very easy
# to publish the wrong binary. Everything here is reproducible from the tag, so
# wiping is safe.
if [ -d "$RELDIR" ] && [ -n "$(ls -A "$RELDIR" 2>/dev/null)" ]; then
    echo "clearing previous contents of $RELDIR"
    rm -rf "$RELDIR"
fi
mkdir -p "$RELDIR"
MANIFEST="$RELDIR/MANIFEST.txt"

{
    echo "Arcsky Xplorer firmware release manifest"
    echo "version string   : $FWSTRING"
    echo "upstream base    : $UPSTREAM"
    echo "git commit       : $(git rev-parse HEAD)"
    echo "git tag          : $(git tag --points-at HEAD | tr '\n' ' ')"
    echo "branch           : $(git rev-parse --abbrev-ref HEAD)"
    echo "tree state       : $([ -n "$DIRTY" ] && echo DIRTY || echo clean)"
    echo "signed with      : $PRIVATE_KEY"
    [ "$DEV_BUILD" = "1" ] && echo "BUILD TYPE       : ENGINEERING - PARAMETERS UNLOCKED, NOT FOR FLIGHT OPS"
    echo
} > "$MANIFEST"

for BOARD in $BOARDS; do
    echo "=== $BOARD ==="
    ./waf configure --board "$BOARD" --signed-fw --private-key "$PRIVATE_KEY"
    ./waf copter

    HWDEF_H="build/$BOARD/hwdef.h"

    # Confirm the version string actually reached this board. Catches the case
    # where a new board hwdef forgot to pick up xplorer_version.inc -- exactly
    # how build/CubeOrangePlus went stale at v1.0.0 while source said v1.0.1.
    grep -qF "#define AP_CUSTOM_FIRMWARE_STRING \"$FWSTRING\"" "$HWDEF_H" \
        || fail "$BOARD did not compile in '$FWSTRING'. Does its hwdef chain include ../include/xplorer_version.inc?
    got: $(grep AP_CUSTOM_FIRMWARE_STRING "$HWDEF_H" || echo '<nothing>')"

    # Confirm signing is actually on, so we cannot ship a 76-byte zero signature
    # that the bootloader will refuse.
    grep -qF "#define AP_SIGNED_FIRMWARE 1" "$HWDEF_H" \
        || fail "$BOARD built without AP_SIGNED_FIRMWARE; it would not boot on a production Cube"

    # The DEV unlock must be present on the DEV board and absent everywhere else.
    if [ "$BOARD" = "$DEV_BOARD" ]; then
        grep -qF "#define XPLORER_DEV_UNLOCK_ENABLED 1" "$HWDEF_H" \
            || fail "$BOARD did not compile in the DEV parameter unlock"
    else
        ! grep -qF "#define XPLORER_DEV_UNLOCK_ENABLED 1" "$HWDEF_H" \
            || fail "$BOARD has the DEV parameter unlock compiled in. Refusing to stage it as a release."
    fi

    APJ_ID=$(sed -n 's/^#define APJ_BOARD_ID \([0-9]*\).*/\1/p' "$HWDEF_H")
    echo "board            : $BOARD (APJ_BOARD_ID $APJ_ID)" >> "$MANIFEST"

    SUFFIX=""
    [ "$DEV_BUILD" = "1" ] && SUFFIX="-DEV"

    for EXT in apj bin abin; do
        SRC="build/$BOARD/bin/arducopter.$EXT"
        [ -f "$SRC" ] || continue
        DST="$RELDIR/xplorer-v$VERSION$SUFFIX-$BOARD-$HASH.$EXT"
        cp "$SRC" "$DST"
        echo "  $(basename "$DST")  sha256=$(sha256sum "$DST" | cut -d' ' -f1)" >> "$MANIFEST"
    done
    echo >> "$MANIFEST"
done

# ------------------------------------------------------- parameter reference --
# Ship the parameter docs with the firmware so support has the matching reference.

if [ -f doc/params/xplorer-params.html ]; then
    cp doc/params/xplorer-params.html "$RELDIR/xplorer-v$VERSION-params.html"
    cp doc/params/xplorer-params.csv  "$RELDIR/xplorer-v$VERSION-params.csv"
    echo "parameter reference: xplorer-v$VERSION-params.html / .csv" >> "$MANIFEST"
fi

# ---------------------------------------------------------- release notes -----
# Generated last, because it stamps in the sha256 sums that only exist once the
# artifacts do. Source is $CHANGELOG; this file is never hand-edited.

if [ -f "$NOTES_GEN" ]; then
    $PYTHON "$NOTES_GEN" --changelog "$CHANGELOG" --manifest "$MANIFEST" \
                         --out "$RELDIR/$NOTES_TXT" \
        || fail "failed to generate $NOTES_TXT"
fi

echo
echo "Staged in $RELDIR:"
ls -1 "$RELDIR"
echo
cat "$MANIFEST"
if [ "$DEV_BUILD" = "1" ]; then
    echo
    echo "*** ENGINEERING BUILD - parameters unlocked. Do NOT publish to the"
    echo "*** release/update channel. It shares APJ_BOARD_ID with production and"
    echo "*** will load onto any Xplorer."
fi
