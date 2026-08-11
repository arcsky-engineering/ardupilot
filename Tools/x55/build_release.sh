#!/bin/bash
#
# Build a traceable Arcsky X55 firmware release.
#
# The point of this script is that a fielded binary's version string can always
# be resolved back to exact source. It refuses to produce a "release" build that
# nothing could later be reconstructed from, and it writes a manifest recording
# what went into each artifact.
#
# Usage:
#   Tools/x55/build_release.sh                 # verify, build all boards, stage artifacts
#   Tools/x55/build_release.sh --check         # verify only; no build (CI / pre-flight)
#   Tools/x55/build_release.sh --board CubeOrangePlus
#   Tools/x55/build_release.sh --dev           # engineering build: allow dirty tree, no tag
#
# Run from a Cygwin shell -- the same environment ./waf is configured in.

set -e

cd "$(dirname "$0")/../.."

VERSION_INC=libraries/AP_HAL_ChibiOS/hwdef/include/x55_version.inc
CHANGELOG=doc/X55-FIRMWARE-CHANGELOG.md
NOTES_GEN=Tools/x55/gen_release_notes.py
NOTES_TXT=X55_ArduCopter_Release_Notes.txt
BOARDS="CubeOrange CubeOrangePlus"
CONFIGURE_OPTS="--enable-opendroneid"
PYTHON="${PYTHON:-python}"

CHECK_ONLY=0
ALLOW_DIRTY=0
CHECK_TAG=1

while [ $# -gt 0 ]; do
    case "$1" in
        --check)        CHECK_ONLY=1 ;;
        --dev)          ALLOW_DIRTY=1; CHECK_TAG=0 ;;
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

VERSION=$(sed -n 's/^define AP_CUSTOM_FIRMWARE_STRING "X55 v\([^"]*\)".*/\1/p' "$VERSION_INC")
[ -n "$VERSION" ] || fail "could not parse a version out of $VERSION_INC"

FWSTRING="X55 v$VERSION"
TAG="x55-fw-v$VERSION"
# --short=8 to match GIT_VERSION exactly (Tools/ardupilotwaf/git_submodule.py),
# so the hash in the filename is the same hash the boot banner reports.
HASH=$(git rev-parse --short=8 HEAD)
UPSTREAM=$(sed -n 's/^#define THISFIRMWARE "\(.*\)"/\1/p' ArduCopter/version.h)

echo "X55 firmware release"
echo "  version string : $FWSTRING"
echo "  upstream base  : $UPSTREAM"
echo "  commit         : $HASH"
echo "  boards         : $BOARDS"
echo

# ---------------------------------------------------------------- gate: tree --
# GIT_VERSION is a bare short hash (Tools/ardupilotwaf/boards.py) with no dirty
# marker, so a binary built from an uncommitted tree reports a hash whose code it
# is not. That is the single easiest way to lose traceability, so block it.
#
# The '-c core.autocrlf=true' is load-bearing, not decoration. This tree is
# checked out by Windows Git, whose system config
# (C:/Program Files/Git/etc/gitconfig) sets core.autocrlf=true, so every text
# file is CRLF on disk. Cygwin ships its own git at /usr/bin/git which never
# reads that config, defaults to autocrlf=false, and therefore reports all ~4500
# text files as modified -- and this script is normally run from Cygwin, because
# that is where waf is configured. Forcing the setting for this one call makes
# the answer identical from either git, and it propagates into submodules too.

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
    git tag -a $TAG -m \"X55 firmware v$VERSION\"
Or pass --dev for an engineering build."
    fi
fi

# ------------------------------------------------------------ gate: changelog -
# A hard failure, not a warning. Undocumented firmware is how 77e5940f (the DB300
# Remote ID work) reached a release with nothing written up anywhere. The notes
# are also generated from this file, so a missing entry means a missing release
# note, not just a missing line in a changelog.

if [ ! -f "$CHANGELOG" ]; then
    fail "$CHANGELOG is missing -- it is the source of the release notes"
fi

if [ "$CHECK_TAG" = "1" ]; then
    $PYTHON "$NOTES_GEN" --changelog "$CHANGELOG" --check --version "$VERSION" \
        || fail "changelog gate failed (see above). Pass --dev to bypass for an engineering build."
else
    # engineering build: still confirm the changelog parses, but do not demand an
    # entry for a version that may not exist yet
    $PYTHON "$NOTES_GEN" --changelog "$CHANGELOG" --check \
        || fail "$CHANGELOG does not parse (see above)"
fi

if [ "$CHECK_ONLY" = "1" ]; then
    echo "OK: $FWSTRING is consistent and releasable from $HASH"
    exit 0
fi

# ----------------------------------------------------------------- build ------

RELDIR="release/x55-v$VERSION"
mkdir -p "$RELDIR"
MANIFEST="$RELDIR/MANIFEST.txt"

{
    echo "Arcsky X55 firmware release manifest"
    echo "version string   : $FWSTRING"
    echo "upstream base    : $UPSTREAM"
    echo "git commit       : $(git rev-parse HEAD)"
    echo "git tag          : $(git tag --points-at HEAD | tr '\n' ' ')"
    echo "branch           : $(git rev-parse --abbrev-ref HEAD)"
    echo "tree state       : $([ -n "$DIRTY" ] && echo DIRTY || echo clean)"
    echo "configure opts   : $CONFIGURE_OPTS"
    echo
} > "$MANIFEST"

for BOARD in $BOARDS; do
    echo "=== $BOARD ==="
    ./waf configure --board "$BOARD" $CONFIGURE_OPTS
    ./waf copter

    # Confirm the version string actually reached this board. Catches the case
    # where a new board hwdef forgot to include x55_version.inc.
    HWDEF_H="build/$BOARD/hwdef.h"
    if ! grep -qF "#define AP_CUSTOM_FIRMWARE_STRING \"$FWSTRING\"" "$HWDEF_H"; then
        fail "$BOARD did not compile in '$FWSTRING'. Does its hwdef.dat include ../include/x55_version.inc?
    got: $(grep AP_CUSTOM_FIRMWARE_STRING "$HWDEF_H" || echo '<nothing>')"
    fi

    APJ_ID=$(sed -n 's/^#define APJ_BOARD_ID \([0-9]*\).*/\1/p' "$HWDEF_H")
    echo "board            : $BOARD (APJ_BOARD_ID $APJ_ID)" >> "$MANIFEST"

    for EXT in apj bin abin; do
        SRC="build/$BOARD/bin/arducopter.$EXT"
        [ -f "$SRC" ] || continue
        DST="$RELDIR/x55-v$VERSION-$BOARD-$HASH.$EXT"
        cp "$SRC" "$DST"
        echo "  $(basename "$DST")  sha256=$(sha256sum "$DST" | cut -d' ' -f1)" >> "$MANIFEST"
    done
    echo >> "$MANIFEST"
done

# ---------------------------------------------------------- release notes -----
# Generated last, because it stamps in the sha256 sums that only exist once the
# artifacts do. Source is $CHANGELOG; this file is never hand-edited.

$PYTHON "$NOTES_GEN" --changelog "$CHANGELOG" --manifest "$MANIFEST" \
                     --out "$RELDIR/$NOTES_TXT" \
    || fail "failed to generate $NOTES_TXT"

echo
echo "Staged in $RELDIR:"
ls -1 "$RELDIR"
echo
cat "$MANIFEST"
echo
echo "Release notes: $RELDIR/$NOTES_TXT"
echo "Copy the .apj files and that .txt to the Dropbox release folder by hand."
