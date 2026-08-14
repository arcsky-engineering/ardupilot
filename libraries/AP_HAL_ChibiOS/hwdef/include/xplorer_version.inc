# Arcsky Xplorer firmware identity.
#
# This is the SINGLE SOURCE OF TRUTH for the Xplorer firmware version string.
# Every board hwdef the Xplorer ships on picks this up, so bumping the version
# here covers all of them at once and they cannot drift apart:
#
#   CubeOrangePlus              <- includes this file directly
#     CubeOrangePlus-ODID       <- includes CubeOrangePlus/hwdef.dat
#       CubeOrangePlus-ODID-DEV <- includes CubeOrangePlus-ODID/hwdef.dat
#
# Release checklist:
#   1. bump the version below
#   2. add an entry to doc/XPLORER-FIRMWARE-CHANGELOG.md
#   3. commit, then tag:  git tag -a xplorer-fw-v<X.Y.Z> -m "Xplorer firmware v<X.Y.Z>"
#   4. build with Tools/xplorer/build_release.sh -- it enforces 1-3, verifies the
#      string actually reached each board, and records a manifest with checksums
#
# Do NOT edit ArduCopter/version.h. THISFIRMWARE and FIRMWARE_VERSION must stay at
# the upstream 4.6.2 values: ground stations use them to select parameter metadata,
# and the autotest scripts parse that line. AP_CUSTOM_FIRMWARE_STRING is
# ArduPilot's supported vendor hook -- it replaces fw_string / fw_short_string
# (boot banner, AUTOPILOT_VERSION) while keeping the upstream string in
# fw_string_original, which the dataflash MSG line records alongside it as
# "Xplorer v<X.Y.Z> [ArduCopter V4.6.2]". See
# libraries/AP_Common/AP_FWVersionDefine.h and
# libraries/AP_Logger/LoggerMessageWriter.cpp.
#
# The DEV build (CubeOrangePlus-ODID-DEV) deliberately reports this SAME version
# string -- it is the same firmware baseline. Its DEV-ness is signalled by the
# MAV_SEVERITY_WARNING "DEV BUILD - PARAMS UNLOCKED" banner that follows this one,
# by the apj "summary" field, and by the release filename. Keeping one version
# number means there is only ever one place to bump.

define AP_CUSTOM_FIRMWARE_STRING "Xplorer v1.0.2"
