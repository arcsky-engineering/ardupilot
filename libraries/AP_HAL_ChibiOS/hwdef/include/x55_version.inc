# Arcsky X55 firmware identity.
#
# This is the SINGLE SOURCE OF TRUTH for the X55 firmware version string. Every
# board hwdef the X55 ships on includes this file, so bumping the version here
# covers all of them at once and they cannot drift apart.
#
# Release checklist:
#   1. bump the version below
#   2. add an entry to doc/X55-FIRMWARE-CHANGELOG.md
#   3. commit, then tag:  git tag -a x55-fw-v<X.Y.Z> -m "X55 firmware v<X.Y.Z>"
#   4. build with Tools/x55/build_release.sh -- it enforces 1-3 and records a manifest
#
# Do NOT edit ArduCopter/version.h. THISFIRMWARE and FIRMWARE_VERSION must stay at
# the upstream 4.4.4 values: ground stations use them to select parameter metadata,
# and the autotest scripts parse that line. AP_CUSTOM_FIRMWARE_STRING is
# ArduPilot's supported vendor hook -- it replaces fw_string / fw_short_string
# (boot banner, AUTOPILOT_VERSION) while keeping the upstream string in
# fw_string_original, which the dataflash MSG line records alongside it. See
# libraries/AP_Common/AP_FWVersionDefine.h and
# libraries/AP_Logger/LoggerMessageWriter.cpp.

define AP_CUSTOM_FIRMWARE_STRING "X55 v1.0.0"
