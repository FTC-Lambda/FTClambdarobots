#!/usr/bin/env sh
# Re-download the official BIOBUZZ (FTC 2026-2027) documents into this folder.
# FIRST publishes Team Updates every Thursday, so run this before relying on the manual.
# Usage: cd docs/game/reference && ./fetch.sh
set -eu
cd "$(dirname "$0")"

B=https://ftc-resources.firstinspires.org/ftc

dl() {
	curl -fsSL -o "$1" -w "%{http_code} %{size_download}B  $1\n" "$2"
}

dl BIOBUZZ-Competition-Manual.pdf        "$B/game/manual"
dl Team-Update-Latest.pdf                "$B/game/tu-latest"
dl Team-Update-Combined.pdf              "$B/game/tu-combined"
dl Team-QA-Instructions.pdf              "$B/game/q-a-help"
dl AprilTags-Printable-US-Letter.pdf     "$B/archive/2027/field/apriltag-us"
dl AprilTags-Printable-A4.pdf            "$B/archive/2027/field/apriltag-a4"
dl AprilTag-Production-Art.pdf           "$B/archive/2027/field/apriltag-art"
dl Initial-Field-Element-Assembly-Guide.pdf "$B/archive/2027/field/initialfieldguide"
dl Event-Field-Setup-Guide.pdf           "$B/archive/2027/field/eventfieldguide"
dl Field-Acceptance-Checklist.pdf        "$B/archive/2027/field/field-check"
dl Field-Mitigation-Guide.pdf            "$B/archive/2027/field/field-mitigation-guide"
dl Field-Reset-Guide.pdf                 "$B/archive/2027/event/field-reset"
dl Field-CAD-STEP.zip                    "$B/archive/2027/field/field-cad-step"
dl BIOBUZZ-Gameplay-Card.pdf             "https://www.firstinspires.org/hubfs/web/program/ftc/biobuzz-gamepostcard.pdf"

# The manual's cover page states its version, e.g. "Version: TU08".
if command -v pdftotext >/dev/null 2>&1; then
	pdftotext -l 1 BIOBUZZ-Competition-Manual.pdf - | grep -m1 "Version:" || true
fi
