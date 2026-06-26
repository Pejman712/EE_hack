#!/usr/bin/env bash
# Generate the WendyOS template-registry payload for `go2-foxglove`, matching the
# layout and conventions of github.com/wendylabsinc/templates.
#
# This repo root is the *deployable* app (real default values); this script emits
# the *template-source* form (Go text/template `{{.VAR}}` placeholders) that the
# registry serves to `wendy init --template go2-foxglove --language python`.
#
# Output (mirrors <registry>/python/go2-foxglove/):
#
#   dist/templates/python/go2-foxglove/
#     template.json        # variables (APP_ID, FOXGLOVE_PORT, GO2_IP, ...)
#     wendy.json           # appId -> {{.APP_ID}}
#     README.md  foxglove-layout.json
#     bridge/ camera/ ros2/ recorder/ sit_stand/ slam/ nav2/   # with placeholders
#
# To publish: copy dist/templates/python/go2-foxglove/ into the registry's
# python/ dir, and add/refresh the registry's top-level meta.json entry from
# meta-entry.json in this repo.
#
# Placeholders follow the registry's convention: substitute the Dockerfile ENV /
# EXPOSE declarations and cyclonedds.xml, and leave code/entrypoint fallback
# defaults literal (the ENV value overrides them at runtime). Rendering with the
# default answers reproduces the deployable repo.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/dist/templates/python/go2-foxglove"

APP_DIRS=(bridge camera ros2 recorder sit_stand slam nav2)
APP_FILES=(wendy.json README.md foxglove-layout.json template.json)

rm -rf "$OUT"
mkdir -p "$OUT"
for f in "${APP_FILES[@]}"; do cp "$ROOT/$f" "$OUT/$f"; done
for d in "${APP_DIRS[@]}"; do cp -r "$ROOT/$d" "$OUT/$d"; done

find "$OUT" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
find "$OUT" -name '*.pyc' -delete 2>/dev/null || true

# --- appId ------------------------------------------------------------------
sed -i 's/"appId": "go2-foxglove"/"appId": "{{.APP_ID}}"/' "$OUT/wendy.json"

# --- GO2_DDS_ADDRESS: the static cyclonedds.xml of the non-auto-binding svcs --
for x in bridge ros2 sit_stand; do
  sed -i 's/192\.168\.123\.18/{{.GO2_DDS_ADDRESS}}/g' "$OUT/$x/cyclonedds.xml"
done

# --- GO2_IP: the Dockerfile ENV of every service that declares it ------------
for d in camera recorder slam nav2; do
  sed -i 's/GO2_IP=192\.168\.123\.161/GO2_IP={{.GO2_IP}}/' "$OUT/$d/Dockerfile"
done

# --- service ports (ENV + EXPOSE) -------------------------------------------
sed -i 's/FOXGLOVE_PORT=8765/FOXGLOVE_PORT={{.FOXGLOVE_PORT}}/; s/^EXPOSE 8765$/EXPOSE {{.FOXGLOVE_PORT}}/' "$OUT/bridge/Dockerfile"
sed -i 's/PORT=7000/PORT={{.RECORDER_PORT}}/; s/^EXPOSE 7000$/EXPOSE {{.RECORDER_PORT}}/' "$OUT/recorder/Dockerfile"
sed -i 's/PORT=7100/PORT={{.NAV2_PORT}}/; s/^EXPOSE 7100$/EXPOSE {{.NAV2_PORT}}/' "$OUT/nav2/Dockerfile"

echo "Wrote template payload to: $OUT"
echo "Placeholders applied:"
grep -rn '{{\.' "$OUT" | sed "s#$OUT/#  #"
