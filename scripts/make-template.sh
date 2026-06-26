#!/usr/bin/env bash
# Generate the WendyOS template-registry payload for `go2-foxglove` from this
# repo. The repo root is the deployable app (real values); this script produces
# the *template-source* form (Go text/template `{{.VAR}}` placeholders) that a
# templates registry serves to `wendy init --template go2-foxglove`.
#
# Output layout (matches the registry's <template>/<language>/ convention):
#
#   dist/templates/go2-foxglove/
#     meta.json            # wizard: phases/questions (APP_ID, GO2_IP, ...)
#     template.json        # variables + defaults for the python language variant
#     python/              # the app source, with {{.VAR}} placeholders applied
#       wendy.json bridge/ camera/ ros2/ recorder/ sit_stand/ slam/ nav2/ ...
#
# Rendering this payload with the default answers reproduces the deployable repo
# byte-for-byte. Verify with:  scripts/make-template.sh && wendy json validate \
#   --help >/dev/null  (see README "Deploy" for the full check).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="$ROOT/dist/templates/go2-foxglove"
SRC="$OUT/python"

# Files copied verbatim into the payload (the app itself). README and the two
# manifests are handled separately.
APP_PATHS=(wendy.json foxglove-layout.json .gitignore README.md \
           bridge camera ros2 recorder sit_stand slam nav2)

rm -rf "$OUT"
mkdir -p "$SRC"
cp "$ROOT/meta.json" "$OUT/meta.json"
cp "$ROOT/template.json" "$OUT/template.json"

for p in "${APP_PATHS[@]}"; do
  [ -e "$ROOT/$p" ] || continue
  cp -r "$ROOT/$p" "$SRC/$p"
done

# Drop build cruft that may have been copied.
find "$SRC" -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
find "$SRC" -name '*.pyc' -delete 2>/dev/null || true

# --- Apply placeholders -----------------------------------------------------
# IPs are unambiguous literals; substitute them everywhere EXCEPT README.md,
# which intentionally documents the defaults as prose.
ip_files() { grep -rlF "$1" "$SRC" --include='*.py' --include='Dockerfile' \
               --include='*.sh' --include='*.xml' 2>/dev/null || true; }

while IFS= read -r f; do [ -n "$f" ] && sed -i 's/192\.168\.123\.161/{{.GO2_IP}}/g' "$f"; done < <(ip_files 192.168.123.161)
while IFS= read -r f; do [ -n "$f" ] && sed -i 's/192\.168\.123\.18/{{.GO2_DDS_ADDRESS}}/g' "$f"; done < <(ip_files 192.168.123.18)

# appId is the one wendy.json value every scaffold must set.
sed -i 's/"appId": "go2-foxglove"/"appId": "{{.APP_ID}}"/' "$SRC/wendy.json"

# FOXGLOVE_PORT: only in the bridge (README keeps the literal default as prose).
sed -i 's/FOXGLOVE_PORT=8765/FOXGLOVE_PORT={{.FOXGLOVE_PORT}}/; s/^EXPOSE 8765$/EXPOSE {{.FOXGLOVE_PORT}}/' "$SRC/bridge/Dockerfile"
sed -i 's/get("FOXGLOVE_PORT", "8765")/get("FOXGLOVE_PORT", "{{.FOXGLOVE_PORT}}")/' "$SRC/bridge/app.py"

echo "Wrote template payload to: $OUT"
echo "Placeholders applied:"
grep -rn '{{\.' "$SRC" | sed "s#$SRC/#  #"
