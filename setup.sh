#!/usr/bin/env bash
# Idempotent: clone pinned repos and fetch pinned assets into the host checkout.
set -euo pipefail
cd "$(dirname "$0")"

clone() { [ -d "$1" ] || git clone "$2" "$1"; git -C "$1" checkout "$3"; }
clone pal_mjlab https://github.com/pal-robotics/pal_mjlab.git 52f5d6ed5d
clone LIBERO https://github.com/Lifelong-Robot-Learning/LIBERO.git 8f1084e313

fetch() { [ -s "$2" ] || { mkdir -p "$(dirname "$2")"; curl -fsSL "$1" -o "$2.part" && mv "$2.part" "$2"; }; }

GSO=6ff8d275ce
for pair in Cole_Hardware_Mug_Classic_Blue:cup Threshold_Bead_Cereal_Bowl_White:bowl; do
  name=${pair%%:*}; dst=assets/${pair##*:}
  files=$(curl -fsSL "https://api.github.com/repos/kevinzakka/mujoco_scanned_objects/contents/models/$name?ref=$GSO" \
          | python3 -c "import json,sys; [print(e['name']) for e in json.load(sys.stdin) if e['type']=='file']")
  [ -n "$files" ] || { echo "no files listed for $name" >&2; exit 1; }
  for f in $files; do
    fetch "https://raw.githubusercontent.com/kevinzakka/mujoco_scanned_objects/$GSO/models/$name/$f" "$dst/$f"
  done
done

RS=https://raw.githubusercontent.com/ARISE-Initiative/robosuite/v1.5.2/robosuite/models/assets
fetch $RS/objects/bottle.xml assets/bottle/objects/bottle.xml
fetch $RS/objects/meshes/bottle.stl assets/bottle/objects/meshes/bottle.stl
fetch $RS/textures/glass.png assets/bottle/textures/glass.png
