#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cmake -S "$ROOT" -B "$ROOT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/build" -j
python3 "$ROOT/tools/validate_scene.py"
rm -rf "$ROOT/release/frames"
mkdir -p "$ROOT/release"
"$ROOT/build/thousand_dice" \
  --mode render \
  --cache "$ROOT/output/simulation.dice1k" \
  --output "$ROOT/release" \
  --width 640 --height 1138 --spp 8 --depth 6 --threads 0 \
  --frame-start 0 --frame-end 107 --no-motion-blur
ffmpeg -y -framerate 30 -start_number 0 \
  -i "$ROOT/release/frames/frame_%04d.ppm" -frames:v 108 \
  -c:v libx264 -preset slow -crf 14 -profile:v high -level 4.1 \
  -pix_fmt yuv420p -movflags +faststart -an \
  "$ROOT/release/thousand_dice_metrology_master_640x1138.mp4"
ffmpeg -y -framerate 30 -start_number 0 \
  -i "$ROOT/release/frames/frame_%04d.ppm" -frames:v 108 \
  -vf "scale=540:960:flags=lanczos+accurate_rnd+full_chroma_int,format=yuv420p" \
  -c:v libx264 -preset slow -crf 16 -profile:v baseline -level 3.1 \
  -pix_fmt yuv420p -movflags +faststart -an \
  "$ROOT/release/thousand_dice_metrology_mobile_540x960.mp4"
