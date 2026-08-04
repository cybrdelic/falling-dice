#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

OUT="${1:-output}"
mkdir -p "$OUT"

./build/metamaterial_compression \
  --mode sim --output "$OUT" \
  --cells-x 2 --cells-y 4 --cells-z 2 \
  --load-steps 40 --unload-steps 30 \
  --max-strain 0.50 --max-iterations 800 \
  --contact-stiffness 1000000 --stl-quality 18

./build/metamaterial_compression \
  --mode render --cache "$OUT/compression.meta3" --output "$OUT/render" \
  --width 540 --height 720 --spp 8 --depth 8 --subframes 2

ffmpeg -y -framerate 24 \
  -i "$OUT/render/frames/frame_%04d.ppm" \
  -vf "hqdn3d=1.4:1.4:0:0" \
  -c:v libx264 -pix_fmt yuv420p -crf 16 -movflags +faststart \
  "$OUT/metamaterial-compression.mp4"

python3 tools/plot_metrics.py "$OUT/compression_metrics.csv" "$OUT/plots"
