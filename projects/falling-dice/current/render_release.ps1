$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
cmake -S $Root -B "$Root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$Root/build" --config Release -j
Remove-Item "$Root/artifacts/frames" -Recurse -Force -ErrorAction SilentlyContinue
New-Item "$Root/artifacts" -ItemType Directory -Force | Out-Null
& "$Root/build/thousand_dice" `
  --mode render `
  --cache "$Root/output/simulation.dice1k" `
  --output "$Root/artifacts" `
  --width 960 --height 1708 --spp 4 --depth 5 --threads 0 `
  --frame-start 0 --frame-end 115 --no-motion-blur
ffmpeg -y -framerate 30 -start_number 0 `
  -i "$Root/artifacts/frames/frame_%04d.ppm" -frames:v 116 `
  -c:v libx264 -preset slow -crf 14 -profile:v high -level 4.1 `
  -pix_fmt yuv420p -movflags +faststart `
  "$Root/artifacts/thousand_dice_practical_aa_master_960x1708.mp4"
ffmpeg -y -framerate 30 -start_number 0 `
  -i "$Root/artifacts/frames/frame_%04d.ppm" -frames:v 116 `
  -vf "scale=540:960:flags=lanczos+accurate_rnd+full_chroma_int,format=yuv420p" `
  -c:v libx264 -preset slow -crf 16 -profile:v baseline -level 3.1 `
  -pix_fmt yuv420p -movflags +faststart -an `
  "$Root/artifacts/thousand_dice_practical_aa_mobile_540x960.mp4"
