$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j

$Out = if ($args.Count -gt 0) { $args[0] } else { "output" }
New-Item -ItemType Directory -Force -Path $Out | Out-Null

$Exe = if (Test-Path "build/Release/metamaterial_compression.exe") {
    "build/Release/metamaterial_compression.exe"
} else {
    "build/metamaterial_compression.exe"
}

& $Exe --mode sim --output $Out `
    --cells-x 2 --cells-y 4 --cells-z 2 `
    --load-steps 40 --unload-steps 30 `
    --max-strain 0.50 --max-iterations 800 `
    --contact-stiffness 1000000 --stl-quality 18

& $Exe --mode render --cache "$Out/compression.meta3" --output "$Out/render" `
    --width 540 --height 720 --spp 8 --depth 8 --subframes 2

ffmpeg -y -framerate 24 -i "$Out/render/frames/frame_%04d.ppm" `
    -vf "hqdn3d=1.4:1.4:0:0" `
    -c:v libx264 -pix_fmt yuv420p -crf 16 -movflags +faststart `
    "$Out/metamaterial-compression.mp4"

python tools/plot_metrics.py "$Out/compression_metrics.csv" "$Out/plots"
