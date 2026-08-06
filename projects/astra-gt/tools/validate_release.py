#!/usr/bin/env python3
import json
import sys
from pathlib import Path
from PIL import Image

root = Path(sys.argv[1])
validation = root / "validation"
validation.mkdir(parents=True, exist_ok=True)
manifest = json.loads((root / "manifest.json").read_text())
parts = manifest["parts"]
errors = []

if len(parts) < 40:
    errors.append(f"semantic part count too low: {len(parts)}")
invalid = [p["name"] for p in parts if not p["valid_brep"]]
if invalid:
    errors.append("invalid B-reps: " + ", ".join(invalid))

required_materials = {"paint", "glass", "rubber", "wheel_metal", "brake_disc", "caliper", "seal"}
materials = {p["material"] for p in parts}
missing_materials = sorted(required_materials - materials)
if missing_materials:
    errors.append("missing semantic materials: " + ", ".join(missing_materials))

for path in [root / "cad" / "ASTRA_GT_render_assembly.step", root / "cad" / "ASTRA_GT_full_package.step"]:
    if not path.exists() or path.stat().st_size < 10000:
        errors.append(f"missing or implausibly small STEP: {path}")

ply_files = sorted((root / "mesh" / "parts").glob("*.ply"))
if len(ply_files) < 40:
    errors.append(f"insufficient PLY parts: {len(ply_files)}")

render_info = {}
for name in ["hero_front.png", "hero_rear.png", "side.png", "front_low.png"]:
    path = root / "renders" / name
    if not path.exists():
        errors.append(f"missing render: {name}")
        continue
    with Image.open(path) as image:
        image.verify()
    with Image.open(path) as image:
        render_info[name] = {"width": image.width, "height": image.height, "mode": image.mode}
        if image.width < 1000 or image.height < 600:
            errors.append(f"render below release resolution: {name} {image.size}")

report = {
    "schema": "cybrdelic.astra-gt.release-validation.v1",
    "passed": not errors,
    "semantic_parts": len(parts),
    "invalid_breps": invalid,
    "materials": sorted(materials),
    "ply_parts": len(ply_files),
    "renders": render_info,
    "errors": errors,
}
(validation / "release_validation.json").write_text(json.dumps(report, indent=2) + "\n")
if errors:
    raise SystemExit("\n".join(errors))
print(json.dumps(report, indent=2))
