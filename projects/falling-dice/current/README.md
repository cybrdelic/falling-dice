# Thousand Dice — Collision-Validated Metrology Rig

A C++20 rigid-body animation and custom CPU path tracer for **1,000 independently simulated 16 mm dice**.

This release rebuilds the visible scene around the deterministic no-creep physics cache. It is not a color pass over the earlier tabletop render. The support system, collision visibility, set geometry, cameras, dice construction, lighting, filtering, pacing, and validation were redesigned.

## Principal corrections

- The moving lift leaf and flush fixture base are rendered directly from their serialized collision bodies.
- The lift's four telescoping posts are generated beneath the physical leaf and are omitted automatically if their current-frame AABBs could touch a die.
- Every renderer-only solid is rejected if its world AABB enters the measured all-frame dice sweep, expanded by 15 mm.
- The analytic work surface has a cutout over the collidable fixture base, eliminating the former coplanar floor/base ambiguity.
- The final pile no longer contains the renderer-only actuator pillar shown intersecting the dice in the previous release.
- The set is now a deep concrete metrology bay with an embedded graphite stage, rear gantry, scale ruler, and control hardware placed outside the scatter envelope.
- Three deliberate locked-off tripod shots replace a single composition that left the settled pile at the bottom of a mostly empty frame.
- Dice use a tighter 0.72 mm molded edge radius, restrained ivory lot variation, recessed-pip shading, cavity rims, and a small rendering clearance to reduce visible collision-skin overlap.
- The renderer uses broad photographic area lights, stable tent-filtered sampling, robust firefly rejection, instance-guided spatial reconstruction, and no temporal denoising or optical-flow processing.
- The release is intentionally silent so the visual and physical pass can be judged independently.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Validate scene clearance

```bash
python3 tools/validate_scene.py
```

Expected result:

```text
renderer-only prop envelope failures: 0
dynamic lift-sleeve/die AABB overlaps: 0
scene collision-visibility validation: PASS
```

## Reproduce the production render

```bash
./build/thousand_dice \
  --mode render \
  --cache output/simulation.dice1k \
  --output release \
  --width 640 --height 1138 \
  --spp 8 --depth 6 --threads 0 \
  --frame-start 0 --frame-end 107 \
  --no-motion-blur
```

Then run `render_release.sh` to encode the 640 × 1138 master and 540 × 960 mobile delivery.
