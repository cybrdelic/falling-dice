# Falling Dice: C++ rigid-body simulation, CPU rendering, and metamaterial compression

This repository preserves the complete source produced during the falling-dice / magnetic-tower development thread and starts the next project: a **graded metamaterial compression test**.

## Current media

### 1,000 independently simulated dice

[**Watch the falling-dice preview (MP4)**](media/falling-dice-preview.mp4)

[![Falling-dice contact sheet](media/falling-dice-contact.jpg)](media/falling-dice-preview.mp4)

![Final dice pile](media/falling-dice-final.jpg)

### Metamaterial compression — Phase 1

[**Watch the metamaterial compression preview (MP4)**](media/metamaterial-preview.mp4)

[![Metamaterial compression sequence](media/metamaterial-contact.jpg)](media/metamaterial-preview.mp4)

![Buckling lattice](media/metamaterial-buckling.jpg)

![Force–displacement curve](media/force-displacement.jpg)

## Source

[**Download the complete source bundle**](source/falling-dice-complete-source.zip)

The bundle contains:

- The current 1,000-die rigid-body simulator and CPU path tracer.
- Every source snapshot generated during the conversation, grouped into dice, magnetic, and million-scale archives.
- The new metamaterial compression project.
- CMake build files, Windows/Linux scripts, validators, technical notes, and limitation reports.

## Metamaterial compression project

The started Phase-1 system includes:

- Parametric graded body-centred-cubic lattice generation.
- Nodal mass derived from polymer density and strut volume.
- XPBD axial and bending constraints with deterministic buckling imperfections.
- Progressive plastic shortening and damage.
- Node-level self-contact and moving-platen contact.
- Force–displacement, absorbed-energy, and damage output.
- Deterministic simulation cache, CPU capsule path tracer, and prototype STL export.

This is a reduced-order beam-lattice prototype, not yet a calibrated nonlinear finite-element model. Phase 2 will add geometrically exact beam/FEM elements, capsule self-contact, watertight implicit meshing, material calibration against printed coupons, and inverse design.

## Build

```bash
unzip source/falling-dice-complete-source.zip -d source-tree
cd source-tree
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
```

## Documentation

- [Engineering lessons](docs/ENGINEERING_LESSONS.md)
- [Complete source-history index](docs/SOURCE_HISTORY.md)
- [Falling-dice project notes](projects/falling-dice/README.md)
- [Metamaterial compression notes](projects/metamaterial-compression/README.md)

## License

MIT.