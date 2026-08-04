# Falling Dice — C++ simulation, rendering, and metamaterial research

This repository contains the maintained **1,000-die rigid-body simulation and renderer**, the rebuilt **nonlinear metamaterial compression solver**, and a compressed archive of the earlier source revisions from the development thread.

The code is committed directly under `projects/`; it is not README-only.

## Falling dice

[**Watch the C++ falling-dice preview**](media/falling-dice-preview.mp4)

[![Falling-dice sequence](media/falling-dice-contact.jpg)](media/falling-dice-preview.mp4)

![Final dice pile](media/falling-dice-final.jpg)

Source: [`projects/falling-dice/current`](projects/falling-dice/current)

- [`src/sim.cpp`](projects/falling-dice/current/src/sim.cpp) — dense rigid-body/contact simulation
- [`src/render.cpp`](projects/falling-dice/current/src/render.cpp) — custom CPU rendering/path-tracing pipeline
- [`src/main.cpp`](projects/falling-dice/current/src/main.cpp) — cache, CLI, orchestration, and outputs
- [`tools/validate_scene.py`](projects/falling-dice/current/tools/validate_scene.py) — collider/renderer scene audit

## Nonlinear metamaterial compression

[**Watch the load/unload compression preview**](media/metamaterial-compression-v2.mp4)

[![Compression sequence](media/metamaterial-compression-v2-contact.jpg)](media/metamaterial-compression-v2.mp4)

![Buckled graded lattice](media/metamaterial-compression-v2-buckling.jpg)

![Force–displacement hysteresis](media/metamaterial-compression-v2-force.jpg)

![Step-refinement audit](media/metamaterial-compression-v2-convergence.jpg)

Source: [`projects/metamaterial-compression`](projects/metamaterial-compression)

The current implementation replaces the earlier point-mass/XPBD prototype with:

- Six mechanical degrees of freedom per lattice node
- Geometrically nonlinear axial, shear, bending, and torsional beam energy
- Incremental elastoplastic state evolution and damage
- Beam self-contact and rigid platen contact
- Incremental quasi-static equilibrium with L-BFGS and safeguarded line search
- Loading and unloading with direct platen-reaction extraction
- Energy, SEA, efficiency, and permanent-set metrics
- Objectivity, finite-difference gradient, convergence, and penetration checks
- A custom CPU capsule/sphere path tracer
- Watertight implicit-union STL export

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --config Release -j
ctest --test-dir build --output-on-failure
```

## Historical source

[Download the source-history bundle](source/falling-dice-complete-source.zip).

## Engineering boundary

This is an engineering research codebase, not a certified material model. The nonlinear architecture, contact pipeline, manufacturing exporter, and numerical tests are implemented; physical prediction still requires calibration to the exact material, printer, orientation, temperature, strain rate, friction, and failure behavior.

## License

MIT.
