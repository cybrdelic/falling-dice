# Falling Dice + Nonlinear Metamaterial Compression

A browsable C++20 research repository containing the maintained **1,000-die rigid-body/rendering project** and the rebuilt **nonlinear beam-lattice compression solver**.

The source is committed directly under `projects/`; it is not hidden behind a README-only archive.

## Falling dice

[**Play the C++ falling-dice preview**](media/falling-dice-preview.mp4)

[![Falling-dice sequence](media/falling-dice-contact.jpg)](media/falling-dice-preview.mp4)

Source: [`projects/falling-dice/current`](projects/falling-dice/current)

Core implementation:

- [`src/sim.cpp`](projects/falling-dice/current/src/sim.cpp) — dense rigid-body/contact simulation
- [`src/render.cpp`](projects/falling-dice/current/src/render.cpp) — custom CPU rendering/path-tracing pipeline
- [`src/main.cpp`](projects/falling-dice/current/src/main.cpp) — cache, CLI, orchestration, and outputs
- [`tools/validate_scene.py`](projects/falling-dice/current/tools/validate_scene.py) — collider/renderer scene audit

## Nonlinear metamaterial compression

[**Play the load/unload compression preview**](media/metamaterial-compression-preview.mp4)

[![Compression sequence](media/metamaterial-compression-contact.jpg)](media/metamaterial-compression-preview.mp4)

![Force–displacement hysteresis](media/metamaterial-force-displacement.jpg)

Source: [`projects/metamaterial-compression/industry-grade`](projects/metamaterial-compression/industry-grade)

The current implementation replaces the earlier point-mass/XPBD prototype with:

- Six mechanical degrees of freedom per lattice node
- Geometrically nonlinear axial, shear, bending, and torsional beam energy
- Incremental elastoplastic state evolution and damage
- Beam self-contact and rigid platen contact
- Incremental quasi-static equilibrium with L-BFGS and safeguarded line search
- Loading and unloading with direct platen-reaction extraction
- Energy, specific-energy-absorption, efficiency, and permanent-set metrics
- Objectivity, finite-difference gradient, convergence, and penetration checks
- A custom CPU capsule/sphere path tracer
- Watertight implicit-union STL export

Key files:

- [`src/sim.cpp`](projects/metamaterial-compression/industry-grade/src/sim.cpp) — nonlinear mechanics, contact, plasticity, and solve loop
- [`src/render.cpp`](projects/metamaterial-compression/industry-grade/src/render.cpp) — C++ visualization renderer
- [`src/main.cpp`](projects/metamaterial-compression/industry-grade/src/main.cpp) — CLI, validation, cache, and manufacturing export
- [`docs/NUMERICAL_METHOD.md`](projects/metamaterial-compression/industry-grade/docs/NUMERICAL_METHOD.md)
- [`docs/VALIDATION.md`](projects/metamaterial-compression/industry-grade/docs/VALIDATION.md)
- [`docs/MATERIAL_CALIBRATION.md`](projects/metamaterial-compression/industry-grade/docs/MATERIAL_CALIBRATION.md)
- [`docs/PRINTING.md`](projects/metamaterial-compression/industry-grade/docs/PRINTING.md)

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --config Release -j
ctest --test-dir build --output-on-failure
```

Run numerical validation:

```bash
./build/projects/metamaterial-compression/metamaterial_compression \
  --mode validate --quick --output validation
```

Run a compression simulation:

```bash
./build/projects/metamaterial-compression/metamaterial_compression \
  --mode sim --output output/metamaterial
```

## Engineering boundary

This is an engineering research codebase, not a certified material model. The nonlinear architecture, contact pipeline, manufacturing exporter, and numerical tests are implemented; physical prediction still requires calibration to the exact material, printer, orientation, temperature, strain rate, friction, and failure behavior.

See [`docs/PROJECT_STATUS.md`](docs/PROJECT_STATUS.md).

## License

MIT.
