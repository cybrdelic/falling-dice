# Falling Dice and Metamaterial Mechanics

This repository contains the maintained C++ implementation from the falling-dice development thread and the replacement for the original compression prototype: a nonlinear beam-lattice metamaterial solver with verification, load/unload analysis, self-contact, manufacturing-mesh export, and a custom CPU path tracer.

## 1,000-die rigid-body project

The browsable source is in [`projects/falling-dice/current`](projects/falling-dice/current). It includes the 1,000-body solver, renderer, collision/scene validator, release scripts, validation report, and scene audit.

## Nonlinear metamaterial compression baseline

The maintained replacement is in [`projects/metamaterial-compression/industry-fem`](projects/metamaterial-compression/industry-fem). It implements:

- six-degree-of-freedom geometrically nonlinear beam nodes
- axial, shear, bending, and torsional mechanics
- bilinear elastoplasticity, hardening, damage, and geometric fracture gaps
- curved-beam capsule self-contact and beam/platen contact
- L-BFGS quasi-static equilibrium with energy line search
- displacement-controlled loading and elastic unloading to unilateral release
- variational reaction-force extraction and energy metrics
- CTest verification and 20/30/40/60-increment convergence data
- a custom CPU path tracer
- a single-component watertight implicit-union STL exporter

The old XPBD truss prototype is retained under [`projects/metamaterial-compression/prototype-xpbd`](projects/metamaterial-compression/prototype-xpbd) for comparison, not as the maintained mechanics implementation.

## Repository layout

```text
projects/falling-dice/current/                       Maintained 1,000-die simulation and renderer
projects/metamaterial-compression/industry-fem/      Maintained nonlinear beam-lattice project
projects/metamaterial-compression/prototype-xpbd/    Superseded prototype retained for history
docs/                                                Audits and engineering notes
media/                                               README stills and compact MP4 previews
```

## Build everything

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --config Release -j
ctest --test-dir build --output-on-failure
```

## Status

The nonlinear solver is an engineering-grade, verification-first baseline, but its reference PETG material card is still provisional. It must be calibrated against printed coupons before its force and energy values are treated as guaranteed physical predictions. The code and limitations are visible rather than hidden behind the README.

## License

MIT.
