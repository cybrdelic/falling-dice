# Metamaterial Compression Engineering v2

A deterministic C++20 research implementation for designing, compressing, unloading, rendering, and manufacturing a graded cellular energy absorber.

This is the replacement for the original XPBD truss prototype. The mechanics are now based on six-degree-of-freedom nonlinear beam elements, explicit beam self-contact, rigid platen contact, incremental quasi-static equilibrium, loading/unloading, objective reaction-force extraction, and a watertight implicit-union manufacturing mesh.

## Delivered reference specimen

| Property | Value |
|---|---:|
| Topology | Graded BCC / accordion trigger lattice |
| Cells | 2 × 4 × 2 |
| Nominal envelope | 20 × 40 × 20 mm |
| Nonlinear nodes | 365 |
| Beam elements | 456 |
| Model mass | 6.414 g |
| Strut radius range | 0.82–1.22 mm |
| Maximum compression | 50% engineering strain |
| Equilibrium states | 71 (40 loading + 30 unloading + tare) |
| Material baseline | FFF TPU 95A, provisional isotropic parameters |
| Platen condition | Lubricated / frictionless normal contact |

## Reference numerical result

| Metric | Result |
|---|---:|
| Peak reaction force | 76.445 N |
| Peak engineering stress | 0.191 MPa |
| Loading work | 0.698 J |
| Recovered work | 0.482 J |
| Model-dissipated work | 0.215 J |
| Model SEA | 33.56 J/kg |
| Permanent set | 15.47% |
| Maximum contact penetration | 0.0177 mm |
| Converged equilibrium states | 71 / 71 |

The peak-force and loading-work predictions are step-refined to the low-single-digit-percent level between the 28/20 and 40/30 increment runs. The unloading dissipation and permanent-set outputs remain more step-sensitive and must be treated as preliminary until the constitutive law is calibrated against printed cyclic coupons.


## Included reference outputs

[Watch the reference compression cycle](assets/media/metamaterial_compression_master_540x720.mp4)

[![Compression sequence](assets/media/metamaterial_compression_contact_sheet.png)](assets/media/metamaterial_compression_master_540x720.mp4)

| Initial | 50% compressed | Unloaded |
|---|---|---|
| ![Initial specimen](assets/media/metamaterial_initial_hero.png) | ![Compressed specimen](assets/media/metamaterial_compressed_hero.png) | ![Unloaded specimen](assets/media/metamaterial_unloaded_hero.png) |

![Force-displacement hysteresis](assets/plots/force_displacement_hysteresis.png)

The printable reference specimen is provided at `assets/print/graded_bcc_energy_absorber_20x40x20mm.stl`. Its edge-incidence audit reports zero boundary edges and zero nonmanifold edges.

## What is implemented

- Nodal translation plus quaternion rotation: six DOF per node.
- Geometrically nonlinear corotational/Cosserat-style beam kinematics.
- Axial, shear, bending, and torsional strain energy.
- Incremental return mapping for axial and rotational plasticity.
- Deterministic stress-free geometric imperfections and a graded trigger band.
- Capsule–capsule, sphere–capsule, and sphere–sphere self-contact.
- Rigid top/bottom platen contact and direct reaction-force measurement.
- Incremental energy minimization with L-BFGS, Armijo line search, steepest-descent recovery, and adaptive displacement-step bisection.
- Load–unload work, recovery, dissipated-work, SEA, permanent-set, residual, and penetration diagnostics.
- Rigid-motion objectivity and finite-difference gradient verification.
- Deterministic `META3` state cache.
- Custom CPU path tracer using true capsules and spheres for the lattice geometry.
- Watertight implicit-union STL export with edge-incidence validation.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Windows with Visual Studio:

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j
```

## Validate the mechanics

```bash
./build/metamaterial_compression \
  --mode validate \
  --output validation \
  --cells-x 2 --cells-y 4 --cells-z 2
```

## Reproduce the reference simulation

```bash
./build/metamaterial_compression \
  --mode sim \
  --output output \
  --cells-x 2 --cells-y 4 --cells-z 2 \
  --load-steps 40 --unload-steps 30 \
  --max-strain 0.50 \
  --max-iterations 800 \
  --contact-stiffness 1000000 \
  --stl-quality 18
```

## Render the equilibrium path

```bash
./build/metamaterial_compression \
  --mode render \
  --cache output/compression.meta3 \
  --output render \
  --width 540 --height 720 \
  --spp 8 --depth 8 --subframes 2

ffmpeg -y -framerate 24 \
  -i render/frames/frame_%04d.ppm \
  -vf "hqdn3d=1.4:1.4:0:0" \
  -c:v libx264 -pix_fmt yuv420p -crf 16 -movflags +faststart \
  render/metamaterial-compression.mp4
```

The interpolated video is a visualization of a quasi-static equilibrium path. It is not a time-accurate dynamic-impact simulation.

## Source map

- `include/sim.hpp` — material, topology, solver, and diagnostic interfaces.
- `src/sim.cpp` — beam mechanics, contact, nonlinear solve, cache state, and STL generation.
- `src/render.cpp` — capsule/sphere/box path tracing, materials, lighting, reconstruction, and camera.
- `src/main.cpp` — CLI, cache I/O, validation, rendering, convergence, and metrics.
- `docs/NUMERICAL_METHOD.md` — formulation and assumptions.
- `docs/VALIDATION.md` — numerical verification and step-refinement audit.
- `docs/MATERIAL_CALIBRATION.md` — required experimental calibration workflow.
- `docs/PRINTING.md` — prototype manufacturing guidance.

## Engineering status

The code is an engineering-grade research foundation: it rejects failed equilibrium states, exposes numerical residuals, verifies gradients and objectivity, performs exact centreline contact for the reduced-order beam geometry, and exports a watertight printable solid.

It is **not a certified material prediction**. The default TPU values are provisional, printed FFF material is anisotropic and rate-dependent, fracture is not yet modeled, and the current platen model is lubricated. Use printed coupon data before making design, safety, or product-performance claims.

## License

MIT.
