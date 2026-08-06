# ASTRA GT — exact C++ / OpenCASCADE vehicle program

ASTRA GT is a single original 2+2 grand-touring coupe developed as an exact-geometry replacement for the earlier polygonal multi-car generator. The authoritative model is a semantic OpenCASCADE `TopoDS_Shape` assembly authored in millimetres. Every render mesh is tessellated directly from those exact shapes; no independently authored render proxy or image-generation output is used.

## Implemented product definition

- 4780 mm overall length, 1920 mm nominal body width, 1390 mm nominal roof height.
- 2860 mm wheelbase; 1640/1660 mm front/rear track.
- 275/35R20 front and 295/35R20 rear tire envelopes.
- Exact B-spline through-section lower body and cabin solids.
- Boolean wheel houses, cooling aperture, brake ducts, side-glass apertures, windshield aperture and backlight aperture.
- Independent hood, decklid, doors, roof, panoramic insert, four side windows, windshield and backlight.
- Semantic seals, handles, mirrors, lamps, cooling inserts, underbody, rockers, tunnel and crash beams.
- Four independent wheel/tire/rim/rotor/caliper assemblies.
- Suspension links and dampers tied to axle hardpoints.
- Four seats, dashboard, console and steering wheel.
- Powertrain, energy-storage and luggage envelopes retained in the full-package CAD but excluded from beauty renders.

## Build

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DASTRA_WARNINGS_AS_ERRORS=ON
cmake --build build --parallel
./build/astra_gt generated/astra_gt
```

The build requires OpenCASCADE with Foundation, Modeling Data, Modeling Algorithms and Data Exchange modules.

## Outputs

- `generated/astra_gt/cad/ASTRA_GT_render_assembly.step`
- `generated/astra_gt/cad/ASTRA_GT_full_package.step`
- native `.brep` assemblies and per-part STEP/BREP files
- per-part binary PLY files derived from the exact B-reps
- PBRT-v4 scenes for front, rear, side and low-front views
- native C++ rasterized front/side/top diagnostic views
- a machine-readable geometry manifest

## Release boundary

The release gate checks compilation, exact B-rep validity, semantic part separation, deterministic tessellation, STEP read-back, package dimensions and image decoding. It does not claim crashworthiness, FMVSS certification, physical prototype correlation, production tooling release or OEM Class-A sign-off. Those claims remain blocked rather than inferred from render quality.
