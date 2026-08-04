# Scene and renderer audit

## Defects found in the prior release

1. **Renderer/collider mismatch:** a decorative actuator pillar occupied the final pile but did not exist in the rigid-body world.
2. **Coplanar ambiguity:** the analytic tabletop and flush fixture top could compete for the same ray hit.
3. **Unexplained floating platform:** the moving support leaf had no readable load path to the work surface.
4. **Poor composition:** one camera had to fit the tall tower, leaving the final pile stranded beneath a large blank wall.
5. **Weak scale cues:** the background and tabletop did not make 16 mm dice read at a believable physical scale.
6. **Overrounded dice:** the former 1.62 mm render bevel made the dice resemble foam blocks.
7. **Patchy materials:** broad beige/white lot differences read as procedural texturing rather than manufacturing tolerance.
8. **Flat pips:** pips were primarily color masks and lacked a convincing cavity rim.
9. **Salt-and-pepper reconstruction:** low-sample glossy paths and aggressive filtering left grain on large surfaces.
10. **Confusing props:** bright vertical measurement bars could be read as additional clipping or stray geometry.
11. **Excessive dead time:** the deterministic cache is already static by roughly frame 100, so the release now ends at frame 107 rather than carrying a long repeated hold.

## Implemented corrections

- Collision-derived base and lift geometry.
- Dynamic, current-frame-validated telescoping support posts.
- Conservative all-frame clearance invariant for every render-only solid.
- Floor cutout matching the collidable base footprint.
- Rear-only set architecture outside the measured dice sweep.
- Three locked cameras with hard editorial cuts and no camera interpolation.
- Smaller molded edge radius and a 0.10 mm per-side rendering clearance.
- Narrow ivory lot distribution and lower-frequency material variation.
- Pip inlay, cavity-rim darkening, and recessed normal response.
- Broad area lights and stronger instance-aware spatial reconstruction.
- Removal of bright marker bars, temporal filtering, generated audio, and long terminal padding.

## Remaining physical limitation

The deterministic cache still records a maximum transient die-to-die penetration of approximately **7.33 mm** during the densest impact and approximately **1.05 mm** in buried late contacts. The renderer no longer introduces object clipping, and the 0.20 mm visual diameter clearance reduces visible collision-skin contact, but the buried contact error remains a solver limitation. Eliminating it requires a new higher-frequency TGS/CCD simulation rather than another scene or shading adjustment.
