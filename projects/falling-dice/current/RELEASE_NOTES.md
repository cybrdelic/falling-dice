# Release notes — collision-validated metrology scene

This release is a structural scene and renderer overhaul around the existing deterministic 1,000-die no-creep cache.

## Visible clipping fixes

- Removed the non-collidable actuator pillar that occupied the final pile.
- Rendered the fixture base and moving lift leaf directly from serialized collision bodies.
- Added a tabletop cutout over the fixture footprint to remove coplanar floor/base ambiguity.
- Added telescoping supports only where current-frame die AABBs prove they cannot intersect.
- Added an all-frame clearance invariant for every renderer-only solid.

## Additional improvements found during the audit

- Replaced the unexplained floating support with a readable load path.
- Moved architecture, ruler, control box, and gantry outside the complete dice sweep.
- Removed bright vertical props that resembled clipping artifacts.
- Replaced one compromised camera with three locked compositions and hard cuts.
- Shortened the edit to the useful 108-frame range; no long terminal padding remains.
- Reduced the dice edge radius from the prior foam-like treatment.
- Narrowed lot-to-lot color variation and rebuilt pips as recessed inlays with cavity rims.
- Added a small render-only collision-skin clearance to reduce visible buried overlap.
- Rebuilt the floor, wall, fixture, and lighting as a restrained metrology set with coherent scale cues.
- Strengthened instance-aware spatial reconstruction while retaining silhouettes, pips, and contact gaps.
- Kept the release silent; no synthetic foley obscures evaluation of the motion.

## Honest remaining limitation

The source cache still contains a 7.33 mm peak transient penetration during its densest impact and approximately 1.05 mm in one buried late contact. The release eliminates renderer-only prop clipping; it does not claim that the inherited rigid-body contact error has been eliminated.
