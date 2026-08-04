# Numerical method

## 1. Configuration space

Each node carries a world-space position `x_i` and an orientation quaternion `R_i`. The nonlinear solve therefore has six generalized coordinates per node: three translations and three incremental rotation-vector components.

The state update is multiplicative in rotation:

```text
x_i = x_i,base + L_cell * u_i
R_i = exp(theta_i) R_i,base
```

This avoids treating finite rotations as additive Euler angles.

## 2. Beam formulation

Every printable strut is subdivided into multiple beam elements. Each element stores its stress-free length, material director, circular-section area, second moment, polar moment, layer, family, and plastic internal state.

The incremental potential contains:

- axial extension based on current length;
- director-to-centreline shear energy;
- relative-rotation bending energy;
- torsional energy about the current beam axis;
- isotropic hardening terms;
- incremental plastic dissipation;
- gravity;
- contact penalties;
- a weak horizontal centering potential to remove uninteresting rigid drift.

Axial plasticity uses a one-dimensional return map. Bending and torsion use vector/scalar rotational return maps. Plastic state is committed only after a converged equilibrium increment.

## 3. Geometry and imperfection

The nominal topology is BCC. Lower layers receive reduced radii and larger stress-free pre-curvature. Cell-centre offsets alternate through the height, creating a printable accordion trigger rather than applying nonphysical lateral forces during compression.

Each physical strut is subdivided into three elements, so buckling and hinge localization are represented by actual intermediate nodes rather than by a straight line connecting only the original lattice vertices.

## 4. Contact

The reduced-order mechanics uses:

- joint spheres;
- strut capsules;
- capsule–capsule closest points;
- sphere–capsule closest points;
- sphere–sphere closest points;
- a spatial hash for broad-phase candidate generation;
- rigid horizontal platen planes.

Connected and immediately adjacent shapes are excluded from self-contact. Contact is frictionless in the delivered reference case, representing lubricated compression platens. This assumption is explicit because platen friction materially changes barreling and collapse mode.

Reaction force is the sum of top-platen normal penalty forces after taring the initial gravity/contact equilibrium. It is not inferred from internal beam-force magnitudes.

## 5. Nonlinear equilibrium

Each prescribed platen position is solved by minimizing the incremental potential. The main optimizer is limited-memory BFGS with Armijo backtracking. If the quasi-Newton metric becomes invalid after a contact active-set change, the solver retries from a steepest-descent direction.

A state is accepted only when:

- the scaled generalized-force residual satisfies the equilibrium criterion;
- maximum contact penetration is below the configured tolerance;
- total incremental potential is finite.

If a target displacement cannot be solved, the displacement interval is bisected adaptively. A failed refined state is rejected with an exception; it is never silently committed.

## 6. Loading and unloading

The delivered reference path contains:

1. gravity/contact equilibrium and load-cell tare;
2. displacement-controlled loading to 50% engineering strain;
3. displacement-controlled unloading to the original platen position.

Work is integrated by the trapezoidal rule along the reaction-force/displacement curve. Reported dissipated work is loading work minus recovered unloading work. Because the default material law is provisional and the unloading branch remains step-sensitive, this value is a model output rather than a validated physical property.

## 7. Manufacturing geometry

The STL generator evaluates the union signed-distance field of all undeformed strut capsules and joint spheres, polygonizes it on a Cartesian grid, repairs tiny ambiguous marching-cubes boundary loops deterministically, and performs an edge-incidence audit.

The reference mesh has zero boundary edges and zero nonmanifold edges. It is a single implicit union, not thousands of overlapping cylinder/sphere shells delegated to the slicer.

## 8. Rendering

The renderer uses:

- capsule intersections for struts;
- sphere intersections for joints;
- rounded boxes for platens and machine components;
- a BVH over moving primitives;
- GGX polymer/coated-metal materials;
- area-light next-event estimation and multiple-importance sampling;
- ACES tone mapping;
- a robust per-pixel sample estimator;
- normal/depth/albedo-guided spatial reconstruction.

No temporal denoising or optical-flow interpolation is used. Video subframes are deterministic interpolation between converged equilibrium configurations.
