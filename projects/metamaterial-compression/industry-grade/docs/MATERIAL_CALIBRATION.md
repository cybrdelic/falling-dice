# Material and process calibration

The default material block is a provisional isotropic TPU 95A baseline. Do not treat it as a manufacturer-independent property set.

## Required specimens

Print all specimens using the same printer, nozzle, layer height, extrusion width, orientation, drying history, temperature, speed, and cooling settings intended for the lattice.

Minimum calibration set:

1. Axial tensile coupons in at least two raster/orientation directions.
2. Short compression cylinders or blocks for compressive modulus and barreling.
3. Three-point or four-point bending coupons for effective bending response.
4. Cyclic tension/compression at several strain amplitudes.
5. Stress-relaxation or hold tests at several strains.
6. At least three replicate lattice coupons for force–displacement repeatability.

## Parameters to fit

Current solver parameters:

- density;
- effective Young's modulus;
- effective Poisson ratio;
- axial yield stress;
- isotropic hardening modulus;
- bending hardening ratio;
- torsional hardening ratio;
- geometric imperfection amplitude;
- effective strut and joint radii.

The current model does not yet expose a Prony-series or hyperelastic parameter block. For TPU, adding and calibrating rate-dependent viscoelastic branches is the next constitutive upgrade.

## Calibration sequence

1. Measure printed strut diameters and joint radii by microscopy or calibrated macro imaging.
2. Fit density from printed coupon mass and reconstructed volume.
3. Fit small-strain modulus from the initial linear region.
4. Fit yield/hardening from monotonic loading.
5. Fit bending parameters from flexural coupons.
6. Validate against a lattice coupon not used during fitting.
7. Repeat at the intended compression rate and temperature.
8. Require repeatable force–displacement and unloading response before using SEA or permanent-set predictions.

## Acceptance criteria

Suggested internal release gates:

- initial stiffness error below 10%;
- peak/plateau force error below 10%;
- loading-work error below 10%;
- densification strain error below 5 percentage points;
- unloading-work and permanent-set error below 15%;
- mesh/step refinement changes below 5% for the reported primary metric;
- at least three physical repeats with reported scatter.
