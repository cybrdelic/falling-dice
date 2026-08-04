# Engineering Release Notes

## Scope

This release replaces the original point-mass/XPBD truss demonstration with a deterministic nonlinear beam-and-contact research solver suitable for numerical method development, printed-coupon correlation, topology comparison, and manufacturing iteration.

## Reference model

- Graded BCC / accordion-trigger lattice
- 2 × 4 × 2 cells
- 20 × 40 × 20 mm nominal envelope
- 365 six-DOF nodes
- 456 nonlinear beam elements
- 6.414 g modeled mass
- 50% compression, then unloading
- 71 converged equilibrium states

## Reference result

- Peak reaction force: 76.445 N
- Peak engineering stress: 0.191 MPa
- Loading work: 0.6977 J
- Recovered work: 0.4824 J
- Model-dissipated work: 0.2153 J
- Model SEA: 33.56 J/kg
- Permanent set: 15.47%
- Maximum contact penetration: 0.0177 mm

## Numerical status

- Rigid-motion objectivity: PASS
- Finite-difference gradient verification: PASS
- Finite state checks: PASS
- All reference increments converged: PASS
- Energy accounting: PASS
- Watertight STL edge audit: PASS

The primary loading outputs are step-refined to low-single-digit percentage changes between the 28/20 and 40/30 load/unload schedules. Unloading dissipation and permanent set remain more step-sensitive and should be considered preliminary.

## Physical-prediction boundary

The solver architecture is engineering-grade, but the included TPU parameters are provisional. It is not a certified material model. Before using the results for product claims, calibrate against printed tension, compression, cyclic, and lattice coupons. FFF anisotropy, rate dependence, viscoelasticity, fracture, interlayer damage, and platen friction require measured parameters.

The MP4 visualizes interpolation between converged quasi-static equilibrium states. It is not a time-accurate dynamic impact simulation.
