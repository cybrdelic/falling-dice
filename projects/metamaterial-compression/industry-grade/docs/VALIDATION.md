# Numerical validation report

## Reference model

```text
Topology:                  graded BCC / accordion trigger
Cells:                     2 × 4 × 2
Nominal envelope:          20 × 40 × 20 mm
Nodes:                     365
Beam elements:             456
Model mass:                0.006413882 kg
Reference area:            0.0004 m²
Maximum engineering strain: 0.50
Equilibrium states:        71
```

## Verification tests

| Test | Result |
|---|---:|
| Rigid-motion objectivity error | 4.47e-15 — PASS |
| Finite-difference gradient relative error | 3.63e-3 — PASS |
| Finite state records | PASS |
| Converged equilibrium fraction | 1.000 |
| Energy-accounting sign check | PASS |
| Maximum contact penetration | 0.0177 mm — PASS |
| Watertight STL | PASS |
| STL boundary edges | 0 |
| STL nonmanifold edges | 0 |

## Reference outputs

| Metric | Value |
|---|---:|
| Peak force | 76.445 N |
| Peak engineering stress | 0.1911 MPa |
| Loading work | 0.6977 J |
| Recovered work | 0.4824 J |
| Model-dissipated work | 0.2153 J |
| Specific model-dissipated work | 33.56 J/kg |
| Permanent set | 15.47% |

## Step refinement

| Loading / unloading increments | Peak force (N) | Loading work (J) | Dissipated work (J) | Permanent set |
|---:|---:|---:|---:|---:|
| 20 / 14 | 79.482 | 0.667 | 0.137 | 10.71% |
| 28 / 20 | 77.599 | 0.680 | 0.168 | 10.74% |
| 40 / 30 | 76.445 | 0.698 | 0.215 | 15.47% |

From 28/20 to 40/30, peak force changes by approximately 1.5% and loading work by approximately 2.6%. Those primary loading metrics show useful numerical convergence.

The unloading dissipation and permanent set are not yet converged to the same standard. They depend on post-buckling branch selection and the provisional rate-independent plastic law. They remain **WARN / calibration required** rather than production-qualified material values.

## Validation boundary

The test suite establishes numerical consistency of the implemented equations. It does not establish correspondence to a specific printed TPU coupon. Experimental validation requires measured geometry, print process, material conditioning, rate, temperature, cyclic response, platen friction, and repeatability data.
