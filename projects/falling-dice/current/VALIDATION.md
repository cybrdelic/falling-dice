# Validation report

## Deterministic rigid-body cache

```text
cache schema:                    DICE1K1 / version 10
frames available:                143
frame rate:                      30 FPS
serialized bodies per frame:     1,002
independent dice:                1,000
physics fixture bodies:          2
nonmagnetic build:                yes
```

The release edit uses frames **0–107**, for **108 frames / 3.60 seconds**. The final rendered frame has zero recorded linear and angular velocity across the dice cache.

```text
frame 107 mean speed:             0 m/s
frame 107 maximum speed:          0 m/s
frame 107 mean angular speed:     0 rad/s
frame 107 maximum angular speed:  0 rad/s
```

## Collision-visible scene validation

`tools/validate_scene.py` parses every cached body state, measures the complete dice sweep, and checks all renderer-only solids against that envelope. It also checks every generated lift-post AABB against every die AABB in the corresponding frame.

```text
measured dice sweep lo:  -0.324452766  -0.003148698  -0.329793376 m
measured dice sweep hi:   0.281077185   0.390320000   0.259161172 m
renderer-only prop envelope failures: 0
dynamic lift-sleeve/die overlaps:      0
scene collision-visibility validation: PASS
```

The flush fixture base and moving leaf are not renderer-only props. They are rendered directly from the two serialized collision bodies in the cache.

## Known rigid-body limitation

```text
peak transient penetration:       7.329948 mm at frame 53
late buried penetration:          1.050527 mm at frame 107
```

This release removes renderer/collider clipping and slightly reduces visible collision-skin overlap by rendering the dice 0.10 mm smaller per side. It does not claim to eliminate the buried contact error in the deterministic physics cache.

## Render and video validation

The production sequence contains 108 contiguous C++ path-traced frames, numbered 0 through 107.

```text
native render:                    640 × 1138
samples per pixel:                8
maximum path depth:               6
recorded frame rate:              30 FPS
release duration:                 3.600000 s
motion blur:                      disabled
camera motion:                    three locked shots / hard cuts
soundtrack:                       none
```

Both release encodes completed a full FFmpeg decode pass with no reported errors.
