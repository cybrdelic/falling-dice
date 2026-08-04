#!/usr/bin/env python3
"""Validate renderer-only geometry against the deterministic dice cache."""
from __future__ import annotations

import ctypes
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "output" / "simulation.dice1k"


class Header(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_char * 8),
        ("version", ctypes.c_uint32),
        ("frames", ctypes.c_uint32),
        ("bodies", ctypes.c_uint32),
        ("diagnosticBytes", ctypes.c_uint32),
        ("snapshotBytes", ctypes.c_uint32),
        ("fps", ctypes.c_uint32),
    ]


class Snapshot(ctypes.Structure):
    _fields_ = [
        ("p", ctypes.c_double * 3),
        ("q", ctypes.c_double * 4),
        ("v", ctypes.c_double * 3),
        ("w", ctypes.c_double * 3),
        ("half", ctypes.c_double * 3),
        ("material", ctypes.c_int32),
        ("id", ctypes.c_int32),
        ("kind", ctypes.c_int32),
        ("moduleClass", ctypes.c_int32),
        ("layer", ctypes.c_int32),
        ("slot", ctypes.c_int32),
    ]


def rotation_extent(q: list[float], half: list[float]) -> list[float]:
    w, x, y, z = q
    r = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return [sum(abs(r[i][j]) * half[j] for j in range(3)) for i in range(3)]


def overlaps(a_lo, a_hi, b_lo, b_hi) -> bool:
    return all(a_lo[k] <= b_hi[k] and a_hi[k] >= b_lo[k] for k in range(3))


# Renderer-only axis-aligned props. Physics base and moving support are excluded:
# they are serialized collision bodies and rendered from the cache itself.
PROPS = {
    "front bench fascia": ((0, -0.065, 0.61), (0.78, 0.065, 0.035)),
    "left bench fascia": ((-0.745, -0.065, 0.16), (0.035, 0.065, 0.49)),
    "right bench fascia": ((0.745, -0.065, 0.16), (0.035, 0.065, 0.49)),
    "concrete wall": ((0, 0.43, -1.08), (1.50, 0.53, 0.035)),
    "wall baseboard": ((0, 0.050, -1.025), (0.84, 0.050, 0.025)),
    "left gantry column": ((-0.72, 0.26, -0.52), (0.028, 0.26, 0.028)),
    "right gantry column": ((0.72, 0.26, -0.52), (0.028, 0.26, 0.028)),
    "gantry crossbar": ((0, 0.515, -0.52), (0.748, 0.027, 0.030)),
    "rear backplate": ((0, 0.265, -0.535), (0.430, 0.215, 0.012)),
    "control box": ((0.82, 0.12, -0.575), (0.075, 0.110, 0.045)),
    "control indicator": ((0.82, 0.150, -0.528), (0.011, 0.011, 0.004)),
    "reference ruler": ((0.455, 0.003, 0.135), (0.115, 0.003, 0.018)),
    "calibration block": ((-0.43, 0.024, 0.105), (0.050, 0.024, 0.034)),
    "calibration cap": ((-0.43, 0.050, 0.105), (0.036, 0.003, 0.025)),
}
for tick in range(13):
    x = 0.345 + tick * (0.220 / 12.0)
    depth = 0.012 if tick % 6 == 0 else (0.008 if tick % 3 == 0 else 0.005)
    PROPS[f"ruler tick {tick}"] = ((x, 0.0068, 0.135), (0.00055, 0.00065, 0.5 * depth))


def main() -> int:
    with CACHE.open("rb") as fh:
        header = Header.from_buffer_copy(fh.read(ctypes.sizeof(Header)))
        if bytes(header.magic).rstrip(b"\0") != b"DICE1K1":
            raise RuntimeError("unexpected cache magic")

        swept_lo = [math.inf] * 3
        swept_hi = [-math.inf] * 3
        frames: list[tuple[list[tuple[list[float], list[float]]], Snapshot | None]] = []

        for _frame in range(header.frames):
            fh.read(header.diagnosticBytes)
            dice_bounds: list[tuple[list[float], list[float]]] = []
            leaf: Snapshot | None = None
            for _body in range(header.bodies):
                snap = Snapshot.from_buffer_copy(fh.read(header.snapshotBytes))
                if snap.kind == 0:
                    p = list(snap.p)
                    e = rotation_extent(list(snap.q), list(snap.half))
                    lo = [p[k] - e[k] for k in range(3)]
                    hi = [p[k] + e[k] for k in range(3)]
                    dice_bounds.append((lo, hi))
                    for k in range(3):
                        swept_lo[k] = min(swept_lo[k], lo[k])
                        swept_hi[k] = max(swept_hi[k], hi[k])
                elif snap.kind == 2 and snap.half[1] < 0.020:
                    leaf = snap
            frames.append((dice_bounds, leaf))

    expanded_lo = [swept_lo[k] - 0.015 for k in range(3)]
    expanded_hi = [swept_hi[k] + 0.015 for k in range(3)]

    prop_failures = []
    for name, (center, half) in PROPS.items():
        lo = [center[k] - half[k] for k in range(3)]
        hi = [center[k] + half[k] for k in range(3)]
        if overlaps(lo, hi, expanded_lo, expanded_hi):
            prop_failures.append(name)

    sleeve_overlaps = 0
    for dice_bounds, leaf in frames:
        if leaf is None:
            continue
        sleeve_top = leaf.p[1] - leaf.half[1] - 0.00035
        if sleeve_top <= 0.0015:
            continue
        centers = [
            (leaf.p[0] + px, 0.5 * sleeve_top, leaf.p[2] + pz)
            for px in (-0.050, 0.050)
            for pz in (-0.026, 0.026)
        ]
        parts = [(center, (0.0065, 0.5 * sleeve_top, 0.0065)) for center in centers]
        parts.append(((leaf.p[0], 0.5 * sleeve_top, leaf.p[2]),
                      (0.018, 0.5 * sleeve_top, 0.014)))
        for center, half in parts:
            lo = [center[k] - half[k] for k in range(3)]
            hi = [center[k] + half[k] for k in range(3)]
            if any(overlaps(lo, hi, die_lo, die_hi) for die_lo, die_hi in dice_bounds):
                sleeve_overlaps += 1

    print(f"cache frames: {header.frames}")
    print(f"bodies per frame: {header.bodies}")
    print("measured dice swept AABB:")
    print("  lo:", " ".join(f"{v:.9f}" for v in swept_lo))
    print("  hi:", " ".join(f"{v:.9f}" for v in swept_hi))
    print(f"renderer-only prop envelope failures: {len(prop_failures)}")
    if prop_failures:
        for name in prop_failures:
            print("  FAIL:", name)
    print(f"dynamic lift-sleeve/die AABB overlaps: {sleeve_overlaps}")

    if prop_failures or sleeve_overlaps:
        return 1
    print("scene collision-visibility validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
