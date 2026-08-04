#!/usr/bin/env python3
"""Generate engineering plots from compression_metrics.csv.

Usage:
    python tools/plot_metrics.py output/compression_metrics.csv output/plots
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def load_rows(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open(newline="") as handle:
        for raw in csv.DictReader(handle):
            row: dict[str, float] = {}
            for key, value in raw.items():
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    continue
            rows.append(row)
    if not rows:
        raise RuntimeError(f"no metric rows in {path}")
    return rows


def save_force_displacement(rows: list[dict[str, float]], destination: Path) -> None:
    loading = [r for r in rows if int(r["loading"]) == 1]
    unloading = [r for r in rows if int(r["loading"]) == 0]
    plt.figure(figsize=(8, 6), dpi=180)
    plt.plot([1000 * r["displacement_m"] for r in loading],
             [r["reaction_force_n"] for r in loading], marker="o", markersize=3,
             linewidth=1.6, label="Loading")
    plt.plot([1000 * r["displacement_m"] for r in unloading],
             [r["reaction_force_n"] for r in unloading], marker="o", markersize=3,
             linewidth=1.6, label="Unloading")
    plt.xlabel("Platen displacement (mm)")
    plt.ylabel("Reaction force (N)")
    plt.title("Force–displacement response")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(destination)
    plt.close()


def save_stress_strain(rows: list[dict[str, float]], destination: Path) -> None:
    loading = [r for r in rows if int(r["loading"]) == 1]
    unloading = [r for r in rows if int(r["loading"]) == 0]
    plt.figure(figsize=(8, 6), dpi=180)
    plt.plot([100 * r["engineering_strain"] for r in loading],
             [r["engineering_stress_pa"] / 1e6 for r in loading], marker="o",
             markersize=3, linewidth=1.6, label="Loading")
    plt.plot([100 * r["engineering_strain"] for r in unloading],
             [r["engineering_stress_pa"] / 1e6 for r in unloading], marker="o",
             markersize=3, linewidth=1.6, label="Unloading")
    plt.xlabel("Engineering strain (%)")
    plt.ylabel("Engineering stress (MPa)")
    plt.title("Stress–strain response")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(destination)
    plt.close()


def save_energy(rows: list[dict[str, float]], destination: Path) -> None:
    strain = [100 * r["engineering_strain"] for r in rows]
    plt.figure(figsize=(8, 6), dpi=180)
    plt.plot(strain, [r["loading_work_j"] for r in rows], linewidth=1.6,
             label="Loading work")
    plt.plot(strain, [r["recovered_work_j"] for r in rows], linewidth=1.6,
             label="Recovered work")
    plt.plot(strain, [r["dissipated_work_j"] for r in rows], linewidth=1.6,
             label="Dissipated work")
    plt.plot(strain, [r["elastic_energy_j"] for r in rows], linewidth=1.2,
             label="Current elastic energy")
    plt.xlabel("Engineering strain (%)")
    plt.ylabel("Energy (J)")
    plt.title("Incremental energy accounting")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(destination)
    plt.close()


def save_solver(rows: list[dict[str, float]], destination: Path) -> None:
    frames = [int(r["frame"]) for r in rows]
    residual = [max(r["residual_rms"], 1e-12) for r in rows]
    penetration = [max(1000 * r["max_penetration_m"], 1e-9) for r in rows]
    plt.figure(figsize=(8, 6), dpi=180)
    plt.semilogy(frames, residual, marker="o", markersize=2.5, linewidth=1.3,
                 label="RMS equilibrium residual")
    plt.semilogy(frames, penetration, marker="o", markersize=2.5, linewidth=1.3,
                 label="Maximum penetration (mm)")
    plt.xlabel("Equilibrium state")
    plt.ylabel("Log-scale diagnostic magnitude")
    plt.title("Solver diagnostics")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(destination)
    plt.close()


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    source = Path(sys.argv[1])
    destination = Path(sys.argv[2])
    destination.mkdir(parents=True, exist_ok=True)
    rows = load_rows(source)
    save_force_displacement(rows, destination / "force_displacement.png")
    save_stress_strain(rows, destination / "stress_strain.png")
    save_energy(rows, destination / "energy_accounting.png")
    save_solver(rows, destination / "solver_diagnostics.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
