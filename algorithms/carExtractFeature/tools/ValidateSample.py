"""Reproducible real-sample parameter sweep and diagnostic figures (numpy/matplotlib).

Usage: python ValidateSample.py input.pcd build/bin/Release/CarLapEdgeInspect.exe output-dir
Put the host's install/bin on PATH before running, so the inspector can find PCL DLLs.
No ground truth is inferred from the detector itself; this reports repeatability, not accuracy.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

local_packages = Path(__file__).resolve().parents[1] / "build" / "python-packages"
if local_packages.exists():
    sys.path.insert(0, str(local_packages))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_pcd(path):
    with open(path, "rb") as stream:
        header = {}
        while True:
            line = stream.readline().decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            header[key] = values
            if key == "DATA":
                break
        names = header["FIELDS"]
        dtype = []
        for i, (name, size, kind, count) in enumerate(zip(names, header["SIZE"], header["TYPE"], header.get("COUNT", ["1"] * len(names)))):
            code = {"F": "f", "I": "i", "U": "u"}[kind]
            dtype.append((name if name != "_" else "padding_" + str(i), "<" + code + size, (int(count),)) if int(count) > 1 else
                         (name if name != "_" else "padding_" + str(i), "<" + code + size))
        if header["DATA"] != ["binary"]:
            raise ValueError("Validation reader expects binary, uncompressed PCD")
        cloud = np.frombuffer(stream.read(), dtype=np.dtype(dtype))
        return np.column_stack([cloud[name] for name in ("x", "y", "z")]).astype(float)


def read_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True, ndmin=1)


def compare(base, variant, spacing):
    a = {int(p["section_id"]): p for p in base}
    b = {int(p["section_id"]): p for p in variant}
    shared = sorted(a.keys() & b.keys())
    distances = [np.linalg.norm([a[k][axis] - b[k][axis] for axis in ("x", "y", "z")]) for k in shared]
    return {"common_sections": len(shared), "lost_sections": len(a.keys() - b.keys()), "gained_sections": len(b.keys() - a.keys()),
            "p95_displacement": float(np.quantile(distances, .95)) if distances else None,
            "max_displacement": float(max(distances)) if distances else None,
            "p95_within_one_spacing": bool(distances and np.quantile(distances, .95) <= spacing)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("inspector", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--plots-only", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    configurations = [("default", 1., 1., 1.)]
    for field in range(3):
        for factor in (.8, 1.2, 1.5):
            values = [1., 1., 1.]
            values[field] = factor
            configurations.append((f"{['neighborhood','smoothing','evidence'][field]}_{factor}", *values))
    if not args.plots_only:
        for name, *values in configurations:
            subprocess.run([str(args.inspector.resolve()), str(args.input.resolve()), str((args.output / name).resolve()), *map(str, values)], check=True, capture_output=True, text=True)
    base_dir = args.output / "default"
    report = json.loads((base_dir / "report.json").read_text())
    measured = read_csv(base_dir / "measured.csv")
    trajectory = read_csv(base_dir / "trajectory.csv")
    summary = {"ground_truth_available": False, "baseline": report, "variants": []}
    for name, *values in configurations[1:]:
        folder = args.output / name
        if not folder.exists():
            continue
        variant_report = json.loads((folder / "report.json").read_text())
        summary["variants"].append({"name": name, "parameters": values,
            "measured": compare(measured, read_csv(folder / "measured.csv"), report["point_spacing"]),
            "trajectory": compare(trajectory, read_csv(folder / "trajectory.csv"), report["point_spacing"]),
            "measured_points": variant_report["measured_points"], "trajectory_points": variant_report["trajectory_points"],
            "trajectory_segments": variant_report["trajectory_segments"]})
    (args.output / "sensitivity.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    cloud = read_pcd(args.input)
    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .2})
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), constrained_layout=True)
    for ax, axis, label in [(axes[0], 0, "X"), (axes[1], 2, "Z")]:
        ax.scatter(cloud[::3, 1], cloud[::3, axis], s=.3, color="#7e9aab", alpha=.45, rasterized=True)
        ax.scatter(measured["y"], measured[label.lower()], s=3, color="#dd9200", label="Measured edge")
        for i, segment in enumerate(np.unique(trajectory["segment_id"])):
            p = trajectory[trajectory["segment_id"] == segment]
            ax.plot(p["y"], p[label.lower()], color="#d62728", lw=1, label="Fitted trajectory" if i == 0 else None)
        ax.set(xlabel="Y (input units)", ylabel=label + " (input units)")
        ax.legend(loc="best")
    valid = np.zeros(report["sections"])
    valid[trajectory["section_id"].astype(int)] = 1
    valid[measured["section_id"].astype(int)] = 2
    axes[2].scatter(np.arange(len(valid)), valid, s=2, c=valid, cmap="viridis")
    axes[2].set(yticks=[0, 1, 2], yticklabels=["Missing", "Interpolated", "Measured"], xlabel="Recovered section ID", ylabel="Coverage")
    fig.suptitle("Upper lap edge - real sample diagnostics (no ground-truth accuracy claim)")
    fig.savefig(args.output / "overview.png", dpi=160)
    plt.close(fig)
    normal = np.array(report["scan_normal"])
    station = cloud @ normal
    boundaries = np.r_[0, np.flatnonzero(np.diff(station) > .1 * report["section_spacing"]) + 1, len(cloud)]
    selected_sections = [0, 100, 350, 500, 850, 1000, 1500, 1798, 2200, 2500, 3000, 3300]
    fig, axes = plt.subplots(3, 4, figsize=(17, 12), constrained_layout=True)
    for ax, section in zip(axes.flat, selected_sections):
        p = cloud[boundaries[section]:boundaries[section + 1]]
        ax.scatter(p[:, 0], p[:, 2], s=20, c="#087e8b", label="Raw section")
        q = measured[measured["section_id"] == section]
        t = trajectory[trajectory["section_id"] == section]
        ax.scatter(q["x"], q["z"], s=100, marker="o", facecolors="none", edgecolors="#d48c00", lw=2, label="Measured")
        ax.scatter(t["x"], t["z"], s=50, marker="x", c="#df2020", lw=2, label="Trajectory")
        ax.set(title=f"Section {section}: measured={len(q)}, trajectory={len(t)}", xlabel="X", ylabel="Z", aspect="equal")
    axes.flat[0].legend(fontsize=8)
    fig.suptitle("Cross-section inspection: check whether each marker lies at the upper-sheet edge")
    fig.savefig(args.output / "sections.png", dpi=160)
    plt.close(fig)
    if len(boundaries) > 1001:
        focus = 1000
        profile = cloud[boundaries[focus]:boundaries[focus + 1]]
        point = measured[measured["section_id"] == focus]
        if len(point):
            x = float(point["x"][0])
            nearby = profile[(profile[:, 0] > x - 5) & (profile[:, 0] < x + 5)]
            fig, ax = plt.subplots(figsize=(8, 5), constrained_layout=True)
            ax.scatter(nearby[:, 0], nearby[:, 2], s=11, c="#087e8b", label="Original scan points")
            ax.scatter(point["x"], point["z"], s=150, c="#e12628", edgecolors="white", linewidths=1.5,
                       zorder=3, label="Measured upper edge")
            ax.set(xlabel="X (input units)", ylabel="Z (input units)", title="Section 1000: upper-sheet endpoint")
            ax.set_aspect("equal")
            ax.legend()
            fig.savefig(args.output / "section1000_zoom.png", dpi=180)
            plt.close(fig)
    print(json.dumps({"variants": len(summary["variants"]), "measured": len(measured), "trajectory": len(trajectory),
                      "all_p95_pass": all(v["measured"]["p95_within_one_spacing"] and v["trajectory"]["p95_within_one_spacing"] for v in summary["variants"])}, indent=2))


if __name__ == "__main__":
    main()
