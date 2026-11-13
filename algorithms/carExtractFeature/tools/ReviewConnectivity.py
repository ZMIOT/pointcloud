"""Compare old/new real-sample edge points against original scan sections."""
import sys
from pathlib import Path
local_packages = Path(__file__).resolve().parents[1] / "build/python-packages"
if local_packages.exists():
    sys.path.insert(0, str(local_packages))
from ValidateSample import read_pcd, read_csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[3]
sample = root / "examples/carExtractFeature/2.pcd"
folder = root / "algorithms/carExtractFeature/build/real2"
before = read_csv(folder / "before_connected/measured.csv")
after = read_csv(folder / (sys.argv[1] if len(sys.argv)>1 else "default") / "measured.csv")
before_trajectory = read_csv(folder / "before_connected/trajectory.csv")
after_trajectory = read_csv(folder / (sys.argv[1] if len(sys.argv)>1 else "default") / "trajectory.csv")
report = __import__("json").loads((folder / "default/report.json").read_text())
cloud = read_pcd(sample)
s = cloud @ np.asarray(report["scan_normal"])
boundaries = np.r_[0, np.flatnonzero(np.diff(s) > .1 * report["section_spacing"]) + 1, len(cloud)]
frames = [130, 831, 976, 985, 986, 987, 1000, 1177]
fig, axes = plt.subplots(2, 4, figsize=(17, 9), constrained_layout=True)
for ax, fi in zip(axes.flat, frames):
    profile = cloud[boundaries[fi]:boundaries[fi+1]]
    old = before[before["section_id"] == fi]
    new = after[after["section_id"] == fi]
    cx = float(old["x"][0])
    crop = profile[(profile[:,0]>cx-5)&(profile[:,0]<cx+4)]
    ax.scatter(crop[:,0],crop[:,2],s=10,c="#087e8b",label="Raw scan")
    ax.scatter(old["x"],old["z"],s=80,c="crimson",marker="x",label="Previous")
    ax.scatter(new["x"],new["z"],s=80,edgecolors="limegreen",marker="o",facecolors="none",linewidths=2,label="Connected run tip")
    ax.set(title=f"Section {fi}",xlabel="X",ylabel="Z",aspect="equal")
axes.flat[0].legend()
fig.savefig(folder / "connectivity_review.png",dpi=160)
fi = 986
profile = cloud[boundaries[fi]:boundaries[fi+1]]
old = before_trajectory[before_trajectory["section_id"] == fi]
new = after_trajectory[after_trajectory["section_id"] == fi]
cx = float(old["x"][0])
crop = profile[(profile[:,0]>cx-5)&(profile[:,0]<cx+4)]
fig, ax = plt.subplots(figsize=(8,5), constrained_layout=True)
ax.scatter(crop[:,0],crop[:,2],s=12,c="#087e8b",label="Original scan")
ax.scatter(old["x"],old["z"],s=120,c="crimson",marker="x",linewidths=2,label="Previous trajectory")
ax.scatter(new["x"],new["z"],s=140,c="limegreen",edgecolors="black",linewidths=0.6,label="Corrected trajectory")
ax.set(title="Section 986: detached piece versus continuous sheet",xlabel="X",ylabel="Z",aspect="equal")
ax.legend()
fig.savefig(folder / "section986_trajectory_correction.png",dpi=180)
