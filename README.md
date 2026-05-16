# Mobile Robot Kinematics & Perception — Simulation Studies

> Mathematical modeling and numerical simulation of mobile robot systems, built from first principles using Python.

## Table of Contents
- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Kinematics](#kinematics)
- [Motion Control](#motion-control)
- [Algorithms](#algorithms)
- [Perception](#perception)
- [How to Run](#how-to-run)
- [Known Limitations](#known-limitations)
- [License](#license)

---

## Overview

This repository contains simulation studies covering core topics in mobile robotics — from kinematic modeling and numerical integration, through trajectory generation and feedback control, to sensor-based perception. Each section derives the underlying mathematics by hand, implements it in Python, and visualizes the results.

Topics covered:
- State space modeling of wheeled robots and aerial vehicles
- Numerical integration using Euler's method
- Differential flatness for trajectory planning
- Open-loop and closed-loop (PD) trajectory tracking
- Motion planning theory — A*, PRM, RRT — and Lyapunov stability
- 2D image filtering by spatial correlation
- LiDAR line extraction via Split-and-Merge

All simulations are implemented in Jupyter notebooks. Most plot outputs are committed and render directly on GitHub; see each section's README for specifics.

---

## Repository Structure

```text
robotics-simulations/
├── kinematics/
│   ├── README.md
│   └── unicycle_and_differential_drive.ipynb
├── motion_control/
│   ├── README.md
│   ├── differential_flatness_and_open_loop_control.ipynb
│   └── figures/
│       ├── q1di_4_basis_3d.png
│       └── q1dii_4_basis_3d.png
├── algorithms/
│   ├── README.md
│   ├── NOTES.md
│   └── closed_loop_trajectory_tracking.ipynb
├── perception/
│   ├── README.md
│   ├── image_filtering_and_line_extraction.ipynb
│   └── data/
│       ├── parrot.png
│       ├── rangeData_4_9_360.xlsx
│       ├── rangeData_5_5_180.xlsx
│       └── rangeData_7_2_90.xlsx
└── requirements.txt
```

---

## Kinematics

**Folder:** [`kinematics/`](kinematics/)

Derives and simulates the kinematic state space equations for four robot models — unicycle, differential drive, simplified car, and planar quadrotor. Implements Euler's method from scratch to integrate the equations forward in time, and analyzes how timestep size affects simulation accuracy.

See [`kinematics/README.md`](kinematics/README.md) for full details.

---

## Motion Control

**Folder:** [`motion_control/`](motion_control/)

Introduces **differential flatness** as a planning tool: derives the flat-output structure for a nonholonomic integrator and for the dynamically-extended unicycle, builds polynomial trajectories from boundary conditions via a linear-system solve (using 4-basis and 6-basis representations), and recovers the corresponding state and control histories analytically. Then implements **open-loop trajectory tracking** by Euler-integrating the unicycle forward under the computed controls — first noise-free, then under Gaussian disturbances on velocity and heading, which makes the open-loop strategy visibly drift. That failure motivates the closed-loop work in [`algorithms/`](algorithms/).

See [`motion_control/README.md`](motion_control/README.md) for full details.

---

## Algorithms

**Folder:** [`algorithms/`](algorithms/)

Two threads: (1) a **PD trajectory-tracking controller** that rescues the drifting open-loop simulation from `motion_control/` by feeding back position and velocity errors at every step — implemented for two gain settings (`kp = 1, kd = 2` and `kp = 4, kd = 4`); and (2) a theoretical study of **motion planning algorithms** — `A*`, PRM, RRT — and **Lyapunov stability** for pose stabilization and control synthesis. The simulation work is in the notebook; the theory deep-dive is in a separate [`NOTES.md`](algorithms/NOTES.md) alongside the section README.

See [`algorithms/README.md`](algorithms/README.md) for full details.

---

## Perception

**Folder:** [`perception/`](perception/)

Two perception primitives implemented from scratch: (1) **2D image filtering by spatial correlation** — the dot-product form of correlation applied to a grayscale photo with a 3×3 box-blur filter, demonstrating the building block beneath edge detection, sharpening, and Gaussian smoothing; and (2) **LiDAR line extraction by recursive Split-and-Merge** — fitting straight-line segments to three 2D range-scan datasets taken from different robot positions in a simulated indoor room, recovering the geometric structure of walls and obstacles from raw `(ρ, θ)` measurements.

See [`perception/README.md`](perception/README.md) for full details.

---

## How to Run

**Requirements:** Python 3.8+, Jupyter notebook or JupyterLab

Install dependencies:
```bash
pip install -r requirements.txt
```

Open any notebook:
```bash
jupyter notebook kinematics/unicycle_and_differential_drive.ipynb
```

Run all cells top to bottom. Most plots render inline and are committed to the notebooks. A few cells in `algorithms/` (the closed-loop runs) do not have outputs committed — see [`algorithms/README.md`](algorithms/README.md) for details. The animation cells in `kinematics/` produce an interactive HTML5 video that requires running the notebook locally; it does not render in GitHub's static preview.

---

## Known Limitations

Per-section limitations are documented in each section's README. Highlights:

- **Kinematics:** A copy-paste bug in the differential-drive Q3 cell makes the Δt = 0.5 and Δt = 0.01 blocks silently use `dt = 0.1`; the unicycle Q2 simulation is not affected. The planar-quadrotor model is derived on paper but not simulated.
- **Motion Control:** Only `Δt = 0.01` runs are retained for the open-loop tracking; the Q2c integration cell originally had `plt.show()` inside the loop, producing 1500 intermediate frames embedded as base64 PNGs — outputs were stripped from that cell with a clean post-loop overlay added below it.
- **Algorithms:** The two closed-loop tracking cells ([9] and [11]) do not have plot outputs committed; the original Q7a cell crashed with an `IndexError` from a grid-length mismatch which is now fixed in place. The notebook's RNG is unseeded so noise realizations differ between runs.
- **Perception:** The Split-and-Merge implementation includes the recursive split but not the merge step; `MAX_P2P_DIST` is declared but unused, which produces visible "jumping" segments across unrelated objects in the LiDAR plots. The line fit is Cartesian-LSQ rather than the polar LSQ called for by the assignment.

---

## License

MIT — see [LICENSE](LICENSE) file. Free for educational reuse.

## Author

GitHub: [asifulshiam](https://github.com/asifulshiam)
