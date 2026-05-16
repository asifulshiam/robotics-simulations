# Algorithms — Closed-Loop Control & Motion Planning

This section covers two threads:

1. **A closed-loop trajectory tracking controller** that fixes the fragility of the open-loop control studied in [`motion_control/`](../motion_control/). Open-loop control breaks down under noise — feedback restores tracking by continuously correcting state error against a planned reference.
2. **A theoretical study of motion planning algorithms** — `A*` grid search, Probabilistic Road Maps (PRM), Rapidly-exploring Random Trees (RRT) — and their comparison to the differential-flatness method used earlier. These are written up in [`NOTES.md`](NOTES.md) alongside the Lyapunov-stability derivations.

## Files

```text
algorithms/
├── README.md
├── NOTES.md                                  ← theory writeups: A* / PRM / RRT / Lyapunov stability
└── closed_loop_trajectory_tracking.ipynb     ← simulation: closed-loop controller under noise
```

The notebook contains a recap of the open-loop chain (trajectory generation → open-loop tracking → open-loop with noise) before introducing the new closed-loop controller. This recap keeps the notebook self-contained — running it top-to-bottom reproduces the full story without depending on cells from `motion_control/`.

---

## Why closed-loop control

The open-loop controller in `motion_control/` worked perfectly in a deterministic simulation: it pre-computed the controls `a(t)` and `ω(t)` analytically from the planned trajectory, then advanced the robot state by Euler integration. Under noise injected into the velocity and heading updates, those controls were no longer correct — the robot drifted, and there was no mechanism to correct the drift because the controller never observed the actual state.

A **closed-loop** (feedback) controller measures the state error at every timestep and modifies its commanded acceleration and angular velocity in response. The error driver: how far is the robot from where the plan says it should be, in both position and velocity?

## The PD trajectory-tracking law

Take the same extended unicycle dynamics from `motion_control/`:

```text
ẋ = V · cos(θ)
ẏ = V · sin(θ)
V̇ = a
θ̇ = ω
```

Differentiating the position equations once more gives the relation between accelerations `(ẍ, ÿ)` and the controls `(a, ω)`:

```text
| ẍ |   | cos(θ)   -V·sin(θ) | | a |
|   | = |                    | |   |
| ÿ |   | sin(θ)    V·cos(θ) | | ω |
```

The trajectory-tracking controller replaces `(ẍ, ÿ)` with a **PD feedback law** that drives the position and velocity errors to zero:

```text
ẍ_cmd = ẍ_d - kp · (x - x_d) - kd · (ẋ - ẋ_d)
ÿ_cmd = ÿ_d - kp · (y - y_d) - kd · (ẏ - ẏ_d)
```

Each axis has a proportional term `-kp · (position_error)` that pulls the robot back toward the desired position, and a derivative term `-kd · (velocity_error)` that damps the response. Standard PD shape: stiff enough to track quickly, damped enough not to oscillate.

Substituting `(ẍ_cmd, ÿ_cmd)` for `(ẍ, ÿ)` in the `2×2` system above and inverting (using the **desired** trajectory's `θ_d, ẋ_d` rather than the current `θ, V` — this linearizes the matrix to the planned reference frame):

```text
| cos(θ_d)   -ẏ_d | | a |   | ẍ_d - kp·(x - x_d) - kd·(ẋ - ẋ_d) |
|                 | |   | = |                                    |
| sin(θ_d)    ẋ_d | | ω |   | ÿ_d - kp·(y - y_d) - kd·(ẏ - ẏ_d) |
```

Solving for `(a, ω)` at each timestep gives the commanded controls. The matrix on the left is invertible as long as `(ẋ_d, ẏ_d) ≠ (0, 0)` — i.e. the planned trajectory has nonzero velocity, which it does by construction here (the boundary velocity is `V = 0.5`).

## Numerical results

Both runs use:
- Trajectory: same differentially-flat path from the [extended-unicycle trajectory generation](../motion_control/#trajectory-generation) in `motion_control/` (boundary conditions `(0,0,0.5,-π/2)` → `(5,5,0.5,-π/2)`, `T = 15`)
- Noise: `V ← V + Δt·a + N(0, 0.01)`, `θ ← θ + Δt·ω + N(0, 0.001)` — matches the open-loop noise case so results are directly comparable to the [open-loop failure mode shown in `motion_control/`](../motion_control/#part-3--open-loop-tracking-under-noise)
- Timestep: `Δt = 0.01`

The notebook plots the desired trajectory (blue) overlaid with the simulated robot path (red) for each gain pair.

### Q7a — Closed-loop tracking with `kp = 1, kd = 2`

The robot tracks the trajectory closely. Compared to the open-loop-with-noise case in `motion_control/` where the path visibly diverged, the PD correction keeps the simulated path nearly on top of the reference.

### Q7b — Closed-loop tracking with `kp = 4, kd = 4`

Stiffer proportional gain and slightly higher damping. Tracks even more tightly to the reference. With a wider noise injection or longer horizon, the stiffer gains would be expected to reject disturbances more aggressively at the cost of more control effort. For these noise levels, both gain pairs succeed; the higher pair gives marginally cleaner tracking.

The qualitative point of the comparison: a closed-loop controller with sensible PD gains rescues the system from the open-loop failure mode. Once feedback is present, the tuning question is one of speed vs. effort, not of whether tracking is possible at all.

---

## Theoretical study (in `NOTES.md`)

The notebook only implements Q7 — the closed-loop simulation above. The rest of the section's work (Q1–Q6) is theoretical and is written up in [`NOTES.md`](NOTES.md):

- **Q1** — `A*` vs differential flatness — when each is the right tool, what the trade-offs are
- **Q2** — Pose stabilization via a Lyapunov function — showing `V̇ < 0` for the chosen control law on a unicycle (proving asymptotic stability to the origin)
- **Q3** — Lyapunov-based control design — given a nonlinear system and a candidate Lyapunov function, find the control law `u` that stabilizes it
- **Q4 / Q5 / Q6** — Step-by-step descriptions of `A*` grid search, PRM, and RRT, with the practical limitations of each

The `NOTES.md` content is the substantive theory portion of the work and explains the algorithmic vocabulary referenced informally above (e.g. "differential flatness" as an alternative to `A*` for trajectory generation in obstacle-free space).

---

## How to Run

```bash
cd algorithms
pip install -r ../requirements.txt
jupyter notebook closed_loop_trajectory_tracking.ipynb
```

Run cells top-to-bottom. The full sequence reproduces:
1. The DF planned trajectory (3rd-order polynomial in `(x, y)`)
2. Open-loop tracking without noise (perfect)
3. Open-loop tracking with noise (visibly fails)
4. Closed-loop tracking with `kp=1, kd=2`
5. Closed-loop tracking with `kp=4, kd=4`

**Note: cells [9] and [11] (the closed-loop runs for Q7a and Q7b) do not have plot outputs committed.** They render correctly when run locally, but the committed notebook intentionally leaves them empty — the original Q7a cell raised an `IndexError` because it used the trajectory arrays from cell [2] (length 151, computed on `Δt = 0.1`) while iterating on the dense grid from cell [3] (length 1501, `Δt = 0.01`); the fix is to recompute the desired trajectory and its derivatives on the dense grid before the loop, which the current Q7a cell does. Q7b was newly added so it has no committed output either. Re-run both cells locally to generate the plots. Cells [2], [5], [7] (trajectory + open-loop recap) retain their committed outputs and render on GitHub directly.

## Known Limitations

- **Cells [9] and [11] outputs are not committed.** See the note in "How to Run" above. Running the notebook top-to-bottom locally reproduces both closed-loop plots and they look like the README claims — the trajectory tracks closely under both gain settings, with the higher gains tracking slightly tighter.
- **The recap cells [3], [5], [7] depend on cell-execution order.** They share variable names (`xd`, `yd`, `V`, `theta`, etc.) and each cell re-initializes them at the top, but the cells must run in order. Cells [9] and [11] each re-initialize their state arrays explicitly to avoid this fragility, but cells [5] and [7] inherit `t`, `dt`, etc. from cell [3].
- **Unseeded RNG.** `np.random.normal` is called without seeding, so each run produces a different noise realization. To reproduce a specific plot exactly, add `np.random.seed(<int>)` before the relevant loop.
- **Both runs use `Δt = 0.01` only.** Timestep sensitivity isn't the focus of this section (it was studied in `motion_control/`), so no Δt sweep is included here.

## License

MIT — see [LICENSE](../LICENSE) in the repository root.
