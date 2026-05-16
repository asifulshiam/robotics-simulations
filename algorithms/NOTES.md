# Theory Notes — Lyapunov Stability and Motion Planning Algorithms

These notes accompany [`README.md`](README.md). The README focuses on the closed-loop tracking simulation; this file covers the theoretical material the section studies but doesn't implement in code:

- [Q1 — A* vs Differential Flatness](#q1--a-vs-differential-flatness)
- [Q2 — Pose Stabilization via a Lyapunov Function](#q2--pose-stabilization-via-a-lyapunov-function)
- [Q3 — Lyapunov-Based Control Design](#q3--lyapunov-based-control-design)
- [Q4 — A* Grid-Based Search Algorithm](#q4--a-grid-based-search-algorithm)
- [Q5 — Probabilistic Road Maps (PRM)](#q5--probabilistic-road-maps-prm)
- [Q6 — Rapidly-exploring Random Trees (RRT)](#q6--rapidly-exploring-random-trees-rrt)

---

## Q1 — A* vs Differential Flatness

Both methods can produce a robot trajectory from a starting pose to a goal pose, but they operate in very different ways and have very different cost profiles.

**Differential flatness** works directly in the continuous-time domain. Once a system is shown to be differentially flat (state and controls expressible as functions of a flat output and its derivatives), trajectory design reduces to choosing smooth functions in the flat output space and reading the state and controls off by differentiation. The trajectories are **piecewise** smooth polynomial curves; the math is closed-form. No grid is needed, no graph search is performed, and the computation cost is dominated by solving a small linear system for the polynomial coefficients.

**A\*** is a graph search algorithm. The state space is **discretized** into a grid or graph of nodes, and the algorithm explores nodes outward from the start, expanding the most promising candidates first based on a cost-to-arrive plus heuristic-cost-to-go estimate. The result is the lowest-cost path through the graph that respects the discretized obstacle layout.

The key trade-offs:

| | Differential flatness | A* grid search |
|---|---|---|
| **Computation** | Cheap (small linear solve) | Expensive (potentially exponential in grid size) |
| **Obstacles** | No native handling | First-class — obstacle cells are simply unwalkable |
| **Output shape** | Smooth, polynomial | Piecewise-linear over grid nodes; needs post-smoothing |
| **Dynamic constraints** | Honored by construction (the trajectory IS feasible for the system) | Not native — A* finds geometric paths; feasibility is a separate concern |
| **Optimality** | Not optimal — feasible only | Optimal over the discretized graph (with admissible heuristic) |

The choice between them is driven by what the environment looks like:

- **Obstacle-free 3D space** → use differential flatness. There's nothing for A* to be careful about, and the smooth polynomial trajectories are directly executable by the controller. A* would be wasteful here — discretizing and searching a graph to recover something flatness gives you in closed form.
- **Obstacle-rich environment** → use A* (or one of the sampling-based planners below). Differential flatness has no machinery for steering around obstacles; you'd have to wrap it in an outer loop that re-plans whenever a collision is detected, which defeats the simplicity.

Most real robotics stacks combine both: a global planner like A* (or RRT) handles the topology of getting around obstacles, and a local controller using flatness-style methods generates the smooth trajectory between successive waypoints.

---

## Q2 — Pose Stabilization via a Lyapunov Function

This is a standard example from the course notes: design a smooth feedback law that drives the unicycle from any initial pose to the origin, and prove the law works using Lyapunov's direct method.

**System.** The kinematic unicycle in Cartesian form:

```text
ẋ = V · cos(θ)
ẏ = V · sin(θ)
θ̇ = ω
```

Controls are linear velocity `V` and angular velocity `ω`. The goal is `(x, y, θ) = (0, 0, 0)`.

**Polar coordinates.** Switch to a representation centered on the goal:

```text
ρ = √(x² + y²)                    Euclidean distance to origin
α = atan2(y, x) − θ + π            heading relative to the line back to origin
δ = α + θ                          angle of that line, measured in the world frame
```

In these coordinates the dynamics become:

```text
ρ̇ = −V · cos(α)
α̇ = (V · sin(α)) / ρ − ω
δ̇ = (V · sin(α)) / ρ
```

This is a useful change of variables because `ρ` directly measures "how close are we to the goal" — driving `ρ → 0` is the primary objective, and `α, δ` describe the angles we still need to correct.

**Candidate Lyapunov function.** Choose

```text
V(ρ, α, δ) = ½ ρ² + ½ (α² + k₃ δ²)
```

with `k₃ > 0`. This is positive definite (zero only at the origin, positive everywhere else) — the first condition for a valid Lyapunov function.

**Control law.** Pick

```text
V_cmd = k₁ · ρ · cos(α)
ω     = k₂ · α + k₁ · (sin(α)·cos(α) / α) · (α + k₃ · δ)
```

with `k₁, k₂, k₃ > 0`.

**Derivative along trajectories.** Compute `V̇` using the chain rule and the polar dynamics. After substituting and simplifying (the `(sin α cos α / α) · (α + k₃ δ)` cross-term in `ω` is constructed exactly to cancel the corresponding cross-term in `V̇`):

```text
V̇ = ρ · ρ̇ + α · α̇ + k₃ · δ · δ̇
  = ρ · (−V·cos α) + α · ((V·sin α)/ρ − ω) + k₃ · δ · (V·sin α / ρ)
  = −k₁ · ρ² · cos²(α) − k₂ · α²
```

Because `k₁ > 0` and `k₂ > 0`, the right-hand side is `≤ 0` everywhere, and `= 0` only when `ρ = 0` and `α = 0`. By Lyapunov's direct method this proves the closed-loop system is stable, and by LaSalle's invariance principle it is asymptotically stable to the origin.

The geometric interpretation: the `−k₁ ρ² cos²(α)` term shrinks the distance whenever the robot is pointing roughly toward the origin (so `cos α` is nonzero), and the `−k₂ α²` term steers the heading toward the homing direction.

---

## Q3 — Lyapunov-Based Control Design

Given a nonlinear system and a candidate Lyapunov function, the design task is to choose the control `u` that makes the function strictly decrease.

**System.** Two-state nonlinear system:

```text
ẋ₁ = −x₁ + x₂³
ẋ₂ = −x₂ + u
```

`u` is the control input.

**Candidate Lyapunov function.**

```text
V(x) = ½ x₁² + ¼ x₂⁴
```

Positive definite (even powers of both states), zero only at the origin.

**Derivative.**

```text
V̇ = x₁ · ẋ₁ + x₂³ · ẋ₂
  = x₁ · (−x₁ + x₂³) + x₂³ · (−x₂ + u)
  = −x₁² + x₁·x₂³ − x₂⁴ + x₂³·u
```

The first and third terms are already negative. The two cross terms `x₁·x₂³ + x₂³·u = x₂³ · (x₁ + u)` are the ones that need to be killed. Choose:

```text
u = −x₁
```

This cancels the offending pair:

```text
V̇ = −x₁² + x₁·x₂³ − x₂⁴ − x₁·x₂³
  = −x₁² − x₂⁴
```

Always `≤ 0`, and `= 0` only at the origin. So `u = −x₁` stabilizes the system in the Lyapunov sense.

This is the cleaner cousin of the pose-stabilization example: we *designed* the control specifically so the bad terms in `V̇` cancel, rather than guessing a control and then proving it works. The technique generalizes — pick a `V`, expand `V̇`, identify which terms `u` can influence, and solve for the `u` that drives the result negative.

---

## Q4 — A* Grid-Based Search Algorithm

A* (read "A-star") is a best-first graph search that finds the lowest-cost path from a start node to a goal node by expanding nodes in order of estimated total path cost. The trick that makes it efficient is the heuristic — A* never expands a node whose total estimated cost exceeds the cost of an already-found path.

**Setup.** Discretize the workspace into a grid of nodes. Some nodes are marked as obstacles; the rest are walkable. Mark a `start` node and a `goal` node.

**Cost function for each node `n`:**

```text
f(n) = g(n) + h(n)
```

- `g(n)` — actual accumulated cost from `start` to `n` along the path explored so far
- `h(n)` — heuristic estimate of remaining cost from `n` to `goal` (often straight-line distance, which is admissible because it never overestimates)
- `f(n)` — total estimated path cost through `n`

**The algorithm.** Maintain two sets:
- **Open set** — nodes discovered but not yet expanded (initially: just `start`)
- **Closed set** — nodes already expanded (initially: empty)

Repeat until either the goal is reached or the open set is empty:

1. Pick the node `n` in the open set with the lowest `f(n)`. If `n` is the goal, reconstruct the path by following parent pointers back to `start` and return it.
2. Move `n` from the open set to the closed set.
3. For each neighbor `m` of `n` that is not an obstacle:
   - If `m` is in the closed set, skip it.
   - If `m` is not in the open set, compute its `g`, `h`, `f`, set its parent to `n`, and add it to the open set.
   - If `m` is already in the open set, check if the path through `n` is cheaper than the previously recorded path. If so, update `m`'s cost values and reset its parent to `n`.

If the open set empties before the goal is found, there is no path.

**Cons.**
- **High memory.** Both the open and closed sets can grow large in cluttered or high-dimensional spaces.
- **Slow in complex maps.** Worst-case complexity grows with the number of nodes; long search horizons hurt.
- **Heuristic-sensitive.** A poor heuristic (or no heuristic at all — that reduces A* to Dijkstra's algorithm) destroys the speed advantage.
- **Ignores dynamic constraints.** A* finds geometric paths over the grid. Whether the underlying robot can actually follow that path — given turning-radius limits, acceleration bounds, etc. — is a separate problem. The path may need significant post-processing or may be infeasible.
- **Grid resolution trade-off.** Fine grids give better paths but blow up search cost; coarse grids are fast but may miss feasible routes through narrow passages.

---

## Q5 — Probabilistic Road Maps (PRM)

PRM is a **sampling-based** motion planner that builds a roadmap of the free space ahead of time, then runs a graph search over the roadmap for any subsequent query. Good for static environments where many path queries will be answered against the same map.

**Algorithm.**
1. **Sample** a large number of configurations (nodes) randomly in the workspace, keeping only those that lie in obstacle-free space.
2. **Connect** each sampled node to nearby nodes by straight-line segments — but only keep a connection if the segment doesn't intersect any obstacle.
3. The resulting graph is the **roadmap**. It captures the topology of the free space.
4. **Query**: given a `start` and `goal`, connect them to their nearest roadmap nodes (again, only if the connection is obstacle-free), then run Dijkstra or A* on the roadmap to find a path.

**Cons.**
- **Needs many samples for good coverage.** Sparse sampling misses connectivity; dense sampling is expensive.
- **Not good for real-time use.** The roadmap is built up front; it doesn't help with environments that change.
- **Narrow passages are problematic.** Random sampling rarely places nodes inside narrow gaps, so the roadmap fails to capture them. Special "narrow-passage" sampling biases exist to mitigate this.

PRM's strength is when the same map is queried many times — pay the roadmap construction cost once, answer many path queries cheaply.

---

## Q6 — Rapidly-exploring Random Trees (RRT)

RRT is also sampling-based but builds the search structure incrementally during each query, biased toward unexplored regions of the space. Strong fit for single-query planning in high-dimensional or dynamically-constrained spaces.

**Algorithm.**
1. **Initialize** the tree with the `start` configuration as its root.
2. Repeatedly:
   - **Sample** a random configuration `q_rand` in the workspace.
   - Find the **nearest** node `q_near` already in the tree.
   - **Extend** the tree from `q_near` toward `q_rand` by a small step `Δ`, producing a new candidate node `q_new`.
   - If the segment from `q_near` to `q_new` is obstacle-free, **add** `q_new` to the tree with `q_near` as its parent.
3. **Stop** when the tree reaches (or comes within tolerance of) the `goal`. Trace back through parent pointers to recover the path.

The "rapidly exploring" property comes from the fact that random sampling tends to pull the tree toward regions that haven't been explored yet — large empty areas are filled in quickly because samples there land far from any existing tree node.

**Cons.**
- **Paths are unsmooth.** The tree grows in random directions; the recovered path zigzags. Post-processing (path smoothing, B-spline fitting) is typically required before the path is executable.
- **No optimality guarantee.** Vanilla RRT finds a feasible path, not the shortest one. Extensions like RRT* are asymptotically optimal but more expensive.

RRT is the workhorse for high-dimensional planning problems — robot arms with many joints, manipulation in cluttered scenes — where A*'s gridding cost is prohibitive and PRM's roadmap construction doesn't pay off because each query needs only one path.

---

## License

MIT — see [LICENSE](../LICENSE) in the repository root.
