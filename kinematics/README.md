# Kinematics — Robot State Space Modeling & Euler Integration

This section derives the kinematic state space equations for four robot models from first principles, then implements Euler's method to simulate their motion over time. The timestep sensitivity of the numerical integration is analyzed by comparing trajectories at Δt = 0.1, 0.5, and 0.01.

---

## Notebook

**`unicycle_and_differential_drive.ipynb`**

Contains the full simulation for two of the four derived models:

- **Q2 — Unicycle simulation:** Euler integration with time-varying angular velocity input
- **Q3 — Differential drive simulation:** Euler integration with independent left/right wheel velocity inputs

> Run all cells top to bottom. Plots render inline. Animation cells produce an interactive HTML5 video (local only — does not render on GitHub).

---

## Mathematical Background

### State Space Form

All robot models follow the general nonlinear state space form:

```
ẋ = f(x, u)
```

where `x` is the state vector (position + orientation), `u` is the control input, and `ẋ` describes how the state evolves. Since the dynamics are nonlinear (cos θ, sin θ terms), this cannot be written as the standard linear `ẋ = Ax + Bu` form — the A matrix is zero and the B matrix is state-dependent.

---

### Q1 — Robot Models Derived

#### Q1a — Unicycle

The simplest wheeled robot model. A single point with position (x, y) and heading θ.

```
State:   x = [x, y, θ]ᵀ
Control: u = [v, ω]ᵀ    (translational velocity, angular velocity)

ẋ = v·cos θ
ẏ = v·sin θ
θ̇ = ω
```

Physical constraints: |v| ≤ v_max, |ω| ≤ ω_max

#### Q1b — Differential Drive

A two-wheeled robot where each wheel is driven independently. Turning is achieved by spinning the wheels at different speeds.

```
State:   x = [x, y, θ]ᵀ
Control: u = [ωr, ωl]ᵀ  (right wheel angular velocity, left wheel angular velocity)
Params:  r = wheel radius, L = axle length

v  = (r/2)(ωl + ωr)     ← derived linear velocity
ω  = (r/L)(ωr - ωl)     ← derived angular velocity

ẋ = v·cos θ
ẏ = v·sin θ
θ̇ = ω
```

Turning conditions:
- ωl > ωr → turn right
- ωr > ωl → turn left
- ωl = ωr → go straight

#### Q1c — Simplified Car

Models a car-like robot with front-wheel steering. Unlike the unicycle, orientation change depends on translational velocity and steering angle.

```
State:   x = [x, y, θ]ᵀ
Control: u = [v, φ]ᵀ    (translational velocity, steering angle)
Params:  L = wheelbase

ẋ = v·cos θ
ẏ = v·sin θ
θ̇ = (v/L)·tan φ
```

Constraints: |v| ≤ v_max, |φ| < π/2

#### Q1d — Planar Quadrotor

A 2D model of a quadrotor with two rotors generating forces T1 and T2.

```
State:   x = [x, vx, y, vy, φ, ω]ᵀ
Control: u = [T1, T2]ᵀ  (rotor thrust forces)
Params:  m = mass, ℓ = rotor arm length, Izz = moment of inertia

ẋ  = vx
v̇x = -(T1+T2)·sin φ / m
ẏ  = vy
v̇y = (T1+T2)·cos φ / m - g
φ̇  = ω
ω̇  = (T2-T1)·ℓ / Izz
```

Hover condition: T1 + T2 = mg
Ascent: T1 + T2 > mg
Descent: T1 + T2 < mg
Rotate right: T1 > T2

---

### Euler's Method

The discrete update rule derived from the definition of the derivative:

```
dx/dt = lim(Δt→0) [x(t+Δt) - x(t)] / Δt
```

For a finite timestep Δt, this gives the forward Euler approximation:

```
x[i+1] = x[i] + ẋ[i] · Δt
```

Applied to the unicycle:
```
x[i+1]     = x[i]     + v · cos(θ[i]) · Δt
y[i+1]     = y[i]     + v · sin(θ[i]) · Δt
θ[i+1]     = θ[i]     + ω[i] · Δt
```

The key insight: given initial conditions `x(0), y(0), θ(0)` and a control sequence, the entire trajectory is computed iteratively — each step uses only the previous step's values.

---

### Q2 — Timestep Analysis (Unicycle)

Three timesteps compared — same control input, same initial conditions:

| Δt | Trajectory shape | Accuracy |
|----|-----------------|----------|
| 0.01 | Smooth curves, tight loops | High — secant approximates tangent well |
| 0.1 | Slightly less smooth | Good — reasonable for most purposes |
| 0.5 | Blocky, rectangular segments | Poor — large steps miss the curve |

The x vs y trajectory plot makes this visually clear: smaller Δt produces smoother, more physically accurate paths. This is the fundamental trade-off in numerical integration — accuracy vs. computation cost.

---

## Known Limitations

- **Q3 — Differential-drive timestep bug.** The differential-drive simulation blocks intended for Δt = 0.5 and Δt = 0.01 both hardcode `dt = 0.1` due to a copy-paste error. The timestep comparison for the differential drive model therefore does not reflect three distinct integration resolutions — all three runs use `dt = 0.1`. The Q2 unicycle simulation does not have this issue and correctly demonstrates all three timesteps.
- **Q1d — Quadrotor matrix form.** The planar quadrotor state space derivation is conceptually correct but the B matrix in the written derivation has formatting inconsistencies. The full simulation of the quadrotor model is not implemented in the notebook.
- **No saved outputs.** Notebook outputs are not committed. Run locally to reproduce plots and animations.

## License

MIT — see [LICENSE](../LICENSE)
