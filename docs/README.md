# Duckiebot — path tracking PID vs LQR (ROS 2 / Gazebo)

Simulation of a differential-drive robot following a closed, stadium-shaped path, controlled in closed loop on a full dynamic model (kinematics + motor dynamics). Two interchangeable controllers: an LQR state-feedback law and a cascade PID, synthesised on the same model and compared under identical conditions.

The physical model, the state-matrix derivation and the parameter choices are documented separately in `system_model.md` / `platform_constraints.md`; this README describes the **ROS 2 implementation** and how the pieces fit together.

---

## 1. Architecture

Four nodes, chained by topics. A single direction of flow, plus the feedback loop that closes through the Gazebo physics.

```
Gazebo (physics + gazebo_ros2_control)
   │ /joint_states            [ω_L, ω_R]   ~985 Hz (physics loop)
   ├──────────────────────────────────────────┐
   ▼                                          │
path_error_node       (C++, ~985 Hz)          │
   │ /path_error              [e_l, e_θ, κ]   │
   ▼                                          ▼
lqr | pid_controller_node  (C++, ~197 Hz) ◄───┘
   │ /motor_voltage_cmd       [V_L, V_R]
   ▼
motor_node.py         (Python, ~197 Hz)
   │ /wheel_effort_controller/commands   [τ_L, τ_R]
   ▼
Gazebo  (closed loop)
```

The controller (LQR or PID) reads `/joint_states` directly in addition to `/path_error`: `e_l` and `e_θ` come from the estimator, `ω_L`/`ω_R` come from the raw encoders.

**Timing — driven by the measurement, not by a timer.** All four nodes run on the arrival of `/joint_states` (published at ~985 Hz from the Gazebo physics loop), not on a ROS timer. A timer under `use_sim_time` is clocked by `/clock`, published at only 10 Hz by default under Gazebo Classic: the nodes then ran at ~20 Hz instead of the intended rate, which injected enough delay to destabilise the loop. The controllers decimate the measurement by 5 (~197 Hz); the estimator and the motor node process every message.

**Estimator / controller separation**: `path_error_node` knows nothing about the controller, and the controller knows nothing about the path geometry (it only receives Kappa). Replacing the LQR with a PID touches neither the estimator, and vice versa.

### ros2_control startup chain

```
duckiebot.xacro               declares the <ros2_control> contract (per-joint interfaces)
   → robot_state_publisher    compiles the xacro, exposes the robot_description parameter
   → gazebo_ros2_control plugin  reads that parameter + controller_config.yaml
   → controller_manager          instantiates the controllers
        ├─ joint_state_broadcaster  → publishes /joint_states
        └─ wheel_effort_controller  → relays the torques
```

## 2. Running the simulation

```bash
cd ~/ros2_ws
colcon build --packages-select duckiebot
source install/setup.bash
```

Three terminals (each with the workspace sourced):

```bash
# 1 — physics + ros2_control controllers
ros2 launch duckiebot gazebo.launch.py

# 2 — control loop (estimator + controller + motor)
ros2 launch duckiebot control.launch.py controller:=lqr   # or controller:=pid

```

The `controller:=lqr|pid` choice is **exclusive**: only one of the two nodes is instantiated.

Model-only visualisation, without physics (geometry, axes, wheelbase):

```bash
ros2 launch duckiebot display.launch.py
```

---

## 3. Directory layout

```
urdf/duckiebot.xacro          robot model + Gazebo parameters + ros2_control contract
src/path_error_node.cpp       estimator: odometry → path error
src/lqr_controller_node.cpp   LQR controller + curvature feedforward
src/pid_controller_node.cpp   cascade PID controller + speed loop
scripts/motor_node.py         motor model: voltage → torque
launch/gazebo.launch.py       physics + spawn + controller activation
launch/control.launch.py      control loop, argument controller:=lqr|pid
launch/display.launch.py      RViz only, without Gazebo
config/controller_config.yaml ros2_control controllers
config/lqr_params.yaml        LQR gain matrix K
config/pid_params.yaml        PID gains and saturations
config/robot_params.yaml      shared physical parameters (Ke, r, L, vr, v_max…)
config/scenario.yaml          initial pose (shared: Gazebo spawn ↔ estimator)
```

---

## 4. Robot model (`duckiebot.xacro`)

Structure: `base_link` (empty) → `chassis` (fixed) → 2 wheels (`continuous`) + `caster` (fixed).

### Masses and inertias — traceability

| Quantity | Value | Source |
|---|---|---|
| `chassis` mass | 0.935 kg | Total to 1 kg (total − wheels − caster) |
| chassis `izz` | 4.1e-3 | point-mass decomposition (battery, camera, RPi at their real positions) |
| chassis `ixx` / `iyy` | 1.76e-3 / 2.50e-3 | homogeneous solid-box formula — approximation |
| wheel `iyy` | 2.5e-4 | model `J`, rotor + gearbox reflected to the wheel side (×N²) |
| caster inertia | 1.28e-7 | homogeneous solid sphere, (2/5)·m·r² |
| wheel `damping` | 0.0457 | = Kt·Ke/R (see §5) |

`ixx`/`iyy` do not enter any equation of the control model (2D, only `izz` matters): they only affect Gazebo's internal 3D physics (roll/pitch), invisible on flat ground with gentle dynamics.

The `chassis` link aggregates the whole robot except the wheels and caster — Gazebo has a single rigid body, unlike the block-by-block decomposition used to compute `izz`.

### Wheel/ground contact parameters

| Parameter | Role |
|---|---|
| `mu1` / `mu2` | Coulomb friction / contact friction (grip) (1.0 wheels = adhesion, 0.01 caster = slides freely) |
| `kp` | contact stiffness (1e7 = near-rigid) |
| `kd` | contact damping, prevents bounce |
| `minDepth` | interpenetration tolerated without correction, to avoid constant micro-corrections |
| `maxVel` | interpenetration-correction speed — set to 0: everything rests on `kp/kd`, avoids visible bounces on stiff contact |

### Ground truth

The `libgazebo_ros_p3d` plugin publishes the exact pose of `base_link` on `/ground_truth/odom`, outside the control loop. Used only as a reference to measure odometry drift.

---

## 5. Motor model and computation split

Per-wheel equation:

```
J·ω̇ = (Kt/R)·V − (Kt·Ke/R)·ω
        ↑ forcing term   ↑ back-EMF (self-braking)
```

The back-EMF term appears by substituting the current `i = (V − Ke·ω)/R` into `J·ω̇ = Kt·i`.

**This term is carried by the joint's ODE damping, not by `motor_node.py`.** The URDF damping applies exactly `τ = −b·ω` (N·m·s/rad); setting `b = Kt·Ke/R = 0.478²/5 = 0.0457`, the form and the value match the back-EMF. Without it, Gazebo would apply a torque with no resistance and the speed would diverge.

`motor_node.py` therefore computes `i = V/R` then `τ = Kt·i` — without the ω term, which would otherwise be double-counted. Max torque: `Kt·V_max/R = 0.478 N·m`, consistent with the `±1.0` bounds of the `effort` interface.

---

## 6. Control nodes

### `path_error_node.cpp` — state estimator

Input `/joint_states`, output `/path_error` = `[e_l, e_θ, κ]` (~985 Hz, `dt` measured from `header.stamp`).

1. **Forward kinematics**: `v = r(ω_L+ω_R)/2`, `ω = r(ω_R−ω_L)/L`
2. **Odometry** (explicit Euler): `x += v·cos(θ)·dt`, `y += v·sin(θ)·dt`, `θ += ω·dt`
3. **Projection onto the stadium**:

```
px = xc + clamp(x−xc, −a, a)      py = yc          (nearest point on the central segment)
d  = hypot(x−px, y−py)
e_l     = R − d
th_p    = atan2(y−py, x−px) + π/2                  (tangent ⟂ to the radius)
e_theta = atan2(sin(θ−th_p), cos(θ−th_p))          (wrapped to [−π, π])
κ       = 1/R if |x−xc| > a, else 0
```

**Safety**: if the pose falls on the central segment itself, `atan2(0,0)` is undefined. The node publishes nothing rather than propagate a NaN into the closed loop.

### `lqr_controller_node.cpp` — controller

Inputs `/joint_states` + `/path_error`, output `/motor_voltage_cmd` = `[V_L, V_R]` (~197 Hz).

```
ω_L_nom = vr·(1 − κ·L/2)/r        V_nom_L = Ke·ω_L_nom
ω_R_nom = vr·(1 + κ·L/2)/r        V_nom_R = Ke·ω_R_nom

x   = [e_l, e_θ, ω_L − ω_L_nom, ω_R − ω_R_nom]
u   = −K·x
V_L = clamp(V_nom_L + u_L, ±U_max)      (same for V_R)
```

**Feedforward / feedback separation** — the central design point of this node.

The nominal is computed per wheel from kappa(κ) alone: it is the inverse kinematics applied to `(v = vr, ω = κ·vr)`, i.e. what would have to be commanded if tracking were perfect. No measured error is involved. On a straight (κ=0) both wheels have the same nominal; on a turn, the inner wheel spins slower.

`K` is fixed (computed offline).

The nominal is recomputed every cycle: on a stadium, κ changes along the way.
`u = −K·x` is a **deviation** from `V_nom`, not an absolute voltage.

### `pid_controller_node.cpp` — controller (alternative to the LQR)

Inputs `/joint_states` + `/path_error`, output `/motor_voltage_cmd`, ~197 Hz.
Same feedforward `V_nom(κ)` as the LQR; only the feedback changes.

Three SISO loops, decomposed:

- **differential mode** (cascade): `pid_pos` (slow) `e_l → e_θ_ref`, then `pid_cap` (fast) `e_θ_ref → V_rot`.
- **common mode** (PI): `pid_vit` regulates `(ω_L+ω_R)/2` towards the nominal. Symmetric output `+V_avg`.

Recomposition: `V_L = V_nom_L + V_avg − V_rot`, `V_R = V_nom_R + V_avg + V_rot`.

**PID vs LQR.** The LQR is one matrix K: it looks at the four states at once (e_l, e_θ, the two speeds) and computes the voltages accounting for the links between them. The PID is three separate loops, each on its own quantity, ignoring one another. Correcting the speed does not disturb the heading, so three independent loops are enough.

### `motor_node.py` — motor model

Input `/motor_voltage_cmd` (V), output `/wheel_effort_controller/commands` (N·m) on each command received (triggered by `/motor_voltage_cmd`, not by a timer). Converts voltage → current → torque (`i = V/R`, `τ = Kt·i`) and bridges units between the command (Volts) and the `effort` interface of `ros2_control` (N·m).

---

## 7. Configuration

| File | Contents |
|---|---|
| `controller_config.yaml` | `joint_state_broadcaster` + `wheel_effort_controller` (passthrough, `effort` interface). `update_rate: 1000` Hz. |
| `robot_params.yaml` | physical parameters shared by both controllers: `Ke`, `wheel_radius`, `wheel_base`, `vr`, `v_max` (5 V, HAT supply). Single source of the physics. |
| `lqr_params.yaml` | matrix `K` (2×4 flattened), computed offline on the nominal linearised model. |
| `pid_params.yaml` | gains of the three loops (`kp/ki/kd` × pos/cap/vit), saturations (`theta_ref_max`, `v_rot_max`, `v_avg_max`) and `decimation`. |
| `scenario.yaml` | initial pose, **single source** read by `gazebo.launch.py` (spawn) and `control.launch.py` (estimator init) — guarantees both start from the same point. |

With the default stadium (center (0,0), R = 0.5, a = 0.5), the pose `(0, −0.5, 0)` places the robot exactly on the path, tangent: `e_l = 0`, `e_θ = 0`.

---