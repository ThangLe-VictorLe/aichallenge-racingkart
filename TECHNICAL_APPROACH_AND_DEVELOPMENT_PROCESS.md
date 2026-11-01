# Technical Approach & Autonomous Racing Kart Development Process

**Summarized by AI**
**Author:** Thang Le (ThangLe-VictorLe)  
**Project:** Autonomous Racing Kart – Multi-Purpose Curvilinear MPC & Racing AI  
**System Architecture:** ROS 2 (Humble) / Autoware / AWSIM Digital Twin  
**Core Components:** `MPC.py`, `mpc_controller.py`, `reference_path.py`, `opponent_profiler.py`, `v2x_vehicle_tracker.py`, `config.yaml`, `ref_vel.yaml`, `steer_gain.yaml`

---

## 1. Executive Summary & Architectural Overview

The goal of this project was to develop a resilient, real-time, high-speed autonomous racing stack for an Ackermann-steered racing kart capable of aggressive wheel-to-wheel racing, proactive overtaking, obstacle avoidance, and autonomous recovery from crash or boundary-trap scenarios.

The baseline stack utilized a spatial/curvilinear Model Predictive Control (MPC) formulation tracking a Frenet reference path. However, in head-to-head racing scenarios, standard MPC approaches suffer from:
1. **Computational latency & solver infeasibility** under dense dynamic constraints.
2. **Hesitation and oscillation (chattering)** behind opponents when gaps open and close.
3. **Lack of tactical foresight** regarding opponent habits and corner lines.
4. **Permanent race termination** if the kart is pinned against track boundaries or loses solver feasibility.

Through iterative development, rigorous logging, and telemetry analysis, the stack was upgraded with:
- **Dynamic Opponent State Estimation & Predictive Trajectory Planning:** Track-wide statistical learning of opponent racing styles and future lines via EWMA.
- **Hierarchical Overtaking & Tailing Procedure:** Strategic side selection, gap bottleneck checking, velocity matching with safe deltas, lateral offset positioning, and dynamic launch triggers.
- **Autonomous Multi-State Recovery & Trap Escape:** FSM-driven reversing, reverse kinematic steering, margin relaxation, and post-recovery cooldown.
- **Performance Optimization & QP Caching ("MPC Never Dies"):** Matrix precomputation, vectorized raycasting, dynamic prediction horizon (*N*) hot-swapping, and decoupled safety margins.
- **Track Dynamics Optimization:** Dynamic steer-gain velocity interpolation, waypoint pacenotes, and minimal-curvature racelines.

```
                      +---------------------------------------+
                      |           V2X Telemetry Feed          |
                      +-------------------+-------------------+
                                          |
                                          v
                      +---------------------------------------+
                      |         V2X Vehicle Tracker           |
                      |  - Position jump rejection            |
                      |  - Velocity estimation & filtering    |
                      +-------------------+-------------------+
                                          |
                +-------------------------+-------------------------+
                |                                                   |
                v                                                   v
+-------------------------------+               +-----------------------------------+
|       Opponent Profiler       |               | Dynamic Obstacle Projection       |
| - Lap-by-lap progress         |               | - Frenet s & d projection         |
| - EWMA lateral error (ey)     |               | - Effective radius (R_eff)        |
| - Yaw relative to track (epsi)|               | - Squeezed static margin handling |
| - Strategic side reasoning    |               +-----------------+-----------------+
+---------------+---------------+                                 |
                |                                                 |
                +------------------------+------------------------+
                                         |
                                         v
               +---------------------------------------------------+
               |  Reference Path / Dynamic Corridor Generator      |
               |  - Decision Hierarchy: Strategy -> Bias -> Width  |
               |  - Strategic Memory Retention (30 wps / 5s)       |
               |  - Bottleneck Width Check (check_corridor_width)  |
               +-------------------------+-------------------------+
                                         |
                                         v
               +---------------------------------------------------+
               |               MPC Controller FSM                  |
               |  - NORMAL / TAILING / LAUNCH / NARROW / RECOVERY  |
               |  - Smooth velocity ramp-down                      |
               |  - Dynamic steer-gain velocity interpolation      |
               |  - Infeasibility step-relaxation                  |
               +-------------------------+-------------------------+
                                         |
                                         v
               +---------------------------------------------------+
               |          High-Performance OSQP MPC Solver         |
               |  - Cached P & A matrices                          |
               |  - Dynamic N-horizon hot swap (10 <-> 30)         |
               +-------------------------+-------------------------+
                                         |
                                         v
                         Ackermann Control Output (v, delta, a)
```

---

## 2. Dynamic Opponent State Estimation & Predictive Trajectory Planning

### 2.1 Theoretical Motivation
In autonomous wheel-to-wheel racing, reactive avoidance is insufficient. If the ego vehicle merely reacts to the opponent’s instantaneous bounding box, it gets trapped in sub-optimal racing lines, risks entering closing gaps, or gets pushed off the track on corner exits. Proactive racing requires understanding where the opponent is *likely* to be several waypoints ahead and exploiting their structural line tendencies.

### 2.2 Architectural Implementation (`opponent_profiler.py` & `v2x_vehicle_tracker.py`)
1. **Telemetry Ingestion & Filtering:**
   - Ingests `V2XVehiclePositionArray` messages from the digital twin.
   - Tracks individual vehicle IDs (`vid`) independently.
   - Enforces noise filtering: ignores position jumps (> 5.0 m) that represent telemetry glitches or simulation teleportation, resetting velocity vectors when anomalies occur.
2. **Continuous Track-Wide Statistical Profiling:**
   - Projects each opponent's Cartesian position $(x_{opp}, y_{opp})$ and heading yaw $\psi_{opp}$ onto the Frenet reference path.
   - Computes lateral displacement $e_y$:

$$
e_y = \cos(\psi_{wp}) \cdot (y_{opp} - y_{wp}) - \sin(\psi_{wp}) \cdot (x_{opp} - x_{wp})
$$

   where $e_y > 0$ indicates the opponent is to the left of the raceline, and $e_y < 0$ indicates the right.
   - Computes heading divergence relative to track tangent:

$$
\Delta \psi = \text{atan2}\left(\sin(\psi_{opp} - \psi_{wp}), \cos(\psi_{opp} - \psi_{wp})\right)
$$

   - Rather than relying solely on single-lap snapshots, introduced **Exponentially Weighted Moving Average (EWMA)** updating ($\alpha = 0.2$):

$$
\bar{e}_{y}[k] \leftarrow \alpha \cdot e_y[k] + (1 - \alpha) \cdot \bar{e}_{y}[k]
$$

$$
\bar{\Delta \psi}[k] \leftarrow \alpha \cdot \Delta \psi[k] + (1 - \alpha) \cdot \bar{\Delta \psi}[k]
$$

   This allows continuous learning across laps while gracefully adapting if an opponent alters their defensive posture.
3. **Corner Strategic Reasoning (`get_strategy_for_corner`):**
   - Evaluates the opponent's historical line at upcoming corner waypoints:
     - **Opponent Far Left ($e_y > 1.0\text{ m}$):** Recommends `"RIGHT"`, unless the opponent's yaw divergence reveals an aggressive cut across the track ($\Delta \psi < -0.2\text{ rad}$), in which case it recommends `"LEFT"`.
     - **Opponent Far Right ($e_y < -1.0\text{ m}$):** Recommends `"LEFT"`, unless opponent is cutting left ($\Delta \psi > 0.2\text{ rad}$), reversing recommendation to `"RIGHT"`.
     - **Opponent Blocking Center ($|e_y| < 0.5\text{ m}$):** Evaluates track curvature $\kappa$. If $\kappa > 0.005$ (left-hand turn), recommends `"LEFT"` (inside dive); if $\kappa < -0.005$ (right-hand turn), recommends `"RIGHT"`.
   - Restricts strategic side forcing during the opponent's initial out-lap until sufficient track history is collected.

---

## 3. Hierarchical Overtaking & Tailing Procedure

### 3.1 Distinct Opponent Classification (Fast vs. Slow / NPC)
Racing karts interact differently with competitive peers versus slow-moving or disabled vehicles. In `config.yaml` and `mpc_controller.py`, distinct behavioural profiles were established based on opponent velocity:
- **Fast Opponents ($v_{opp} \ge 20\text{ km/h}$):**
  - Moderate tailing distance (12 m), higher tailing speed delta (2.0 m/s), small launch safe width (0.1 m), shorter prediction horizon ($N = 10$) for agile transient response.
  - Overtaking steer gain interpolated between 1.6 and 2.0.
- **Slow Opponents / NPCs ($v_{opp} < 20\text{ km/h}$):**
  - Extended tailing awareness distance (25 m), earlier lateral offset positioning, enlarged prediction horizon ($N = 20$) to plan smooth, wide-radius sweeping lines around stationary obstacles.
  - Low-speed overtaking steer gain increased up to 2.5 for rapid turning authority.

### 3.2 Side Choice Protocol & Memory Retention
Corridor generation in `reference_path.py` was rebuilt with a strict 4-level decision hierarchy to prevent the MPC from oscillating between left and right sides of an opponent:
1. **Strategic Intent:** Evaluates `OpponentProfiler` recommendations based on past cornering habits.
2. **Track-Specific Corridor Bias (Pacenotes):** Pre-programmed tactical zones (`corridor_bias` in `config.yaml`) dictating which side is physically and strategically superior on specific track segments (e.g., inside line on turn entries).
3. **Geometric Width Selection with Hysteresis:** If neither strategy nor bias applies, compares available segment widths ($W_{left}$ vs. $W_{right}$). A significant margin threshold (0.2 m) is required to overturn the prevailing choice, penalizing center transitions:

$$
\text{Choice} = \begin{cases} 
\text{LEFT}, & \text{if } W_{left} > W_{right} + 0.2\text{ m} \\ 
\text{RIGHT}, & \text{if } W_{right} > W_{left} + 0.2\text{ m} \\ 
\text{Smooth Continuity}, & \text{otherwise} 
\end{cases}
$$

4. **Strategic Decision Memory:** Once a passing side is selected, the decision is locked in memory for 30 waypoints or 5.0 seconds. This prevents boundary flickering when crossing into transition zones.

### 3.3 Horizon Bottleneck Verification (`check_corridor_width`)
Previous iterations suffered from the "MPC swerve bug": the vehicle would initiate an overtake because the immediate next waypoint was open, only to encounter a closed wall 10 steps later, causing solver failure or emergency braking.
- Implemented `check_corridor_width()` outside of the MPC solver.
- Simulates the entire $N$-step horizon corridor.
- Computes the minimum physical bottleneck width ($W_{bottleneck}$) along the prospective corridor.
- Overtake / Launch mode is only unlocked if $W_{bottleneck} \ge \text{launch\_safe\_width}$. Otherwise, the kart holds in tailing mode.

### 3.4 Speed Matching, Safe Delta & Smooth Velocity Ramp-Down
- **Approach Velocity:** During tailing, maximum allowable speed is bounded by opponent speed:

$$
v_{target} = v_{opp} + \max(\Delta v_{min}, \Delta s \cdot K_{tail})
$$

- **Physical Deceleration Ramp:** Directly lowering $v_{max}$ in the MPC constraints causes the QP solver to assume the vehicle is already travelling slowly, planning sharp steer angles while physical momentum is still high. Implemented a rate-limited ramp:

$$
v_{cmd}^{k} = \max\left(v_{target}, v_{cmd}^{k-1} - 0.8 \cdot |a_{min}| \cdot \Delta t\right)
$$

  ensuring downstream actuators and the low-level PID controller do not saturate.
- **Lateral Offset Tailing:** While waiting for a gap, the kart applies a lateral shift (`tailing_lateral_offset = 0.7m`) toward the intended passing edge. This gives the kart an open sightline, creates clean airflow/sensor coverage, and positions it directly for launch without needing a lateral jerk when launching.

### 3.5 No-Overtake Zones & Emergency Pass Override
- **Two-Tier Zones:**
  - *Strict No-Overtake Zones:* Enforce conservative distance keeping in high-risk track sectors (narrow chicanes, blind turns).
  - *Prep Zones:* Suppress full-throttle launch but allow tight tailing to prepare for slipstreaming out of corner exits.
- **Crash Override:** If an opponent kart crashes or slows below 4.0 km/h, all no-overtake zone restrictions are instantly bypassed to prevent gridlocks.

---

## 4. Multi-State Kart Recovery & Boundary Escape System

### 4.1 Problem Definition
In competitive simulator testing and physical runs, collisions or aggressive defensive pinches can push the kart into the barriers or leave it wedged at an awkward angle relative to the raceline. In standard MPC, once the vehicle deviates significantly from the track or points backwards, the spatial formulation becomes mathematically ill-posed, resulting in QP infeasibility and permanent stalling.

### 4.2 State Machine Architecture
Implemented an autonomous recovery Finite State Machine embedded within `mpc_controller.py`:

```
                       +----------------------+
                       |        NORMAL        |
                       +----------+-----------+
                                  |
            Stuck timer expired   |   Corridor < 0.15m
           (or solver infeasible) |
                                  v
                       +----------------------+
       +-------------->|      RECOVERING      |<--------------+
       |               | (Reverse Gear,       |               |
       |               |  Reverse Kinematics) |               |
       |               +----------+-----------+               |
       |                          |                           |
       |        Reverse min time  |                           |
       |        & Solver Feasible |                           |
       |                          v                           |
       |               +----------------------+               |
       |               |       COOLDOWN       |               |
       |               | (Forward Drive,      |               |
       |               |  Trap Escape Gain)   |               |
       |               +----------+-----------+               |
       |                          |                           |
       |            |ey| < 0.05m  |  Stuck re-detected        |
       |            or timeout    +---------------------------+
       |                          v
       |               +----------------------+
       +---------------+        NORMAL        |
                       +----------------------+
```

1. **Intelligent Stuck Detection:**
   - Evaluates forward intent vs. actual telemetry velocity ($|v| < 2.0\text{ km/h}$).
   - **Dynamic Stuck Timer:** If the controller is commanding high forward velocity ($v_{cmd} > 1.0\text{ m/s}$) or experiencing solver infeasibility, stuck recovery triggers rapidly (0.5 s). If the kart is intentionally holding behind an opponent, a 3.0-second grace period is granted.
   - Suppresses stuck detection during race start (first 8.0 seconds) to prevent false triggering on the grid.
2. **Reverse Maneuvering (`RECOVERING`):**
   - Transmits `GearCommand.REVERSE`.
   - Commands fixed reverse velocity (5.0 km/h).
   - **Reverse Kinematic Steering Law:** Implemented closed-loop steering in reverse based on heading deviation $e_\psi$:

$$
\delta_{reverse} = \text{clip}(0.5 \cdot e_\psi, -0.314, 0.314)
$$

   This rotates the kart's nose toward the center of the track while backing away from the barrier.
   - Enforces a high recovery steer gain (2.0x).
3. **Trap Escape Cooldown (`COOLDOWN`):**
   - After reversing for at least 2.0 seconds and verifying MPC solver feasibility, shifts back to `GearCommand.DRIVE`.
   - Modifies cost matrices dynamically:
     - `cooldown_Q = [5000000.0, 5000000.0, 2000000.0]` (relaxes orientation penalty to prevent nose-diving into the wall).
     - `cooldown_R = [100000.0, 0.0]` (zeroes steer effort penalty to maximize steering actuation freedom).
   - Applies `post_recovery_steer_gain = 2.0`.
   - Exits cooldown once $|e_y| < 0.05\text{ m}$ or when the cooldown timer (10 s) expires.
4. **Corridor Narrow State (`NARROW`):**
   - If available corridor width drops below 0.15 m, the system halts immediately and shifts to reverse braking, preventing the solver from outputting erroneous acceleration.
5. **Gate 3 Alley Exit Protocol:**
   - Dedicated pitlane exit controller navigating narrow alleys ($|e_y| > 7.0\text{ m}$) using lookahead pure-pursuit before smoothly transferring control to MPC.

---

## 5. Performance Optimization & QP Stability ("MPC Never Dies")

### 5.1 QP Matrix Caching & Precomputation (`MPC.py`)
Profiling revealed that constructing sparse equality and inequality matrices on every control cycle (40 Hz) was consuming massive CPU overhead, causing cycle overruns when dynamic obstacles were in range.
- **Precomputed Cost Structure:**
  - Quadratic objective matrix $P$ is constructed once and stored.
  - Linear cost vector $q$ is updated via in-place vector assignment:

$$
q = \begin{bmatrix} -Q \cdot x_r \\ -R \cdot u_r \\ -Q_N \cdot x_{r,N} \end{bmatrix}
$$

- **Static Equality Block Diagonal Caching:**
  - Dynamic equality matrix $A_{eq}$ contains repeated structural zero-blocks and coordinate projection identities.
  - Pre-allocated constant sparse blocks ($Z_{top,A}, Z_{right,A}, Z_{top,B}$) and updated only non-zero spatial state transition terms, avoiding full-matrix re-allocation.

### 5.2 Decoupled Safety Margins
Previously, a single safety margin was applied to both track boundaries and dynamic obstacles. This created an impossible trade-off: widening margins caused the kart to fail overtakes on narrow sections; narrowing margins caused high-speed wall clipping.
- **Decoupled Margins:**
  - `static_safety_margin` (0.9 m): strictly protects track boundaries and walls.
  - `safety_margin` (0.73 m): protects against dynamic opponent karts.
  - `squeezed_static_margin` (0.8 m): dynamically applied during active tailing/launch mode to squeeze through tight gaps without sacrificing open-track safety.

### 5.3 Vectorized Raycasting & Constraint Merging (`reference_path.py`)
- Replaced iterative Bresenham and cell-by-cell loops with vectorized NumPy array operations.
- Obstacle boundaries are projected directly into Frenet coordinates $(s, d)$ with circular wrap-around handling:

$$
s_{dist} = (s_{wp} - s_{obs} + L/2) \bmod L - L/2
$$

$$
w = \sqrt{R_{eff}^2 - s_{dist}^2}
$$

  allowing instant computation of upper and lower bounds $[l_b, u_b]$ for the entire $N$-step horizon.

### 5.4 Progressive Infeasibility Relaxation
When the OSQP solver fails to find a strictly feasible solution (e.g., pinched corridor or extreme initial state error):
- Implemented a 5-stage progressive relaxation loop:

$$
\text{margin}_{relaxed} = \text{margin} \times \frac{5 - i}{5.0}, \quad i \in \{1, 2, 3, 4, 5\}
$$

- Progressively shrinks safety margins down to 0% to find the closest valid exit path.
- If relaxation level $\ge 4$ or status remains infeasible, increments the `infeasibility_counter`, which directly informs the Stuck Detection state machine to reverse out before a hard crash occurs.

### 5.5 Dynamic Horizon (*N*) Hot-Swapping
- Standard driving utilizes $N = 30$ (0.75 s lookahead at 40 Hz) for smooth track contouring.
- In active avoidance/launch mode against fast opponents, $N$ is dynamically swapped to 10 or 20, cutting QP matrix dimensions and solver iteration time by over 60% to eliminate control lag.

---

## 6. Vehicle Dynamics, Trajectory & Pacenote Tuning

### 6.1 Trajectory Evolution
1. **`mintime2`:** Minimum time trajectory generated via optimal control. Achieved high apex speeds but exhibited extreme tire slip angles that caused spin-outs when perturbed.
2. **`traj_mincurv` (Preliminary & Final Optimized):** Shifted to minimum-curvature racelines (`raceline_optimized_traj_mincurv.csv`). Flattening corner radii significantly reduced lateral accelerations, delivering consistent lap times (40.6 s per lap) and wide passing corridors.

### 6.2 Speed-Dependent Dynamic Steer Gain Interpolation
Fixed steering gain failed across wide speed envelopes: high gain at high speeds induced high-frequency steering oscillation (shimmy), while low gain at low speeds caused understeer when navigating tight hairpin corners or reversing.
- Implemented piecewise linear interpolation in `mpc_controller.py`:

$$
\text{Gain}(v) = \text{interp}\left(v, [v_1, \dots, v_k], [g_1, \dots, g_k]\right)
$$

- Normal low speed ($\le 10\text{ km/h}$): gain up to 2.5.
- High speed ($\ge 35\text{ km/h}$): gain tightened to 1.0 - 1.2 for high-speed tracking stability.

### 6.3 Segmented Track Pacenotes (`ref_vel.yaml` & `steer_gain.yaml`)
- Segmented the entire raceline into 5-waypoint intervals.
- Calibrated entry speeds, mid-corner velocities, and steer sensitivities for specific track sectors (hairpins, esses, high-speed sweepers, pit straight).

---

## 7. Chronological Commit History & Milestone Log

| Commit | Date | Key Enhancements & Milestones |
| :--- | :--- | :--- |
| `f5d00d0` | Jul 6 | Initial workspace setup; IMU corrector core implementation. |
| `483d193` | Jul 25 | Track CSV tools, lanelet map conversion, custom MPC tuning infrastructure. |
| `107fe3f` | Jul 25 | Integration of minimum-time trajectory profiles (`mintime2`). |
| `47d6ec5` | Jul 25 | Tire slip modeling exploration; dynamic control testing. |
| `e04b280` | Aug 1 | Shift to `traj_mincurv` raceline; achieved baseline 46.22 s lap. |
| `4eba749` | Aug 3 | Initial development of automated wall recovery system. |
| `a463c0e` | Aug 6 | Complete wall recovery via reverse gear shifting. |
| `912bca0` | Aug 6 | Full recovery coverage in all collision orientations; fallback to standardized `traj_mincurv`. |
| `59d6609` | Aug 11 | Optimized mincurv raceline; lap record lowered to 40.6 s; V2X visualization layer. |
| `5f93147` | Aug 14 | **"MPC Never Dies":** Identified computational bottlenecks; vectorized obstacle raycasting; instant constraint merging. |
| `e970d59` | Aug 14 | **QP Precomputation:** Cached $P$ matrix and sparse equality blocks; eliminated solver lag. |
| `ae6e87f` / `f839c5e` | Aug 15 | Steering & velocity overrides in recovery; cooldown mode with heading alignment; low-speed passing logic. |
| `2af854a` | Aug 17 | Opponent velocity calculation; pre-solve speed ramp down; dynamic launch distance. |
| `3776fc8` | Aug 18 | Frenet $(s, d)$ coordinate mapping for all waypoints; fixed alignment crash bugs. |
| `2432eaa` | Aug 19 | Formalized No-Overtake Zones; explicit Launch Mode activation; instant recovery trigger on crash. |
| `4a48046` | Aug 21 | Smart launch mode by step count; track pacenotes and bias; emergency pass override for stalled karts (< 5 km/h). |
| `ee10eb3` | Aug 21 | Distinct Fast vs Slow opponent profiles; strict no-overtake zones; full matrix raycasting vectorization. |
| `510a72f` | Aug 22 | Dynamic stuck timer (active driving vs. intentional stop grace period); dynamic steer gain interpolation by velocity. |
| `85631ab` | Aug 25 | **Dynamic Opponent State Estimation:** Initial `OpponentProfiler` recording opponent driving habits, lateral deviations, and styles. |
| `2849e00` | Aug 27 | Continuous learning via EWMA ($\alpha = 0.2$); blind start launch override (first 50 wps). |
| `13ddd9d` | Aug 28 | Strict velocity matching; safe delta for gap waiting; lateral offset tailing; strict 4-level side choice hierarchy. |
| `c6ff54b` | Aug 29 | Strategic memory retention (30 wps / 5.0 s); heading $\psi$ strategic reasoning; lateral offset applied across all tailing states. |
| `2be5ad3` | Aug 31 | Corridor width bottleneck checking (`check_corridor_width`); full stop gate passing; unblocked Safety Gates 1, 2, and 3. |
| `3dd1a33` | Aug 31 | Final tuning of waypoint reference velocities and dynamic steer gains. |
| `ca9f330` | Sep 1 | Decoupled wall and opponent safety margins; center-pull lateral offset on launch; solver infeasibility step relaxation. |

---

## 8. Quantitative Results & Key Takeaways

1. **Lap Time & Speed Improvement:**
   - Improved lap times from initial > 53 s down to a consistent **40.6 s** on the optimized minimal-curvature trajectory.
2. **Computational Determinism:**
   - Matrix caching and array vectorization reduced MPC control loop execution times from > 45 ms (causing dropped deadlines) down to a stable **< 15 ms**, maintaining a rock-solid 40 Hz control frequency even in crowded traffic.
3. **Zero-Stall Reliability:**
   - Multi-state recovery completely eliminated permanent freeze and terminal wall-crash scenarios across comprehensive safety-gate and head-to-head simulations.
4. **Racecraft Intelligence:**
   - The combination of EWMA opponent profiling, bottleneck corridor verification, and strategic memory allowed the kart to execute decisive overtakes without hesitation, clip-ins, or unnecessary off-line dives.
