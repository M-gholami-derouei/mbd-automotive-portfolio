# Adaptive Cruise Control — Model-Based Design
### A complete MPC-based ACC system built from first principles in MATLAB/Simulink

---

## Overview

This project implements a full **Adaptive Cruise Control (ACC)** system using **Model-Based Design (MBD)** in MATLAB/Simulink R2024b. The system controls a vehicle's longitudinal motion by maintaining a target cruise speed when no lead vehicle is present, and a safe following gap when a lead vehicle is detected.

The project was built entirely from first principles — from deriving the vehicle dynamics equations to justifying every design parameter physically. No pre-built Vehicle Dynamics Blockset templates were used for the plant model.

---

## System Architecture

The model consists of three interconnected subsystems:

```
F_driver ──────────────────────────────────────────┐
                                                    ├──► Switch ──► VehiclePlant ──► v_ego
MPC Controller ──► F_traction ─────────────────────┘         ▲                        │
       ▲                                                       │                        │
       │                                                       └────────────────────────┤
  [v_ref; d_ref] ◄── mode ◄── Stateflow                                                 │
  [OV weights]                    ▲                                                      ▼
       │                     is_detected                Environment ◄──────────── v_ego │
  v_lead ──────────────────────────────────────────────────────┘
```

### VehiclePlant
Models longitudinal vehicle dynamics derived from Newton's second law:

$$m\dot{v} = F_{traction} - C_{rr}mg - \frac{1}{2}C_D\rho A_f v^2 - mg\sin(\theta)$$

A static friction model prevents physically incorrect reverse acceleration from rest.

### Environment
Generates the lead vehicle velocity profile (5-phase scenario) and computes the gap dynamics:

$$\dot{d} = v_{lead} - v_{ego}, \quad d(0) = d_0$$

### Stateflow Mode Logic
Hierarchical state machine managing three operating modes:

| Mode | Value | Condition | MPC Objective |
|------|-------|-----------|---------------|
| Standby | 1 | ACC engaged, initializing | Hold current speed |
| Following | 2 | d < d_max (lead detected) | Track d_ref |
| Speed Control | 3 | d ≥ d_max (no lead) | Track v_ref |

![Stateflow Chart](images/stateflow_chart.png)
*Figure 1: Hierarchical Stateflow state machine*

---

## Design Decisions

### Why MPC over PID?

The ACC plant has:
- **Input constraints**: F_traction ∈ [−3000, 4500] N
- **State constraints**: v ≥ 0 (no reverse under ACC)
- **Competing objectives**: speed tracking AND gap maintenance

MPC handles all of these natively through constrained optimization. A PID would require separate mode-switching logic with no optimality guarantee.

### Why Mode Switching?

With **one manipulated variable** (F_traction) and **two outputs** (v, d), simultaneous perfect tracking of both is mathematically impossible. The fundamental insight is:

- When no lead vehicle exists → d is undefined → only track v_ref
- When lead vehicle detected → safety dominates → only track d_ref, speed is secondary

Stateflow implements this switching by changing MPC output weights at runtime via the `y.wt` port of the MPC Controller block.

### MPC Prediction Model

The nonlinear plant is linearized via Taylor expansion around operating point v_op = 60 km/h:

$$\dot{x} = Ax + Bu + Ev_{lead}$$

$$A = \begin{bmatrix} -\frac{C_D \rho A_f v_{op}}{m} & 0 \\ -1 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} \frac{1}{m} \\ 0 \end{bmatrix}, \quad E = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$$

States: x = [v, d]ᵀ — Manipulated variable: F_traction — Measured disturbance: v_lead

### MPC Horizon Justification

| Parameter | Value | Physical Justification |
|-----------|-------|----------------------|
| Sample time Ts | 0.1 s | Captures vehicle dynamics bandwidth (~1-2 Hz) |
| Prediction horizon Np | 50 steps (5 s) | Covers worst-case emergency braking: t_stop = v_max/a_max = 33.3/8 ≈ 4.1 s |
| Control horizon Nc | 25 steps (2.5 s) | Covers velocity-matching maneuver; halves QP decision variables |

### Dynamic Gap Reference

The following distance reference is velocity-dependent:

$$d_{ref} = v_{ego} \cdot t_{headway} + d_{min}$$

A fixed gap reference would be either dangerously small at high speed or unnecessarily conservative at low speed. This formulation ensures TTC ≥ t_headway at all times.

---

## Vehicle Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Vehicle mass | m | 1500 | kg |
| Air density | ρ | 1.2 | kg/m³ |
| Frontal area | A_f | 2.1 | m² |
| Drag coefficient | C_D | 0.30 | — |
| Rolling resistance | C_rr | 0.015 | — |
| Time headway | t_headway | 2.0 | s |
| Min standstill gap | d_min | 5 | m |
| Radar range | d_max | 150 | m |

---

## Lead Vehicle Scenario

A 5-phase velocity profile exercises all critical ACC behaviors:

| Phase | Time | v_lead | Behavior Tested |
|-------|------|--------|----------------|
| 1 | 0–30 s | 0 → 60 km/h | Acceleration from rest |
| 2 | 30–90 s | 60 km/h | Speed holding |
| 3 | 90–150 s | 60 → 120 km/h | Following acceleration |
| 4 | 150–210 s | 120 → 40 km/h | **Critical braking scenario** |
| 5 | 210–300 s | 40 km/h | Gap stabilization |

---

## Results

### Speed Control Mode
v_ego tracking a constant reference of 15 m/s from rest. Clean first-order response with no overshoot after weight tuning.

![Speed Control Result](images/speed_control_result.png)
*Figure 2: F_traction (top) and v_ego (bottom) in Speed Control mode. v_ego settles exactly at v_ref = 15 m/s.*

---

### Following Mode
Gap tracking with dynamic d_ref = v_ego · t_headway + d_min. The MPC maintains the following distance throughout all 5 phases of the lead vehicle scenario.

![Following Mode Result](images/following_mode_result.png)
*Figure 3: d_ref (top) and d (bottom) in Following mode. Gap tracks reference closely and remains always positive.*

---

### Full System with Mode Switching
Complete simulation showing Stateflow mode transitions driven by gap distance relative to radar detection range.

![Full System Result](images/full_system_result.png)
*Figure 4: d_ref, d, v_ego (km/h), and mode signal. Mode switches between Following (2) and Speed Control (3) based on d vs d_max.*

---

## How to Run

### Prerequisites
- MATLAB R2024b
- Simulink
- Model Predictive Control Toolbox
- Stateflow
- Embedded Coder (optional)

### Steps

```matlab
% 1. Open MATLAB and navigate to the project folder
cd('path/to/adaptive-cruise-control')

% 2. Load vehicle and MPC parameters
run('scripts/parameters.m')

% 3. Build the MPC prediction model and controller object
run('scripts/MPC_linear_model.m')

% 4. Open and run the Simulink model
open_system('model/VehiclePlant.slx')
sim('model/VehiclePlant.slx')
```

---

## Project Structure

```
adaptive-cruise-control/
├── model/
│   └── VehiclePlant.slx          # Complete Simulink model
├── scripts/
│   ├── parameters.m               # Vehicle and MPC parameters
│   └── MPC_linear_model.m         # State-space model and MPC object
├── docs/
│   └── MPC_Design_Parameters.pdf  # Detailed parameter justification report
├── images/
│   ├── stateflow_chart.png
│   ├── speed_control_result.png
│   ├── following_mode_result.png
│   └── full_system_result.png
└── README.md
```

---

## Key Engineering Insights

**On the 1-input 2-output problem:** With only F_traction as the manipulated variable, simultaneous perfect tracking of both speed and gap is mathematically impossible. The mode-switching architecture resolves this by making only one objective active at a time — a physically correct and practically elegant solution.

**On static friction:** Rolling resistance only opposes motion when v > 0. At rest, the net force cannot be negative — the vehicle stays stationary until traction exceeds the static friction threshold. This was implemented with a conditional logic block in Simulink.

**On linearization:** The nonlinear drag term v² was linearized around v_op = 60 km/h using Taylor expansion. The prediction model is therefore most accurate near highway cruise conditions, which represents the dominant operating regime for ACC.

---

## Author

**Hassan Gholami**
Control & Automation Engineer — Milan, Italy
[LinkedIn](https://linkedin.com/in/gholami-hassan) | [Email](mailto:m.gholami.derouei@gmail.com)
