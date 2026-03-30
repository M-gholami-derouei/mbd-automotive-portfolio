# Battery Thermal Management System — Model-Based Design
### MPC-based BTMS for Battery Electric Vehicle using MATLAB/Simulink + Simscape

---

## Overview

This project implements a full **Battery Thermal Management System (BTMS)** for a Battery Electric Vehicle (BEV) using **Model-Based Design (MBD)** in MATLAB/Simulink R2024b with Simscape thermal modeling. The system controls battery pack temperature by commanding coolant mass flow rate through a liquid cooling circuit, maintaining cell temperature within the safe operating window while preventing thermal runaway.

The project was built entirely from first principles — from deriving the cell heat generation mechanisms to justifying every linearization assumption physically. No pre-built battery thermal templates were used for the plant model.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Battery Thermal Physics](#battery-thermal-physics)
3. [Heat Generation Mechanisms](#heat-generation-mechanisms)
4. [Thermal Model Derivation](#thermal-model-derivation)
5. [Radiation Neglect Justification](#radiation-neglect-justification)
6. [Lumped Capacitance Validity](#lumped-capacitance-validity)
7. [Coolant Loop Architecture](#coolant-loop-architecture)
8. [Complete Two-State Nonlinear Model](#complete-two-state-nonlinear-model)
9. [Entropic Coefficient Assumptions](#entropic-coefficient-assumptions)
10. [Actuator Chain and Manipulated Variable](#actuator-chain-and-manipulated-variable)
11. [Operating Point Selection](#operating-point-selection)
12. [Linearization](#linearization)
13. [MPC Formulation](#mpc-formulation)
14. [Stateflow Thermal Mode Logic](#stateflow-thermal-mode-logic)
15. [Safety Constraints and Thermal Runaway](#safety-constraints-and-thermal-runaway)
16. [Design Assumptions Summary](#design-assumptions-summary)
17. [Vehicle and System Parameters](#vehicle-and-system-parameters)
18. [How to Run](#how-to-run)
19. [Project Structure](#project-structure)

---

## System Architecture

The BTMS controls battery cell temperature by regulating coolant mass flow rate through a liquid cooling circuit. The system consists of four interconnected subsystems:

```
[Current I] ──────────────────────────────────────────────────────────────────────┐
[SoC] ───────────────────────────────────────────────────────────────────────┐    │
                                                                             │    │
MPC Controller ──► m_cool_dot ──► Pump ──► Cold Plate ──► Chiller Circuit   │    │
      ▲                                         │                            │    │
      │                                         ▼                            │    │
 [T_cell; T_cool] ◄────────── SimscapePlant ◄──┘◄───── BatteryCell ◄────────┘────┘
      │                            ▲
 [T_cool_in; I]                    │
 [T_amb; SoC] ──────── Disturbances/Scheduled Parameters
      │
  Stateflow ──► Thermal Mode (Normal / Warning / Critical)
      ▲
  T_cell threshold logic
```

---

## Battery Thermal Physics

### Why BTMS is safety-critical

A lithium-ion battery cell is not a perfect energy converter. The difference between electrical energy input and useful output manifests as **heat generation inside the cell**. Unlike an engine overheating — which is a linear, recoverable degradation — uncontrolled battery heating can trigger **thermal runaway**: a self-sustaining exothermic chain reaction where:

1. Elevated temperature accelerates decomposition of the electrolyte and cathode material
2. Decomposition releases oxygen
3. Oxygen reacts with the flammable electrolyte
4. The reaction generates more heat, further accelerating decomposition
5. The process becomes self-reinforcing and **cannot be stopped once initiated**

The result is fire or explosion — not a stalled vehicle. This is the fundamental engineering motivation for treating BTMS as a safety-critical control system, not merely a performance optimization.

### Safe Operating Window

For lithium-ion NMC (Nickel Manganese Cobalt Oxide) cells:

| Zone | Temperature Range | Consequence |
|------|------------------|-------------|
| Optimal performance | 25°C — 40°C | Best efficiency, lowest degradation |
| Acceptable | 15°C — 45°C | Mild degradation |
| Warning | 45°C — 60°C | Accelerated aging, capacity fade |
| **Danger** | **>60°C** | **Thermal runaway risk** |
| Cold limit | <15°C | Lithium plating, permanent damage |

The BTMS must therefore maintain $T_{cell}$ within $[15°C, 45°C]$ under normal operation, with $60°C$ as a hard safety constraint. Below $15°C$, the system may need to provide heating — outside the scope of this project.

---

## Heat Generation Mechanisms

A lithium-ion cell produces heat through **two distinct physical mechanisms**:

### 1. Joule Heating (Irreversible)

Current flowing through the cell's internal resistance produces resistive heating:

$$\dot{Q}_{ohm} = I^2 R_{int}$$

This term is:
- Always **positive** regardless of current direction
- The **dominant** heat generation mechanism under normal operation
- Quadratic in current — heat generation increases rapidly under high load

### 2. Entropic Heat (Reversible)

During the electrochemical reaction, the cell exchanges heat with its surroundings due to the **entropy change** of the reaction. This is analogous to the thermal expansion work in a thermodynamic cycle:

$$\dot{Q}_{ent} = T_{cell} \cdot I \cdot \frac{\partial U_{OCV}}{\partial T_{cell}}\bigg|_{SoC}$$

Where:
- $U_{OCV}$ is the open-circuit voltage (equilibrium electrochemical potential)
- $\frac{\partial U_{OCV}}{\partial T_{cell}}$ is the **entropic coefficient** — a thermodynamic property of the cell
- For NMC cells, $\frac{\partial U_{OCV}}{\partial T_{cell}} < 0$ (typically $\approx -0.0002$ V/K)
- **The sign of this term depends on current direction** — it is not always additive to $\dot{Q}_{ohm}$

**Sign analysis by operating mode:**

| Mode | $I$ convention | $\frac{\partial U}{\partial T}$ | $\dot{Q}_{ent}$ | Thermal effect |
|------|---------------|----------------------------------|-----------------|----------------|
| Discharge | $I > 0$ | $< 0$ | $< 0$ | Endothermic — reduces net heat generation |
| Charge | $I < 0$ | $< 0$ | $> 0$ | Exothermic — adds to net heat generation |

At moderate discharge ($I = 10$ A, $T_{cell} = 300$ K, $\frac{\partial U}{\partial T} \approx -0.0002$ V/K):
$$\dot{Q}_{ent} \approx 300 \times 10 \times (-0.0002) = -0.6 \text{ W}$$

Compared to Joule heating: $\dot{Q}_{ohm} = 10^2 \times 0.05 = 5$ W — the entropic contribution is approximately **12% of the dominant term**.

The entropic term cannot be treated as negligible, for two distinct reasons:

1. **During charging**, $\dot{Q}_{ent} > 0$and adds to Joule heating. Omitting it produces a systematic 12% underestimate of total heat generation — the MPC predicts lower $T_{cell}$ than reality, accumulating bias toward a delayed cooling response.

2. **During discharge**, $\dot{Q}_{ent} < 0$ and partially offsets Joule heating. Omitting it produces a systematic overestimate of heat generation — the MPC is more conservative than required, commanding higher-than-necessary pump speeds. In a dual-use BTMS (charge and discharge), omitting the entropic term therefore introduces a sign-dependent model error that cannot be bounded without knowing the operating mode. Retaining it ensures the prediction model is accurate in both modes without mode-dependent correction.

> **Note:** Neglecting $\dot{Q}_{ent}$ during discharge specifically is conservative (overestimates heating), not dangerous. However, a prediction model with a sign-dependent 12% error in the heat generation term is not an acceptable foundation for an MPC operating across both charge and discharge cycles. The term is retained for correctness.

---

## Thermal Model Derivation

### Lumped Parameter Assumption

The cell is modeled as a **uniform mass at a single temperature** $T_{cell}(t)$. The starting point is the heat conduction PDE governing the interior of the cell:

$$\rho c_p \frac{\partial T}{\partial t} = k\nabla^2 T + \dot{q}_{gen}$$

This PDE governs heat diffusion **inside the cell body only**. The cooling does not appear in the PDE — it enters as a **boundary condition** at the cell surface, coupling the surface temperature to the coolant:

$$-k\frac{\partial T}{\partial n}\bigg|_{surface} = \frac{T_{surface} - T_{cool}}{R_{th}}$$

The lumped capacitance assumption performs three operations simultaneously:

**1. Eliminates the spatial operator.**
Uniform temperature means $\nabla^2 T = 0$ everywhere inside the cell. The diffusion term vanishes.

**2. Integrates the PDE over the cell volume.**
Converting volumetric heat generation $\dot{q}_{gen}$ [W/m³] to total heat generation $\dot{Q}_{gen}$ [W]:

$$\int_V \rho c_p \frac{\partial T}{\partial t} \, dV = \int_V \dot{q}_{gen} \, dV \quad \Rightarrow \quad m_{cell} c_{p,cell} \dot{T}_{cell} = \dot{Q}_{gen}$$

**3. Absorbs the boundary condition into the ODE.**
With uniform temperature, $T_{surface} = T_{cell}$. The surface boundary condition becomes a heat removal term in the energy balance:

$$\dot{Q}_{cool} = \frac{T_{cell} - T_{cool}}{R_{th}}$$

Combining all three steps yields the lumped ODE:

$$\boxed{m_{cell} c_{p,cell} \dot{T}_{cell} = \dot{Q}_{gen} - \frac{T_{cell} - T_{cool}}{R_{th}}}$$

> **Note:** The cooling term $\dot{Q}_{cool}$ does not originate from the PDE body — it originates from the surface boundary condition. This distinction is physically important: the PDE governs internal conduction; the boundary condition governs the thermal interface between the cell and its cooling circuit. The lumped assumption collapses both into a single ODE by assuming the internal resistance to heat flow is negligible relative to the external resistance at the surface — an assumption quantified by the Biot number (see [Lumped Capacitance Validity](#lumped-capacitance-validity)).

### Heat Removal Mechanisms

Three mechanisms could remove heat from the cell surface:

**Conduction** through TIM (Thermal Interface Materials) and cold plate:
$$\dot{Q}_{cond} = \frac{T_{cell} - T_{cool}}{R_{th}}, \quad R_{th} = R_{TIM} + R_{cold\,plate}$$

**Convection** to coolant (captured in $R_{th}$ above for the cold plate circuit)

**Radiation** to surroundings:
$$\dot{Q}_{rad} = \varepsilon \sigma A (T_{cell}^4 - T_{amb}^4)$$

### Radiation Neglect Justification

At BTMS operating conditions ($T_{cell} = 60°C = 333K$, $T_{amb} = 20°C = 293K$, $A = 0.01$ m²):

$$\dot{Q}_{rad} = 0.9 \times 5.67\times10^{-8} \times 0.01 \times (333^4 - 293^4) \approx 2.5 \text{ W}$$

$$\dot{Q}_{conv} = hA(T_{cell} - T_{cool}) = 500 \times 0.01 \times 40 = 200 \text{ W}$$

$$\frac{\dot{Q}_{rad}}{\dot{Q}_{conv}} = \frac{2.5}{200} = 1.25\%$$

**Radiation contributes 1.25% of convective heat removal under liquid cooling. Neglected.**

### Cell Governing Equation

Combining both heat generation mechanisms and the single dominant heat removal path:

$$\boxed{m_{cell} c_{p,cell} \dot{T}_{cell} = I^2 R_{int} + T_{cell} \cdot I \cdot \frac{\partial U_{OCV}}{\partial T_{cell}}\bigg|_{SoC} - \frac{T_{cell} - T_{cool}}{R_{th}}}$$

---

## Lumped Capacitance Validity

The lumped capacitance assumption requires the **Biot number** to be small:

$$Bi = \frac{h L_c}{k_{cell}} \ll 1$$

For a **cylindrical** 18650 cell, the characteristic length is the ratio of volume to surface area. For a long cylinder of radius $r$ and length $L$, the lateral surface dominates:

$$L_c = \frac{V}{A_s} = \frac{\pi r^2 L}{2\pi r L} = \frac{r}{2}$$

With $r = 9$ mm:

$$L_c = \frac{0.009}{2} = 0.0045 \text{ m}$$

> **Note:** The formula $L_c = r/3$ applies to a **sphere** ($V/A_s = \frac{4}{3}\pi r^3 / 4\pi r^2 = r/3$) and is incorrect for a cylinder. Using it here would underestimate the Biot number by 33%.

With:
- Cell thermal conductivity (radial): $k_{cell} \approx 1$ W/mK
- Liquid cooling convective coefficient: $h \approx 500$ W/m²K

$$Bi = \frac{500 \times 0.0045}{1} = \mathbf{2.25}$$

**$Bi \gg 1$ — the lumped assumption is not valid for a single cell.** Meaningful radial temperature gradients exist. However, for the purposes of an MPC prediction model — which requires a low-order linear state-space representation — the lumped model is the standard engineering simplification used throughout BTMS literature. A distributed parameter model (PDE-based, implemented via Simscape finite element discretization) represents a natural extension of this work.

This assumption is explicitly acknowledged in the model and its implications noted: the MPC prediction model will underestimate the peak temperature at the cell core, which must be compensated by **conservative safety margins on the temperature constraint**. With $Bi = 2.25$, the core-to-surface temperature difference is non-negligible, and the 60°C hard constraint applied to the surface sensor measurement must be interpreted as a surrogate for the core temperature — the actual safety margin to thermal runaway onset is smaller than the nominal 20°C margin suggests.

---

## Coolant Loop Architecture

### Architectural Clarification

A critical architectural decision was made during the modeling phase. A naive BTMS architecture might connect the coolant loop directly to ambient via a radiator:

$$\text{Cell} \rightarrow \text{Cold plate} \rightarrow \text{Radiator} \rightarrow \text{Ambient}$$

This is **incorrect for a BEV BTMS**. In this architecture, $T_{cool}$ is bounded below by $T_{amb}$, which means in a European summer ($T_{amb} = 30°C$), the coolant cannot be driven below $30°C$ — insufficient to maintain $T_{cell}$ in the optimal window of $25°C$–$40°C$ under load.

The correct BEV architecture uses a **refrigeration circuit (chiller)**:

$$\text{Cell} \rightarrow \text{Cold plate} \rightarrow \text{Chiller (heat exchanger)} \rightarrow \text{Refrigerant circuit} \rightarrow \text{Condenser} \rightarrow \text{Ambient}$$

The coolant loop **never contacts ambient directly**. The chiller exchanges heat with a refrigerant circuit — identical in principle to automotive air conditioning. This decouples $T_{cool}$ from $T_{amb}$, allowing the coolant to be driven below ambient temperature by the compressor.

### Coolant as Second State

With this architecture, modeling the coolant temperature as a **second dynamic state** (rather than a known input) is necessary because:

1. The MPC must predict $T_{cool}$ evolution to anticipate pump commands
2. The coolant thermal mass introduces its own dynamics — a step change in pump speed does not instantly change $T_{cool}$
3. Anticipatory control (commanding the pump before $T_{cell}$ exceeds limits) requires knowledge of how fast $T_{cool}$ responds

### Coolant Energy Balance

Applying a lumped energy balance to the coolant in the cold plate (control volume A):

$$\boxed{m_{cool} c_{p,cool} \dot{T}_{cool} = \frac{T_{cell} - T_{cool}}{R_{th}} - \dot{m}_{cool} c_{p,cool}(T_{cool} - T_{cool,in})}$$

Where:
- $\frac{T_{cell} - T_{cool}}{R_{th}}$: Heat entering coolant from cell (same term as in cell equation, opposite sign — energy conserved)
- $\dot{m}_{cool} c_{p,cool}(T_{cool} - T_{cool,in})$: Advective heat removal — warm coolant leaving cold plate, cool coolant returning from chiller
- $T_{cool,in}$: Chiller outlet temperature — treated as a **measured disturbance** from a sensor at the chiller outlet
- $\dot{m}_{cool}$: Coolant mass flow rate — the **manipulated variable** commanded by the MPC via pump speed

**Plug-flow assumption:** This formulation implicitly treats the cold plate as a **plug-flow volume with spatially uniform coolant temperature**, using the cold plate outlet temperature as representative of the entire coolant mass $m_{cool}$. In reality, a temperature gradient exists along the flow path — the coolant entering the cold plate is at $T_{cool,in}$ and exits at $T_{cool}$. The lumped model collapses this gradient into a single state. A consequence is that at low flow rates the residence time increases — the fluid heats up more per pass — which the energy balance captures correctly through the advective term, provided the outlet temperature is used as the representative state. This approximation is standard in low-order BTMS models and its accuracy improves as the cold plate thermal mass decreases relative to the flow-rate-driven advective term.

**Note on pipe heat loss:** Heat exchange between coolant and ambient through the pipe wall was considered. The pipe surface area ($A_{pipe} = \pi D L \approx 0.03$ m²) is negligible compared to the cold plate effective area, and the pipe is thermally insulated. This term is linear and adds no complexity, but its contribution is less than 5% of cold plate heat transfer and is therefore neglected.

---

## Entropic Coefficient Assumptions

The entropic coefficient $\frac{\partial U_{OCV}}{\partial T_{cell}}$ is a function of both SoC and $T_{cell}$ in general:

$$\frac{\partial U_{OCV}}{\partial T_{cell}} = f(SoC, T_{cell})$$

**Assumption:** Over the normal BTMS operating range of $15°C$ to $45°C$, the temperature dependence of the entropic coefficient is weak. It is therefore modeled as a function of SoC only:

$$\frac{\partial U_{OCV}}{\partial T_{cell}} \approx f(SoC)$$

implemented as a **1D lookup table** scheduled by SoC, provided by the Battery Management System (BMS) at every timestep.

**Justification:** This assumption is consistent with published characterization data for lithium-ion NMC cells in the literature. The temperature sensitivity of the entropic coefficient introduces a secondary nonlinearity ($T_{cell}$ appearing inside the coefficient which multiplies $T_{cell}$) that is negligible over the operating range relative to the primary thermal dynamics.

**Extension:** A 2D lookup table scheduled by both SoC and $T_{cell}$ represents a natural extension for higher-fidelity applications.

**SoC dynamics** are governed by Coulomb counting:

$$\dot{SoC} = -\frac{I}{Q_{nom}}$$

SoC is estimated by the BMS via Coulomb counting or Kalman filtering and provided as a scheduled parameter — not a state predicted by the BTMS-MPC.

---

## Actuator Chain and Manipulated Variable

### Why $\dot{m}_{cool}$ and not $h_{rad}$

The MPC manipulated variable is **coolant mass flow rate** $\dot{m}_{cool}$ [kg/s], commanded via pump voltage. The full actuator chain is:

$$V_{pump} \xrightarrow{K_\omega} \omega_{pump} \xrightarrow{K_Q} Q_{cool} \xrightarrow{\rho_{cool}} \dot{m}_{cool}$$

This chain is approximately linear in the operating range, making $\dot{m}_{cool}$ a clean MPC manipulated variable. The pump voltage-to-flow-rate map is inverted outside the MPC as a static actuator block.

### Bilinearity of the Manipulated Variable

The coolant equation contains the term:

$$\dot{m}_{cool} \cdot c_{p,cool} \cdot (T_{cool} - T_{cool,in})$$

This is a **product of the manipulated variable and a state** — a bilinear term. Even with $\dot{m}_{cool}$ as MV, the system is not linear in the standard sense:

$$\dot{x} \neq Ax + Bu \quad \text{(exactly)}$$

because $B$ would depend on $T_{cool}$ and $A$ would depend on $\dot{m}_{cool}$.

**Resolution:** Taylor linearization around the operating point — identical procedure to the ACC project where the nonlinear drag term $v^2$ was linearized. After linearization, the bilinear product contributes to both the $A$ and $B$ matrices as constant terms evaluated at the operating point.

**Alternative considered and rejected:** Input transformation $v = \dot{m}_{cool}(T_{cool} - T_{cool,in})$ would linearize the equation exactly. Rejected because:
1. $v$ is not physically commandable
2. Recovering $\dot{m}_{cool}$ requires dividing by $(T_{cool} - T_{cool,in})$ — undefined when $T_{cool} = T_{cool,in}$ (near equilibrium)
3. Constraint specification on $\dot{m}_{cool}$ becomes non-trivial
4. The inversion block becomes singular near equilibrium — unacceptable in a safety-critical system

---

## Complete Two-State Nonlinear Model

The complete nonlinear system before linearization:

$$\dot{T}_{cell} = \frac{1}{m_{cell}c_{p,cell}}\left(I^2 R_{int} + T_{cell} \cdot I \cdot \frac{\partial U_{OCV}}{\partial T_{cell}}\bigg|_{SoC} - \frac{T_{cell} - T_{cool}}{R_{th}}\right)$$

$$\dot{T}_{cool} = \frac{1}{m_{cool}c_{p,cool}}\left(\frac{T_{cell} - T_{cool}}{R_{th}} - \dot{m}_{cool} c_{p,cool}(T_{cool} - T_{cool,in})\right)$$

**System variables:**

| Symbol | Role | Source |
|--------|------|--------|
| $x = [T_{cell}, T_{cool}]^T$ | States | Modeled / Measured |
| $u = \dot{m}_{cool}$ | Manipulated variable | MPC output → pump |
| $d = [I, T_{cool,in}]^T$ | Measured disturbances | BMS + chiller outlet sensor |
| $T_{amb}$ | Environmental input | Ambient sensor |
| $SoC$ | Scheduled parameter | BMS (Coulomb counting / Kalman) |

---

## Operating Point Selection

The MPC prediction model is derived by linearizing the nonlinear system around a chosen operating point. The operating point represents the **dominant steady-state condition** the system is designed around.

### Operating Point Justification

| Variable | Value | Physical Justification |
|----------|-------|----------------------|
| $T_{cell,op}$ | 35°C | Center of optimal performance window (25°C–40°C) |
| $T_{cool,op}$ | 28°C | 7°C below $T_{cell,op}$ — sufficient driving temperature difference for cold plate heat transfer; satisfies $T_{amb} < T_{cool} < T_{cell}$ |
| $T_{cool,in,op}$ | 20°C | Chiller outlet at moderate load — decoupled from ambient by refrigeration circuit |
| $T_{amb,op}$ | 30°C | European summer design scenario |
| $I_{op}$ | ~1 A per cell | Highway cruise: $P_{cruise} \approx 20$ kW, $V_{pack} = 400$ V → $I_{pack} = 50$ A; with $N_p \approx 50$ parallel strings → $I_{cell} \approx 1$ A (C-rate ≈ 0.33C for 3 Ah cell — physically reasonable for sustained cruise) |
| $\dot{m}_{cool,op}$ | TBD from steady-state | From $\dot{T}_{cool}=0$ condition at operating point |

### Temperature Ordering Constraint

The three temperatures must always satisfy:

$$T_{cool,in} < T_{cool} < T_{cell}$$

This is a consequence of the **second law of thermodynamics** — heat flows from hot to cold. Violation of this ordering means the coolant is absorbing heat from both sides simultaneously, which is physically impossible in a passive heat exchange configuration. The operating point was verified to satisfy this constraint.

### Steady-State Consistency Check

At the operating point, $\dot{T}_{cool}\big|_{op} = 0$:

$$0 = \frac{T_{cell,op} - T_{cool,op}}{R_{th}} - \dot{m}_{cool,op} c_{p,cool}(T_{cool,op} - T_{cool,in,op})$$

This equation determines the required $\dot{m}_{cool,op}$ given the thermal resistance and operating temperatures — ensuring the operating point is physically achievable.

### Gain Scheduling

The prediction model is valid in a **neighbourhood** of the operating point. For large temperature excursions, a single linearization introduces significant model error. A **gain-scheduled** approach is implemented via Stateflow, with **three linearization points** corresponding to the three thermal operating modes:

| Model | Linearization point | Active in mode |
|-------|--------------------|--------------------|
| **Model 1** | $T_{cell,op,1} = 25°C$ | Normal (cold regime) |
| **Model 2** | $T_{cell,op,2} = 35°C$ | Normal / Warning |
| **Model 3** | $T_{cell,op,3} = 50°C$ | Warning (upper) / Critical |

Stateflow switches between prediction models based on measured $T_{cell}$, identical in concept to the mode-switching architecture of the ACC project. Three linearization points are required — not two — because the system spans a 30°C range across its three modes, and a single Warning/Critical model at 35°C would carry unacceptable linearization error at 50°C+ where the MPC prediction quality is most critical.

---

## Linearization

### Taylor Expansion

The nonlinear system $\dot{x} = f(x, u, d)$ is linearized via first-order Taylor expansion around $(x_{op}, u_{op}, d_{op})$:

$$\Delta\dot{x} = A\Delta x + B\Delta u + E\Delta d$$

Where $\Delta(\cdot) = (\cdot) - (\cdot)_{op}$ denotes deviation from operating point.

### Jacobian Matrices

**A matrix** (state Jacobian):

$$A = \begin{bmatrix}
\dfrac{I_{op}\frac{\partial U}{\partial T_{cell}} - \frac{1}{R_{th}}}{m_{cell}c_{p,cell}} & \dfrac{1}{m_{cell}c_{p,cell}R_{th}} \\[12pt]
\dfrac{1}{m_{cool}c_{p,cool}R_{th}} & -\dfrac{1}{m_{cool}c_{p,cool}R_{th}} - \dfrac{\dot{m}_{cool,op}}{m_{cool}}
\end{bmatrix}$$

**B matrix** (input Jacobian):

$$B = \begin{bmatrix}
0 \\[6pt]
-\dfrac{T_{cool,op} - T_{cool,in,op}}{m_{cool}}
\end{bmatrix}$$

*$B_1 = 0$: Mass flow rate does not directly affect cell temperature — it acts through $T_{cool}$ only.*

**E matrix** (disturbance Jacobian):

$$E = \begin{bmatrix}
\dfrac{2I_{op}R_{int} + T_{cell,op}\frac{\partial U}{\partial T_{cell}}}{m_{cell}c_{p,cell}} & 0 \\[12pt]
0 & \dfrac{\dot{m}_{cool,op}}{m_{cool}}
\end{bmatrix}$$

*$E_{12} = 0$: Chiller inlet temperature has no direct path to cell dynamics.*
*$E_{21} = 0$: Current has no direct path to coolant dynamics.*

### Physical Interpretation of Zero Entries

The sparsity structure of $B$ and $E$ encodes the **physical topology** of the system:
- Cell temperature is only affected by current (through $E_{11}$) and by coolant temperature (through $A_{12}$) — not directly by pump speed
- Coolant temperature is only affected by pump speed (through $B_2$) and chiller inlet (through $E_{22}$) — not directly by current
- This clean separation reflects the physical decoupling between the electrical and hydraulic circuits

---

## MPC Formulation

### Prediction Model

The MPC uses the linearized state-space model:

$$\Delta\dot{x} = A\Delta x + B\Delta u + E\Delta d$$
$$\Delta y = C\Delta x, \quad C = I_2$$

Discretized at sample time $T_s$ using zero-order hold.

### Cost Function

$$J = \sum_{k=1}^{N_p} \|\Delta y(k)\|_W^2 + \sum_{k=0}^{N_c-1} \|\Delta u(k)\|_R^2$$

### Constraints

| Constraint | Physical meaning |
|-----------|-----------------|
| $T_{cell} \leq 45°C$ (soft) | Upper bound of acceptable window |
| $T_{cell} \leq 60°C$ (hard) | Thermal runaway prevention |
| $T_{cell} \geq 15°C$ (soft) | Lower bound — lithium plating prevention |
| $\dot{m}_{cool} \in [\dot{m}_{min}, \dot{m}_{max}]$ | Pump physical limits |
| $\Delta\dot{m}_{cool} \leq \Delta\dot{m}_{max}$ | Rate of change limit — pump wear |

### Horizon Justification

*(To be completed during MPC design phase — following same methodology as ACC project: worst-case thermal event duration drives $N_p$; control maneuver time drives $N_c$.)*

---

## Stateflow Thermal Mode Logic

A hierarchical Stateflow state machine manages three operating modes:

| Mode | Condition | MPC Objective | Action |
|------|-----------|---------------|--------|
| **Normal** | $T_{cell} \leq 40°C$ | Track $T_{cell,ref} = 35°C$ | Low pump speed, Model 1/2 |
| **Warning** | $40°C < T_{cell} \leq 55°C$ | Aggressive cooling | High pump speed, Model 2/3 |
| **Critical** | $T_{cell} > 55°C$ | Emergency — maximize cooling | Max pump speed, Model 3, alert |

**Note on threshold selection:** The mode-switching thresholds (40°C and 55°C) are set **below** the physical zone boundaries defined in the Safe Operating Window table (45°C and 60°C respectively). This is intentional — the thresholds provide a **5°C buffer** on each boundary, ensuring that:
1. The MPC is not operating in Normal mode right up to the edge of the acceptable zone, leaving no room to react to load transients
2. The switch to more aggressive control and the tighter linearization model (Model 3) occurs while the system still has thermal headroom — not after it has already entered the danger zone
3. A sensor failure or measurement lag does not delay a mode transition until the physical limit is already exceeded

Mode switching changes:
- MPC output weights (prioritize $T_{cell}$ tracking vs energy efficiency)
- Active linearization model (gain scheduling across Model 1, 2, 3)
- Constraint tightening (reduced $T_{cell}$ upper bound in Warning mode to create buffer)

---

## Safety Constraints and Thermal Runaway

**Hard constraint:** $T_{cell} \leq 60°C$ at all times. Implemented as a hard output constraint in the MPC QP formulation — the optimizer will never produce a solution that violates this bound, at the cost of relaxing other objectives if necessary.

**Soft constraint:** $T_{cell} \leq 45°C$ with a slack variable — violations are penalized but permitted transiently during aggressive load transients.

**Why the hard constraint is set at 60°C and not higher:** Thermal runaway onset temperature for NMC cells is typically reported in the range of $80°C$–$150°C$ depending on SoC and cell chemistry. The 60°C hard constraint provides a **20°C minimum safety margin** above the optimal window, accounting for:
- Model uncertainty from lumped capacitance assumption ($Bi = 2.25$ implies non-trivial core-to-surface temperature difference — the surface measurement used by the MPC underestimates the core temperature, so the effective margin at the core is smaller than 20°C)
- Sensor measurement delay and noise
- MPC prediction horizon limitations during extreme transients

---

## Design Assumptions Summary

| # | Assumption | Justification | Impact if violated |
|---|-----------|--------------|-------------------|
| 1 | Lumped capacitance for cell | $Bi = 2.25$ — invalid strictly; standard in BTMS literature | MPC underestimates peak core temperature; compensated by conservative hard constraint at 60°C |
| 2 | Radiation neglected | 1.25% of convective term at operating conditions | Negligible |
| 3 | Pipe heat loss neglected | <5% of cold plate heat transfer; pipe insulated | Negligible |
| 4 | Entropic coefficient $= f(SoC)$ only | Weak $T_{cell}$ dependence over 15°C–45°C range | ~Secondary error in $\dot{Q}_{ent}$ |
| 5 | Entropic term retained (not neglected) | 12% of Joule heating; sign-dependent error across charge/discharge without it | Charge: underestimates heat; Discharge: overestimates — bidirectional error unacceptable in dual-use model |
| 6 | Plug-flow, uniform coolant temperature | Flow dynamics much faster than thermal dynamics; outlet temperature used as representative state | Overestimates coolant thermal mass at low flow; gradient along flow path collapsed to single state |
| 7 | $T_{cool,in}$ as measured disturbance | Chiller dynamics modeled separately by refrigeration controller | Chiller transients appear as disturbance steps |
| 8 | Three linearizations, one per Stateflow mode | Valid in neighbourhood of each $T_{op}$; three points span full 25°C–55°C operating range | Model error increases far from each operating point |
| 9 | $\dot{m}_{cool}$ as MV (not $V_{pump}$) | Linear actuator map in operating range | Actuator nonlinearity introduces small error |

---

## Vehicle and System Parameters

*(To be populated during Simscape implementation using Incropera, EES fluid property tables, and published 18650 cell datasheets.)*

| Parameter | Symbol | Value | Unit | Source |
|-----------|--------|-------|------|--------|
| Cell mass | $m_{cell}$ | TBD | kg | Datasheet |
| Cell specific heat | $c_{p,cell}$ | TBD | J/kgK | Literature |
| Cell internal resistance | $R_{int}$ | TBD | Ω | Datasheet |
| Cell capacity | $Q_{nom}$ | 3 | Ah | 18650 standard |
| TIM thermal resistance | $R_{TIM}$ | TBD | K/W | Datasheet |
| Cold plate thermal resistance | $R_{cold plate}$ | TBD | K/W | First principles |
| Total thermal resistance | $R_{th} = R_{TIM} + R_{cold plate}$ | TBD | K/W | — |
| Coolant mass (cold plate) | $m_{cool}$ | TBD | kg | Geometry |
| Coolant specific heat (50% glycol) | $c_{p,cool}$ | TBD | J/kgK | EES |
| Pack voltage | $V_{pack}$ | 400 | V | Typical BEV |
| Highway cruise power | $P_{cruise}$ | 20,000 | W | Literature |
| Number of parallel strings | $N_p$ | ~50 | — | Derived |
| Cell current at cruise | $I_{op}$ | ~1 | A | Derived (0.33C) |

---

## How to Run

### Prerequisites
- MATLAB R2024b
- Simulink
- Simscape
- Simscape Thermal
- Model Predictive Control Toolbox
- Stateflow

### Steps

```matlab
% 1. Navigate to project folder
cd('path/to/btms-project')

% 2. Load system parameters
run('scripts/parameters.m')

% 3. Build linearized prediction model and MPC object
run('scripts/MPC_thermal_model.m')

% 4. Open and run Simulink model
open_system('model/BTMS.slx')
sim('model/BTMS.slx')
```

---

## Project Structure

```
btms-mpc/
├── model/
│   └── BTMS.slx                    # Complete Simulink/Simscape model
├── scripts/
│   ├── parameters.m                # System and MPC parameters
│   └── MPC_thermal_model.m         # Linearized state-space and MPC object
├── lookup_tables/
│   └── entropic_coefficient.mat    # dU/dT vs SoC lookup table
├── images/
│   ├── stateflow_chart.png
│   ├── simscape_plant.png
│   └── results/
└── README.md
```

---

## Author

**Hassan Gholami**
Control & Automation Engineer — Milan, Italy
[LinkedIn](https://linkedin.com/in/gholami-hassan) | [Email](mailto:m.gholami.derouei@gmail.com)

## Status
Work in progress
