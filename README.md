# PID-Controlled DC Motor Speed Control
### MATLAB & Simulink Simulation | Control Systems | Signal Analysis

---

## Project Overview

This project models a DC motor as a second-order dynamic system and designs a PID controller to regulate its angular speed to a desired setpoint. The work covers the full control engineering workflow — from mathematical modelling and open-loop analysis through to closed-loop design, performance quantification, frequency-domain stability verification, and disturbance rejection testing.

**Tools used:** MATLAB R2024a, Simulink, Control System Toolbox  
**Domain:** Control Systems, Signal Processing, Power Electronics  
**Target applications:** Motor drives, robotics, industrial automation, EV traction systems

---

## Results Summary

| Metric | Value |
|---|---|
| Target speed | 10 rad/s |
| Rise time | 0.132 s |
| Settling time | 0.257 s |
| Overshoot | 1.03 % |
| Steady-state error | ~0 % |
| Phase margin | ~95° |
| Gain margin | > 20 dB |

---

## System Model

The DC motor is modelled from first principles using its electrical (armature) and mechanical equations:

**Electrical:** `V = L·(di/dt) + R·i + Kₑ·ω`  
**Mechanical:** `J·(dω/dt) = Kₜ·i − B·ω`

Applying the Laplace transform yields the second-order transfer function:

```
G(s) =           Kₜ
       ─────────────────────────────
       (Js + B)(Ls + R) + Kₑ·Kₜ
```

Which simplifies with the given parameters to:

```
G(s) =          0.01
       ─────────────────────────────
       0.005s² + 0.06s + 0.1001
```

**Motor parameters used:**

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Armature resistance | R | 1 | Ω |
| Armature inductance | L | 0.5 | H |
| Torque constant | Kₜ | 0.01 | N·m/A |
| Back-EMF constant | Kₑ | 0.01 | V·s/rad |
| Rotor inertia | J | 0.01 | kg·m² |
| Viscous friction | B | 0.1 | N·m·s/rad |

The system is second-order because two energy storage elements exist — inductance L (electrical) and inertia J (mechanical).

---

## PID Controller Design

The closed-loop structure:

```
Setpoint → [Σ] → [PID] → [Motor G(s)] → Speed output
              ↑                               |
              └──────── feedback ─────────────┘
```

**Controller gains:**

| Gain | Value | Role |
|---|---|---|
| Kp = 100 | Proportional | Fast response to current error |
| Ki = 200 | Integral | Eliminates steady-state error |
| Kd = 10 | Derivative | Damps overshoot, predicts future error |

**Tuning logic:**
1. Set Ki = Kd = 0. Increase Kp until fast rise without excessive overshoot → Kp = 100
2. Add Ki and increase until steady-state error reaches zero → Ki = 200  
3. Add Kd to suppress remaining overshoot from integral action → Kd = 10

---

## Figures

### Figure 1 — Open-loop step response
![Open loop](figures/P1_openloop.png)

The uncontrolled motor settles at ~0.1 rad/s from a 1V step input. DC gain = Kₜ/(BR + KₑKₜ) ≈ 0.0999. This establishes the baseline problem — the motor is slow, low-gain, and cannot track a setpoint without a controller.

---

### Figure 2 — PID tuning comparison (P vs PI vs PID)
![Comparison](figures/P1_comparison.png)

Three controllers on the same plant isolate the contribution of each term:
- **P only:** Fast rise but settles at ~9.5 rad/s — 5% steady-state error is mathematically guaranteed since the proportional force approaches zero as error reduces
- **PI:** Eliminates offset completely (integral forces exact tracking) but causes ~35% overshoot and two oscillation cycles
- **Full PID:** Rise time 0.132s, overshoot 1.03%, settling time 0.257s — derivative term brakes before overshoot occurs

---

### Figure 3 — Disturbance rejection
![Disturbance](figures/P1_disturbance.png)

A load torque disturbance is injected at t = 5s. The output speed shows negligible deviation and recovers immediately. The integral term continuously accumulates new error caused by the disturbance and corrects it before it builds. This property is critical in real applications under variable load — conveyor systems, CNC axes, EV drivetrains.

---

### Figure 4 — Open-loop Bode plot with stability margins
![Bode](figures/P1_bode.png)

Gain crossover frequency ≈ 10 rad/s. Phase at crossover ≈ −85°, giving a phase margin of approximately 95°. This is well above the minimum acceptable margin of 45°, confirming robust stability. The system can tolerate significant parameter variation without going unstable.

---

### Figure 5 — Root locus
![Root locus](figures/P1_rootlocus.png)

Both closed-loop poles are located in the left-half s-plane at approximately −6 on the real axis, confirming stability. The distance from the imaginary axis determines how quickly disturbances decay — larger negative real part means faster decay.

---

## Key Engineering Insights

**Why not P-only?** Proportional control cannot eliminate steady-state error. As the motor approaches setpoint, the error shrinks, so the corrective force shrinks — the output always settles slightly below target.

**Why does integral cause overshoot?** The integrator accumulates energy during the rise. When the output reaches setpoint the integrator has stored significant energy and continues pushing — causing overshoot. Derivative action counteracts this by detecting the rapid approach and applying a predictive brake.

**Simulation limitations:**
- Model assumes linear motor behaviour — real motors have friction nonlinearities, magnetic saturation, and backlash not captured by the transfer function
- Ideal sensor feedback assumed — real hardware introduces measurement noise and delay, which severely degrades derivative term performance
- No actuator saturation modelling beyond basic clamping — integral windup during saturation requires dedicated anti-windup logic

---

## File Structure

```
PID_Motor_Control/
├── README.md
├── pid_motor_control.m       ← main MATLAB script
├── motor_simulink.slx        ← Simulink closed-loop model
└── figures/
    ├── P1_openloop.png
    ├── P1_comparison.png
    ├── P1_disturbance.png
    ├── P1_bode.png
    └── P1_rootlocus.png
```

---

## How to Run

1. Open MATLAB R2024a or later
2. Run `pid_motor_control.m` — generates all 5 figures and prints performance metrics to Command Window
3. Open `motor_simulink.slx` in Simulink — run simulation to see closed-loop step response

---

## Author

**Nakul**  
B.E. Electrical and Electronics Engineering  
IEEE Member | Embedded Systems | Power Electronics | Control Systems
