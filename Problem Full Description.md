# Magnetic Levitation System – Digital Control and Observer Design

## Overview
This project investigates the performance of a **magnetic levitation system** under discrete-time state feedback and observer-based control. The main goal is to study the effect of **state and output noise** on the system's performance when digital controllers are applied to a continuous-time plant.  

The system chosen is a **magnetically suspended ball**, a classic magnetic levitation example with practical applications in high-speed trains. The study covers modeling, discretization, state feedback design, observer design, and noise analysis.

---

## System Description
The magnetic levitation system consists of a ball suspended in midair by an electromagnetic coil. The coil current produces a magnetic force that counteracts gravity, allowing precise vertical positioning.  

**Variables:**
- $h$ : Vertical position of the ball (m)  
- $i$ : Current through the electromagnet (A)  
- $V$ : Applied voltage (control signal) (V)  
- $M$ : Mass of the ball (kg)  
- $g$ : Gravity (m/s²)  
- $L$ : Inductance (H)  
- $R$ : Resistance (Ω)  
- $K$ : Magnetic force coefficient  

**Parameter values:**
- $M = 0.05\ \mathrm{kg}$  
- $K = 0.0001$  
- $L = 0.01\ \mathrm{H}$  
- $R = 1\ \Omega$  
- $g = 9.81\ \mathrm{m/s^2}$  

**Equilibrium condition:**

$$
h_\text{eq} = \frac{K i_\text{eq}^2}{Mg}, \quad \frac{dh}{dt} = 0
$$

The system is linearized about $h = 0.01\,\mathrm{m}$ (nominal current ≈ 7 A), resulting in the following **state-space model**:

$$
\dot{x} = A x + B u, \quad y = C x
$$

where

$$
x = 
\begin{bmatrix} 
\Delta h \\ 
\Delta \dot{h} \\ 
\Delta i 
\end{bmatrix}, \quad
A = 
\begin{bmatrix} 
0 & 1 & 0 \\ 
980 & 0 & -2.8 \\ 
0 & 0 & -100 
\end{bmatrix}, \quad
B = 
\begin{bmatrix} 
0 \\ 0 \\ 100 
\end{bmatrix}, \quad
C = 
\begin{bmatrix} 
1 & 0 & 0 
\end{bmatrix}
$$

---

## Project Requirements & Implementation

### a) Nonlinear Model and Linearization
- Deduce the nonlinear state-space equations of the magnetic levitation system.  
- Linearize the system using **Taylor Series Expansion** around the equilibrium point.  
- Verify that the resulting matrices match the linearized model above.  

### b) Discretization
- Discretize the linear continuous-time model using **three different sampling periods** ($T_s = 0.001, 0.005, 0.01$ s) using **zero-order hold**.  

### c) Continuous vs Discrete Simulation
- For each discrete system, simulate and plot $x(kT)$ versus $k$ under a **step input**.  
- Compare discrete responses to the continuous-time system response.

### d) Discrete State Feedback Design
- Select appropriate **pole locations** for each discrete system.  
- Compute the **state feedback gains** using pole placement.  

### e) Closed-loop Continuous Response
- Apply each **discrete feedback gain** to the continuous system.  
- Simulate the **continuous-time step response** and evaluate system performance.

### f) Noisy System Analysis
- Introduce **state and/or output noise**.  
- Study the impact on system performance under state feedback control.

### g) Full-order Observer Design
- For each discrete system, design a **full-order observer**.  
- Verify system performance with **observer-based state feedback**.

### h) Observer applied to Continuous System
- Apply each discrete observer and its corresponding feedback gain to the continuous system.  
- Simulate the **continuous-time step response** and analyze observer accuracy.

### i) Noisy Observer-based System
- Study the effect of **state and output noise** on the observer-based closed-loop system.  

---

