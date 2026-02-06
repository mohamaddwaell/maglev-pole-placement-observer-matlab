# Magnetic Levitation System – Digital Control and Observer Design

## Overview
This project investigates the performance of a magnetic levitation system
under discrete-time state feedback and observer-based control.
The effects of sampling time, state/output noise, and digital implementation
on a continuous-time nonlinear plant are analyzed.

## System Description
A magnetically suspended ball is modeled using nonlinear dynamics derived
from electromagnetic force balance and electrical circuit equations.
The system is linearized around an equilibrium operating point.

State vector:
x = [Δh, Δḣ, Δi]ᵀ

## Control Strategy
- Continuous-time model linearization
- Zero-order-hold discretization with multiple sampling periods
- Discrete pole placement via state feedback
- Digital controller applied to continuous plant
- Full-order discrete-time observer design

## Noise Analysis
The impact of:
- Process noise (state disturbances)
- Measurement noise (output disturbances)

on both closed-loop performance and observer accuracy is evaluated.

## Key Observations
- Smaller sampling periods better approximate continuous dynamics
- Discrete controllers degrade performance when applied to continuous plants
  as sampling time increases
- Observer convergence strongly depends on pole placement and noise level
