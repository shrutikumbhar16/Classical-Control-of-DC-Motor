# Classical-Control-of-DC-Motor
Python GUI simulation of a DC motor comparing P, PI, PD, and PID controllers using speed feedback, with plots of angular speed, position, and armature current.



## Overview

This project implements a Python GUI simulation of a DC motor to study and compare four feedback controllers: Proportional (P), Proportional-Integral (PI), Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID).

The simulation allows users to change motor inductance and controller gains to observe their effects on the motor's dynamic response.

## Features

- Interactive graphical user interface (GUI).
- Adjustable armature inductance (La).
- Adjustable controller gains: Kp, Ki, and Kd.
- Comparison of P, PI, PD, and PID controllers.
- Plots of three motor state variables:
  - Angular speed (ω)
  - Angular position (θ)
  - Armature current (ia)
- Numerical simulation of the DC motor's electrical and mechanical dynamics.

## Motor Model

The motor is described by the following electrical equation:

Va(t) = Ra ia(t) + La dia(t)/dt + Kb ω(t)

The mechanical equation is:

J dω(t)/dt + Bω(t) = Kt ia(t) - TL(t)

Angular position is calculated using:

dθ(t)/dt = ω(t)

Where:

- Va: Armature voltage
- Ra: Armature resistance
- La: Armature inductance
- ia: Armature current
- Kb: Back-EMF constant
- Kt: Motor torque constant
- ω: Angular speed
- θ: Angular position
- J: Moment of inertia
- B: Viscous friction coefficient
- TL: Load torque

## Transfer Function

The voltage-to-speed transfer function is:

G(s) = Ω(s) / Va(s)

G(s) = Kt / [(La s + Ra)(J s + B) + Kt Kb]

The voltage-to-position transfer function is:

Gθ(s) = G(s) / s

For the nominal motor parameters used in the simulation, the transfer function can be calculated by substituting the corresponding parameter values.

## Controllers

The simulation compares the following controllers:

- **P:** C(s) = Kp
- **PI:** C(s) = Kp + Ki/s
- **PD:** C(s) = Kp + Kd s
- **PID:** C(s) = Kp + Ki/s + Kd s

For unity negative feedback, the closed-loop transfer function is:

T(s) = C(s)G(s) / [1 + C(s)G(s)]

The feedback variable is angular speed.

## Requirements

- Python 3
- NumPy
- Matplotlib
- Tkinter (usually included with standard Windows Python installations)

## Installation

Install Python if it is not already installed.

Install the required Python libraries from the terminal:

```bash
py -m pip install numpy matplotlib
```

## How to Run

1. Download or clone this repository.
2. Open the project folder in VS Code.
3. Open the terminal in the project folder.
4. Run the application:

```bash
py CSPML_LAB_7_28.py
```

5. Enter the desired motor inductance, controller gains, and simulation duration.
6. Run the simulation to compare the responses of the four controllers.

## Understanding the Plots

- **Angular Speed:** Shows how the motor speed responds to the reference input.
- **Angular Position:** Shows the accumulated angular displacement obtained by integrating speed.
- **Armature Current:** Shows the electrical current response and helps compare current demand across controllers.

Changing motor inductance and controller gains affects tracking performance, transient response, and current demand.

## Assumptions

- The motor is represented by a linear electrical and mechanical model.
- Motor parameters are treated as constant.
- Load torque is assumed to be zero for the nominal model.
- Initial conditions are zero.

## Technologies Used

- Python
- NumPy
- Matplotlib
- Tkinter



