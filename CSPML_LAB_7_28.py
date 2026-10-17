"""Lab 7: DC motor feedback simulation with P, PI, PD and PID.

Model:
  La * dia/dt = Va_motor - Ra*ia - Kb*omega
  J  * domega/dt = Kt*ia - B*omega - TL
  dtheta/dt = omega

The feedback model uses a unit-step speed reference r(t)=1*u(t), with speed
as the only feedback signal. Separately, the open-loop motor voltage step is
Va(t)=1*u(t) V, whose Laplace transform is Va(s)=1/s. In closed loop, the
controller output becomes the motor-terminal voltage, so it cannot also be
held fixed at 1 V.

Only La, Kp, Ki, Kd and simulation duration are editable. DT and other motor
parameters are fixed. No pre-built step-response function is used.
"""
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Fixed motor parameters (SI units)
RA = 2.0
J = 0.02
B = 0.01
KT = 0.1
KB = 0.1
TL = 0.0
DT = 0.001
# Explicit 1 V armature-voltage unit step for the OPEN-LOOP motor model:
VA_STEP = 1.0  # Va(t) = 1*u(t) V, hence Va(s) = 1/s

# Unit-step reference used at the summing junction in the CLOSED-LOOP model.
# This is a speed reference (rad/s), not the physical armature voltage.
SPEED_REFERENCE = 1.0
CONTROLLERS = ("P", "PI", "PD", "PID")


def motor_tf_coefficients(la):
    """Full motor transfer functions from armature voltage to speed/position.

    Electrical: Va(s) = (La*s + Ra)*Ia(s) + Kb*Omega(s)
    Mechanical: (J*s + B)*Omega(s) = Kt*Ia(s) - TL(s)

    With TL(s)=0:
      G_speed(s) = Omega(s)/Va(s)
                 = Kt / [(La*s + Ra)(J*s + B) + Kt*Kb]
                 = Kt / [a2*s^2 + a1*s + a0]
      G_position(s) = Theta(s)/Va(s) = G_speed(s)/s
    """
    return la * J, la * B + RA * J, RA * B + KT * KB


def simulate(la, duration, controller, kp, ki, kd):
    """Forward-Euler integration of motor states with speed feedback."""
    n_steps = max(1, int(np.ceil(duration / DT)))
    t = np.linspace(0.0, duration, n_steps + 1)
    h = t[1] - t[0]
    n = len(t)

    current = np.zeros(n)
    speed = np.zeros(n)
    position = np.zeros(n)
    motor_voltage = np.zeros(n)
    error_integral = 0.0
    previous_error = SPEED_REFERENCE

    for k in range(n - 1):
        # Unit-step signal is applied at the summing junction.
        error = SPEED_REFERENCE - speed[k]
        error_integral += error * h
        # Zero initial derivative avoids a numerical derivative kick at t=0.
        derivative = 0.0 if k == 0 else (error - previous_error) / h

        p = kp * error
        i = ki * error_integral
        d = kd * derivative
        if controller == "P":
            motor_voltage[k] = p
        elif controller == "PI":
            motor_voltage[k] = p + i
        elif controller == "PD":
            motor_voltage[k] = p + d
        else:
            motor_voltage[k] = p + i + d

        di_dt = (motor_voltage[k] - RA * current[k] - KB * speed[k]) / la
        domega_dt = (KT * current[k] - B * speed[k] - TL) / J
        current[k + 1] = current[k] + h * di_dt
        speed[k + 1] = speed[k] + h * domega_dt
        position[k + 1] = position[k] + h * speed[k]
        previous_error = error

    motor_voltage[-1] = motor_voltage[-2]
    if not all(np.all(np.isfinite(x)) for x in (current, speed, position, motor_voltage)):
        raise FloatingPointError(
            f"{controller} response became non-finite. Reduce the gains or simulation time."
        )
    return t, speed, position, current, motor_voltage


class MotorGUI:
    def __init__(self, root):
        self.root = root
        root.title("Lab 7 — DC Motor Feedback Study")
        root.geometry("1100x850")
        root.minsize(900, 680)
        self.inputs = {
            "La (H)": tk.StringVar(value="0.5"),
            "Kp": tk.StringVar(value="0.6"),
            "Ki": tk.StringVar(value="0.3"),
            "Kd": tk.StringVar(value="0.09"),
            "Simulation time (s)": tk.StringVar(value="10"),
        }
        self._build_ui()
        self.simulate()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        ttk.Label(main, text="DC Motor Feedback Control",
                  font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(
            main,
            text="Compare P, PI, PD and PID. Fixed unit-step input; speed-only feedback."
        ).pack(anchor="w", pady=(2, 8))

        params = ttk.LabelFrame(main, text="Tunable parameters", padding=8)
        params.pack(fill="x")
        for col, (label, var) in enumerate(self.inputs.items()):
            cell = ttk.Frame(params)
            cell.grid(row=0, column=col, padx=7, pady=4, sticky="ew")
            ttk.Label(cell, text=label).pack(anchor="w")
            ttk.Entry(cell, textvariable=var, width=13).pack(anchor="w")
            params.columnconfigure(col, weight=1)

        button_row = ttk.Frame(params)
        button_row.grid(row=1, column=0, columnspan=len(self.inputs), sticky="ew", pady=(8, 0))
        ttk.Label(button_row, text="Fixed: Ra, J, B, Kt, Kb, TL and dt = 0.001 s").pack(side="left")
        ttk.Button(button_row, text="Simulate", command=self.simulate).pack(side="right")

        self.status = ttk.Label(main, text="Ready.")
        self.status.pack(anchor="w", pady=6)

        self.fig, self.axes = plt.subplots(3, 1, figsize=(9.5, 6.4), sharex=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=main)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        ttk.Label(
            main,
            text="Outputs: speed ω (rad/s), position θ (rad), armature current ia (A). "
                 "Motor voltage is calculated by the selected controller and printed in the terminal.",
            wraplength=1050
        ).pack(anchor="w", pady=(4, 0))

    def simulate(self):
        try:
            la = float(self.inputs["La (H)"].get())
            kp = float(self.inputs["Kp"].get())
            ki = float(self.inputs["Ki"].get())
            kd = float(self.inputs["Kd"].get())
            duration = float(self.inputs["Simulation time (s)"].get())
            values = np.array([la, kp, ki, kd, duration])
            if not np.all(np.isfinite(values)):
                raise ValueError("Enter finite numeric values.")
            if la <= 0:
                raise ValueError("La must be greater than zero.")
            if min(kp, ki, kd) < 0:
                raise ValueError("Kp, Ki and Kd must be non-negative.")
            if not 0 < duration <= 300:
                raise ValueError("Simulation time must be greater than 0 and at most 300 seconds.")
        except ValueError as exc:
            messagebox.showerror("Invalid parameters", str(exc))
            return

        a2, a1, a0 = motor_tf_coefficients(la)
        print("\n--- DC MOTOR MODEL ---")
        print("Electrical equation: Va(s) = (La*s + Ra)*Ia(s) + Kb*Omega(s)")
        print("Mechanical equation: (J*s + B)*Omega(s) = Kt*Ia(s) - TL(s)")
        print("FULL motor TF: G_speed(s)=Omega(s)/Va(s)=Kt/[(La*s+Ra)(J*s+B)+Kt*Kb]")
        print(f"Expanded G_speed(s) = {KT:g}/({a2:.8g}s^2 + {a1:.8g}s + {a0:.8g})")
        print(f"G_position(s)=Theta(s)/Va(s)=G_speed(s)/s = {KT:g}/({a2:.8g}s^3 + {a1:.8g}s^2 + {a0:.8g}s)")
        print(f"OPEN-LOOP voltage step explicitly: Va(t) = {VA_STEP:g}*u(t) V; Va(s) = {VA_STEP:g}/s = {VA_STEP:g}/s")
        print(f"CLOSED-LOOP reference: r(t) = {SPEED_REFERENCE:g}*u(t); speed feedback gain = 1.")
        print("Important: in closed loop the controller output is the motor voltage; it is not held fixed at 1 V.")
        print("Controller | Final speed(rad/s) | Final position(rad) | Final current(A) | Final motor voltage(V)")
        for ax in self.axes:
            ax.clear()

        line_styles = {
            "P": ("-", 1.8),
            "PI": ("--", 1.8),
            "PD": ("-.", 2.2),
            "PID": (":", 2.4),
        }
        try:
            for name in CONTROLLERS:
                t, speed, position, current, voltage = simulate(
                    la, duration, name, kp, ki, kd
                )
                style, width = line_styles[name]
                self.axes[0].plot(t, speed, label=name, linestyle=style, linewidth=width)
                self.axes[1].plot(t, position, label=name, linestyle=style, linewidth=width)
                self.axes[2].plot(t, current, label=name, linestyle=style, linewidth=width)
                print(f"{name:10s} | {speed[-1]:18.6f} | {position[-1]:20.6f} | "
                      f"{current[-1]:16.6f} | {voltage[-1]:21.6f}")
                indices = sorted(set([0, min(1, len(t)-1), min(10, len(t)-1),
                                      min(100, len(t)-1), len(t)-1]))
                print("  Va samples:", ", ".join(
                    f"t={t[idx]:.3f}s:{voltage[idx]:.6f}V" for idx in indices
                ))
        except (FloatingPointError, OverflowError) as exc:
            messagebox.showerror("Simulation failed", str(exc))
            self.status.config(text="Simulation failed. Try smaller gains.")
            return

        self.axes[0].axhline(SPEED_REFERENCE, color="black", linestyle=":", linewidth=1.1,
                             label="Step level = 1")
        self.axes[0].set_title("Speed response")
        self.axes[0].set_ylabel("Speed ω (rad/s)")
        self.axes[1].set_title("Position (integral of speed)")
        self.axes[1].set_ylabel("Position θ (rad)")
        self.axes[2].set_title("Armature current")
        self.axes[2].set_ylabel("Current ia (A)")
        self.axes[2].set_xlabel("Time (s)")
        for ax in self.axes:
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize="small")
        self.fig.tight_layout()
        self.canvas.draw_idle()
        self.status.config(text="Simulation complete. Compare the four curves in all three plots.")


if __name__ == "__main__":
    root = tk.Tk()
    MotorGUI(root)
    root.mainloop()
