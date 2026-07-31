"""
pid.py — PID on Duckiebot simulation.

"""

import time
import numpy as np
from numpy.linalg import inv, eigvals, matrix_rank, matrix_power
import matplotlib.pyplot as plt

# Parameters (platform_constraints.md)
Kt = 0.478      # N.m/A   torque constant
Ke = 0.478      # V.s/rad back-EMF constant
R  = 5.0        # Ohm     set by order of magnitude
J  = 2.5e-4     # kg.m2   wheel-side inertia
m  = 1.0        # kg      robot mass
r  = 0.033      # m       wheel radius
Iz = 4.1e-3     # kg.m2   yaw inertia (point-mass decomposition)
L  = 0.102      # m       wheel separation
vr = 0.2        # m/s     nominal reference speed (linearization point)

class PID:
    """
    PID controller.
    u(t) = Kp*e + Ki*integral(e) + Kd*de/dt

    """

    def __init__(self, Kp, Ki, Kd, dt,
                 out_min=None, out_max=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max

        self._integral = 0.0
        self._prev_measurement = None
        self._prev_derivative = 0.0
        self.setpoint = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = None
        self._prev_derivative = 0.0

    def _clamp(self, value, lo, hi):
        if lo is not None:
            value = max(lo, value)
        if hi is not None:
            value = min(hi, value)
        return value

    def update(self, measurement, setpoint=None, dt=None):
        if setpoint is not None:
            self.setpoint = setpoint
        dt = dt if dt is not None else self.dt

        error = self.setpoint - measurement

        # proportional term
        p_term = self.Kp * error

        # integral term
        self._integral += error * dt
        i_term = self.Ki * self._integral

        # derivative term, computed on the measurement to avoid
        # derivative kick when the setpoint changes abruptly
        if self._prev_measurement is None:
            raw_derivative = 0.0
        else:
            raw_derivative = -(measurement - self._prev_measurement) / dt

        derivative = raw_derivative

        self._prev_derivative = derivative
        self._prev_measurement = measurement
        d_term = self.Kd * derivative

        output = p_term + i_term + d_term
        clamped = self._clamp(output, self.out_min, self.out_max)

        # back-calculate anti-windup: undo the integral step if saturated
        if clamped != output and self.Ki != 0.0:
            self._integral -= (output - clamped) / self.Ki

        return clamped

def build_matrices(Kt=Kt, Ke=Ke, R=R, J=J, m=m, r=r, Iz=Iz, L=L, vr=vr):
    """Return (A, B) of the linearized model"""
    alpha = m * r**2 / 4
    beta  = Iz * r**2 / L**2

    # Inter-wheel coupling matrix
    M = np.array([[J + alpha + beta, alpha - beta],
                  [alpha - beta,     J + alpha + beta]])
    Minv = inv(M)

    A_omega = -(Kt * Ke / R) * Minv     # acts on [omega_L, omega_R]
    B_omega =  (Kt / R)      * Minv     # acts on [V_L, V_R]

    A = np.zeros((4, 4))
    A[0, 1] = vr
    A[1, 2] = -r / L
    A[1, 3] =  r / L
    A[2, 2], A[2, 3] = A_omega[0]
    A[3, 2], A[3, 3] = A_omega[1]

    B = np.zeros((4, 2))
    B[2, 0], B[2, 1] = B_omega[0]
    B[3, 0], B[3, 1] = B_omega[1]

    return A, B

def draw_simu(t, y, titre="States", xlabel="Time (s)", ylabel=""):
    plt.figure(figsize=(8, 4))
    x_arr = np.array(y)
    for i in range(x_arr.shape[1]):
        plt.plot(t, x_arr[:,i], label=ylabel[i] if ylabel else None)
    plt.legend()
    plt.title(titre)
    plt.grid(True)
    plt.show(block=False)

def closed_loop_cascade(dt:float, x0:list, A:tuple, B:tuple, pid_pos:PID, pid_cap:PID, T=6):
    """
    Cascade regulation.
    - pid_pos  (outer, slow) : e_l → desired heading e_theta_ref
    - pid_cap  (inner, fast) : (e_theta_ref - e_theta) → V_rot
    Setpoint of both loops = 0 for position; the heading follows e_theta_ref.
    """
    x = x0.copy().astype(float)
    x_list, u_list = [], []

    # physical bounds on the desired heading
    THETA_REF_MAX = 1.0   # rad, ~30°

    for i in range(int(T / dt)):
        e_l   = x[0]
        e_th  = x[1]

        # outer loop (position): outputs a desired heading
        # setpoint = 0 (we want e_l = 0)
        e_theta_ref = pid_pos.update(e_l, setpoint=0.0)
        e_theta_ref = max(-THETA_REF_MAX, min(THETA_REF_MAX, e_theta_ref))

        # inner loop (heading): follows e_theta_ref
        # measurement = e_th, setpoint = e_theta_ref
        V_rot = pid_cap.update(e_th, setpoint=e_theta_ref)

        # reconstruct the voltages (deviation from nominal, V_nom=0)
        V_L, V_R = -V_rot, +V_rot
        u = np.array([V_L, V_R])

        # integration
        x_dot = A @ x + B @ u
        x = x + x_dot * dt

        x_list.append(x)
        u_list.append(u)

    t = np.arange(int(T / dt)) * dt
    return t, x_list, u_list

def compute_metrics(t, x_list, u_list, settle_frac=0.05):
    """
    Three controller-comparison metrics, on the regulation scenario.
    - tracking error: mean and max of |e_l|
    - convergence time: instant after which |e_l| stays below settle_frac * |e_l(0)|
    - control effort: RMS of u (aggregated over both wheels)
    """
    x = np.array(x_list)          # (n, 4)
    u = np.array(u_list)          # (n, 2)
    t = np.asarray(t)

    e_l = x[:, 0]                 # lateral error

    # 1. tracking error
    err_mean = np.mean(np.abs(e_l))
    err_max  = np.max(np.abs(e_l))

    # 2. convergence time (settling time)
    band = settle_frac * abs(e_l[0])          # e.g. 5% of the initial error
    outside = np.abs(e_l) > band
    if np.any(outside):
        last_out = np.max(np.where(outside))  # last instant outside the band
        t_settle = t[last_out]
    else:
        t_settle = 0.0                         # never left the band

    # 3. control effort (RMS)
    u_rms = np.sqrt(np.mean(u**2))

    print("err_mean", err_mean)
    print("err_max", err_max)
    print("t_settle", t_settle)
    print("u_rms", u_rms)

    return {
        "err_mean": err_mean,
        "err_max":  err_max,
        "t_settle": t_settle,
        "u_rms":    u_rms,
    }

if __name__ == "__main__":

    plt.ion()  # interactive mode

    # Get A B from model
    A, B = build_matrices()

    #----------------------------------------------------------------#
    #-------------------------- PID INIT-------------------------#
    #----------------------------------------------------------------#
    # Simulate robot with PID
    dt=0.001 # Simulation time
    pid_pos = PID(Kp=3, Ki=0, Kd=0, dt=dt, out_min=-5, out_max=5)
    pid_cac = PID(Kp=3, Ki=0, Kd=0, dt=dt, out_min=-5, out_max=5)

    #----------------------------------------------------------------#
    #---------------------- Simulation init----------------------#
    #----------------------------------------------------------------#
    V_nom = 0 # nominal motor voltage - to be measured for 0.2 m/s
    V_rot = 0 # motor voltage command
    x_state, u_command = [], []
    x_init = np.array([0.1, -0.1, 0, 0]) # Initial state: [e_l, e_θ, ω_L, ω_R]

    #----------------------------------------------------------------#
    #-------------------- Simulation and Graph-------------------#
    #----------------------------------------------------------------#
    t, x_state, u_command = closed_loop_cascade(dt, x_init, A, B, pid_pos, pid_cac)
    x_state = np.array(x_state)
    draw_simu(np.arange(len(x_state))*dt, x_state[:, 0:2], titre="Lateral and heading error", xlabel="Time (s)", ylabel=["e_l[m]", "e_th[rad]"])
    draw_simu(np.arange(len(x_state))*dt, x_state[:, 2:4], titre="Robot state evolution", xlabel="Time (s)", ylabel=["ω_L", "ω_R"])
    test = [(u[1] - u[0]) / 2 for u in u_command]
    test = np.array(test).reshape(-1, 1)
    draw_simu(np.arange(len(x_state))*dt, test, titre="Differential steering effort", xlabel="Time (s)", ylabel=["(u_r - u_l)/2 [V]"])

    compute_metrics(t, x_state, u_command)

    plt.ioff()
    plt.show()