"""
lqr.py — LQR on Duckiebot simulation.

"""

import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are, solve_discrete_are
from scipy.signal import cont2discrete

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

def comput_K_LQR_Discrete(A, B, Q, R):
    """ Return K gains in discrete domain"""
    dt = 0.005  # 200 Hz
    Ad, Bd, _, _, _ = cont2discrete((A, B, np.eye(4), 0), dt)
    P = solve_discrete_are(Ad, Bd, Q, R)
    K_discrete = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
    return K_discrete

def build_matrices(Kt=Kt, Ke=Ke, R=R, J=J, m=m, r=r, Iz=Iz, L=L, vr=vr):
    """Return (A[2:,2:], B) of the linearized model, the motor block."""
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

def run_motor(A, B, u, dt=0.001):
    """Simulate the motor dynamics."""
    x_act = np.array([0.0, 0.0]) # motor pos = 0
    x = []
    for i in range(int(6/dt)):
        x_dot = A @ x_act + B @ u
        x_act = x_act + x_dot * dt
        x.append(x_act)
    t = np.arange(int(6/dt)) * dt
    return t, x

def draw_simu(t, y, titre="States", xlabel="Time (s)", ylabel=""):
    """Plot information"""
    plt.figure(figsize=(8, 4))
    x_arr = np.array(y)
    for i in range(x_arr.shape[1]):
        plt.plot(t, x_arr[:,i], label=ylabel[i] if ylabel else None)
    plt.legend()
    plt.title(titre)
    plt.grid(True)
    plt.show(block=False)

def comput_K_LQR(A, B, Q, R):
    """Compute LQR gain K in continuous domain"""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

def sanity_checks(A, B, K):
    """Check closed-loop stability"""
    eigs_cl = np.linalg.eigvals(A - B @ K)
    print(f"Closed-loop eigenvalues : {eigs_cl.round(3)}")
    all_stable = all(e.real < 0 for e in eigs_cl)
    print(f"→ {'STABLE ✓' if all_stable else 'UNSTABLE ✗'}")

def closed_loop(dt, x, A, B, K):
    """Closed-loop dynamics"""
    u_list = []
    x_list = []
    x_act = x.copy()
    for i in range(int(6/dt)):
        u = -K @ x_act
        x_dot = A @ x_act + B @ u
        x_act = x_act + x_dot * dt
        u_list.append(u)
        x_list.append(x_act)

    t = np.arange(int(6/dt)) * dt
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

    np.set_printoptions(precision=4, suppress=True)

    #----------------------------------------------------------------#
    #------------------------- Motor simu------------------------#
    #----------------------------------------------------------------#
    A, B = build_matrices()
    A = A[2:,2:]
    B = B[2:]

    u_command = np.array([4.5, 4.5]) # voltage input
    t, pos_motors = run_motor(A, B, u_command)

    # 1st-order info for the motor
    omega = np.array(pos_motors)[:, 0]       # ω_L
    omega_ss = omega[-1]                     # ≈ 9.42
    seuil = 0.632 * omega_ss                 # ≈ 5.96
    idx = np.argmax(omega >= seuil)          # first crossing above threshold
    print("measured τ :", t[idx]*1000, "ms")

    # Plot motor information
    # draw_simu(t, pos_motors, titre="Motor position", xlabel="Time (s)", ylabel="Angular pos")

    #----------------------------------------------------------------#
    #--------------------- LQR control design--------------------#
    #----------------------------------------------------------------#
    # Model parameters
    A, B = build_matrices()
    # Define LQR cost matrices
    Q = np.zeros((4, 4))
    Q[0, 0] = 100.0  # penalize position
    Q[1, 1] = 10.0   # penalize position
    Q[2, 2] = 0.1    # penalize motor speed R
    Q[3, 3] = 0.1    # penalize motor speed L
    R = np.zeros((2, 2))
    R[0, 0] = 1.0 # penalize control effort L
    R[1, 1] = 1.0 # penalize control effort R

    K = comput_K_LQR(A, B, Q, R)
    print("LQR gain K:", K)
    sanity_checks(A, B, K)

    K = comput_K_LQR_Discrete(A, B, Q, R)
    print("LQR gain K:", K)
    sanity_checks(A, B, K)

    #----------------------------------------------------------------#
    #----------------------- LQR simulation----------------------#
    #----------------------------------------------------------------#
    # state init
    x0 = np.array([0.1, -0.1, 0, 0])
    dt = 0.001
    t, x_state, u_command = closed_loop(dt, x0, A, B, K)

    # Plot simulation
    draw_simu(t, x_state, titre="Robot state evolution", xlabel="Time (s)", ylabel=["e_l", "e_th",  "ω_L", "ω_R"])
    draw_simu(t, u_command, titre="Robot control input", xlabel="Time (s)", ylabel=["u_l", "u_r"])

    compute_metrics(t, x_state, u_command)

    plt.ioff()
    plt.show()