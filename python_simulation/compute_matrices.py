"""
compute_matrices.py — Duckiebot differential drive, linearized model.

Computes the state-space matrices A (4x4) and B (4x2) for the state
  x = [e_l, e_theta, omega_L, omega_R],  u = [V_L, V_R]
from the physical parameters in platform_constraints.md, then runs sanity checks (time constants, controllability).

Model: coupled motor dynamics (mass + yaw inertia), see system_model.md

"""

import numpy as np
from numpy.linalg import inv, eigvals, matrix_rank, matrix_power
import matplotlib.pyplot as plt

#  Parameters (platform_constraints.md)
Kt = 0.478      # N.m/A   torque constant
Ke = 0.478      # V.s/rad back-EMF constant
R  = 5.0        # Ohm     set by order of magnitude
J  = 2.5e-4     # kg.m2   wheel-side inertia
m  = 1.0        # kg      robot mass
r  = 0.033      # m       wheel radius
Iz = 4.1e-3     # kg.m2   yaw inertia
L  = 0.102      # m       wheel separation
vr = 0.2        # m/s     nominal reference speed (linearization point)


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


def sanity_checks(A, B):
    """Print the model's physical-consistency checks"""
    # motor block = bottom-right 2x2 submatrix
    A_omega = A[2:, 2:]
    ev = eigvals(A_omega)
    print("Motor-block eigenvalues (1/s) :", np.round(ev, 2))
    print("Time constants tau (ms)     :", np.round(-1e3 / ev, 1))

    # controllability
    C = B.copy()
    for i in range(1, 4):
        C = np.hstack([C, matrix_power(A, i) @ B])
    print("Controllability matrix rank   :", matrix_rank(C), "/ 4")


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    A, B = build_matrices()
    print("=== A (4x4) ===\n", A)
    print("\n=== B (4x2) ===\n", B)
    print("\n Sanity checks ")
    sanity_checks(A, B)