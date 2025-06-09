import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from cosserat_nordbo.cosserat_rod_estimation.txt_reader import *
plt.style.use('seaborn-poster')
from cosserat_nordbo.cosserat_rod_estimation.utils import *
import numpy as np



def corde_Kse_Kbt(diameter=1e-3, E=3e9, poisson=0.4):
    """
    Calcule les matrices Kse et Kbt pour une corde cylindrique.
    
    Paramètres :
        diameter : diamètre de la corde (m)
        E : module de Young (Pa)
        poisson : coefficient de Poisson

    Retour :
        Kse, Kbt : matrices (3x3) de raideur de cisaillement/extension et flexion/torsion
    """
    # Section transversale
    r = diameter / 2
    A = np.pi * r**2

    # Module de cisaillement
    G = E / (2 * (1 + poisson))

    # Moments d'inertie pour une section circulaire
    Ixx = Iyy = (np.pi * r**4) / 4 / 4  # = πd⁴/64
    Izz = Ixx + Iyy  # moment polaire

    # Matrices de raideur
    Kse = np.diag([G * A, G * A, E * A])
    Kbt = np.diag([E * Ixx, E * Iyy, E * Izz])

    return Kse, Kbt


def hat(vec):
    """Opérateur hat pour un vecteur 3D"""
    x, y, z = vec
    return np.array([[0, -z, y],
                     [z, 0, -x],
                     [-y, x, 0]])

def project_to_SO3(R):
    """Projette une matrice R 3x3 dans SO(3) via QR"""
    Q, _ = np.linalg.qr(R)
    # Assure la positivité du déterminant
    if np.linalg.det(Q) < 0:
        Q[:, 2] *= -1
    return Q







def solve_cosserat_ivp(d, L, E, poisson, rho, position, rotation, n0, m0,print_=False,n_element=51):
    """
    Résout les équations statiques d'une tige de Cosserat avec `solve_ivp`.

    Paramètres :
        d : diamètre de la corde (m)
        L : longueur de la corde (m)
        E : module de Young (Pa)
        poisson : coefficient de Poisson
        rho : masse volumique (kg/m³)
        position : position initiale p0 (np.array de shape (3,))
        rotation : matrice de rotation R0 (np.array de shape (3,3))
        n0 : force interne initiale (np.array de shape (3,))
        m0 : moment interne initial (np.array de shape (3,))

    Retour :
        sol : solution renvoyée par `solve_ivp`
    """
    Kse, Kbt = corde_Kse_Kbt(diameter=d, E=E, poisson=poisson)
    A = np.pi * (d / 2)**2  # Section transversale
    g = 9.81                # Gravité

    last_s = None  # Variable pour stocker le dernier s pour le debug


    def Cosserat(s, Gamma, v_star=np.array([0, 0, 1]), u_star=np.zeros(3), f=np.zeros(3), l=np.zeros(3)):
        """
        Calcule dGamma/ds à partir de l'état Gamma et des paramètres mécaniques.
        
        Gamma : np.array de taille 18 (3 + 9 + 3 + 3)
            [p(3), R(9), v(3), u(3)]
        Kse : (3,3) matrice de raideur de cisaillement/extension
        Kbt : (3,3) matrice de raideur de flexion/torsion
        v_star : vecteur de vitesse de référence (3,)
        u_star : vecteur de rotation de référence (3,)
        f : force externe (3,)
        l : moment externe (3,)
        
        Retourne :
            dGamma/ds : np.array de taille 18
        """
        #print(f"--- Debug: s = {s} ---")
        #print(f"Gamma = {Gamma}")

        p = Gamma[0:3]
        #print(f"p = {p}")
        R = Gamma[3:12].reshape((3, 3))
        #print(f"R (reshaped) = \n{R}")
        n = Gamma[12:15]
        #print(f"n = {n}")
        m = Gamma[15:18]
        #print(f"m = {m}")

        last_v = Gamma[18:21]
        #print(f"last_v = {last_v}")
        last_u = Gamma[21:24]
        #print(f"last_u = {last_u}")

        v = np.linalg.inv(Kse) @ R.T @ n + v_star
        #print(f"v = {v}")
        u = np.linalg.inv(Kbt) @ m + u_star
        #print(f"u = {u}")

        dv = v - last_v
        #print(f"dv = {dv}")
        du = u - last_u
        #print(f"du = {du}")

        # Équations (6) et (7)
        dp = R @ v
        #print(f"dp = {dp}")
        dR = R @ hat(u)
        #print(f"dR = \n{dR}")

        f = np.array([0, 0, -A * rho * g])
        #print(f"f (external force) = {f}")
        dn = -f
        #print(f"dn = {dn}")
        dm = -np.cross(dp, n) - l
        #print(f"dm = {dm}")

        # Recompose dGamma/ds
        dGamma = np.zeros_like(Gamma)
        dGamma[0:3] = dp
        dGamma[3:12] = dR.reshape(-1)
        dGamma[12:15] = dn
        dGamma[15:18] = dm
        dGamma[18:21] = dv
        dGamma[21:24] = du

        #print(f"dGamma = {dGamma}")
        #print("--- End Debug ---\n")

        return dGamma

    def Cosserat_wrapper(s, Gamma):
        

        
        res = Cosserat(
            s, Gamma,
            v_star=np.array([0, 0, 1]),
            u_star=np.zeros(3),
            f=np.array([0, 0, -A * rho * g]),
            l=np.zeros(3)
        )

        

        return res

    # Initial state Gamma0
    Gamma0 = np.concatenate([
        position.flatten(),        # p0
        rotation.flatten(),        # R0
        n0.flatten(),              # n0
        m0.flatten(),              # m0
        np.zeros(3),               # v0
        np.zeros(3)                # u0
    ])

    # Solve
    t_eval = np.linspace(0, L, n_element)
    sol = solve_ivp(Cosserat_wrapper, [0.0, L], Gamma0, t_eval=t_eval)

    p = sol.y[:3,-1]

    if print_:
        print("p:", p)
        print("T3:", T3)
        print("distance:", np.linalg.norm(p - T3))
        print("R :" , sol.y[3:12,-1].reshape(3,3))
        print("R3 :", R3)
        print("diff angle:", rotation_angle_between(R3, sol.y[3:12,-1].reshape(3,3)))

    return sol
"""
sol = solve_cosserat_ivp(
    d=0.01, L=0.60, E=3e9, poisson=0.4, rho=1000,
    position=np.array([0, 0, 0]),
    rotation=np.eye(3),
    n0=np.array([0, 0, 0]),
    m0=np.array([0, 0, 0])
)

plt.figure(figsize=(12, 8))


# Calculate theta_y (rotation around y-axis) as a function of s
theta_y = np.arctan2(sol.y[5], sol.y[3])  # Assuming R[0, 0] = sol.y[3] and R[2, 0] = sol.y[5]
# Plot theta_y vs s
plt.subplot(321)
plt.plot(sol.t, theta_y)
plt.xlabel('s')
plt.ylabel('theta_y (radians)')
plt.title('Rotation around y-axis (theta_y) vs s')
plt.grid()

# Plot x vs z
plt.subplot(322)
plt.plot(sol.y[0], sol.y[2])
plt.xlabel('x')
plt.ylabel('z')
plt.title('x vs z')
plt.grid()

# Plot s vs p components
plt.subplot(323)
plt.plot(sol.t, sol.y[0], label='p_x')
plt.plot(sol.t, sol.y[1], label='p_y')
plt.plot(sol.t, sol.y[2], label='p_z')
plt.xlabel('s')
plt.ylabel('p components')
plt.legend()
plt.title('Components of p vs s')
plt.grid()

# Plot components of n vs s
plt.subplot(324)
plt.plot(sol.t, sol.y[12], label='n_x')
plt.plot(sol.t, sol.y[13], label='n_y')
plt.plot(sol.t, sol.y[14], label='n_z')
plt.xlabel('s')
plt.ylabel('n components')
plt.legend()
plt.title('Components of n vs s')
plt.grid()

# Plot components of m vs s
plt.subplot(325)
plt.plot(sol.t, sol.y[15], label='m_x')
plt.plot(sol.t, sol.y[16], label='m_y')
plt.plot(sol.t, sol.y[17], label='m_z')
plt.xlabel('s')
plt.ylabel('m components')
plt.legend()
plt.title('Components of m vs s')
plt.grid()

# Plot determinant of R vs s
plt.subplot(326)
det_R = [np.linalg.det(sol.y[3:12, i].reshape((3, 3))) for i in range(sol.y.shape[1])]
plt.plot(sol.t, det_R)
plt.xlabel('s')
plt.ylabel('det(R)')
plt.title('Determinant of R vs s')
plt.grid()

# Plot components of v vs s
plt.figure(figsize=(12, 6))
plt.subplot(211)
plt.plot(sol.t, sol.y[18], label='v_x')
plt.plot(sol.t, sol.y[19], label='v_y')
plt.plot(sol.t, sol.y[20], label='v_z')
plt.xlabel('s')
plt.ylabel('v components')
plt.legend()
plt.title('Components of v vs s')
plt.grid()

# Plot components of u vs s
plt.subplot(212)
plt.plot(sol.t, sol.y[21], label='u_x')
plt.plot(sol.t, sol.y[22], label='u_y')
plt.plot(sol.t, sol.y[23], label='u_z')
plt.xlabel('s')
plt.ylabel('u components')
plt.legend()
plt.title('Components of u vs s')
plt.grid()

plt.tight_layout()
plt.show()
"""