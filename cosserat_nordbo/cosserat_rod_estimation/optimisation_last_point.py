from utils import *
from txt_reader import *
from txt_reader import show_plot
from parameters_computing import dist_dernier_point,solve_cosserat_ivp
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from minimise_with_ftol import minimize_with_early_stop

# Seuil d'arrêt sur la fonction objectif
threshold = 1e-3

# Exception personnalisée pour stopper l’optimisation
class EarlyStoppingException(Exception):
    pass

# Callback appelé à chaque itération
def early_stop_callback(xk):
    fx = dist_dernier_point(xk)
    print(f"[Callback] f(x) = {fx:.6f}")
    if fx < threshold:
        raise EarlyStoppingException()

# Initialisation des paramètres


(d, E, poisson, rho) = (0.01, 1e7, 0.48, 1400)

n0s = np.linspace(np.array([-0.6,0,0.1]), np.array([-0.9,0,0.1]), 3)
#print("nos shape:", n0s.shape)
m0s = np.linspace(np.array([0.0,0.015,0]), np.array([0.0,0.03,0]),3)
#print("mos shape:", m0s.shape)


n0s = np.array([forces[0]])
m0s = np.array([torques[0],torques1[0],torques2[0]])
#m0s = -np.array([torques1[0]])


Es = np.logspace(6.5, 7.5, 3)  # De 10^6 à 10^9 avec 3 points
sols = []
wrenches = []
youngs = []
for m0 in m0s : 
    for n0 in n0s:
        for E in Es:
            sol = solve_cosserat_ivp(
                d=d, L=0.60, E=E, poisson=poisson, rho=rho,
                position=T2,
                rotation=R2,
                n0=n0,
                m0=m0
            )
            sols.append(sol)
            wrenches.append((n0, m0))
            youngs.append(E)
        """print("----------------------")
        print("n0:", n0)
        print("m0:", m0)"""


if sols is not None:
    colors = plt.cm.viridis(np.linspace(0, 1, len(sols)))
    for sol, color,(n0,m0),E in zip(sols, colors,wrenches,youngs):
        plot_cable(sol, color, ax, ax2, ax3, T3,n0=n0, m0=m0,E=E)

show_plot()

initial_guess = np.array([-12, 0, 0, 0, 1.8, 0])
bounds = [(-1,1), (-1,1), (-1,1), (-0.1,0.1), (-0.1,0.1), (-0.1,0.1)]
initial_guess = np.array([-14.75811189 ,  0.09086251 , -3.18448662,  
                          -0.37966638,1.86092011,0.75027306])


initial_guess = np.vstack((forces[0], torques[0])).flatten()*10

initial_guess = np.array([-1.2, 0, 0, 0, -0.5, 0])*0.001

initial_guess = np.array([-0.08666602, -0.04876035,  0.02571622,
                            0.00113228 ,-0.01012694 ,-0.00479059])

initial_guess = np.array([-0.10964993, -0.03731836, -0.0529525, 0.00133102, -0.0062091, -0.00411043])



initial_guess = np.array([-0.18283517, -0.0971982, -0.21271606, -0.00527533, 0.01916997, -0.01073625])
initial_guess = np.array([-0.17854694, -0.11992282, -0.19540045, -0.01104545, 0.01688951, -0.00278103])
# Options pour L-BFGS-B
initial_guess =  np.array([-0.5, -0.0015237688, -0.0029728646,  0.000273025,  -0.005, 0.004 ])

initial_guess = np.zeros_like(initial_guess)

initial_guess =  np.array([-0.00765882, -0.01858921,  0.04926925, -0.002018116,  0.00268144 ,
  0.009196357])

print("Initial guess:", initial_guess)

sol = solve_cosserat_ivp(
    d=0.01, L=0.60, E=10e6, poisson=0.4, rho=1400,
    position=T2,
    rotation=R2,
    n0=initial_guess[:3],
    m0=initial_guess[3:6]
)

plot_cable(sol, 'blue', ax, ax2, ax3, T3,n0=initial_guess[:3], m0=initial_guess[3:6])
#show_plot()



"""
result = minimize(
        dist_dernier_point,
        initial_guess,
        method='L-BFGS-B',
        #bounds=bounds,
        options=options,
        callback=early_stop_callback
    )
"""

options = {
    'disp': True,         # Affiche les messages de convergence
    'gtol': 1e-8,         # Tolérance sur le gradient (plus petit = plus précis)
    'ftol': 1e-12,        # Réduction relative minimale de f requise
    'maxiter': 500,       # Nombre maximal d'itérations
    'maxls': 40           # Nombre max de recherches linéaires (important si f remonte)
}


result = minimize_with_early_stop(
    fun=dist_dernier_point,
    x0=initial_guess,
    options=options,
)

# Print the results
print("Optimized parameters:", result.x)
print("initial_guess = ", numpy_array_to_string(result.x))
print("Minimum value of dist_dernier_point:", result.fun)
"""
d,E,poisson,rho = result.x
sol = solve_cosserat_ivp(
    d=d, L=0.60, E=E, poisson=poisson, rho=rho,
    position=T2,
    rotation=R2,
    n0=forces[0],
    m0=torques[0]
)
"""

n0 = result.x[:3]
m0 = result.x[3:6]

(d, E, poisson, rho) = [0.01, 10e8, 0.4, 1400]

(d, E, poisson, rho) = (0.01, 10e6, 0.48, 1400)

sol= solve_cosserat_ivp(
    d=d, L=0.60, E=E, poisson=poisson, rho=rho,
    position=T2,
    rotation=R2,
    n0=n0,
    m0=m0,
    print_=True
    
    )


# Plot the optimized cable
plot_cable(sol, 'red', ax, ax2, ax3, T3,n0=n0, m0=m0)
show_plot()




"""


from parameters_computing import *

fig = plt.figure(figsize=(16, 10))

# 3D plot (large on the left)
ax = fig.add_subplot(121, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_box_aspect([1, 1, 1])  # For isotropic axes

# Frame 1: Identity
R1 = np.eye(3)
T1 = np.array([0, 0, 0])
plot_frame(ax, R1, T1, length=0.8, name="F0")

R2 = rotations2[0]
T2 = positions2[0]
plot_frame(ax, R2, T2, length=0.8, name="start")

R3 = rotations1[0]
T3 = positions1[0]
plot_frame(ax, R3, T3, length=0.8, name="end")



# Create additional 2D subplots for the cable visualization
ax2 = fig.add_subplot(222)  # z vs x
ax2.set_xlabel('x')
ax2.set_ylabel('z')
ax2.set_title('z vs x')
ax2.grid()

ax3 = fig.add_subplot(224)  # z vs y
ax3.set_xlabel('y')
ax3.set_ylabel('z')
ax3.set_title('z vs y')
ax3.grid()


plot_cable(sol, 'red', ax, ax2, ax3, T3)
show_plot()


"""