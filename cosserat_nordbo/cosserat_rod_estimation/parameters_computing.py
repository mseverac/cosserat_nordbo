import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import geometry_msgs.msg
from utils import *
from txt_reader import *
from test_solve_ivp import solve_cosserat_ivp
import random


#distance entre le capteur de force et le point d'attache de la corde
l=0.15
r=np.array([0,0,l])

sols = None

# Usage of the function

  # Generate 10 distinct colors using the Viridis colormap

(d, E, poisson, rho) = [0.01, 10e8, 0.4, 1400]

n0s = np.linspace(np.array([-12,0,0]), np.array([-12,0,0]), 1)
#print("nos shape:", n0s.shape)
m0s = np.linspace(np.array([0.0,1.8,0]), np.array([-0.0,1.8,0]),1)
#print("mos shape:", m0s.shape)

sols = []
wrenches = []
for m0 in m0s : 
    for n0 in n0s:
        sol = solve_cosserat_ivp(
            d=d, L=0.60, E=E, poisson=poisson, rho=rho,
            position=T2,
            rotation=R2,
            n0=n0,
            m0=m0
        )
        sols.append(sol)
        wrenches.append((n0, m0))
        """print("----------------------")
        print("n0:", n0)
        print("m0:", m0)"""

        
fig, ax, ax2, ax3 = initialize_plot()


sols = None
if sols is not None:
    colors = plt.cm.viridis(np.linspace(0, 1, len(sols)))
    for sol, color,(n0,m0) in zip(sols, colors,wrenches):
        plot_cable(sol, color, ax, ax2, ax3, T3,n0=n0, m0=m0)





def dist_dernier_point(x,plot=False,print_=True): 

    if len(x) == 4:

        (d, E, poisson, rho) = x
        n0=forces[0],
        m0=torques[0]

    elif len(x) == 10:
        (d,E,poisson,rho) = x[:4]
        n0 = x[4:7]
        m0 = x[7:10]

    elif len(x) == 6:
        (d, E, poisson, rho) = (0.01, 10e6, 0.48, 1400)

        n0 = x[:3]
        m0 = x[3:6]

    elif len(x) == 7 :

        n0 = x[:3]
        m0 = x[3:6]
        E = x[6]
        poisson = 0.4
        rho = 1400
        d = 0.01


    

    
    sol = solve_cosserat_ivp(
        d=d, L=0.60, E=E, poisson=poisson, rho=rho,
        position=T2,
        rotation=R2,
        n0=n0,
        m0=m0
    )

    #if plot :
     #   plot_cable(sol, 'red', ax, ax2, ax3, T3)

    #print("sol shape:", sol.y.shape)

    p = sol.y[:3,-1]
    if print_:
        print("x:", x)
        print("p:", p)
        print("T3:", T3)
        print("distance:", np.linalg.norm(p - T3))
        print("R :" , sol.y[3:12,-1])
        print("R3 :", R3)
        print("diff angle:", rotation_angle_between(R3, sol.y[3:12,-1].reshape(3,3)))
    #print("p:", p)
    #print("T3:", T3)
    #print("R :" , sol.y[3:12,-1])
    #print("R3 :", R3)
    #print("diff angle:", rotation_angle_between(R3, sol.y[3:12,-1].reshape(3,3)))
    return np.linalg.norm(p - T3)#+0.0*np.abs(rotation_angle_between(R3, sol.y[3:12,-1].reshape(3,3)))

#print(dist_dernier_point(0.01, 1e8, 0.4, 1000))


