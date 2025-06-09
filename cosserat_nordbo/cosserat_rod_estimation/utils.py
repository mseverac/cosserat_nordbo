from scipy.spatial.transform import Rotation as R
import numpy as np
import os
import matplotlib.pyplot as plt

def rotation_angle_between(R1, R2):
    # Rotation relative : R = R1^T * R2
    R_rel = R.from_matrix(R1.T @ R2)
    # Angle de rotation (entre -pi et pi)
    angle = R_rel.magnitude()
    return angle if angle <= np.pi else angle - 2 * np.pi


def numpy_array_to_string(array,print_=False):
    if print_:
        print(f"Converting array to string: {array}")
    array_str = np.array2string(array, separator=', ', precision=8, suppress_small=True)
    return f"np.array({array_str})"




def plot_frame(ax, R, T, length=1.0, name=None):
    """
    Trace un repère orthonormé 3D à partir d'une matrice de rotation R (3x3) et d'une translation T (3,)
    """
    # Origine
    origin = T.reshape(3)
    
    # Axes : colonnes de R
    x_axis = R[:, 0]
    y_axis = R[:, 1]
    z_axis = R[:, 2]
    
    # Tracer les 3 axes
    ax.quiver(*origin, *(x_axis * length), color='r', label='x' if name is None else f'{name}_x')
    ax.quiver(*origin, *(y_axis * length), color='g', label='y' if name is None else f'{name}_y')
    ax.quiver(*origin, *(z_axis * length), color='b', label='z' if name is None else f'{name}_z')
    
    # Optionnel : nom de l'origine
    if name:
        ax.text(*origin, f'{name}', fontsize=12, color='k')

# Exemple d'utilisation :


"""
sol = solve_cosserat_ivp(
    d=0.01, L=0.60, E=3e9, poisson=0.4, rho=1000,
    position=T2,
    rotation=R2,
    n0=forces[0],
    m0=torques[0]
)


sols = []
ds = []
Es = []
poissons = []
rhos = []
for _ in range(10):
    d = 0.01 + random.uniform(-0.001, 0.001)
    L = 0.60  # Keeping L constant
    E = pow(10,random.uniform(7,9))
    poisson = 0.4 + random.uniform(-0.2, 0.4)
    rho = 1000 + random.uniform(-100, 600)
    ds.append(d)
    Es.append(E)
    poissons.append(poisson)
    rhos.append(rho)
    
    sol = solve_cosserat_ivp(
        d=d, L=L, E=E, poisson=poisson, rho=rho,
        position=T2,
        rotation=R2,
        n0=forces[0],
        m0=torques[0]
    )

    sols.append(sol)
for i in range(2):
    d = 0.01 
    L = 0.60  
    E = 10e8
    poisson = 0.9
    rho = 1000 
    if i == 1:
        rho = 10000

    sols.append(solve_cosserat_ivp(
        d=d, L=L, E=E, poisson=poisson, rho=rho,
        position=T2,
        rotation=R2,
        n0=forces[0],
        m0=torques[0]
    ))

"""


def format_array_to_string(array):
    """
    Convert a NumPy array of floats to a string with numbers truncated to 3 decimal places.
    """
    return np.array2string(array, formatter={'float_kind': lambda x: f"{x:.3f}"})


def format_scientific_notation(value):
    """
        Convert a float to a string in scientific notation with 2 decimal places.
        """
    return f"{value:.2e}"


def plot_cable(sol, color, ax, ax2, ax3, T3,n0=None, m0=None,E=None):
    """
    Plot the cable represented by sol on the provided axes.

    Parameters:
    - sol: The solution object containing the cable data.
    - color: The color to use for the cable.
    - ax: The 3D axis for the cable plot.
    - ax2: The 2D axis for the z vs x plot.
    - ax3: The 2D axis for the z vs y plot.
    - ax4: The 2D axis for the x, y, z vs s plot.
    - T3: The reference point to scatter on the 2D plots.
    """
    # Plot the position of each point of the cable in the 3D figure

    if isinstance(sol, np.ndarray):
        ax.plot(sol[0], sol[1], sol[2], label='Cable', color=color, linewidth=2)

        if E is not None:
            ax2.plot(sol[0], sol[2], color=color,label="E : "+format_scientific_notation(E))
            ax2.legend()

        else:
            ax2.plot(sol[0], sol[2], color=color)


        ax2.scatter(T3[0], T3[2], color='green', marker='x')

        # Plot z vs y
        if n0 is not None : 
        
            ax3.plot(sol[1], sol[2], color=color, label=f"n0 : {format_array_to_string(n0)} ,m0 : {format_array_to_string(m0)}")
            ax3.legend()

        else:
            ax3.plot(sol[1], sol[2], color=color)

        
        ax3.scatter(T3[1], T3[2], color='green', marker='x')
        







    else:
        ax.plot(sol.y[0], sol.y[1], sol.y[2], label='Cable', color=color, linewidth=2)

        # Optionally, scatter the points for better visualization
        #ax.scatter(sol.y[0], sol.y[1], sol.y[2], color=color, s=10, label='Cable Points')

        # Plot z vs x
        if E is not None:
            ax2.plot(sol.y[0], sol.y[2], color=color,label="E : "+format_scientific_notation(E))
            ax2.legend()

        else:
            ax2.plot(sol.y[0], sol.y[2], color=color)


        ax2.scatter(T3[0], T3[2], color='green', marker='x')

        # Plot z vs y
        if n0 is not None : 
        
            ax3.plot(sol.y[1], sol.y[2], color=color, label=f"n0 : {format_array_to_string(n0)} ,m0 : {format_array_to_string(m0)}")
            ax3.legend()

        
        ax3.scatter(T3[1], T3[2], color='green', marker='x')
        





def show_plot(block=True, title=None,plot=True,folder="plots",save=True):


    if title is not None and save:
        if not os.path.exists(folder):
            os.makedirs(folder)
        plt.savefig(f"{folder}/{title}.png")
        print(f"Plot saved as {folder}/{title}.png")
    if plot:
        plt.tight_layout()
        plt.show(block=block)



