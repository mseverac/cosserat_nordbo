__doc__ = """Fixed joint example - rod attached at both ends with sag due to extra length."""

from pprint import pp
import numpy as np
import elastica as ea




import matplotlib.pyplot as plt


from cosserat_nordbo.cosserat_rod_estimation.deltaZ_poly import deltaZ_poly


from cosserat_nordbo.cosserat_rod_estimation.utils import plot_frame


from cosserat_nordbo.cosserat_rod_estimation.bc_cases_postprocessing import (
    plot_position,
    plot_orientation,
    plot_video,
    plot_video_xy,
    plot_video_xz,
)


import numpy as np
import elastica as ea
import time

class GeneralConstraintSimulator(
    ea.BaseSystemCollection,
    ea.Constraints,
    ea.Connections,
    ea.Forcing,
    ea.Damping,
    ea.CallBacks,
):
    pass



def cosserat_get_cable_state(start, end,start_rotation = np.matrix([[0,0,1],[0,1,0],[-1,0,0]],dtype=float), 
                             end_rotation =  np.matrix([[0,0,1],[0,1,0],[-1,0,0]],dtype=float) ,
                             rod_length=0.6, n_elem=50,E=1e7, poisson_ratio=0.5,final_time=0.3,plot=False):
    """Simule un câble avec les deux extrémités fixées et retourne pp_list."""
    
    print(f"Start: {start_rotation}, End: {end_rotation}")
    # Simulation container
    sim = GeneralConstraintSimulator()

    normal = np.array(start_rotation[:, 0]).flatten()
    direction = np.array(start_rotation[:, 2]).flatten()

    end_direction = np.array(end_rotation[:, 2]).flatten()
    end_normal = np.array(end_rotation[:, 0]).flatten()

    print(f"start normal: {normal}")
    print(f"end normal: {end_normal}")
    print(f"start direction: {direction}")
    print(f"end direction: {end_direction}")

    

    # Parameters
    
    base_radius = 0.01
    density = 1400
    shear_modulus = E / (1 + poisson_ratio)

    initial_position = deltaZ_poly(start, end, L=rod_length, plot=False, nb_points=n_elem+1, print_distance=False)

    dt = 1e-5
    damping_constant = 2.0
    total_steps = int(final_time / dt)
    diagnostic_step_skip = 5

    # Create rod
    rod = ea.CosseratRod.straight_rod(
        n_elem,
        start,
        direction,
        normal,
        rod_length,
        base_radius,
        density,
        youngs_modulus=E,
        shear_modulus=shear_modulus,
    )

    # Initial conditions
    rod.velocity_collection[:] = 0.0
    rod.position_collection[2, :] += 0.001 * np.random.randn(n_elem + 1)
    rod.position_collection = initial_position.transpose()
    """

    print("Rod director:")
    print(rod.director_collection[:, :, 0])

    print("Start rotation:")
    print(start_rotation.T)

    print("Error:")
    print(rod.director_collection[:, :, 0].T - start_rotation)

    print("rod director:")
    print(rod.director_collection[:, :, -1])

    print("End rotation:")
    print(end_rotation.T)

    print("Error:")
    print(rod.director_collection[:, :, -1].T - end_rotation)

    """
    rod.director_collection[:, :, 0] = start_rotation.T
    rod.director_collection[:, :, -1] = end_rotation.T
    

    sim.append(rod)

    # Constraints
    sim.constrain(rod).using(
        ea.GeneralConstraint,
        constrained_position_idx=(-1,),
        constrained_director_idx=(-1,),
        translational_constraint_selector=np.array([True, True, True]),
        rotational_constraint_selector=np.array([True, True, True]),
    )

    sim.constrain(rod).using(
        ea.GeneralConstraint,
        constrained_position_idx=(0,),
        constrained_director_idx=(0,),
        translational_constraint_selector=np.array([True, True, True]),
        rotational_constraint_selector=np.array([True, True, True]),
    )

    # Add gravity
    sim.add_forcing_to(rod).using(
        ea.GravityForces,
        acc_gravity=np.array([0.0, 0.0, -9.81])
    )

    # Add damping
    sim.dampen(rod).using(
        ea.AnalyticalLinearDamper,
        damping_constant=damping_constant,
        time_step=dt,
    )

    # Data collection
    class MyCustomCallback(ea.CallBackBaseClass):
        def __init__(self, step_skip, callback_params):
            super().__init__()
            self.step_skip = step_skip
            self.callback_params = callback_params

        def make_callback(self, system, time, current_step):
            if current_step % self.step_skip == 0:
                self.callback_params["time"].append(time)
                self.callback_params["position"].append(system.position_collection.copy())
                self.callback_params["velocity"].append(system.velocity_collection.copy())
                self.callback_params["directors"].append(system.director_collection.copy())
                self.callback_params["internal_forces"].append(system.internal_forces.copy())
                self.callback_params["internal_torques"].append(system.internal_torques.copy())

    pp_list = {
        "time": [],
        "position": [],
        "velocity": [],
        "directors": [],
        "internal_forces": [],
        "internal_torques": [],
    }

    sim.collect_diagnostics(rod).using(
        MyCustomCallback,
        step_skip=diagnostic_step_skip,
        callback_params=pp_list
    )

    # Finalize and simulate
    sim.finalize()
    timestepper = ea.PositionVerlet()
    ea.integrate(timestepper, sim, final_time, total_steps)

    if plot:
        plot_all_components(pp_list, rod_length=rod_length, plot_3d=True)

    return pp_list


































def plot_all_components(pp_list, rod_length=0.6, plot_3d=True,frames=None):
    # Récupérer les dernières données
    last_step = -1
    positions = np.array(pp_list["position"][last_step])  # Shape (3, n_elem+1)
    velocities = np.array(pp_list["velocity"][last_step])  # Shape (3, n_elem+1)
    directors = np.array(pp_list["directors"][last_step])  # Shape (3, 3, n_elem)
    internal_forces = np.array(pp_list["internal_forces"][last_step])  # Shape (3, n_elem)
    internal_torques = np.array(pp_list["internal_torques"][last_step])  # Shape (3, n_elem)

    positions_initial = np.array(pp_list["position"][0])  # Shape (3, n_elem+1)
    
    # Calculer la position le long du câble (normalisée)
    s = np.linspace(0, rod_length, positions.shape[1])
    if plot_3d :
        
        # Combine both plots into a single figure with subplots
        fig, axs = plt.subplots(2, 1, figsize=(10, 12))

        # Plot the position of the 3 quartiles over time
        mid_idx = positions.shape[1] // 2
        quarter_idx = positions.shape[1] // 4
        three_quarter_idx = 3 * positions.shape[1] // 4

        time = np.array(pp_list["time"])
        mid_positions = [pos[2, mid_idx] for pos in pp_list["position"]]
        quarter_positions = [pos[2, quarter_idx] for pos in pp_list["position"]]
        three_quarter_positions = [pos[2, three_quarter_idx] for pos in pp_list["position"]]

        axs[0].plot(time, mid_positions, label='Midpoint (1/2)', color='b')
        axs[0].plot(time, quarter_positions, label='First Quartile (1/4)', color='r')
        axs[0].plot(time, three_quarter_positions, label='Third Quartile (3/4)', color='g')

        axs[0].set_title('Position of Quartiles Along the Cable Over Time')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Position Z (m)')
        axs[0].legend()
        axs[0].grid(True)

        # Plot the 3D shape of the cable
        ax = fig.add_subplot(212, projection='3d')

        ax.set_box_aspect([1, 1, 1])  # For isotropic axes

        ax.set_xlim([-0.3, 0.3])
        ax.set_ylim([0.5, 0.9])
        ax.set_zlim([-0.0, 0.5])

        ax.plot(positions[0], positions[1], positions[2], label='Rod Shape', color='b')
        ax.plot(positions_initial[0], positions_initial[1], positions_initial[2], label='Initial Shape', color='orange', linestyle='--')
        ax.scatter(positions[0, 0], positions[1, 0], positions[2, 0], color='r', label='Start Point')
        ax.scatter(positions[0, -1], positions[1, -1], positions[2, -1], color='g', label='End Point')

        if frames is not None:

            print("Plotting frames for start and end positions.")
            rotations1, start = frames[0]
            rotations2, end = frames[1]
            
            plot_frame(ax, rotations1, start, length=0.05, name="Start")
            plot_frame(ax, rotations2, end, length=0.05, name="End")

        ax.set_title('3D Shape of the Cable')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()

        plt.tight_layout()
        plt.show()

    # Créer une figure avec plusieurs sous-graphiques
    else : 
        
        plt.figure(figsize=(20, 25))
        
        # 1. Positions
        plt.subplot(6, 1, 1)
        plt.plot(s, positions[2], label='Z')
        plt.title('Position Components Along the Rod')
        plt.xlabel('Position along rod (m)')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.grid(True)
        
        # 2. Vitesses
        plt.subplot(6, 1, 2)
        plt.plot(s, velocities[0], label='Vx')
        plt.plot(s, velocities[1], label='Vy')
        plt.plot(s, velocities[2], label='Vz')
        plt.title('Velocity Components Along the Rod')
        plt.xlabel('Position along rod (m)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.grid(True)
        
        # 3. Directeurs (orientations)
        plt.subplot(6, 1, 3)
        plt.plot(s[:-1], directors[0,0,:], label='D1 X')
        plt.plot(s[:-1], directors[0,1,:], label='D1 Y')
        plt.plot(s[:-1], directors[0,2,:], label='D1 Z')
        plt.plot(s[:-1], directors[1,0,:], '--', label='D2 X')
        plt.plot(s[:-1], directors[1,1,:], '--', label='D2 Y')
        plt.plot(s[:-1], directors[1,2,:], '--', label='D2 Z')
        plt.plot(s[:-1], directors[2,0,:], ':', label='D3 X')
        plt.plot(s[:-1], directors[2,1,:], ':', label='D3 Y')
        plt.plot(s[:-1], directors[2,2,:], ':', label='D3 Z')
        plt.title('Director Components Along the Rod')
        plt.xlabel('Position along rod (m)')
        plt.ylabel('Director Components')
        plt.legend()
        plt.grid(True)
        
        # 4. Forces internes
        plt.subplot(6, 1, 4)
        plt.plot(s[:], internal_forces[0], label='Fx')
        plt.plot(s[:], internal_forces[1], label='Fy')
        plt.plot(s[:], internal_forces[2], label='Fz')
        plt.title('Internal Forces Along the Rod')
        plt.xlabel('Position along rod (m)')
        plt.ylabel('Force (N)')
        plt.legend()
        plt.grid(True)
        
        # 5. Moments internes
        plt.subplot(6, 1, 5)
        plt.plot(s[:-1], internal_torques[0], label='Mx')
        plt.plot(s[:-1], internal_torques[1], label='My')
        plt.plot(s[:-1], internal_torques[2], label='Mz')
        plt.title('Internal Torques Along the Rod')
        plt.xlabel('Position along rod (m)')
        plt.ylabel('Torque (Nm)')
        plt.legend()
        plt.grid(True)
        
        # 6. Position X du milieu de la corde en fonction du temps
        plt.subplot(6, 1, 6)
        mid_idx = positions.shape[1] // 2
        quarter_idx = positions.shape[1] // 4
        three_quarter_idx = 3 * positions.shape[1] // 4

        time = np.array(pp_list["time"])
        mid_x_positions = [pos[2, mid_idx] for pos in pp_list["position"]]
        quarter_x_positions = [pos[2, quarter_idx] for pos in pp_list["position"]]
        three_quarter_x_positions = [pos[2, three_quarter_idx] for pos in pp_list["position"]]

        plt.plot(time, mid_x_positions, label='X (milieu)')
        plt.plot(time, quarter_x_positions, label='X (1/4)')
        plt.plot(time, three_quarter_x_positions, label='X (3/4)')

        plt.title('Position X à Différents Points de la Corde en Fonction du Temps')
        plt.xlabel('Temps (s)')
        plt.ylabel('Position X (m)')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("rod_suspended_both_ends_components.png")
        plt.show()

