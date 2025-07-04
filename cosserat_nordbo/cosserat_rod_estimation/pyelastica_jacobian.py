# Après votre simulation, appelez cette fonction

from cosserat_nordbo.cosserat_rod_estimation.both_ends_fixed import * 

from cosserat_nordbo.cosserat_rod_estimation.txt_reader import *
from cosserat_nordbo.cosserat_rod_estimation.test_dx import compute_perturbed_inputs
from copy import *




def compute_pyelastica_jacobian(start, end, R1, R2,
                                 n_elem=49, E=3e7, poisson=0.5, rho=1400,
                                 d=0.01, L=0.60,
                                 final_time_init=0.04,
                                 final_time_ds=0.04,
                                 plot_cables=False,
                                 points3d: np.ndarray = None):

    
    if points3d is None :
    

        pp_list = cosserat_get_cable_state(start, end, 
                                            start_rotation = R1, end_rotation = R2,
                                            final_time=final_time_init,
                                            n_elem=n_elem,E=E,base_radius=d,poisson_ratio=poisson,density=rho,
                                            )

        
        Jac = None

        #plot_all_components(pp_list,frames=frames)


        last_step = -1
        positions = np.array(pp_list["position"][last_step])  # Shape (3, n_elem+1)

    else : 
        positions = points3d.transpose()

    perturbed_pos = compute_perturbed_inputs(start,end,R1,R2,0.003,0.1)

    for i, (s, e, R1p, R2p) in enumerate(perturbed_pos):

        """print(f"Perturbation {i+1}:")
        print(f"Start perturbed: {s} ||| start : {start}")
        print(f"End perturbed: {e}||| start : {end}")
        print(f"Start Rotation perturbed:\n{R1p}\n||| start : \n{R1}\n")
        print(f"End Rotation perturbed:\n{R2p}\n||| start : \n{R2}\n")
"""
        pp_list1 = cosserat_get_cable_state(
            s, e,
            start_rotation=R1p, end_rotation=R2p,
            final_time=final_time_ds,
            damping_constant=1.0,
            n_elem=n_elem,

        )



        #plot_all_components(pp_list1, frames=frames)

        

        if plot_cables :

            fig, ax, ax2, ax3 = initialize_plot()

            plot_cable(np.array(pp_list1["position"][-1]),"red",ax,ax2,ax3,e)
            plot_cable(np.array(positions),"blue",ax,ax2,ax3,end)

            show_plot()

        ds = (np.array(pp_list1["position"][-1]).flatten()-np.array(positions).flatten()).reshape(-1,1)

        if Jac is None :
            Jac = ds.copy()
        

        else :
            Jac = np.hstack([Jac,ds])

    return Jac,np.array(positions).transpose()


  





def afficher_tableau_beau(tableau: np.ndarray, titre: str = "Affichage du tableau") -> None:
    """
    Affiche un tableau 2D NumPy avec une mise en page esthétique.
    
    Args:
        tableau (np.ndarray): Tableau 2D à afficher.
        titre (str): Titre du graphique.
    """
    if tableau.ndim != 2:
        raise ValueError("Le tableau doit être 2D")

    plt.figure(figsize=(10, 8))
    cmap = plt.cm.viridis  # colormap élégante, tu peux essayer 'plasma', 'magma', etc.
    im = plt.imshow(tableau, cmap=cmap, aspect='auto')

    # Ajouter les valeurs dans la heatmap (si le tableau n'est pas trop grand)
    if tableau.shape[0] <= 50 and tableau.shape[1] <= 50:
        for i in range(tableau.shape[0]):
            for j in range(tableau.shape[1]):
                plt.text(j, i, f"{tableau[i, j]:.3f}", 
                         ha="center", va="center", color="white", fontsize=6)

    plt.title(titre, fontsize=16)
    plt.colorbar(im, fraction=0.046, pad=0.04)
    plt.xlabel("Colonnes")
    plt.ylabel("Lignes")
    plt.xticks(np.arange(tableau.shape[1]))
    plt.yticks(np.arange(tableau.shape[0]))
    plt.tight_layout()
    plt.grid(False)
    plt.show()

"""def test_jac():
    import time


    t1 = time.time()

    #afficher_tableau_beau(J,"jacobienne")
    J = compute_pyelastica_jacobian(start, end, rotations2[i], rotations1[i],
                                n_elem=49, E=3e7, poisson=0.5, rho=1400,
                                d=0.01, L=0.60,
                                final_time_init=0.04,
                                final_time_ds=0.04,
                                plot_cables=True,
                                
                                )

    print("jac shape :",J.shape)



    start = positions2[i]
    end = positions1[i]



    frames =(
                (rotations2[i], positions2[i]),
                (rotations1[i], positions1[i]))

    print(f"rotations2[i]: {rotations2[i]}")
    print(f"positions2[i]: {positions2[i]}")
    print(f"rotations1[i]: {rotations1[i]}")
    print(f"positions1[i]: {positions1[i]}")

    t2 = time.time()


    print("computation time = ",t2-t1)"""
