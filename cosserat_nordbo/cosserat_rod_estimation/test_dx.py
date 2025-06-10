import numpy as np
from scipy.spatial.transform import Rotation as R

# Valeurs initiales (à adapter à ton cas)


def compute_perturbed_inputs(start,end,start_rotation,end_rotation,epsilon,dtheta):

    perturbed_inputs = []

    
    directions = np.eye(3)


    # Perturbations sur START + ROTATION DE START
    for i in range(3):
        # Translation sur start
        start_pert = start + epsilon * directions[i]
        perturbed_inputs.append((
            start_pert, end.copy(),
            start_rotation.copy(), end_rotation.copy()
        ))

    for i in range(3):
        # Rotation autour de chaque axe pour start_rotation
        rotvec = dtheta * directions[i]
        delta_rot = R.from_rotvec(rotvec).as_matrix()
        start_rot_pert =  start_rotation @ delta_rot.T
        perturbed_inputs.append((
            start.copy(), end.copy(),
            start_rot_pert, end_rotation.copy()
        ))

    # Perturbations sur END + ROTATION DE END
    for i in range(3):
        # Translation sur end
        end_pert = end + epsilon * directions[i]
        perturbed_inputs.append((
            start.copy(), end_pert,
            start_rotation.copy(), end_rotation.copy()
        ))

    for i in range(3):
        # Rotation autour de chaque axe pour end_rotation
        rotvec = dtheta * directions[i]
        delta_rot = R.from_rotvec(rotvec).as_matrix()
        end_rot_pert = delta_rot @ end_rotation
        perturbed_inputs.append((
            start.copy(), end.copy(),
            start_rotation.copy(), end_rot_pert
        ))

    return perturbed_inputs

"""
start = np.array([0.0, 0.0, 0.0])
end = np.array([1.0, 0.0, 0.0])
start_rotation = np.eye(3)
end_rotation = np.eye(3)

# Perturbations
epsilon = 1e-3       # petite translation
dtheta = 1e-2        # petite rotation en radians

# Liste des perturbations

# Directions unitaires

perturbed_inputs = compute_perturbed_inputs(start,end,start_rotation,end_rotation,epsilon,dtheta)
# Affichage pour vérification (facultatif)
for i, (s, e, R1, R2) in enumerate(perturbed_inputs):
    print(f"Perturbation {i+1}:")
    print(f"Start: {s}")
    print(f"End: {e}")
    print(f"Start Rotation:\n{R1}")
    print(f"End Rotation:\n{R2}\n")"""
