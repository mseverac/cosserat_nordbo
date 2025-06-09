from cosserat_nordbo.cosserat_rod_estimation.init_gamma0 import get_cosserat_gamma0

from txt_reader import positions1,positions2, rotations1, rotations2

import numpy as np

print("positions1 shape:", positions1.shape)
print("positions2 shape:", positions2.shape)
# Define start and end positions



initial_guess = np.array([-0.17834246, -0.09977787, -0.21975261,
                               -0.00928889,  0.02773056, -0.01566086])



for i in range(20,0,-1):

    start = positions2[i]
    end = positions1[i]



    R1 = rotations2[i]
    R2 = rotations1[i]

    """print("Start position:", start)
    print("End position:", end)
    print("Start rotation:\n", R1)
    print("End rotation:\n", R2)"""

    # Get the Cosserat gamma0
    start,R1,n0,m0 , _= get_cosserat_gamma0(start, end, 
                                R1=R1, R2=R2,
                                n_elem=49, E=30e6, plot=False,print_=True,
                                save=True,
                                triple_opt=False,
                                xtol=1e-4,
                                maxiter = 400,
                                initial_guess=initial_guess
                                )

    """print("Start position after Cosserat:", start)
    print("Start rotation after Cosserat:\n", R1)
    print("n0:", n0)
    print("m0:", m0)"""

    gamma0 = np.concatenate([start, R1.flatten(), n0, m0])

    initial_guess = np.concatenate([n0, m0])

    print("Gamma0:", gamma0)
