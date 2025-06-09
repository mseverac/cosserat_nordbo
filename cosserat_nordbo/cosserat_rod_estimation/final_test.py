# Après votre simulation, appelez cette fonction

from both_ends_fixed import * 

from txt_reader import *



for i in range(0, 30,5):
    """start = np.random.rand(3) * 0.5  # Random start point in a 0.5x0.5x0.5 cube
    end = np.random.rand(3) * 0.5  # Random end point in a 0.5x0.5x0.5 cube


    while np.linalg.norm(start - end) > 0.58:

        start = np.random.rand(3) * 0.5  # Random start point in a 0.5x0.5x0.5 cube
        end = np.random.rand(3) * 0.5  # Random end point in a 0.5x0.5x0.5 cube

        print("Start and end points are too far, skipping this simulation.")
        
    print(f"Simulation {i+1}: start={start}, end={end}")
"""

    start = positions2[i]
    end = positions1[i]


    start_time = time.time()

    """plot_frame(ax, rotations1[i], start, length=0.05, name="Start")
    plot_frame(ax, rotations2[i], end, length=0.05, name="End")

    show_plot()"""

    frames =(
             (rotations2[i], positions2[i]),
             (rotations1[i], positions1[i]))
    
    print(f"rotations2[i]: {rotations2[i]}")
    print(f"positions2[i]: {positions2[i]}")
    print(f"rotations1[i]: {rotations1[i]}")
    print(f"positions1[i]: {positions1[i]}")

    pp_list = cosserat_get_cable_state(start, end, start_rotation = rotations2[i], end_rotation = rotations1[i],final_time=0.06)

    end_time = time.time()
    execution_time = end_time - start_time
    print(f"Execution time for simulation {i+1}: {execution_time:.2f} seconds")

    plot_all_components(pp_list,frames=frames)


    last_step = -1
    positions = np.array(pp_list["position"][last_step])  # Shape (3, n_elem+1)

    print(f"positions end: {positions}")



fps = 30
PLOT_VIDEO = False

if PLOT_VIDEO:

    filename = "rod_suspended_both_ends"
    plot_video(
        pp_list,
        video_name=filename + ".mp4",
        fps=fps,
    )
    

