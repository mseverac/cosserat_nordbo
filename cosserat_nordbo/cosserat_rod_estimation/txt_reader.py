import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from cosserat_nordbo.cosserat_rod_estimation.utils import *


from cosserat_nordbo.cosserat_rod_estimation.utils import plot_frame


#distance entre le capteur de force et le point d'attache de la corde
l=0.15
r=np.array([0,0,l])

sols = None

def parse_vector3(vector_str):
    """Parse a geometry_msgs.msg.Vector3 string into a NumPy array."""
    # print(f"Parsing Vector3: {vector_str}")

    components = vector_str.strip('geometry_msgs.msg.Vector3()').split(',')

    # for i, component in enumerate(components[:3]):
    #     print(f"Component {i}: {component}")
    #     print(f"Parsed value: {component.split('=')[1].strip(')')}")

    x, y, z = [float(component.split('=')[1].strip(")")) for component in components[:3]]
    
    result = np.array([x, y, z])
    # print(f"Parsed Vector3 result: {result}")
    return result

def parse_quaternion(quaternion_str):
    """Parse a geometry_msgs.msg.Quaternion string into a NumPy array."""
    # print(f"Parsing Quaternion: {quaternion_str}")

    components = quaternion_str.strip('geometry_msgs.msg.Quaternion()').split(',')

    # for i, component in enumerate(components[:4]):
    #     print(f"Component {i}: {component}")
    #     print(f"Parsed value: {component.split('=')[1].strip(')')}")

    x, y, z, w = [float(component.split('=')[1].strip(")")) for component in components[:4]]

    result = np.array([x, y, z, w])
    # print(f"Parsed Quaternion result: {result}")
    return result

def quaternion_to_rotation_matrix(quaternion):
    """Convert a quaternion to a rotation matrix."""
    # print(f"Converting Quaternion to Rotation Matrix: {quaternion}")
    rotation_matrix = R.from_quat(quaternion).as_matrix()
    # print(f"Rotation Matrix result: \n{rotation_matrix}")
    return rotation_matrix

def read_wrench_poses(file_path):
    """Read the wrench poses from the file and organize them into NumPy arrays."""
    positions1, rotations1 = [], []
    positions2, rotations2 = [], []
    forces, torques = [], []

    # print(f"Opening file: {file_path}")
    with open(file_path, 'r') as file:
        for line_number, line in enumerate(file, start=1):
            # print(f"Processing line {line_number}: {line.strip()}")
            # Split the line into components
            parts = line.split('geometry_msgs.msg.')
            # print("-------------------")
            # print("Split parts:")
            # for i, part in enumerate(parts):
            #     print(f"Part {i}:")
            #     print(part)
            #     print(part.strip())
            # print("-------------------")

            # Parse the first transform
            translation1 = parse_vector3(parts[2].strip())
            quaternion1 = parse_quaternion(parts[3].strip())
            # print(f"Parsed translation1: {translation1}")
            # print(f"Parsed quaternion1: {quaternion1}")
            positions1.append(translation1)
            rotations1.append(quaternion_to_rotation_matrix(quaternion1))

            # Parse the second transform
            translation2 = parse_vector3(parts[5].strip())
            quaternion2 = parse_quaternion(parts[6].strip())
            # print(f"Parsed translation2: {translation2}")
            # print(f"Parsed quaternion2: {quaternion2}")
            positions2.append(translation2)
            rotations2.append(quaternion_to_rotation_matrix(quaternion2))

            # Parse the wrench
            force = parse_vector3(parts[8].strip())
            torque = parse_vector3(parts[9].strip())
            # print(f"Parsed force: {force}")
            # print(f"Parsed torque: {torque}")
            forces.append(force)
            torques.append(torque)

    # Convert lists to NumPy arrays
    positions1 = np.array(positions1)
    rotations1 = np.array(rotations1)
    positions2 = np.array(positions2)
    rotations2 = np.array(rotations2)
    forces = np.array(forces)
    torques = np.array(torques)

    """print("Final shapes of arrays:")
    print(f"positions1 shape: {positions1.shape}")
    print(f"rotations1 shape: {rotations1.shape}")
    print(f"positions2 shape: {positions2.shape}")
    print(f"rotations2 shape: {rotations2.shape}")
    print(f"forces shape: {forces.shape}")
    print(f"torques shape: {torques.shape}")"""

    return positions1, rotations1, positions2, rotations2, forces, torques

# Example usage

import os

def test_read():

    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, 'wrench_poses.txt')

    print(f"Reading wrench poses from file: {file_path}")
    positions1, rotations1, positions2, rotations2, forces, torques = read_wrench_poses(file_path)



    torques1 = torques.copy()
    torques2 = torques.copy()


    for i, (torque, force) in enumerate(zip(torques, forces)):
        """print("--------------------")
        print("rotation:", rotations2[i])
        print("torque:", torque)
        print("force:", force)
        print("torque after:", torques[i])
        print("---------------------")
    """
        torques1[i] = torque - np.dot(r, force)
        torques2[i] = torque + np.dot(r, force) 



def initialize_plot(R2 =None,T2=None,R3=None,T3=None):
    
    fig = plt.figure(figsize=(16, 10))

    # 3D plot (large on the left)
    ax = fig.add_subplot(121, projection='3d')
    ax.set_xlim([-0.2, 0.3])
    ax.set_ylim([0.5, 0.99])
    ax.set_zlim([-0.0, 0.5])

    ax.set_box_aspect([1, 1, 1])  # For isotropic axes

    """print("positions1:", positions1[0])
    print("rotations1:", rotations1[0])
    print("positions2:", positions2[0])
    print("rotations2:", rotations2[0])"""

    # Frame 1: Identity
    R1 = np.eye(3)
    T1 = np.array([0, 0, 0])
    plot_frame(ax, R1, T1, length=0.2, name="F0")

    



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



    plot_frame(ax, R1, T1, length=0.2, name="F0")

    if R2 is not None :

        plot_frame(ax, R2, T2, length=0.2, name="start")


    if R3 is not None :

        plot_frame(ax, R3, T3, length=0.2, name="end")

    return fig, ax, ax2, ax3

R1 = np.eye(3)
T1 = np.array([0, 0, 0])

