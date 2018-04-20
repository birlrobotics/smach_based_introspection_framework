from smach_based_introspection_framework.offline_part import util 
import numpy
import ipdb
from pyquaternion import Quaternion

def interpolate_pose_using_slerp(command_matrix, control_dimensions):
    pxyz_idx = util.get_position_xyz_index(control_dimensions)
    qxyzw_idx = util.get_quaternion_xyzw_index(control_dimensions)
    pos_mat = command_matrix[:, pxyz_idx].copy()
    quat_mat = command_matrix[:, qxyzw_idx].copy()


    distance_mat = numpy.linalg.norm(pos_mat[1:]-pos_mat[:-1], axis=1)
    steps = numpy.cumsum(distance_mat, axis=0)
    steps /= steps[-1]

    qxyzw0 = numpy.asarray(quat_mat[0]).reshape(-1)
    qxyzw1 = numpy.asarray(quat_mat[-1]).reshape(-1)

    if numpy.dot(qxyzw0, qxyzw1) < 0:
        qxyzw0 = -qxyzw0
    
    q0 = Quaternion(
        qxyzw0[3],
        qxyzw0[0],
        qxyzw0[1],
        qxyzw0[2],
    )
    q1 = Quaternion(
        qxyzw1[3],
        qxyzw1[0],
        qxyzw1[1],
        qxyzw1[2],
    )
    interpolated_q = [q0]
    for i in steps:
        interpolated_q.append(Quaternion.slerp(q0, q1, i))
    interpolated_mat = [[
        i.elements[1], 
        i.elements[2], 
        i.elements[3], 
        i.elements[0]] for i in interpolated_q]

    command_matrix[:, qxyzw_idx] = interpolated_mat
    return command_matrix

def plot_command_matrix_in_matplotlib(command_matrix, control_dimensions, title):
    from mpl_toolkits.mplot3d import axes3d
    import matplotlib.pyplot as plt
    from tf.transformations import (
        quaternion_inverse,
        quaternion_multiply,
    )

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    pxyz_idx = util.get_position_xyz_index(control_dimensions)
    qxyzw_idx = util.get_quaternion_xyzw_index(control_dimensions)

    for i in range(command_matrix.shape[0]):
        qxyzw = command_matrix[i][qxyzw_idx]
        uvw = quaternion_multiply(
            qxyzw,
            quaternion_multiply(
                [1,0,0,0], quaternion_inverse(qxyzw),
            )
        )
        x, y, z = command_matrix[i][pxyz_idx]
        u, v, w = uvw[:3]
        ax.quiver(x, y, z, u, v, w, length=0.01)
    ax.set_title(title)

    fig.show()

    

        

if __name__ == "__main__":
    import pickle
    command_matrix = numpy.loadtxt("test_data/command_matrix.txt")
    control_dimensions = pickle.load(open("test_data/control_dimensions.pkl", 'r')) 
    
    plot_command_matrix_in_matplotlib(command_matrix, control_dimensions, 'raw')

    command_matrix = interpolate_pose_using_slerp(command_matrix, control_dimensions)

    plot_command_matrix_in_matplotlib(command_matrix, control_dimensions, 'modified')

    raw_input()
