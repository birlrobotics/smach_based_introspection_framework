from smach_based_introspection_framework.offline_part import util 
import numpy
import ipdb

def interpolate_pose_using_slerp(command_matrix, control_dimensions):
    pxyz_idx = util.get_position_xyz_index(control_dimensions)
    qxyzw_idx = util.get_quaternion_xyzw_index(control_dimensions)
    pos_mat = command_matrix[:, pxyz_idx].copy()
    quat_mat = command_matrix[:, qxyzw_idx].copy()

    ipdb.set_trace()

    distance_mat = numpy.linalg.norm(pos_mat[1:]-pos_mat[:-1], axis=1)
    steps = numpy.cumsum(distance_mat, axis=0)
    steps /= steps[-1]

    q0 = numpy.asarray(quat_mat[0]).reshape(-1)
    q1 = numpy.asarray(quat_mat[-1]).reshape(-1)

    #command_matrix[:, ori_column_idx] = command_matrix[-1, ori_column_idx]
    return command_matrix

if __name__ == "__main__":
    import pickle
    command_matrix = numpy.loadtxt("test_data/command_matrix.txt")
    control_dimensions = pickle.load(open("test_data/control_dimensions.pkl", 'r')) 
    interpolate_pose_using_slerp(command_matrix, control_dimensions)
