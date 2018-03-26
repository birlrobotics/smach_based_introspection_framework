import baxter_interface
import rospy
import copy
import numpy
from smach_based_introspection_framework.configurables import (
    dmp_cmd_fields,
)
from birl_skill_management.dmp_management import (
    cook_array_from_object_using_postfixs,
)
from birl_skill_management.util import (
    get_eval_postfix,
    get_moveit_plan,
)
import baxter_interface
from smach_based_introspection_framework.online_part.framework_core.states import (
    get_event_flag,
)
import ipdb
from baxter_core_msgs.msg import EndpointState
from birl_runtime_parameter_filler.util import get_topic_message_once
from control_msgs.msg import FollowJointTrajectoryActionResult
from smach_based_introspection_framework._constant import (
    ANOMALY_DETECTED,
)
import time
from util import introspect_moveit_exec
from birl_dmp.dmp_training.util import generalize_via_dmp
from smach_based_introspection_framework.srv import (
    UpdateGoalVector,
    UpdateGoalVectorRequest,
    UpdateGoalVectorResponse,
)
from tf.transformations import (
    quaternion_multiply,
)

def norm_quaternion(command_matrix, control_dimensions):
    from sklearn import preprocessing
    ori_column_idx = []
    for idx, dim in enumerate(control_dimensions):
        if 'orientation' in dim:
            ori_column_idx.append(idx)

    q = command_matrix[:, ori_column_idx]
    nq = preprocessing.normalize(q)
    command_matrix[:, ori_column_idx] = nq
    return command_matrix

def lock_last_quaternion(command_matrix, control_dimensions):
    ori_column_idx = []
    for idx, dim in enumerate(control_dimensions):
        if 'orientation' in dim:
            ori_column_idx.append(idx)
    command_matrix[:, ori_column_idx] = command_matrix[-1, ori_column_idx]
    return command_matrix
    

def filter_close_points(_mat):
    mat = numpy.flip(_mat, axis=0)

    last = mat[0].copy()
    new_mat = [last.copy()]
    for i in range(mat.shape[0]):
        if numpy.linalg.norm(mat[i]-last) < 0.01:
            continue

        new_mat.append(mat[i].copy())
        last = mat[i].copy()

    return numpy.flip(numpy.matrix(new_mat), axis=0)

def update_goal_vector(vec):
    sp = rospy.ServiceProxy("/observation/update_goal_vector", UpdateGoalVector)
    req = UpdateGoalVectorRequest()
    req.goal_vector = vec
    sp.call(req)

def execute(dmp_model, goal, goal_modification_info=None):
    list_of_postfix = get_eval_postfix(dmp_cmd_fields, 'pose')

    limb = 'right'
    limb_interface = baxter_interface.limb.Limb(limb)


    topic_name = "/robot/limb/%s/endpoint_state"%(limb,)
    topic_type = EndpointState
    endpoint_state_msg = get_topic_message_once(topic_name, topic_type)

    start = numpy.array(cook_array_from_object_using_postfixs(list_of_postfix, endpoint_state_msg.pose))
    end = numpy.array(cook_array_from_object_using_postfixs(list_of_postfix, goal))

    if goal_modification_info is not None:
        new_goal = copy.deepcopy(end)
        if 'translation' in goal_modification_info:
            pxyz_idx = goal_modification_info['translation']['index'] 
            new_goal[pxyz_idx] = end[pxyz_idx]+goal_modification_info['translation']['value']
        if 'quaternion_rotation' in goal_modification_info:
            qxyzw_idx = goal_modification_info['quaternion_rotation']['index']
            rot_q = goal_modification_info['quaternion_rotation']['value']
            new_goal[qxyzw_idx] = quaternion_multiply(rot_q, end[qxyzw_idx])
        end = new_goal

    command_matrix = generalize_via_dmp(start, end, dmp_model)

    command_matrix = norm_quaternion(command_matrix, dmp_cmd_fields)
    command_matrix = lock_last_quaternion(command_matrix, dmp_cmd_fields)
    command_matrix = filter_close_points(command_matrix)

    update_goal_vector(command_matrix[-1].tolist()[0])
    
    robot, group, plan, fraction = get_moveit_plan(command_matrix, dmp_cmd_fields, 'pose')
    rospy.loginfo('moveit plan success rate %s, Press enter to exec'%fraction)
    if rospy.is_shutdown():
        return False
    goal_achieved = introspect_moveit_exec(group, plan)
    return goal_achieved
