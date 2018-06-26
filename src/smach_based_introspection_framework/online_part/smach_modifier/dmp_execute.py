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
    plot_cmd_matrix,
)
from smach_based_introspection_framework.online_part.framework_core.states import (
    get_event_flag,
)
import ipdb
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
from quaternion_interpolation import interpolate_pose_using_slerp

def _get_moveit_plan(robot, group, command_matrix, control_dimensions, control_mode):
    from geometry_msgs.msg import (
        Pose,
    )

    plot_cmd_matrix(command_matrix, control_dimensions, control_mode)
    
    list_of_postfix = get_eval_postfix(control_dimensions, control_mode)
    list_of_poses = []
    for row_no in range(command_matrix.shape[0]):
        pose = Pose() 
        for col_no in range(command_matrix.shape[1]):
            exec_str = 'pose'+list_of_postfix[col_no]+'=command_matrix[row_no, col_no]'
            exec(exec_str)
        list_of_poses.append(pose)



    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)


    plan = None
    fraction = None
    group.set_start_state_to_current_state()
    plan, fraction = group.compute_cartesian_path(
                             list_of_poses,   # waypoints to follow
                             0.1,        # eef_step
                             0.0)         # jump_threshold

    return plan, fraction


def update_goal_vector(vec):
    sp = rospy.ServiceProxy("/observation/update_goal_vector", UpdateGoalVector)
    req = UpdateGoalVectorRequest()
    req.goal_vector = vec
    sp.call(req)

def _get_dmp_plan(robot, group, dmp_model, goal, goal_modification_info=None):
    list_of_postfix = get_eval_postfix(dmp_cmd_fields, 'pose')

    current_pose = group.get_current_pose().pose
    start = numpy.array(cook_array_from_object_using_postfixs(list_of_postfix, current_pose))
    end = numpy.array(cook_array_from_object_using_postfixs(list_of_postfix, goal))

    if goal_modification_info is not None:
        gmst = rospy.Time.now().to_sec()
        new_goal = copy.deepcopy(end)
        if 'translation' in goal_modification_info:
            pxyz_idx = goal_modification_info['translation']['index'] 
            new_goal[pxyz_idx] = end[pxyz_idx]+goal_modification_info['translation']['value']
        if 'quaternion_rotation' in goal_modification_info:
            qxyzw_idx = goal_modification_info['quaternion_rotation']['index']
            rot_q = goal_modification_info['quaternion_rotation']['value']
            new_goal[qxyzw_idx] = quaternion_multiply(rot_q, end[qxyzw_idx])
        end = new_goal
        gmet = rospy.Time.now().to_sec()
        rospy.logdebug("Took %s seconds to modify goal"%(gmet-gmst))

    st = rospy.Time.now().to_sec()
    command_matrix = generalize_via_dmp(start, end, dmp_model)
    rospy.logdebug("Took %s seconds to call generalize_via_dmp"%(rospy.Time.now().to_sec()-st))
    st = rospy.Time.now().to_sec()
    command_matrix = interpolate_pose_using_slerp(command_matrix, dmp_cmd_fields)
    rospy.logdebug("Took %s seconds to call interpolate_pose_using_slerp"%(rospy.Time.now().to_sec()-st))

    if goal_modification_info is None:
        update_goal_vector(numpy.asarray(command_matrix[-1]).reshape(-1).tolist())
    
    st = rospy.Time.now().to_sec()
    plan, fraction = _get_moveit_plan(robot, group, command_matrix, dmp_cmd_fields, 'pose')
    rospy.logdebug("Took %s seconds to call _get_moveit_plan"%(rospy.Time.now().to_sec()-st))

    rospy.loginfo('moveit plan success rate %s'%fraction)
    return plan, fraction

def get_dmp_plan(robot, group, dmp_model, goal, goal_modification_info=None):
    while not rospy.is_shutdown():
        plan, fraction = _get_dmp_plan(robot, group, dmp_model, goal, goal_modification_info)
        if fraction < 1:
            rospy.loginfo("fraction %s, hit 'Enter' to plan again. or enter anything to move on."%fraction)
            s = raw_input()
            if s == '':
                continue
            else:
                break
        else:
            rospy.loginfo("plan succeeded, press Enter to execute.")
            s = raw_input()
            break

    if rospy.is_shutdown():
        raise Exception("rospy.is_shutdown")

    return plan
