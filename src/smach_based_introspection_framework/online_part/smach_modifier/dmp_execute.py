import baxter_interface
import rospy
from smach_based_introspection_framework.online_part.motion_handler import (
    BreakOnAnomalyTrajectoryClient,
)
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
from birl_baxter_dmp.dmp_generalize import dmp_imitate
import ipdb
from baxter_core_msgs.msg import EndpointState
from birl_runtime_parameter_filler.util import get_topic_message_once
from control_msgs.msg import FollowJointTrajectoryActionResult
from smach_based_introspection_framework._constant import (
    ANOMALY_DETECTED,
)
import time

def execute(dmp_model, goal):
    list_of_postfix = get_eval_postfix(dmp_cmd_fields, 'pose')

    limb = 'right'
    limb_interface = baxter_interface.limb.Limb(limb)


    topic_name = "/robot/limb/%s/endpoint_state"%(limb,)
    topic_type = EndpointState
    endpoint_state_msg = get_topic_message_once(topic_name, topic_type)

    start = cook_array_from_object_using_postfixs(list_of_postfix, endpoint_state_msg.pose)
    end = cook_array_from_object_using_postfixs(list_of_postfix, goal)


    command_matrix = dmp_imitate(starting_pose=start, ending_pose=end, weight_mat=dmp_model["basis_weight"], base_fuc=dmp_model["basis_function_type"])
    
    robot, group, plan, fraction = get_moveit_plan(command_matrix, dmp_cmd_fields, 'pose')
    if fraction < 0.5:
        raise Exception("moveit plan success rate < 0.5")
    
    def cb(data):
        cb.result = data
    cb.result = None
    sub_handle = rospy.Subscriber('/robot/limb/right/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, cb)

    group.execute(plan, wait=False)
    s_t = time.time()
    while not cb.result and not rospy.is_shutdown() and time.time()-s_t<10: 
        if get_event_flag() == ANOMALY_DETECTED:
            group.stop()
            return False
    if cb.result is None:
        rospy.logerr("dmp moveit exec did not return after 10 secs. Time out.")
        return False
        
    if cb.result.result.error_code != 0:
        rospy.logerr("dmp moveit exec returns: %s"%cb.result)
        return False
    sub_handle.unregister()
    return True