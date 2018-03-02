import baxter_interface
import rospy
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

def execute(dmp_model, goal):
    list_of_postfix = get_eval_postfix(dmp_cmd_fields, 'pose')

    limb = 'right'
    limb_interface = baxter_interface.limb.Limb(limb)


    topic_name = "/robot/limb/%s/endpoint_state"%(limb,)
    topic_type = EndpointState
    endpoint_state_msg = get_topic_message_once(topic_name, topic_type)

    start = cook_array_from_object_using_postfixs(list_of_postfix, endpoint_state_msg.pose)
    end = cook_array_from_object_using_postfixs(list_of_postfix, goal)

    command_matrix = generalize_via_dmp(start, end, dmp_model)
    
    robot, group, plan, fraction = get_moveit_plan(command_matrix, dmp_cmd_fields, 'pose')
    rospy.loginfo('moveit plan success rate %s, Press enter to exec'%fraction)
    raw_input() 
    if rospy.is_shutdown():
        return False
    goal_achieved = introspect_moveit_exec(group, plan)
    return goal_achieved
