import baxter_interface
import rospy
from smach_based_introspection_framework.online_part.motion_handler import (
    BreakOnAnomalyTrajectoryClient,
)
from smach_based_introspection_framework.online_part.configurables import (
    dmp_cmd_fields,
)
import sos
from smach_based_introspection_framework.online_part.framework_core.states import (
    get_anomaly_t,
    hmm_state_switch_client,
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

def execute(dmp_model, goal):
    list_of_postfix = get_eval_postfix(dmp_cmd_fields, 'pose')

    limb = 'right'
    limb_interface = baxter_interface.limb.Limb(limb)
    start = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    end = cook_array_from_object_using_postfixs(list_of_postfix, goal)
    command_matrix = dmp_imitate(starting_pose=start, ending_pose=end, weight_mat=dmp_model["basis_weight"], base_fuc=dmp_model["basis_function_type"])
    
    robot, group, plan, fraction = get_moveit_plan(command_matrix, dmp_cmd_fields, 'pose')
    if fraction < 0.9:
        raise Exception("moveit plan success rate < 0.9")
    group.execute(plan, wait=False)
    while not group._g.execute_action_client_.getResult():
        if get_event_flag() == ANOMALY_DETECTED:
            group.stop()
            return False
    return True
