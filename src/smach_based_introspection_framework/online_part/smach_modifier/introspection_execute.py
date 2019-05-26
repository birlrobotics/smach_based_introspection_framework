import rospy
import sos
from smach_based_introspection_framework.online_part.framework_core.states import (
    hmm_state_switch_client,
    write_exec_hist,
    set_event_flag,
)
from smach_based_introspection_framework.online_part.robot_screen_visualization.setter import(
    show_everyhing_is_good,
)
from smach_based_introspection_framework._constant import (
    ANOMALY_NOT_DETECTED,
)
import moveit_commander
from util import introspect_moveit_exec
import dmp_execute
import time
import ipdb

get_moveit_vars = None

def execute(self, userdata):
    hmm_state_switch_client(0)
    show_everyhing_is_good()
    set_event_flag(ANOMALY_NOT_DETECTED)
    write_exec_hist(self, type(self).__name__, userdata, self.depend_on_prev_state)

    if hasattr(self, "before_motion"):
        self.before_motion()


    hmm_state_switch_client(self.state_no)


    robot, group = get_moveit_vars()

    pst = rospy.Time.now().to_sec()
    plan = None
    if hasattr(self, "get_modified_plan"):
        plan = self.get_modified_plan(robot,group)

    elif hasattr(self, "get_joint_state_goal"):
        d = self.get_joint_state_goal()
        goal_joint = {k:d[k] for k in group.get_active_joints()}
        group.set_joint_value_target(goal_joint)
        group.set_start_state_to_current_state()
        plan = group.plan()
    elif hasattr(self, "get_pose_goal"):
        goal_pose = self.get_pose_goal()
        if not hasattr(self, 'get_dmp_model'):
            group.set_start_state_to_current_state()
            group.set_pose_target(goal_pose)
            plan = group.plan()
        else:
            dmp_model = self.get_dmp_model()
            group.set_start_state_to_current_state()
            plan = dmp_execute.get_dmp_plan(robot, group, dmp_model, goal_pose)
    pet = rospy.Time.now().to_sec()

    goal_achieved = True
    if plan is not None:
        rospy.logdebug("Took %s seconds to figure out a moveit plan"%(pet-pst))
        mst = rospy.Time.now().to_sec()
        goal_achieved = introspect_moveit_exec(group, plan)
        met = rospy.Time.now().to_sec()
        rospy.logdebug("Took %s seconds to exec moveit plan"%(met-mst))
    hmm_state_switch_client(0)
    if not goal_achieved:
        sos.handle_anomaly_with_qtable(self)
        return "Recovery"

    if hasattr(self, "after_motion"):
        self.after_motion()

    return self.determine_successor()
