import baxter_interface
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

def execute(self, userdata):
    hmm_state_switch_client(0)
    show_everyhing_is_good()
    set_event_flag(ANOMALY_NOT_DETECTED)
    write_exec_hist(self, type(self).__name__, userdata, self.depend_on_prev_state)

    if hasattr(self, "before_motion"):
        self.before_motion()

    goal_achieved = True
    hmm_state_switch_client(self.state_no)


    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.set_max_velocity_scaling_factor(1)
    group.set_max_acceleration_scaling_factor(1)



    if hasattr(self, "get_joint_state_goal"):
        d = self.get_joint_state_goal()
        goal_joint = {k:d[k] for k in group.get_active_joints()}

        group.set_joint_value_target(goal_joint)
        group.go(wait=True)
        goal_achieved = True
    elif hasattr(self, "get_pose_goal"):

        goal_pose = self.get_pose_goal()

        if not hasattr(self, 'get_dmp_model'):
            group.set_start_state_to_current_state()
            group.set_pose_target(goal_pose)
            plan = group.plan()
            goal_achieved = introspect_moveit_exec(group, plan)
        else:
            dmp_model = self.get_dmp_model()
            goal_achieved = dmp_execute.execute(dmp_model, goal_pose)

    hmm_state_switch_client(0)
    if not goal_achieved:
        no_need_to_revert = sos.handle_anomaly(self)
        rospy.loginfo('no_need_to_revert : %s'%no_need_to_revert)
        if no_need_to_revert:
            pass
        else:
            return "Revert"

    if hasattr(self, "after_motion"):
        self.after_motion()

    return self.determine_successor()
