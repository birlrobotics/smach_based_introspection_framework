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

def execute(self, userdata):
    hmm_state_switch_client(0)
    show_everyhing_is_good()
    set_event_flag(ANOMALY_NOT_DETECTED)
    write_exec_hist(self, type(self).__name__, userdata, self.depend_on_prev_state)


    if hasattr(self, "get_pose_goal"):
        if hasattr(self, "before_motion"):
            self.before_motion()

        goal_pose = self.get_pose_goal()

        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander("right_arm")
        group.set_max_velocity_scaling_factor(0.3)
        group.set_max_acceleration_scaling_factor(0.3)
        group.set_start_state_to_current_state()
        group.set_pose_target(goal_pose)
        plan = group.plan()

        hmm_state_switch_client(self.state_no)

        goal_achieved = introspect_moveit_exec(group, plan)
        if not goal_achieved:
            no_need_to_revert = sos.handle_anomaly(self)
            rospy.loginfo('no_need_to_revert : %s'%no_need_to_revert)
            if no_need_to_revert:
                pass
            else:
                return "Revert"
                
        hmm_state_switch_client(0)

        if hasattr(self, "after_motion"):
            self.after_motion()

    return self.determine_successor()
