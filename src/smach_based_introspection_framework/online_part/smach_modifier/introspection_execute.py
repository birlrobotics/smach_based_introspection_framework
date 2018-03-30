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
import time
import ipdb

def get_moveit_vars():
    if not hasattr(get_moveit_vars, 'robot'):
        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander("right_arm")
        group.set_max_velocity_scaling_factor(1)
        group.set_max_acceleration_scaling_factor(1)
        get_moveit_vars.robot = robot
        get_moveit_vars.group = group
    get_moveit_vars.group.clear_pose_targets()
    return get_moveit_vars.robot, get_moveit_vars.group

def execute(self, userdata):
    hmm_state_switch_client(0)
    show_everyhing_is_good()
    set_event_flag(ANOMALY_NOT_DETECTED)
    write_exec_hist(self, type(self).__name__, userdata, self.depend_on_prev_state)

    if hasattr(self, "before_motion"):
        self.before_motion()


    hmm_state_switch_client(self.state_no)


    robot, group = get_moveit_vars()

    pst = time.time()
    plan = None
    if hasattr(self, "get_joint_state_goal"):
        d = self.get_joint_state_goal()
        goal_joint = {k:d[k] for k in group.get_active_joints()}
        group.set_joint_value_target(goal_joint)
        plan = group.plan()
    elif hasattr(self, "get_pose_goal"):
        goal_pose = self.get_pose_goal()
        if not hasattr(self, 'get_dmp_model'):
            group.set_start_state_to_current_state()
            group.set_pose_target(goal_pose)
            plan = group.plan()
        else:
            dmp_model = self.get_dmp_model()
            plan = dmp_execute.get_dmp_plan(robot, group, dmp_model, goal_pose)

    pet = time.time()

    goal_achieved = True
    if plan is not None:
        rospy.loginfo("Took %s seconds to figure out a moveit plan"%(pet-pst))
        mst = time.time()
        goal_achieved = introspect_moveit_exec(group, plan)
        met = time.time()
        rospy.loginfo("Took %s seconds to exec moveit plan"%(met-mst))

    hmm_state_switch_client(0)
    if not goal_achieved:
        robot, group = get_moveit_vars()
        no_need_to_revert = sos.handle_anomaly(self, robot, group)
        rospy.loginfo('no_need_to_revert : %s'%no_need_to_revert)
        if no_need_to_revert:
            pass
        else:
            return "Revert"

    if hasattr(self, "after_motion"):
        self.after_motion()

    return self.determine_successor()
