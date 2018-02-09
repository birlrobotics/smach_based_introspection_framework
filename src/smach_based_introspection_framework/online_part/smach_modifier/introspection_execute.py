import baxter_interface
import rospy
from smach_based_introspection_framework.online_part.motion_handler import (
    BreakOnAnomalyTrajectoryClient,
)
import sos
from smach_based_introspection_framework.online_part.framework_core.states import (
    hmm_state_switch_client,
    write_exec_hist,
)

def execute(self, userdata):
    hmm_state_switch_client(0)
    write_exec_hist(self, type(self).__name__, userdata, self.depend_on_prev_state)
    if hasattr(self, "before_motion"):
        self.before_motion()
    limb = 'right'
    traj = BreakOnAnomalyTrajectoryClient(limb)
    limb_interface = baxter_interface.limb.Limb(limb)
    traj.clear('right')
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, 0.0)
    
    goal_pose = self.get_pose_goal()
    traj.add_pose_point(goal_pose, 4.0)

    hmm_state_switch_client(self.state_no)
    traj.start()

    goal_achieved = traj.wait(5)
    if not goal_achieved:
        traj.stop()
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
