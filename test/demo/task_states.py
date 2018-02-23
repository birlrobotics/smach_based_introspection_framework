import smach
import hardcoded_data
import baxter_interface
import rospy
import copy

class CalibrateForceSensor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 1
        self.depend_on_prev_state = False

    def after_motion(self):
        from std_srvs.srv import Trigger
        try:
            rospy.wait_for_service('/robotiq_wrench_calibration_service', timeout=3)
            trigger = rospy.ServiceProxy('/robotiq_wrench_calibration_service', Trigger)
            resp = trigger()
            rospy.sleep(5)
        except Exception as exc:
            rospy.logerr("calling force sensor calibration failed: %s"%exc)

    def get_pose_goal(self):
        return hardcoded_data.calibration_pose
    
    def determine_successor(self):
        return 'Successful'

class GotoPickHoverWithoutObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Successful'])
        self.state_no = 2
        self.depend_on_prev_state = True

    def get_pose_goal(self):
        return hardcoded_data.hover_pick_object_pose

    def determine_successor(self):
        return 'Successful'

class GoToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Successful'])
        self.state_no = 3
        self.depend_on_prev_state = True 
        
    def before_motion(self):
        limb = 'right'
        baxter_interface.Gripper(limb).open()

    def after_motion(self):
        limb = 'right'
        baxter_interface.Gripper(limb).close()

    def get_pose_goal(self):
        return hardcoded_data.pick_object_pose

    def determine_successor(self):
        return 'Successful'

class GoToPickHoverWithObject(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Successful'])
        self.state_no = 4
        self.depend_on_prev_state = True
        
    def get_pose_goal(self):
        return hardcoded_data.hover_pick_object_pose

    def determine_successor(self):
        return 'Successful'

def assembly_user_defined_sm():
    sm = smach.StateMachine(outcomes=['TaskFailed', 'TaskSuccessful'])
    with sm:
        smach.StateMachine.add(
            CalibrateForceSensor.__name__,
            CalibrateForceSensor(),
            transitions={
                'Successful': GotoPickHoverWithoutObject.__name__,
            }
        )

        smach.StateMachine.add(
            GotoPickHoverWithoutObject.__name__,
            GotoPickHoverWithoutObject(),
            transitions={
                'Successful': GoToPickPosition.__name__,
            }
        )

        smach.StateMachine.add(
			GoToPickPosition.__name__,
			GoToPickPosition(),
            transitions={
                'Successful': GoToPickHoverWithObject.__name__,
            }
        )

        smach.StateMachine.add(
			GoToPickHoverWithObject.__name__,
			GoToPickHoverWithObject(),
            transitions={
                'Successful':'TaskSuccessful'
            }
        )

    return sm
