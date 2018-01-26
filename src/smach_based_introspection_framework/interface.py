from core import (
    get_event_flag,
    set_event_flag,
    AnomalyDiagnosis,
    HumanTeachingRecovery,
    RollBackRecovery,
    write_exec_hist,
    hmm_state_switch_client,
    send_image,
    listen_HMM_anomaly_signal,
)
from constant import (
    ANOMALY_DETECTED,
    ANOMALY_DETECTION_BLOCKED, 
    ANOMALY_NOT_DETECTED,
    RECOVERY_JUST_DONE,
)
import copy

def execute_decorator(original_execute):
    def f(self, userdata): 
        if get_event_flag() == RECOVERY_JUST_DONE:
            set_event_flag(ANOMALY_NOT_DETECTED)
            rospy.loginfo("RECOVERY_JUST_DONE")
            send_image('green.jpg')
            return "Successful"
        else:
            if not hasattr(self, 'state_no'):
                state_no = 0 
            else:
                state_no = self.state_no

            if not hasattr(self, 'depend_on_prev_state'):
                depend_on_prev_state = False 
            else:
                depend_on_prev_state = True 
            write_exec_hist(self, type(self).__name__, userdata, depend_on_prev_state )

            if get_event_flag() != ANOMALY_DETECTION_BLOCKED:
                send_image('green.jpg')
                hmm_state_switch_client(state_no)

            ret = original_execute(self, userdata)

            hmm_state_switch_client(0)
            if get_event_flag() == ANOMALY_DETECTION_BLOCKED:
                set_event_flag(ANOMALY_NOT_DETECTED)
                rospy.loginfo("UnBlock anomlay detection")
                send_image('green.jpg')
            return ret
    return f

def modify_user_defined_sm(sm):
    import smach
    with sm:
        raw_user_states = copy.deepcopy(sm._states)

        # redirect all NeedRecovery to their respective anomay diagnosis
        for user_state in raw_user_states:
            state_name = user_state 
            state_transitions = sm._transitions[state_name]
            if "NeedRecovery" in state_transitions:
                state_instance = sm._states[state_name]
                state_no = state_instance.state_no
                ad_state_name = 'AnomalyDiagnosisForSkillNo%s'%state_no
                htr_state_name = "HumanTeachingRecoveryForSkillNo%s"%state_no
                state_transitions["NeedRecovery"] = ad_state_name

                smach.StateMachine.add(
                    ad_state_name,
                    AnomalyDiagnosis(outcomes=["GoToRollBackRecovery", "GoToHumanTeachingRecovery"]),
                    transitions={
                        'GoToRollBackRecovery': 'RollBackRecovery',
                        'GoToHumanTeachingRecovery': htr_state_name,
                    }
                )

                smach.StateMachine.add(
                    htr_state_name,
                    HumanTeachingRecovery(outcomes=["RecoveryDone"]),
                    transitions={
                        'RecoveryDone': state_name,
                    }
                )

        # build Recovery states automatically
        recovery_outcomes = ['RecoveryFailed']
        recovery_state_transitions = {
            'RecoveryFailed':'TaskFailed'
        }
        for user_state in raw_user_states:
            state_name = user_state 
            recovery_outcomes.append('Reenter_'+state_name)
            recovery_state_transitions['Reenter_'+state_name] = state_name

        smach.StateMachine.add(
			RollBackRecovery.__name__,
			RollBackRecovery(outcomes=recovery_outcomes),
            transitions=recovery_state_transitions
        )

    return sm

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
import rospy
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import baxter_interface
from std_msgs.msg import (
    Header,
)
import struct
class BreakOnAnomalyTrajectoryClient(object):

    def __init__(self, limb,verbose = False):
        # set up the action client
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        
        ## check up if the joint_trajectory action server setup
        ## please rosrun baxter_interface joint_trajectory_server
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

        #clear all  JointTrajectoryPoint
        self.clear(limb)
        self._verbose = verbose
        # enable the IK Service
        ik_ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_ns, SolvePositionIK)
        rospy.wait_for_service(ik_ns, 5.0)
        
        self._gripper = baxter_interface.Gripper(limb)
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()


    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def check_motion_anomaly(self):
        if get_event_flag() == ANOMALY_DETECTED:
            # anomaly detected
            return True 
        else:
            return False

    def wait(self, timeout=rospy.Duration(0.0)):
        if type(timeout) == int:
            timeout = rospy.Duration(timeout)
        timeout_time = rospy.get_rostime() + timeout
        while not rospy.is_shutdown():
            if get_event_flag() == ANOMALY_DETECTED:
                # anomaly detected
                return False

            if timeout != rospy.Duration(0.0) and rospy.get_rostime() > timeout_time:
                rospy.logerr("traj exec timeout")
                # timeout and not finished
                return False

            goal_achieved = self._client.gh.get_result()
            if goal_achieved is not None:
                # finished within timeout and no anomaly detected
                return True

    def result(self):
        return self._client.get_result()

    def add_pose_point(self,pose,time):
        angles = self.ik_request(pose)
        if not angles:
            return 0
        else:
            self.add_point(angles,time)
            return 1

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
  
    def ik_request(self,pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

        limb_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        limb_angles = [limb_joints[joint] for joint in limb_names]
        return limb_angles

def start_instrospection(
    no_state_trainsition_report=False, 
    no_anomaly_detection=False , 
    use_manual_anomaly_signal=False
):
    import core
    core.mode_no_state_trainsition_report = no_state_trainsition_report
    if not no_anomaly_detection:
        listen_HMM_anomaly_signal(use_manual_anomaly_signal)
