from smach_based_introspection_framework._constant import (
    ANOMALY_DETECTED,
    ANOMALY_DETECTION_BLOCKED, 
    ANOMALY_NOT_DETECTED,
    RECOVERY_JUST_DONE,
    ROLLBACK_RECOVERY_TAG,
    RECOVERY_DEMONSTRATED_BY_HUMAN_TAG,
)
import smach
import os
import rospy
import ipdb
from smach_based_introspection_framework.online_part.robot_screen_visualization.setter import(
    show_anomaly_detected,
    show_everyhing_is_good,
)

mode_no_state_trainsition_report = False 
event_flag = 1
execution_history = []
latest_anomaly_t = None

def get_event_flag():
    global event_flag
    return event_flag

def set_event_flag(value):
    global event_flag
    event_flag = value

## @brief record exec history
## @param current_state_name string
## @param current_userdata userdata passed into current state 
## @param depend_on_prev_states True if current state's success depends on previous states 
## @return None
def write_exec_hist(state_instance, current_state_name, current_userdata, depend_on_prev_states):
    import copy
    global execution_history

    saved_userdata = {}
    for k in state_instance._input_keys:
        saved_userdata[k] = copy.deepcopy(current_userdata[k])

    execution_history.append(
        {
            "state_name": current_state_name,
            "saved_userdata": saved_userdata,
            "depend_on_prev_states": depend_on_prev_states
        }
    )

def hmm_state_switch_client(state):
    global mode_no_state_trainsition_report
    if mode_no_state_trainsition_report:
        print 'mode_no_state_trainsition_report'
        return
    rospy.wait_for_service('hmm_state_switch')
    try:

        from birl_baxter_tasks.srv import State_Switch, State_SwitchRequest
        hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch',
                                                    State_Switch)
        req = State_SwitchRequest()
        req.state = state
        resp = hmm_state_switch_proxy(req)
        if resp.finish.data:
            print "Hmm State switch to %d succesfully" %state
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

class AnomalyDiagnosis(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
    def execute(self, userdata):
        global latest_anomaly_t

        hmm_state_switch_client(-1)
        show_anomaly_detected()
        rospy.sleep(5)

        from AnomalyClassification import AnomalyClassification
        ac = AnomalyClassification()
        anomaly_t = latest_anomaly_t
        latest_anomaly_t = None
        if ac.classify_anomaly_at(anomaly_t):
            pass
        else:
            while True:
                rospy.loginfo("input a number to proceed:")
                rospy.loginfo("%s -- roll back recovery"%ROLLBACK_RECOVERY_TAG)
                rospy.loginfo("%s -- human teaching recovery"%RECOVERY_DEMONSTRATED_BY_HUMAN_TAG)
                rospy.loginfo("-4 -- dry-run classifier and dmp")
                rospy.loginfo("-5 -- run classifier and dmp")
                i = raw_input()
                if i == str(ROLLBACK_RECOVERY_TAG):
                    return 'GoToRollBackRecovery'
                elif i == str(RECOVERY_DEMONSTRATED_BY_HUMAN_TAG):
                    return 'GoToHumanTeachingRecovery'


class HumanTeachingRecovery(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
    def execute(self, userdata):
        hmm_state_switch_client(RECOVERY_DEMONSTRATED_BY_HUMAN_TAG)

        while True:
            rospy.loginfo("enter \"s\" to start teaching, \"f\" to finish")
            i = raw_input()
            if i == 's':
                rospy.loginfo("start")
                hmm_state_switch_client(RECOVERY_DEMONSTRATED_BY_HUMAN_TAG)
                break
        while True:
            rospy.loginfo("enter \"f\" to finish")
            i = raw_input()
            if i == 'f':
                rospy.loginfo("end")
                hmm_state_switch_client(0)
                break

        set_event_flag(RECOVERY_JUST_DONE)
        return 'RecoveryDone'
    
class RollBackRecovery(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
        
    def execute(self, userdata):
        global execution_history
        hmm_state_switch_client(ROLLBACK_RECOVERY_TAG)

        rospy.loginfo("Enter RollBackRecovery State...")
        rospy.loginfo("Block anomlay detection")

        history_to_reexecute = None 
        while True:
            if len(execution_history) == 0:
                rospy.loginfo("no execution_history found")
            elif execution_history[-1]['depend_on_prev_states']:
                execution_history.pop()
            else:
                history_to_reexecute = execution_history[-1]
                break

        if history_to_reexecute is None:
            return 'RecoveryFailed'

        state_name = history_to_reexecute['state_name']
        next_state = state_name
        rospy.loginfo('Gonna reenter %s'%(next_state,))

        rospy.loginfo("Block anomlay detection for the next state")
        set_event_flag(ANOMALY_DETECTION_BLOCKED)
        rospy.sleep(5)
        return 'Reenter_'+next_state

def listen_HMM_anomaly_signal(use_manual_anomaly_signal):
    def callback_hmm(msg):
        global latest_anomaly_t
        print msg
        if get_event_flag() != ANOMALY_DETECTION_BLOCKED:
            rospy.logerr("hmm signaled an anomaly")
            set_event_flag(ANOMALY_DETECTED) 
            anomaly_t = msg.stamp.to_sec()
            if latest_anomaly_t is None:
                latest_anomaly_t = anomaly_t

    import std_msgs.msg
    if use_manual_anomaly_signal:
        rospy.Subscriber("/manual_anomaly_signal", std_msgs.msg.Header, callback_hmm)
    else:
        rospy.Subscriber("/anomaly_detection_signal", std_msgs.msg.Header, callback_hmm)

