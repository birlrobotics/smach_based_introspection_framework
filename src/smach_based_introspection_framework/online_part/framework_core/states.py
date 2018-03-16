from smach_based_introspection_framework._constant import (
    ANOMALY_DETECTED,
    ANOMALY_NOT_DETECTED,
    ROLLBACK_RECOVERY_TAG,
)
import smach
import os
import rospy
import ipdb
from smach_based_introspection_framework.online_part.robot_screen_visualization.setter import(
    show_anomaly_detected,
    show_everyhing_is_good,
)
from smach_based_introspection_framework.srv import State_Switch, State_SwitchRequest

mode_no_state_trainsition_report = False 
event_flag = 1
execution_history = []
latest_anomaly_t = None

def get_anomaly_t():
    global latest_anomaly_t
    return latest_anomaly_t

def get_event_flag():
    global event_flag
    return event_flag

def set_event_flag(value):
    global event_flag, latest_anomaly_t
    event_flag = value
    if value == ANOMALY_NOT_DETECTED:
        latest_anomaly_t = None

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
    rospy.wait_for_service('hmm_state_switch')
    try:

        hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch',
                                                    State_Switch)
        req = State_SwitchRequest()
        req.state = state
        resp = hmm_state_switch_proxy(req)
        if resp.finish.data:
            rospy.loginfo("Hmm State switch to %d succesfully" %state)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)

class RollBackRecovery(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes)
        
    def execute(self, userdata):
        global execution_history
        hmm_state_switch_client(ROLLBACK_RECOVERY_TAG)

        rospy.loginfo("Enter RollBackRecovery State...")

        history_to_reexecute = None 
        while True:
            if len(execution_history) == 0:
                rospy.loginfo("no independent state found")
                return 'RecoveryFailed'
            elif execution_history[-1]['depend_on_prev_states']:
                execution_history.pop()
            else:
                history_to_reexecute = execution_history[-1]
                break

        if history_to_reexecute is None:
            rospy.loginfo("Cannot find reverting point in history_to_reexecute. Gonna abort")
            return 'RecoveryFailed'

        rospy.loginfo("About to revert, confirm? Or abort the whole task? Input confirm or abort.")
        while True:
            s = raw_input()
            if s == 'abort':
                return 'RecoveryFailed'
            elif s == 'confirm':
                break
            else:
                rospy.loginfo("input confirm or abort")


        state_name = history_to_reexecute['state_name']
        next_state = state_name
        rospy.loginfo('Gonna reenter %s'%(next_state,))
        rospy.sleep(5)
        return 'Reenter_'+next_state

def listen_HMM_anomaly_signal():
    hmm_state_switch_client(0)
    def callback_hmm(msg):
        global latest_anomaly_t
        if get_event_flag() != ANOMALY_DETECTED:
            rospy.logerr("hmm signaled an anomaly")
            set_event_flag(ANOMALY_DETECTED) 
            anomaly_t = msg.stamp.to_sec()
            if latest_anomaly_t is None:
                latest_anomaly_t = anomaly_t

    import std_msgs.msg
    rospy.Subscriber("/anomaly_detection_signal", std_msgs.msg.Header, callback_hmm)

