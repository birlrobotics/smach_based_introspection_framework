from smach_based_introspection_framework._constant import (
    ANOMALY_DETECTED,
    ANOMALY_NOT_DETECTED,
    ROLLBACK_RECOVERY_TAG,
    Q_TABLE_RECOVERY,
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
reverting_statistics = None
latest_anomaly_type = None

def set_reverting_statistics(value):
    global reverting_statistics
    reverting_statistics = value 

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

def set_latest_anomaly_type(value):
    global latest_anomaly_type
    latest_anomaly_type = value

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
    rospy.wait_for_service('hmm_state_switch', timeout=20)
    hmm_state_switch_proxy = rospy.ServiceProxy('hmm_state_switch',
                                                State_Switch)
    req = State_SwitchRequest()
    req.state = state
    resp = hmm_state_switch_proxy(req)

class CheckQTable(smach.State):
    def __init__(self, outcomes):
        # This outcomes is defined in modify_user_sm.py
        smach.State.__init__(self, outcomes)
        
    def execute(self, userdata):
        global execution_history, reverting_statistics
        hmm_state_switch_client(Q_TABLE_RECOVERY)

        rospy.loginfo("Enter CheckQTable State...")

        current_node = execution_history[-1]['state_name']
        try:
            stat = reverting_statistics[current_node][latest_anomaly_type]
        except KeyError:
            stat = None
        if stat is None:
            next_state = current_node
            rospy.loginfo('reverting_statistics contains no statistics about how to recover anomaly \'%s\' in state \'%s\', gonna reenter itself %s'%(latest_anomaly_type, current_node, next_state,))
        else:
            next_state = stat
        ipdb.set_trace()
        return 'Recovery_with_'+next_state

class RollBackRecovery(smach.State):
    def __init__(self, outcomes):
        # This outcomes is defined in modify_user_sm.py
        smach.State.__init__(self, outcomes)
        
    def execute(self, userdata):
        global execution_history, reverting_statistics
        hmm_state_switch_client(ROLLBACK_RECOVERY_TAG)

        rospy.loginfo("Enter RollBackRecovery State...")

        current_node = execution_history[-1]['state_name']
        if reverting_statistics is None:
            next_state = current_node
            rospy.loginfo('reverting_statistics is None, gonna reenter itself %s'%(next_state,))
        else:
            try:
                stat = reverting_statistics[current_node][latest_anomaly_type]
            except KeyError:
                stat = None
            if stat is None:
                next_state = current_node
                rospy.loginfo('reverting_statistics contains no statistics about how to recover anomaly \'%s\' in state \'%s\', gonna reenter itself %s'%(latest_anomaly_type, current_node, next_state,))
            else:
                # Compute the next state according to votes
                # Should be changed into Q table
                import numpy as np
                names = [i[0] for i in stat.items()]
                counts = np.array([i[1] for i in stat.items()], dtype=np.float64)
                prob = counts/counts.sum()
                idx = np.argwhere(np.random.multinomial(1, prob))[0][0]
                next_state = names[idx]
                rospy.loginfo('prob %s, gonna reenter %s'%(str(prob), next_state,))
            
        return 'Reenter_'+next_state

def listen_HMM_anomaly_signal():
    hmm_state_switch_client(0)
    def callback_hmm(msg):
        global latest_anomaly_t
        if get_event_flag() != ANOMALY_DETECTED:
            set_event_flag(ANOMALY_DETECTED) 
            anomaly_t = msg.stamp.to_sec()
            if latest_anomaly_t is None:
                latest_anomaly_t = anomaly_t

    import std_msgs.msg
    rospy.Subscriber("/anomaly_detection_signal", std_msgs.msg.Header, callback_hmm)

