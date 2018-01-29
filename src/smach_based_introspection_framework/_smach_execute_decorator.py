from constant import (
    ANOMALY_DETECTED,
    ANOMALY_DETECTION_BLOCKED, 
    ANOMALY_NOT_DETECTED,
    RECOVERY_JUST_DONE,
)
from core import (
    get_event_flag,
    set_event_flag,
    write_exec_hist,
    hmm_state_switch_client,
    send_image,
)

def smach_execute_decorator(original_execute):
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
