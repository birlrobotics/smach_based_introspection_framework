import rospy
from moveit_msgs.msg import MoveGroupActionResult, MoveItErrorCodes
import time
from smach_based_introspection_framework.online_part.framework_core.states import (
    set_event_flag,
    get_event_flag,
)
from smach_based_introspection_framework._constant import (
    ANOMALY_DETECTED,
    ANOMALY_NOT_DETECTED,
)
import threading

def introspect_moveit_exec(group, plan, timeout=1000):
    if plan is None or len(plan.joint_trajectory.points) == 0:
        return False

    t = threading.Thread(target=lambda : group.execute(plan, wait=True))
    t.run()
    s_t = time.time()
    while t.is_alive() and not rospy.is_shutdown() and time.time()-s_t<timeout: 
        if get_event_flag() == ANOMALY_DETECTED:
            if time.time()-s_t < 1.0:
                set_event_flag(ANOMALY_NOT_DETECTED)
            else:
                rospy.logerr("hmm signaled an anomaly")
                rospy.logerr("Moveit gonna stop")
                a_t = time.time()
                while time.time()-a_t < 1:
                    group.stop()
                rospy.sleep(0.1)
                group.stop()
                return False
    
    rospy.loginfo("Moveit exec done")
    return True
