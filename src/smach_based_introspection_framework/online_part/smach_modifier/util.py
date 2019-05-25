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
import ipdb
def introspect_moveit_exec(group, plan):
    if plan is None or len(plan.joint_trajectory.points) == 0:
        return False

    timeout = plan.joint_trajectory.points[-1].time_from_start+rospy.Duration(1)

    group.execute(plan, wait=False)

    s_t = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now()-s_t<timeout: 
        if get_event_flag() == ANOMALY_DETECTED:
            time_diff = rospy.Time.now()-s_t
            if time_diff.to_sec() < 1:
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
