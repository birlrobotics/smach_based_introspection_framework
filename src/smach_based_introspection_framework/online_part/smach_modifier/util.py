import rospy
from control_msgs.msg import FollowJointTrajectoryActionResult
import time
from smach_based_introspection_framework.online_part.framework_core.states import (
    get_event_flag,
)
from smach_based_introspection_framework._constant import (
    ANOMALY_DETECTED,
)

def introspect_moveit_exec(group, plan, timeout=1000):
    if plan is None or len(plan.joint_trajectory.points) == 0:
        return False


    def cb(data):
        cb.result = data
    cb.result = None
    sub_handle = rospy.Subscriber('/robot/limb/right/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, cb)

    group.execute(plan, wait=False)
    s_t = time.time()
    while not cb.result and not rospy.is_shutdown() and time.time()-s_t<timeout: 
        if get_event_flag() == ANOMALY_DETECTED:
            group.stop()
            return False
    
    if cb.result is None:
        rospy.logerr("dmp moveit exec did not return after %s secs. Time out."%timeout)
        return False

    if cb.result.result.error_code != 0:
        rospy.logerr("dmp moveit exec returns: %s"%cb.result)
        return False
    sub_handle.unregister()
    return True
