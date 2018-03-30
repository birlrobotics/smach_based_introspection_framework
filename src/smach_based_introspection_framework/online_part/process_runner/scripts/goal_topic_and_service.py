import rospy
from smach_based_introspection_framework.srv import (
    UpdateGoalVector,
    UpdateGoalVectorRequest,
    UpdateGoalVectorResponse,
)
from smach_based_introspection_framework.msg import (
    GoalVector
)
import threading
import copy

access_shared_data = threading.Lock()
shared_goal_vector = None

def scb(req):
    global shared_goal_vector
    access_shared_data.acquire()
    shared_goal_vector = copy.deepcopy(req.goal_vector)
    access_shared_data.release()
    return UpdateGoalVectorResponse(True)

if __name__ == '__main__':
    rospy.init_node("goal_topic_and_service_node")
    rospy.loginfo("goal_topic_and_service.py starts")

    ss = rospy.Service("/observation/update_goal_vector", UpdateGoalVector, scb)
    pub = rospy.Publisher("/observation/goal_vector", GoalVector, queue_size=0)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        access_shared_data.acquire()
        pub.publish(shared_goal_vector)
        access_shared_data.release()

        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break

    rospy.loginfo("goal_topic_and_service.py ends")
