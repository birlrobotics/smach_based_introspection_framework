from rostopics_to_timeseries.RostopicsToTimeseries import OnlineRostopicsToTimeseries
import rospy
from smach_based_introspection_framework.configurables import (
    tfc,
)

if __name__ == '__main__':
    rospy.init_node("timeseries_publisher_node") # Needed to init a ROS node first
    rospy.loginfo("timeseries_publisher.py starts")
    # Pass in topic configuration and timeseries rate
    onrt = OnlineRostopicsToTimeseries(tfc) 
    # Pass in the topic name that publishes the timeseries vector
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic") 
    rospy.loginfo("timeseries_publisher.py exits")
