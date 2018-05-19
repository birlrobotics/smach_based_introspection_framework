from rostopics_to_timeseries.RostopicsToTimeseries import OnlineRostopicsToTimeseries
import multiprocessing 
import rospy
from smach_based_introspection_framework.configurables import (
    tfc,
    anomaly_filtering_scheme,
)

class TimeseriesPubProc(multiprocessing.Process):
    def __init__(
        self, 
        filtering_config, 
        node_name="TimeseriesPubProc_node",
        topic_name="/rostopics_to_timeseries_topic",
    ):
        multiprocessing.Process.__init__(self)     

        self.tfc = filtering_config
        self.node_name = node_name
        self.topic_name = topic_name

    def run(self):
        rospy.init_node(self.node_name, anonymous=True)
        # Pass in topic configuration and timeseries rate
        onrt = OnlineRostopicsToTimeseries(self.tfc) 
        # Pass in the topic name that publishes the timeseries vector
        rospy.loginfo("%s starts timeseries publishing."%self.node_name)
        onrt.start_publishing_timeseries(self.topic_name) 

if __name__ == '__main__':

    ad_timeseries_pub = TimeseriesPubProc(
        tfc,
        node_name="TimeseriesPubProc_node_for_anomaly_detection",
        topic_name="timeseries_topic_for_anomaly_detection",
    )
    ad_timeseries_pub.daemon = True
    ad_timeseries_pub.start()

    ac_timeseries_pub = TimeseriesPubProc(
        anomaly_filtering_scheme,
        node_name="TimeseriesPubProc_node_for_anomaly_classification",
        topic_name="timeseries_topic_for_anomaly_classification",
    )
    ac_timeseries_pub.daemon = True
    ac_timeseries_pub.start()


    rospy.init_node("timeseries_publisher_master_node")
    rospy.loginfo("timeseries_publisher.py starts")
    rospy.spin()
    rospy.loginfo("timeseries_publisher.py exits")
