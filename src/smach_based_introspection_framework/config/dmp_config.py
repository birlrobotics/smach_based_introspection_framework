from rostopics_to_timeseries import TopicMsgFilter
from baxter_core_msgs.msg import EndpointState 
from rostopics_to_timeseries import RosTopicFilteringScheme

class BaxterEndpointFilterForDmpCmd(TopicMsgFilter):
    def __init__(self):
        super(BaxterEndpointFilterForDmpCmd, self).__init__()

    def convert(self, msg):
        return [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ] 

    @staticmethod
    def vector_size():
        return 7

    @staticmethod
    def vector_meaning():
        return [
            'baxter_enpoint_pose.pose.position.x', \
            'baxter_enpoint_pose.pose.position.y', \
            'baxter_enpoint_pose.pose.position.z', \
            'baxter_enpoint_pose.pose.orientation.x', \
            'baxter_enpoint_pose.pose.orientation.y', \
            'baxter_enpoint_pose.pose.orientation.z', \
            'baxter_enpoint_pose.pose.orientation.w', \
        ] 

dmp_cmd_timeseries_hz = 100
dmp_cmd_timeseries_config = RosTopicFilteringScheme(dmp_cmd_timeseries_hz)
dmp_cmd_timeseries_config.add_filter(
    "/robot/limb/right/endpoint_state", 
    EndpointState,
    BaxterEndpointFilterForDmpCmd,
)
