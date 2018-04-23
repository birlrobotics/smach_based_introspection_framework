from geometry_msgs.msg import WrenchStamped
from baxter_core_msgs.msg import EndpointState 
import msg_filters
from rostopics_to_timeseries import RosTopicFilteringScheme

filtering_schemes = []


tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters.WrenchStampedFilter(),
)
filtering_schemes.append(tfc)


tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters.WrenchStampedNormFilter(),
)
filtering_schemes.append(tfc)


tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters.WrenchStampedNormFilter(),
)
tfc.add_filter(
    "/robot/limb/right/endpoint_state", 
    EndpointState,
    msg_filters.BaxterEndpointTwistNormFilter(),
)
filtering_schemes.append(tfc)


tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/robot/limb/right/endpoint_state", 
    EndpointState,
    msg_filters.BaxterEndpointTwistNormFilter(),
)
filtering_schemes.append(tfc)
