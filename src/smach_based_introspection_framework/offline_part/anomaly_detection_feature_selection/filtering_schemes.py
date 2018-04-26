from geometry_msgs.msg import WrenchStamped
from baxter_core_msgs.msg import EndpointState 
import msg_filters
from rostopics_to_timeseries import RosTopicFilteringScheme
from smach_based_introspection_framework.msg import tactile_static
import tactilesensors4.msg

filtering_schemes = []

# A new scheme
tfc = RosTopicFilteringScheme()
tfc.add_filter(
    "/TactileSensor4/StaticData", 
    tactilesensors4.msg.StaticData,
    msg_filters.TactileStaticMaxFilter(),
)
tfc.add_filter(
    "/TactileSensor4/Dynamic", 
    tactilesensors4.msg.Dynamic,
    msg_filters.TactileDynamicAbsMaxFilter(),
)
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
