from geometry_msgs.msg import WrenchStamped
from baxter_core_msgs.msg import EndpointState 
import msg_filters
from rostopics_to_timeseries import RosTopicFilteringScheme
from smach_based_introspection_framework.msg import tactile_static
import tactilesensors4.msg
import itertools

filtering_schemes = []

args = []

args.append([
    [
        "/TactileSensor4/StaticData", 
        tactilesensors4.msg.StaticData,
        msg_filters.TactileStaticStdFilter,
    ],
    [
        "/TactileSensor4/StaticData", 
        tactilesensors4.msg.StaticData,
        msg_filters.TactileStaticMeanFilter,
    ],
    [
        "/TactileSensor4/Dynamic", 
        tactilesensors4.msg.Dynamic,
        msg_filters.TactileDynamicAbsMaxFilter,
    ],
    [
        "/robotiq_force_torque_wrench", 
        WrenchStamped, 
        msg_filters.WrenchStampedNormFilter,
    ],
    [
        "/robot/limb/right/endpoint_state", 
        EndpointState,
        msg_filters.BaxterEndpointTwistNormFilter,
    ],
])

for prod in itertools.product(*[(None, i) for i in args]):
    tfc = RosTopicFilteringScheme()
    for j in prod:
        if j is not None:
            for k in j:
                tfc.add_filter(*k)
    if len(tfc.timeseries_header) != 0:
        filtering_schemes.append(tfc)
