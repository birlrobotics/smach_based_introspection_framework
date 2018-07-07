from rostopics_to_timeseries import RosTopicFilteringScheme
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
from smach_based_introspection_framework.offline_part.anomaly_detection_feature_selection import msg_filters, msg_filters_with_scaling, msg_filters_with_scaling_and_clip
import tactilesensors4.msg
from baxter_core_msgs.msg import EndpointState 
from geometry_msgs.msg import WrenchStamped
from scipy import signal

timeseries_rate = 10
tfc = RosTopicFilteringScheme(timeseries_rate)
tfc.add_filter(
    "/TactileSensor4/StaticData", 
    tactilesensors4.msg.StaticData,
    msg_filters_with_scaling_and_clip.TactileStaticStdScaleClipMaxFilter,
)
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters_with_scaling.WrenchStampedNormFilter,
)
tfc.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters_with_scaling.WrenchStampedFilter,
)
tfc.add_filter(
    "/robot/limb/right/endpoint_state", 
    EndpointState,
    msg_filters_with_scaling.BaxterEndpointTwistNormFilter,
)
tfc.add_filter(
    "/robot/limb/right/endpoint_state", 
    EndpointState,
    msg_filters_with_scaling.BaxterEndpointTwistFilter,
)
tfc.smoother_class = WindowBasedSmoother_factory(signal.boxcar(5))
