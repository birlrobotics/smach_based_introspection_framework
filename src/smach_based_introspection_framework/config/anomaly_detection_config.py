from rostopics_to_timeseries import RosTopicFilteringScheme
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
from smach_based_introspection_framework.offline_part.anomaly_detection_feature_selection import msg_filters, msg_filters_with_scaling_and_clip
import tactilesensors4.msg
from baxter_core_msgs.msg import EndpointState 
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from scipy import signal

anomaly_detection_timeseries_hz = 20
anomaly_detection_timeseries_config = RosTopicFilteringScheme(anomaly_detection_timeseries_hz)

anomaly_detection_timeseries_config.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters.WrenchStampedNormFilter,
)
anomaly_detection_timeseries_config.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    msg_filters.WrenchStampedFilter,
)

'''
anomaly_detection_timeseries_config.add_filter(
    "/joint_states", 
    JointState,
    msg_filters.URjointFilterForPosition,
)
anomaly_detection_timeseries_config.add_filter(
    "/joint_states", 
    JointState,
    msg_filters.URjointFilterForVelocity,
)

# anomaly_detection_timeseries_config.add_filter(
#     "/joint_states", 
#     JointState,
#     msg_filters.URjointFilterForEffort,
# )
'''
anomaly_detection_timeseries_config.smoother_class = WindowBasedSmoother_factory(signal.boxcar(5))
