from rostopics_to_timeseries import RosTopicFilteringScheme
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
import smach_based_introspection_framework.offline_part.anomaly_classification_feature_selection.msg_filters_with_scaling as cl_msg_filters_with_scaling
import tactilesensors4.msg
from baxter_core_msgs.msg import EndpointState 
from geometry_msgs.msg import WrenchStamped
from scipy import signal
from sensor_msgs.msg import JointState

anomaly_classification_timeseries_hz = 10
anomaly_classification_confidence_threshold = 0.7
anomaly_window_size = [2, 2]
anomaly_classification_timeseries_config = RosTopicFilteringScheme(anomaly_classification_timeseries_hz)

anomaly_classification_timeseries_config.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    cl_msg_filters_with_scaling.WrenchStampedNormFilter,
)
anomaly_classification_timeseries_config.add_filter(
    "/robotiq_force_torque_wrench", 
    WrenchStamped, 
    cl_msg_filters_with_scaling.WrenchStampedFilter,
)
anomaly_classification_timeseries_config.add_filter(
    "/joint_states", 
    JointState,
    cl_msg_filters_with_scaling.URjointFilterForPosition,
)
anomaly_classification_timeseries_config.add_filter(
    "/joint_states", 
    JointState,
    cl_msg_filters_with_scaling.URjointFilterForVelocity,
)
anomaly_classification_timeseries_config.add_filter(
    "/joint_states", 
    JointState,
    cl_msg_filters_with_scaling.URjointFilterForEffort,
)
anomaly_classification_timeseries_config.smoother_class = WindowBasedSmoother_factory(signal.boxcar(5))
