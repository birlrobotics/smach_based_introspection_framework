from rostopics_to_timeseries import RosTopicFilteringScheme
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
import smach_based_introspection_framework.offline_part.anomaly_classification_feature_selection.msg_filters_with_scaling as cl_msg_filters_with_scaling
import tactilesensors4.msg
from baxter_core_msgs.msg import EndpointState 
from geometry_msgs.msg import WrenchStamped
from scipy import signal

anomaly_classification_timeseries_hz = 10
anomaly_classification_confidence_threshold = 0.7
anomaly_window_size = [2, 2]
anomaly_classification_timeseries_config = RosTopicFilteringScheme(anomaly_classification_timeseries_hz)
anomaly_classification_timeseries_config.add_filter(
    "/TactileSensor4/StaticData", 
    tactilesensors4.msg.StaticData,
    cl_msg_filters_with_scaling.TactileStaticStdFilter,
)
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
    "/robot/limb/right/endpoint_state", 
    EndpointState,
    cl_msg_filters_with_scaling.BaxterEndpointTwistNormFilter,
)
anomaly_classification_timeseries_config.add_filter(
    "/robot/limb/right/endpoint_state", 
    EndpointState,
    cl_msg_filters_with_scaling.BaxterEndpointTwistFilter,
)
anomaly_classification_timeseries_config.smoother_class = WindowBasedSmoother_factory(signal.boxcar(5))
