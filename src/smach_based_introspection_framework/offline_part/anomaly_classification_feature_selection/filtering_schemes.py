from geometry_msgs.msg import WrenchStamped
from baxter_core_msgs.msg import EndpointState 
import msg_filters
import msg_filters_with_scaling
import msg_filters_with_scaling_and_clip
from rostopics_to_timeseries import RosTopicFilteringScheme
from smach_based_introspection_framework.msg import tactile_static
import tactilesensors4.msg
import itertools
from smach_based_introspection_framework.configurables import (
    anomaly_classification_timeseries_hz,
)
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
from scipy import signal

filtering_schemes = []
'''
fixed_filters = [
    [
        "/TactileSensor4/StaticData", 
        tactilesensors4.msg.StaticData,
        msg_filters.TactileStaticStdFilter,
    ],
    [
        "/robotiq_force_torque_wrench", 
        WrenchStamped, 
        msg_filters.WrenchStampedNormFilter,
    ],
    [
        "/robotiq_force_torque_wrench", 
        WrenchStamped, 
        msg_filters.WrenchStampedFilter,
    ],
    [
        "/robot/limb/right/endpoint_state", 
        EndpointState,
        msg_filters.BaxterEndpointTwistNormFilter,
    ],
    [
        "/robot/limb/right/endpoint_state", 
        EndpointState,
        msg_filters.BaxterEndpointTwistFilter,
    ],
]
'''
fixed_filters = [
    [
        "/TactileSensor4/StaticData", 
        tactilesensors4.msg.StaticData,
        msg_filters_with_scaling.TactileStaticStdFilter,
    ],
    [
        "/robotiq_force_torque_wrench", 
        WrenchStamped, 
        msg_filters_with_scaling.WrenchStampedNormFilter,
    ],
    [
        "/robotiq_force_torque_wrench", 
        WrenchStamped, 
        msg_filters_with_scaling.WrenchStampedFilter,
    ],
    [
        "/robot/limb/right/endpoint_state", 
        EndpointState,
        msg_filters_with_scaling.BaxterEndpointTwistNormFilter,
    ],
    [
        "/robot/limb/right/endpoint_state", 
        EndpointState,
        msg_filters_with_scaling.BaxterEndpointTwistFilter,
    ],
]

filters_args = []

smoother_args = []
smoother_args.append(WindowBasedSmoother_factory(signal.boxcar(5)))
#smoother_args.append(None)
for smoother_class in smoother_args:
    for prod in itertools.product(*[(None, i) for i in filters_args]):
        anomaly_classification_timeseries_config = RosTopicFilteringScheme(anomaly_classification_timeseries_hz)
        if smoother_class is not None:
            anomaly_classification_timeseries_config.smoother_class = smoother_class
        for k in fixed_filters:
            anomaly_classification_timeseries_config.add_filter(*k)

        for j in prod:
            if j is not None:
                for k in j:
                    anomaly_classification_timeseries_config.add_filter(*k)

        if len(anomaly_classification_timeseries_config.timeseries_header) != 0:
            filtering_schemes.append(anomaly_classification_timeseries_config)
