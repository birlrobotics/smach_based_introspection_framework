from geometry_msgs.msg import WrenchStamped
from baxter_core_msgs.msg import EndpointState 
import msg_filters_with_scaling
from rostopics_to_timeseries import RosTopicFilteringScheme
from smach_based_introspection_framework.msg import tactile_static
import tactilesensors4.msg
import itertools
from smach_based_introspection_framework.configurables import (
    timeseries_rate,
)
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
from scipy import signal

filtering_schemes = []

fixed_filters = [
    [
        "/TactileSensor4/StaticData", 
        tactilesensors4.msg.StaticData,
        msg_filters_with_scaling.TactileStaticStdFilter,
    ],
    [
        "/robotiq_force_torque_wrench", 
        WrenchStamped, 
        msg_filters_with_scaling.WrenchStampedForceNormFilter,
    ],
    [
        "/TactileSensor4/Dynamic", 
        tactilesensors4.msg.Dynamic,
        msg_filters_with_scaling.TactileDynamicAbsMaxFilter,
    ],
    [
        "/robotiq_force_torque_wrench", 
        WrenchStamped, 
        msg_filters_with_scaling.WrenchStampedTorqueNormFilter,
    ],
    [
        "/robot/limb/right/endpoint_state", 
        EndpointState,
        msg_filters_with_scaling.BaxterEndpointTwistNormFilter,
    ],
]

filters_args = []

smoother_args = []
smoother_args.append(WindowBasedSmoother_factory(signal.boxcar(5)))

for smoother_class in smoother_args:
    for prod in itertools.product(*[(None, i) for i in filters_args]):
        tfc = RosTopicFilteringScheme(timeseries_rate)
        if smoother_class is not None:
            tfc.smoother_class = smoother_class
        for k in fixed_filters:
            tfc.add_filter(*k)

        for j in prod:
            if j is not None:
                for k in j:
                    tfc.add_filter(*k)

        if len(tfc.timeseries_header) != 0:
            filtering_schemes.append(tfc)
