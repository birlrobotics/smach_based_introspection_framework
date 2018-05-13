from geometry_msgs.msg import WrenchStamped
from baxter_core_msgs.msg import EndpointState 
from parameters_combinator import ListOfParams
from smach_based_introspection_framework.msg import (
    Tag_MultiModal,
)
from rostopics_to_timeseries import RosTopicFilteringScheme
import baxter_core_msgs.msg
import tactilesensors4.msg
import geometry_msgs.msg
from smach_based_introspection_framework.offline_part.anomaly_detection_feature_selection import msg_filters_with_scaling, msg_filters_with_scaling_and_clip
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
from scipy import signal

HUMAN_AS_MODEL_MODE = True

dmp_cmd_fields = [
    '.endpoint_state.pose.position.x',
    '.endpoint_state.pose.position.y',
    '.endpoint_state.pose.position.z',
    '.endpoint_state.pose.orientation.x',
    '.endpoint_state.pose.orientation.y',
    '.endpoint_state.pose.orientation.z',
    '.endpoint_state.pose.orientation.w',
]

data_fields_store = {
    "endpoint_pose": [
        '.endpoint_state.pose.position.x',
        '.endpoint_state.pose.position.y',
        '.endpoint_state.pose.position.z',
        '.endpoint_state.pose.orientation.x',
        '.endpoint_state.pose.orientation.y',
        '.endpoint_state.pose.orientation.z',
        '.endpoint_state.pose.orientation.w'
    ],
    "endpoint_twist": [
        '.endpoint_state.twist.linear.x',
        '.endpoint_state.twist.linear.y',
        '.endpoint_state.twist.linear.z',
        '.endpoint_state.twist.angular.x',
        '.endpoint_state.twist.angular.y',
        '.endpoint_state.twist.angular.z',
    ],
    'wrench': [
         '.wrench_stamped.wrench.force.x',
         '.wrench_stamped.wrench.force.y',
         '.wrench_stamped.wrench.force.z',
         '.wrench_stamped.wrench.torque.x',
         '.wrench_stamped.wrench.torque.y',
         '.wrench_stamped.wrench.torque.z',
    ],
    'wrench_torque': [
         '.wrench_stamped.wrench.torque.x',
         '.wrench_stamped.wrench.torque.y',
         '.wrench_stamped.wrench.torque.z',
    ],
    'wrench_derivative': [
        '.delta_wrench.force.x',
        '.delta_wrench.force.y',
        '.delta_wrench.force.z',
        '.delta_wrench.torque.x',
        '.delta_wrench.torque.y',
        '.delta_wrench.torque.z',
    ],
    'tactile_texel_sum': [
        '.tactile_texel_sum',
    ],

    'tactile_sensor_data':[
        '.tactile_values.tactile_values_0',
        '.tactile_values.tactile_values_1',
        '.tactile_values.tactile_values_2',
        '.tactile_values.tactile_values_3',
        '.tactile_values.tactile_values_4',
        '.tactile_values.tactile_values_5',
        '.tactile_values.tactile_values_6',
        '.tactile_values.tactile_values_7',
        '.tactile_values.tactile_values_8',
        '.tactile_values.tactile_values_9',
],
}
data_type_chosen = 'endpoint_twist_and_wrench'
data_type_split = data_type_chosen.split("_and_")
interested_data_fields = []
for data_type in data_type_split:
    interested_data_fields += data_fields_store[data_type]

score_metric = '_score_metric_sum_of_loglik_'

model_type = 'hmmlearn\'s HMM'
model_config = {
  'n_components': ListOfParams([1,3,5,7, 9]),
  'covariance_type': ListOfParams(['full']),
  'n_iter': 1000,
}

'''
model_type = 'BNPY\'s HMM'
model_config = {
    'n_iteration': 1000,
    'K': 10,
    'alloModel' : 'HDPHMM',
    'obsModel'  : 'Gauss',
    'varMethod' : ListOfParams(['memoVB']),
    'ECovMat'   : ListOfParams(['covdata']), #diagcovdata
}
'''


anomaly_window_size_in_sec = 4
anomaly_resample_hz = 10
anomaly_classification_confidence_threshold = 0.5
anomaly_handcoded_labels = {0: 'object_slip',
                            1: 'tool_collision',
                            2: 'human_collision',
                            3: 'no_object',}
anomaly_classifier_model_path = '/home/birl_wu/baxter_ws/src/SPAI/smach_based_introspection_framework/anomaly_classifier'


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


topics_to_be_recorded_into_rosbag = [
    '/tag_multimodal',
    '/anomaly_detection_signal',
    '/anomaly_detection_metric_gradient_of_loglikelihood',
    '/anomaly_detection_threshold_gradient_of_loglikelihood',
    '/identified_skill_gradient_of_loglikelihood',
    '/robot/limb/right/endpoint_state',
    '/robotiq_force_torque_wrench',
    '/tactile_sensor_data',
    '/observation/goal_vector',
    '/TactileSensor4/StaticData',
    '/TactileSensor4/Dynamic',
    '/TactileSensor4/EulerAngle',
    '/TactileSensor4/Gyroscope',
    '/TactileSensor4/Magnetometer',
    '/TactileSensor4/Accelerometer',
]
