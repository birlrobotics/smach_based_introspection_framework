from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import WrenchStamped
from parameters_combinator import ListOfParams
from smach_based_introspection_framework.msg import (
    Tag_MultiModal,
)

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
data_type_chosen = 'endpoint_twist_and_wrench_and_tactile_sensor_data'
data_type_split = data_type_chosen.split("_and_")
interested_data_fields = []
for data_type in data_type_split:
    interested_data_fields += data_fields_store[data_type]

score_metric = '_score_metric_sum_of_loglik_'

'''
model_type = 'hmmlearn\'s HMM'
model_config = {
  'n_components': ListOfParams([1,3,5,7]),
  'covariance_type': ListOfParams(['diag', 'spherical', 'full', 'tied']),
  'n_iter': 1000,
}
'''
model_type = 'BNPY\'s HMM'
model_config = {
    'n_iteration': 1000,
    'K': 10,
    'alloModel' : 'HDPHMM',
    'obsModel'  : 'AutoRegGauss',
    'varMethod' : ListOfParams(['memoVB', 'moVB']),
    'ECovMat'   : ListOfParams(['eye']),
}


anomaly_window_size_in_sec = 4
anomaly_resample_hz = 10

anomaly_classification_confidence_threshold = 0.5


# New config that is about to remove some old ones above

info_of_topics_to_timeseries = [
#    (
#        "/tag_multimodal",
#        Tag_MultiModal,
#        lambda m: [m.tactile_texel_sum]
#    ),

#      (
#          "/tag_multimodal",
#          Tag_MultiModal,
#          lambda m: [
#              m.wrench_stamped.wrench.force.x,
#              m.wrench_stamped.wrench.force.y,
#              m.wrench_stamped.wrench.force.z,
#              m.wrench_stamped.wrench.torque.x,
#              m.wrench_stamped.wrench.torque.y,
#              m.wrench_stamped.wrench.torque.z,
#          ]
#      ),
]
timeseries_rate = 1

topics_to_be_recorded_into_rosbag = [
    '/tag_multimodal',
    '/anomaly_detection_signal',
    '/anomaly_detection_metric_gradient_of_loglikelihood',
    '/anomaly_detection_threshold_gradient_of_loglikelihood',
    '/identified_skill_gradient_of_loglikelihood',
    '/robot/limb/right/endpoint_state',
    '/robotiq_force_torque_wrench',
    '/tactile_sensor_data',
]

for tu in info_of_topics_to_timeseries:
    if tu[0] not in topics_to_be_recorded_into_rosbag:
        raise Exception("%s not in topics_to_be_recorded_into_rosbag but in info_of_topics_to_timeseries"%tu[0])
