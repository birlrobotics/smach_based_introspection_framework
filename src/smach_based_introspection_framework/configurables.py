from baxter_core_msgs.msg import EndpointState
from geometry_msgs.msg import WrenchStamped

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
}
data_type_chosen = 'endpoint_pose_and_endpoint_twist_and_wrench'        
data_type_split = data_type_chosen.split("_and_")
interested_data_fields = []
for data_type in data_type_split:
    interested_data_fields += data_fields_store[data_type]

model_type = 'hmmlearn\'s HMM'
score_metric = '_score_metric_sum_of_loglik_'
model_config = {
    'hmm_max_train_iteration': 1000,
    'hmm_max_hidden_state_amount': 7,
    'gaussianhmm_covariance_type_string': ['diag', 'spherical', 'full', 'tied'],
}


anomaly_window_size_in_sec = 4
anomaly_resample_hz = 10

anomaly_classification_confidence_threshold = 0.5


# New config that is about to remove some old ones above

info_of_topics_to_timeseries = [
    (
        "/robot/limb/right/endpoint_state",
        EndpointState,
        lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z,\
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w,\
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,\
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z]
    ), 
    (
        "/robotiq_force_torque_wrench",
        WrenchStamped,
        lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,\
            m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z]
    ),
]
timeseries_rate = 100

topics_to_be_recorded_into_rosbag = [
    '/tag_multimodal', 
    '/anomaly_detection_signal',
    '/anomaly_detection_metric_gradient_of_loglikelihood',
    '/anomaly_detection_threshold_gradient_of_loglikelihood',
    '/identified_skill_gradient_of_loglikelihood',
    '/robot/limb/right/endpoint_state',
    '/robotiq_force_torque_wrench',
]

for tu in info_of_topics_to_timeseries:
    if tu[0] not in topics_to_be_recorded_into_rosbag:
        raise Exception("%s not in topics_to_be_recorded_into_rosbag but in info_of_topics_to_timeseries"%tu[0]) 



















