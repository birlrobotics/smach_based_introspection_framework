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
    'wrench': [
         '.wrench_stamped.wrench.force.x',
         '.wrench_stamped.wrench.force.y',
         '.wrench_stamped.wrench.force.z',
         '.wrench_stamped.wrench.torque.x',
         '.wrench_stamped.wrench.torque.y',
         '.wrench_stamped.wrench.torque.z',
    ] 
}
data_type_chosen = 'endpoint_pose_and_wrench'        
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
