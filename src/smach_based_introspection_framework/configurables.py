#!/usr/bin/env python
from parameters_combinator import ListOfParams

HUMAN_AS_MODEL_MODE = False
dmp_cmd_fields = [
    '.endpoint_state.pose.position.x',
    '.endpoint_state.pose.position.y',
    '.endpoint_state.pose.position.z',
    '.endpoint_state.pose.orientation.x',
    '.endpoint_state.pose.orientation.y',
    '.endpoint_state.pose.orientation.z',
    '.endpoint_state.pose.orientation.w',
]

score_metric = '_score_metric_sum_of_loglik_'

'''
model_type = 'hmmlearn\'s HMM'
model_config = {
  'n_components': ListOfParams([1,3,5,7, 9]),
  'covariance_type': ListOfParams(['full']),
  'n_iter': 1000,
}
'''

# bnpy_info
# Supported Allocation model:   ['FiniteHMM',' HDPHMM']
# Supported Observation models: ['Gauss','DiagGauss','ZeroMeanGauss','AutoRegGauss']
# Supported Variational methods:['memoVB']
model_type = 'BNPY\'s HMM'
model_config = {
    'n_iteration': 1000,
    'K': ListOfParams([5,10]),
    'alloModel' : 'FiniteHMM',
    'obsModel'  : ListOfParams(['AutoRegGauss', 'Gauss']),
    'varMethod' : ListOfParams(['memoVB']),
    'ECovMat'   : ListOfParams(['covdata']), #diagcovdata
}
'''
model_type = 'BNPY\'s HMM'
model_config = {
    'n_iteration': 1000,
    'K': ListOfParams([2, 3, 5, 7]),
    'alloModel' : 'HDPHMM',
    'obsModel'  : ListOfParams(['AutoRegGauss']),
    'varMethod' : ListOfParams(['memoVB']),
    'ECovMat'   : ListOfParams(['covdata']), #diagcovdata
}
'''


from smach_based_introspection_framework.config.anomaly_classification_config import (
    anomaly_resample_hz, 
    anomaly_classification_confidence_threshold, 
    anomaly_window_size, 
    anomaly_filtering_scheme,
)

from smach_based_introspection_framework.config.anomaly_detection_config import (
    timeseries_rate,
    tfc, 
)

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
    'timeseries_topic_for_anomaly_detection',
    'timeseries_topic_for_anomaly_classification',
]
