#!/usr/bin/env python
from parameters_combinator import ListOfParams

HUMAN_AS_MODEL_MODE = True

score_metric = '_score_metric_sum_of_loglik_'

model_type = 'hmmlearn\'s HMM'
model_config = {
  'n_components': ListOfParams([3,5,7, 9]),
  'covariance_type': ListOfParams(['full']),
  'n_iter': 1000,
}
'''

# bnpy_info
# Supported Allocation model:   ['FiniteHMM',' HDPHMM']
# Supported Observation models: ['Gauss','DiagGauss','ZeroMeanGauss','AutoRegGauss']
# Supported Variational methods:['EM','VB','soVB', 'memoVB']
model_type = 'BNPY\'s HMM'
model_config = {
    'n_iteration': ListOfParams([1000]),
    'K'          : ListOfParams([5]),
    'alloModel'  : ListOfParams(['HDPHMM']),
    'obsModel'   : ListOfParams(['AutoRegGauss']),
    'varMethod'  : ListOfParams(['memoVB']),
    'ECovMat'    : ListOfParams(['covdata']),
}
'''
train_size = 0.3


from smach_based_introspection_framework.config.anomaly_classification_config import (
    anomaly_classification_timeseries_hz, 
    anomaly_classification_confidence_threshold, 
    anomaly_window_size, 
    anomaly_classification_timeseries_config,
)

from smach_based_introspection_framework.config.anomaly_detection_config import (
    anomaly_detection_timeseries_hz,
    anomaly_detection_timeseries_config, 
)

from smach_based_introspection_framework.config.dmp_config import (
    dmp_cmd_timeseries_hz,
    dmp_cmd_timeseries_config, 
)
dmp_cmd_fields = dmp_cmd_timeseries_config.timeseries_header

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
