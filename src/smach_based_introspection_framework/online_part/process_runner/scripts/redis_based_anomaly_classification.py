#!/usr/bin/env python
import rospy
import pandas as pd
import redis
import multiprocessing
import Queue
from smach_based_introspection_framework.online_part.data_collection.StoreTimeseriesInRedisProc import (
    StoreTimeseriesInRedisProc,
)
from smach_based_introspection_framework.srv import (
    AnomalyClassificationService, 
    AnomalyClassificationServiceResponse,
)

import copy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import time
from smach_based_introspection_framework.configurables import (
    anomaly_window_size,
    anomaly_filtering_scheme,
)
from smach_based_introspection_framework._constant import (
    latest_model_folder,
    realtime_anomaly_plot_folder,
)
from birl_hmm.hmm_training import (hmm_util,)
import glob
import re
from sklearn.externals import joblib
from smach_based_introspection_framework.online_part.anomaly_classifier.Classifier import NormalDistributedConfidenceClassifier

def plot_resampled_anomaly_df(resampled_anomaly_df):
    import datetime
    realtime_anomaly_plot_dir = os.path.join(realtime_anomaly_plot_folder, str(datetime.datetime.now()))

    if not os.path.isdir(realtime_anomaly_plot_dir):
        os.makedirs(realtime_anomaly_plot_dir)

    for dim in resampled_anomaly_df.columns:
        rospy.loginfo("plotting %s"%dim)
        fig, ax = plt.subplots(nrows=1, ncols=1)
        time_x = resampled_anomaly_df.index-resampled_anomaly_df.index[0]
        ax.plot(
            time_x.tolist(),
            resampled_anomaly_df[dim].tolist(), 
        )
        ax.set_title(dim)
        fig.savefig(os.path.join(realtime_anomaly_plot_dir, (dim+'.png').strip('.')))
        plt.close(fig)
        

def classify_against_all_types(mat, happen_in_state):

    models_grouped_by_type = {}
    prog = re.compile(r'anomaly_type_\(?([^\(\)]+)\)?')
    for model_file in glob.glob(os.path.join(latest_model_folder, '*', 'classifier_model')):
        anomaly_type = prog.search(model_file).group(1)
        with open(model_file, 'rb') as f:
            models_grouped_by_type[anomaly_type] = joblib.load(f)

    c = NormalDistributedConfidenceClassifier(models_grouped_by_type)
    predict_label = c.predict(mat)
    
    if predict_label == 'human_collision_with_object':
        predict_label = 'human_collision'
        print ("Warning: This setting only for the experiment testing!")
    
    return [(predict_label, {'confidence':1})]

def cb(req):
    rospy.loginfo(req)
    anomaly_t = req.anomaly_start_time_in_secs
    happen_in_state = req.happen_in_state
    r = redis.Redis(host='localhost', port=6379, db=0)
    search_start = anomaly_t-anomaly_window_size[0]
    search_end = anomaly_t+anomaly_window_size[1]
    rows = r.zrangebyscore("timeseries_topic_for_anomaly_classification", search_start, search_end, withscores=True)
    if len(rows) == 0:
        rospy.logerr("cannot find exec record in redis, redis returned nothing")
        return AnomalyClassificationServiceResponse("__NO_EXEC_RECORD", 0)

    mat = np.array([eval(i[0]) for i in rows])
    ret = classify_against_all_types(mat, happen_in_state)
    if len(ret) == 0:
        return AnomalyClassificationServiceResponse("__NO_CLASSIFIER_FOUND", 0)
    m = max(ret, key=lambda x: x[1]['confidence'])
    rospy.loginfo("classication report: %s"%ret)
    return AnomalyClassificationServiceResponse(m[0], m[1]['confidence'])


if __name__ == '__main__':
    redis_proc = StoreTimeseriesInRedisProc(
        node_name='StoreTimeseriesInRedisProc_node_for_AC',
        topic_name='timeseries_topic_for_anomaly_classification',
    )
    redis_proc.daemon = True
    redis_proc.start()

    rospy.init_node('anomaly_classification_node')

    rospy.loginfo("redis_based_anomaly_classification.py starts")
    s = rospy.Service("AnomalyClassificationService", AnomalyClassificationService, cb) 
    rospy.spin()
    rospy.loginfo('redis_based_anomaly_classification.py exits')
