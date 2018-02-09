import multiprocessing
import Queue
from smach_based_introspection_framework.online_part.data_collection.StoreVectorToRedisProc import (
    StoreVectorToRedisProc,
)
from smach_based_introspection_framework.online_part.data_collection.ConvertTagTopicToInterestedVectorProc import (
    ConvertTagTopicToInterestedVectorProc,
)
from smach_based_introspection_framework.srv import (
    AnomalyClassificationService, 
    AnomalyClassificationServiceResponse,
)
import ipdb
import copy
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import time
from smach_based_introspection_framework.configurables import (
    anomaly_window_size_in_sec, 
    anomaly_resample_hz, 
    interested_data_fields,
)
from smach_based_introspection_framework._constant import (
    latest_model_folder,
    realtime_anomaly_plot_folder,
)
import glob
import re
from sklearn.externals import joblib

def classify_against_all_types(mat):
    ret = []
    prog = re.compile(r'nominal_skill_(\d+)_anomaly_type_(.*)')
    for i in glob.glob(os.path.join(latest_model_folder, '*')):
        m = prog.match(os.path.basename(i))
        if not m:
            continue
        
        tag = m.group(1)
        anomaly_type = m.group(2)
        model = joblib.load(os.path.join(i, 'classifier_model'))
        hmm_model = model['hmm_model']
        threshold_for_classification = model['threshold_for_classification']
        score = hmm_model.score(mat) 
        confidence = score/threshold_for_classification
        ret.append((
            os.path.basename(i), 
            {
                "score":score, 
                "threshold_for_classification":threshold_for_classification, 
                "confidence":confidence,
            },
        ))
        
    return ret

resampled_anomaly_df_queue = Queue.Queue()
def cb(req):
    global resampled_anomaly_df_queue
    print req
    anomaly_t = req.anomaly_start_time_in_secs
    import redis
    r = redis.Redis(host='localhost', port=6379, db=0)
    from birl.robot_introspection_pkg.anomaly_sampling_config import anomaly_window_size_in_sec, anomaly_resample_hz
    search_start = anomaly_t-anomaly_window_size_in_sec/2-1            
    search_end = anomaly_t+anomaly_window_size_in_sec/2+1

    rows = r.zrangebyscore("tag_multimodal_msgs", search_start, search_end, withscores=True)
    if len(rows) == 0:
        rospy.logerr("cannot find exec recrod in redis, redis returned nothing")
        return AnomalyClassificationServiceResponse(-1, -1)

    import pandas as pd
    dimensions = copy.deepcopy(interested_data_fields)
    if '.tag' in dimensions:
        idx_to_del = dimensions.index('.tag')
        del dimensions[idx_to_del]
    search_df = pd.DataFrame(columns=dimensions)
    for item in rows:
        value, score = item
        time = score
        data_frame = eval(value)
        search_df.loc[score] = data_frame 

    new_time_index = np.linspace(anomaly_t-anomaly_window_size_in_sec/2, anomaly_t+anomaly_window_size_in_sec/2, anomaly_window_size_in_sec*anomaly_resample_hz)
    old_time_index = search_df.index
    resampled_anomaly_df = search_df.reindex(old_time_index.union(new_time_index)).interpolate(method='linear', axis=0).ix[new_time_index]
    resampled_anomaly_df_queue.put(resampled_anomaly_df)
    ret = classify_against_all_types(resampled_anomaly_df.values)
    m = max(ret, key=lambda x: x[1]['confidence'])
    print ret
    print m
    return AnomalyClassificationServiceResponse(1, 0.99)


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

if __name__ == '__main__':
    com_queue_of_receiver = multiprocessing.Queue()
    process_receiver = ConvertTagTopicToInterestedVectorProc(
        interested_data_fields,
        com_queue_of_receiver,
        node_name="tagMsgReceiverForOnlineRedisRecorder",
    )
    process_receiver.daemon = True
    process_receiver.start()

    com_queue_of_redis = multiprocessing.Queue()
    redis_zadd_proc = StoreVectorToRedisProc(
        com_queue_of_redis, 
        node_name='RedisZaddProc_node_for_anomaly_classification',
    )
    redis_zadd_proc.daemon = True
    redis_zadd_proc.start()

    import rospy
    rospy.init_node('anomaly_classification_node')

    rospy.loginfo("redis_based_anomaly_classification.py starts")
    s = rospy.Service("AnomalyClassificationService", AnomalyClassificationService, cb) 

    while not rospy.is_shutdown():
        if not resampled_anomaly_df_queue.empty():
            plot_resampled_anomaly_df(resampled_anomaly_df_queue.get())

        try:
            latest_data_tuple = com_queue_of_receiver.get(timeout=1)
        except Queue.Empty:
            continue
        com_queue_of_redis.put(latest_data_tuple)

    rospy.loginfo('redis_based_anomaly_classification.py exits')
