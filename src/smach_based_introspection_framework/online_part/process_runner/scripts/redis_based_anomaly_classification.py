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
    anomaly_window_size_in_sec,
    anomaly_resample_hz,    
    anomaly_classifier_model_path,
    anomaly_handcoded_labels,
)
from smach_based_introspection_framework._constant import (
    latest_model_folder,
    realtime_anomaly_plot_folder,
)
from birl_hmm.hmm_training import (hmm_util,)
import glob
import re
from sklearn.externals import joblib

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

loaded_model  = None
def classify_against_all_types(mat, happen_in_state):

    ret = []
    prog = re.compile(r'nominal_skill_(\d+)_anomaly_type_(.*)')
    for i in glob.glob(os.path.join(latest_model_folder, '*')):
        m = prog.match(os.path.basename(i))
        if not m:
            continue
        
        tag = m.group(1)
        anomaly_type = m.group(2)
        if int(tag) != int(happen_in_state):
            continue

        model = joblib.load(os.path.join(i, 'classifier_model'))
        hmm_model = model['hmm_model']
        '''
        threshold_for_classification = model['threshold_for_classification']
        score = hmm_model.score(mat) 
        confidence = score/threshold_for_classification
        ret.append((
            anomaly_type,
            {
                "score":score, 
                "threshold_for_classification":threshold_for_classification, 
                "confidence":confidence,
            },
        ))

        '''
        confidence = hmm_util.fast_log_curve_calculation(mat, hmm_model)[-1]
        ret.append((
            anomaly_type,
            {
                "confidence":confidence,
                
            },
        ))

    return ret


    '''
    # load the LSTM model
    global loaded_model
    if loaded_model == None:
        rospy.loginfo('loading anomaly_classifier_model')
        import anomaly_model_generation
        loaded_model = anomaly_model_generation.run()
        loaded_model.load_weights(os.path.join(anomaly_classifier_model_path, 'baxter_pnp_weights.h5'))
        loaded_model.compile(loss='binary_crossentropy', optimizer='rmsprop', metrics =['accuracy'])
    else:
        rospy.loginfo('model had been loaded!')
    rospy.loginfo("\nPredicting")
    pred = loaded_model.predict(mat.T[np.newaxis,...], batch_size=128)
    anomaly_type = anomaly_handcoded_labels[np.argmax(pred)]
    rospy.loginfo('Predicted class: ' + anomaly_type)
    
    ret = []
    ret.append((
        anomaly_type,
        {
            "confidence":np.max(pred),
        },
    ))
    return ret
    '''   

def cb(req):
    rospy.loginfo(req)
    anomaly_t = req.anomaly_start_time_in_secs
    happen_in_state = req.happen_in_state
    import redis
    r = redis.Redis(host='localhost', port=6379, db=0)
    search_start = anomaly_t-anomaly_window_size_in_sec/2-1            
    search_end = anomaly_t+anomaly_window_size_in_sec/2+1

    rows = r.zrangebyscore("tag_multimodal_msgs", search_start, search_end, withscores=True)
    if len(rows) == 0:
        rospy.logerr("cannot find exec record in redis, redis returned nothing")
        return AnomalyClassificationServiceResponse("__NO_EXEC_RECORD", 0)

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
    plot_resampled_anomaly_df(resampled_anomaly_df)
    ret = classify_against_all_types(resampled_anomaly_df.values, happen_in_state)
    if len(ret) == 0:
        return AnomalyClassificationServiceResponse("__NO_CLASSIFIER_FOUND", 0)
    m = max(ret, key=lambda x: x[1]['confidence'])
    rospy.loginfo("classication report: %s"%ret)
    return AnomalyClassificationServiceResponse(m[0], m[1]['confidence'])


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
        try:
            latest_data_tuple = com_queue_of_receiver.get(timeout=1)
        except Queue.Empty:
            continue
        except KeyboardInterrupt:
            break
        com_queue_of_redis.put(latest_data_tuple)

    rospy.loginfo('redis_based_anomaly_classification.py exits')
