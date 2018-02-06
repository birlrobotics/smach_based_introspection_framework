import multiprocessing
import Queue
import birl.HMM.hmm_for_baxter_using_only_success_trials.hmm_online_service.data_stream_handler_process as data_stream_handler_process
import birl.HMM.hmm_for_baxter_using_only_success_trials.hmm_online_service.constant as constant 
import birl.robot_introspection_pkg.multi_modal_config as mmc
from anomaly_classification_proxy.srv import (
    AnomalyClassificationService, 
    AnomalyClassificationServiceResponse,
)
import ipdb
import copy
import numpy as np
import matplotlib.pyplot as plt
import os

class RedisTalker(multiprocessing.Process):
    def __init__(
        self,
        com_queue, 
    ):
        multiprocessing.Process.__init__(self)     
        self.com_queue = com_queue
        
    def run(self):
        import redis
        import rospy
        r = redis.Redis(host='localhost', port=6379, db=0)
        print 'delete key \"tag_multimodal_msgs\"', r.delete("tag_multimodal_msgs")
        while not rospy.is_shutdown():
            try:
                latest_data_tuple = self.com_queue.get(1)
            except Queue.Empty:
                continue
            except KeyboardInterrupt:
                break

            data_frame = latest_data_tuple[constant.data_frame_idx]
            smach_state = latest_data_tuple[constant.smach_state_idx]
            data_header = latest_data_tuple[constant.data_header_idx]

            score = data_header.stamp.to_sec()
            value = data_frame
            r.zadd("tag_multimodal_msgs", value, score)
resampled_anomaly_df_queue = Queue.Queue()
def redis_service_callback(req):
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
        rospy.logerr("cannot find exec recrod, redis returned nothing")
        return AnomalyClassificationServiceResponse(-1, -1)

    import pandas as pd
    import birl.robot_introspection_pkg.multi_modal_config as mmc
    dimensions = copy.deepcopy(mmc.interested_data_fields)
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
    return AnomalyClassificationServiceResponse(1, 0.99)


def plot_resampled_anomaly_df(resampled_anomaly_df):
    import datetime
    realtime_anomaly_plot_dir = os.path.join('realtime_anomaly_plot_dir', str(datetime.datetime.now()))

    if not os.path.isdir(realtime_anomaly_plot_dir):
        os.makedirs(realtime_anomaly_plot_dir)

    for dim in resampled_anomaly_df.columns:
        fig, ax = plt.subplots(nrows=1, ncols=1)
        time_x = resampled_anomaly_df.index-resampled_anomaly_df.index[0]
        ax.plot(
            time_x.tolist(),
            resampled_anomaly_df[dim].tolist(), 
        )
        ax.set_title(dim)
        fig.savefig(os.path.join(realtime_anomaly_plot_dir, dim+'.png'))
        plt.close(fig)

if __name__ == '__main__':
    com_queue_of_receiver = multiprocessing.Queue()
    process_receiver = data_stream_handler_process.TagMultimodalTopicHandler(
        mmc.interested_data_fields,
        com_queue_of_receiver,
        node_name="tagMsgReceiverForOnlineRedisRecorder",
    )
    process_receiver.start()

    com_queue_of_redis = multiprocessing.Queue()
    redis_talker = RedisTalker(com_queue_of_redis)
    redis_talker.start()

    import rospy
    rospy.init_node('anomaly_classification_node')

    s = rospy.Service("AnomalyClassificationService", AnomalyClassificationService, redis_service_callback) 

    while not rospy.is_shutdown():
        if not resampled_anomaly_df_queue.empty():
            plot_resampled_anomaly_df(resampled_anomaly_df_queue.get())

        try:
            latest_data_tuple = com_queue_of_receiver.get(1)
        except Queue.Empty:
            continue
        except KeyboardInterrupt:
            break
        com_queue_of_redis.put(latest_data_tuple)
