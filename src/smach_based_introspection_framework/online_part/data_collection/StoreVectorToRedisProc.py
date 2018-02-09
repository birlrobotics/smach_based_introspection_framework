import multiprocessing
from ConvertTagTopicToInterestedVectorProc import (
    data_frame_idx,
    smach_state_idx,
    data_header_idx,
)

class StoreVectorToRedisProc(multiprocessing.Process):
    def __init__(
        self,
        com_queue, 
        node_name="StoreVectorToRedisProc_node",
    ):
        multiprocessing.Process.__init__(self)     
        self.com_queue = com_queue
        self.node_name = node_name
        
    def run(self):
        import rospy
        rospy.init_node(self.node_name, anonymous=True)
        try:
            import redis
            import Queue
            r = redis.Redis(host='localhost', port=6379, db=0)
            rospy.loginfo('delete key \"tag_multimodal_msgs\": %s'%r.delete("tag_multimodal_msgs"))
            while not rospy.is_shutdown():
                try:
                    latest_data_tuple = self.com_queue.get(timeout=1)
                except Queue.Empty:
                    continue
                except KeyboardInterrupt:
                    break

                data_frame = latest_data_tuple[data_frame_idx]
                smach_state = latest_data_tuple[smach_state_idx]
                data_header = latest_data_tuple[data_header_idx]

                score = data_header.stamp.to_sec()
                value = data_frame
                r.zadd("tag_multimodal_msgs", value, score)
        except Exception as e:
            rospy.logerr("StoreVectorToRedisProc error: %s"%e)
        rospy.loginfo("StoreVectorToRedisProc exits")
