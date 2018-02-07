import multiprocessing
import copy
from smach_based_introspection_framework.msg import (
    Tag_MultiModal,
)
data_frame_idx = 0
smach_state_idx = 1
data_header_idx = 2

class TagMultimodalTopicHandler(multiprocessing.Process):
    def __init__(
        self, 
        interested_data_fields, 
        com_queue, 
        node_name="TagMultimodalTopicHandler",
    ):
        multiprocessing.Process.__init__(self)     

        interested_data_fields = copy.deepcopy(interested_data_fields)
    
        # we don't need tag when using HMM.score
        if '.tag' in interested_data_fields:
            tag_idx = interested_data_fields.index('.tag')
            del(interested_data_fields[tag_idx])
        self.interested_data_fields = interested_data_fields
        self.com_queue = com_queue
        self.node_name = node_name

    def callback_multimodal(self, data):
        data_header = data.header
        smach_state = data.tag

        one_frame_data = []
        for field in self.interested_data_fields:
            one_frame_data.append(eval('data'+field))

        self.com_queue.put((one_frame_data, smach_state, data_header))       

    def run(self):
        # set up Subscribers
        import rospy
        rospy.init_node(self.node_name, anonymous=True)
        rospy.Subscriber("/tag_multimodal", Tag_MultiModal, self.callback_multimodal)
        rospy.loginfo('/tag_multimodal subscribed')
        rospy.spin()
        rospy.loginfo("TagMultimodalTopicHandler exits")

class RedisZaddProc(multiprocessing.Process):
    def __init__(
        self,
        com_queue, 
        node_name="RedisZaddProc_node",
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
            rospy.loginfo('RedisZaddProc: delete key \"tag_multimodal_msgs\" %s'%r.delete("tag_multimodal_msgs"))
            while not rospy.is_shutdown():
                try:
                    latest_data_tuple = self.com_queue.get(timeout=1)
                except Queue.Empty:
                    continue

                data_frame = latest_data_tuple[data_frame_idx]
                smach_state = latest_data_tuple[smach_state_idx]
                data_header = latest_data_tuple[data_header_idx]

                score = data_header.stamp.to_sec()
                value = data_frame
                r.zadd("tag_multimodal_msgs", value, score)
        except Exception as e:
            rospy.logerr("RedisZaddProc error: %s"%e)
        rospy.loginfo("RedisZaddProc exits")
