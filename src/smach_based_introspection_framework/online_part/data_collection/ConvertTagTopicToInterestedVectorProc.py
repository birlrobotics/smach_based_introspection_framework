import multiprocessing
import copy
from smach_based_introspection_framework.msg import (
    Tag_MultiModal,
)
data_frame_idx = 0
smach_state_idx = 1
data_header_idx = 2

class ConvertTagTopicToInterestedVectorProc(multiprocessing.Process):
    def __init__(
        self, 
        interested_data_fields, 
        com_queue, 
        node_name="ConvertTagTopicToInterestedVectorProc_node",
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
        try:
            rospy.Subscriber("/tag_multimodal", Tag_MultiModal, self.callback_multimodal)
            rospy.loginfo('ConvertTagTopicToInterestedVectorProc: /tag_multimodal subscribed')
            rospy.spin()
        except Exception as e:
            rospy.logerr("ConvertTagTopicToInterestedVectorProc error: %s"%e)
        rospy.loginfo("ConvertTagTopicToInterestedVectorProc exits")
