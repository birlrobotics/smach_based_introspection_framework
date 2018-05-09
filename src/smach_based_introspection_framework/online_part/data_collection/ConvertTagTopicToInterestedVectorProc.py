import copy
from smach_based_introspection_framework.msg import (
    Tag_MultiModal,
)
from TagTopicMsgDecorator import TagTopicMsgDecorator
import Queue
import multiprocessing 

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
    
        self.tag_topic_msg_decorator = TagTopicMsgDecorator()

    def callback_multimodal(self, data):
        data = self.tag_topic_msg_decorator.decorate(data)

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


if __name__ == "__main__":
    import rospy
    import numpy as np
    import matplotlib.pyplot as plt
    import ipdb

    interested_data_fields = [
         '.wrench_stamped.wrench.force.x',
         '.wrench_stamped.wrench.force.y',
         '.wrench_stamped.wrench.force.z',
         '.delta_wrench.force.x',
         '.delta_wrench.force.y',
         '.delta_wrench.force.z',
    ]

    com_queue_of_receiver = multiprocessing.Queue()
    process_receiver = ConvertTagTopicToInterestedVectorProc(
        interested_data_fields,
        com_queue_of_receiver,
    )
    process_receiver.daemon = True

    process_receiver.start()

    samples = []
    while not rospy.is_shutdown():
        try:
            latest_data_tuple = com_queue_of_receiver.get(timeout=1)
        except Queue.Empty:
            continue
        except KeyboardInterrupt:
            break
        samples.append(latest_data_tuple[data_frame_idx])

    mat = np.array(samples)    
    assert np.array_equal(mat[1:, :3]-mat[:-1, :3], mat[1:, 3:])
    print "Pass wrench derivative test."
#     dimension = mat.shape[1]
#     fig, axs = plt.subplots(nrows=dimension, ncols=1)
#     if dimension == 1:
#         axs = [axs]
#     for idx, field in enumerate(interested_data_fields):
#         ax = axs[idx] 
#         ax.set_title(field)
#         ax.plot(mat[:, idx])
#     plt.show()

