import copy
from rostopics_to_timeseries.msg import (
    Timeseries,
)
from smach_based_introspection_framework.msg import (
    Tag_MultiModal,
)
import Queue
import multiprocessing 

data_frame_idx = 0
smach_state_idx = 1
data_header_idx = 2

class TimeseriesReceiverProc(multiprocessing.Process):
    def __init__(
        self, 
        com_queue, 
        node_name="TimeseriesReceiverProc_node",
        topic_name="/rostopics_to_timeseries_topic",
    ):
        multiprocessing.Process.__init__(self)     

        self.com_queue = com_queue
        self.node_name = node_name
        self.topic_name = topic_name

        self.smach_state = None

    def callback_multimodal(self, msg):
        self.smach_state = msg.tag

    def callback_timeseries(self, msg):
        import rospy
        if self.smach_state is None:
            rospy.logwarn("TimeseriesReceiverProc did not receive smach_state, so no message will be passed via queue.")
        else:
            self.com_queue.put((msg.sample, self.smach_state, msg.header))

    def run(self):
        # set up Subscribers
        import rospy
        rospy.init_node(self.node_name, anonymous=True)
        try:
            rospy.Subscriber("/tag_multimodal", Tag_MultiModal, self.callback_multimodal)
            rospy.Subscriber(self.topic_name, Timeseries, self.callback_timeseries)
            rospy.loginfo('TimeseriesReceiverProc: /tag_multimodal subscribed')
            rospy.spin()
        except Exception as e:
            rospy.logerr("TimeseriesReceiverProc error: %s"%e)
        rospy.loginfo("TimeseriesReceiverProc exits")

if __name__ == "__main__":
    import rospy
    import numpy as np
    import matplotlib.pyplot as plt
    import ipdb

    com_queue_of_receiver = multiprocessing.Queue()
    process_receiver = TimeseriesReceiverProc(
        com_queue_of_receiver,
    )
    process_receiver.daemon = True

    process_receiver.start()

    tups = []
    while not rospy.is_shutdown():
        try:
            latest_data_tuple = com_queue_of_receiver.get(timeout=1)
        except Queue.Empty:
            continue
        except KeyboardInterrupt:
            break
        tups.append(latest_data_tuple)
        rospy.loginfo("Got one")

    ts = [i[data_header_idx].stamp.to_sec() for i in tups]
    mat = np.array([i[data_frame_idx] for i in tups])

    fig, ax = plt.subplots(nrows=1, ncols=1)
    for col_idx in range(mat.shape[1]):
        ax.plot(ts, mat[:, col_idx].reshape(-1), label='No.%s signal'%col_idx)
    ax.legend()
    ax.set_title("Online Signals")
    plt.show()
        
