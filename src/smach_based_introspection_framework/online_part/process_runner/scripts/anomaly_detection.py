#!/usr/bin/env python
import Queue
import multiprocessing 

from smach_based_introspection_framework._constant import (
    latest_model_folder,
)
from smach_based_introspection_framework.online_part.data_collection.TimeseriesReceiverProc import (
    TimeseriesReceiverProc,
    data_frame_idx,
    smach_state_idx,
    data_header_idx,
)
import rospy
import multiprocessing
from sklearn.externals import joblib
import rospy
import numpy as np
from std_msgs.msg import (
    Header,
    Int8,
    Float64,
)
from smach_based_introspection_framework.msg import Float64Stamped
import glob
import re
import os
from smach_based_introspection_framework.online_part.robot_screen_visualization.setter import(
    show_introspection_model_not_found,
)
import ipdb
from smach_based_introspection_framework.online_part.anomaly_detector import(
    Detectors,
)

class IdSkillThenDetectAnomaly(multiprocessing.Process):
    def __init__(
        self, 
        com_queue,
    ):
        multiprocessing.Process.__init__(self)     

        self.com_queue = com_queue

        model_group_by_state = {}
        threshold_constant_group_by_state = {}
        prog = re.compile(r'(skill |tag_)(\d+)')
        for i in glob.glob(os.path.join(latest_model_folder, "*", "introspection_model")):
            tag = int(prog.match(os.path.basename(os.path.dirname(i))).group(2))
            model = joblib.load(i) 
            hmm_model = model['hmm_model']
            threshold_for_introspection = model['threshold_for_introspection']
            model_group_by_state[tag] = hmm_model
            threshold_constant_group_by_state[tag] = threshold_for_introspection 

        self.detector = Detectors.DetectorBasedOnGradientOfLoglikCurve(
            model_group_by_state, 
            threshold_constant_group_by_state,
        )
        self.anomaly_detection_metric = "gradient_of_loglikelihood"
        self.model_group_by_state = model_group_by_state
        self.skip_this_state = False
        self.last_smach_state = None

    def run(self):

        rospy.init_node("IdSkillThenDetectAnomaly", anonymous=True)
        anomaly_detection_signal_pub = rospy.Publisher("/anomaly_detection_signal", Header, queue_size=100)
        anomaly_detection_metric_pub = rospy.Publisher("/scaled_and_stamped_anomaly_detection_metric_%s"%self.anomaly_detection_metric, Float64Stamped, queue_size=100)
        anomaly_detection_threshold_pub = rospy.Publisher("/scaled_and_stamped_anomaly_detection_threshold_%s"%self.anomaly_detection_metric, Float64Stamped, queue_size=100)
        identified_skill_pub = rospy.Publisher("/identified_skill_%s"%self.anomaly_detection_metric, Int8, queue_size=100)

        arrived_state = 0 
        while not rospy.is_shutdown():
            try:
                latest_data_tuple = self.com_queue.get(timeout=1)
            except Queue.Empty:
                continue
            except KeyboardInterrupt:
                break

            smach_state = latest_data_tuple[smach_state_idx]

            if smach_state != self.last_smach_state:
                self.last_smach_state = smach_state
                self.skip_this_state = False
                if smach_state <= 0:
                    self.detector.reset()
                    self.skip_this_state = True
                    continue

                if smach_state not in self.model_group_by_state:
                    rospy.logwarn("introspection_model_not_found")
                    show_introspection_model_not_found()
                    self.detector.reset()
                    self.skip_this_state = True
                    continue
            elif self.skip_this_state:
                continue
                

            data_frame = latest_data_tuple[data_frame_idx]
            data_header = latest_data_tuple[data_header_idx]

            now_skill, anomaly_detected, metric, threshold = self.detector.add_one_smaple_and_identify_skill_and_detect_anomaly(np.array(data_frame).reshape(1,-1), now_skill=smach_state)

            if anomaly_detected:
                anomaly_detection_signal_pub.publish(data_header) 
    
            if now_skill is not None:
                # identified_skill_pub.publish(now_skill)
                pass

            if metric is not None and threshold is not None:
                metric /= abs(threshold)
                threshold /= abs(threshold)
                anomaly_detection_metric_pub.publish(Float64Stamped(data_header, metric))
                anomaly_detection_threshold_pub.publish(Float64Stamped(data_header, threshold))

if __name__ == '__main__':
    com_queue_of_receiver = multiprocessing.Queue()
    process_receiver = TimeseriesReceiverProc(
        com_queue_of_receiver,
        node_name="TimeseriesReceiverProc_node_for_anomaly_detection",
        topic_name="timeseries_topic_for_anomaly_detection",
    )
    process_receiver.daemon = True

    com_queue_of_anomaly_detection = multiprocessing.Queue()
    process_anomaly_detection = IdSkillThenDetectAnomaly(
        com_queue_of_anomaly_detection,    
    )
    process_anomaly_detection.daemon = True



    process_receiver.start()
    process_anomaly_detection.start()


    rospy.init_node("anomaly_detection_node")
    rospy.loginfo('anomaly_detection.py starts')


    while not rospy.is_shutdown():
        try:
            latest_data_tuple = com_queue_of_receiver.get(timeout=1)
        except Queue.Empty:
            continue
        except KeyboardInterrupt:
            break
        com_queue_of_anomaly_detection.put(latest_data_tuple)

    rospy.loginfo('anomaly_detection.py exits')
