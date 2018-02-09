import rospy 
from smach_based_introspection_framework.online_part.process_runner.anomaly_classification_process import (
    AnomalyClassificationProc 
)
from smach_based_introspection_framework.online_part.process_runner.tag_multimodal_topic_process import (
    TagMultimodalTopicProc, 
)
from smach_based_introspection_framework.srv import (
    AnomalyClassificationService,
)
import ipdb

if __name__ == '__main__':
    rospy.init_node("test_anomaly_classification")
    
    tmt_proc = TagMultimodalTopicProc()
    tmt_proc.start()
    ac_proc = AnomalyClassificationProc()
    ac_proc.start()
    rospy.sleep(10)
    sp = rospy.ServiceProxy('AnomalyClassificationService', AnomalyClassificationService)
    try:
        anomaly_t = rospy.Time.now().to_sec()-3
        print "anomaly_t", anomaly_t
        resp = sp(anomaly_t)
        print 'resp', resp
    except rospy.ServiceException as exc:
        rospy.logerr("calling AnomalyClassificationService failed")
        raise Exception("calling AnomalyClassificationService failed")

    rospy.sleep(10)
    tmt_proc.stop()
    ac_proc.stop()
