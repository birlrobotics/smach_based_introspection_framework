import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from smach_based_introspection_framework.online_part.process_runner.redis_based_anomaly_classification_process import (
    RedisBasedAnomalyClassification, 
)
from smach_based_introspection_framework._constant import (
    latest_experiment_record_folder,
)
import os

rr = None

def cb(req):
    global rr
    resp = SetBoolResponse()
    resp.success = True
    resp.message = ''
    try:
        if req.data:
            if rr is None:
                if not os.path.isdir(latest_experiment_record_folder):
                    os.makedirs(latest_experiment_record_folder)
                rr = RedisBasedAnomalyClassification()
                rr.start()
                rospy.loginfo("redis_based_anomaly_classification started")
            else:
                raise Exception("Already started")
        else:
            if rr is not None:
                rr.stop()
                rospy.loginfo("redis_based_anomaly_classification stopped")
                rr = None
            else:
                raise Exception("Never started")
    except Exception as e:
        resp.success = False
        resp.message = str(e)
        rospy.logerr("Error during req %s: %s"%(req, e))

    return resp
        

if __name__ == '__main__':
    rospy.init_node("redis_based_anomaly_classification_node")
    server = rospy.Service(
        "redis_based_anomaly_classification_service", 
        SetBool,
        cb,
    )
    rospy.loginfo("redis_based_anomaly_classification server started")
    rospy.spin()
