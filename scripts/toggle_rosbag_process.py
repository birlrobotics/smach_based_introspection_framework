import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from smach_based_introspection_framework.online_part.process_runner.rosbag_process import (
    RosbagProc
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
                rr = RosbagProc(
                    os.path.join(latest_experiment_record_folder, "record.bag"),
                    ['/tag_multimodal', '/anomaly_detection_signal']
                )
                rr.start()
                rospy.loginfo("toggle_rosbag_process started")
            else:
                raise Exception("Already started")
        else:
            if rr is not None:
                rr.stop()
                rospy.loginfo("toggle_rosbag_process stopped")
                rr = None
            else:
                raise Exception("Never started")
    except Exception as e:
        resp.success = False
        resp.message = str(e)
        rospy.logerr("Error during req %s: %s"%(req, e))

    return resp
        

if __name__ == '__main__':
    rospy.init_node("toggle_rosbag_process_node")
    server = rospy.Service(
        "toggle_rosbag_process_service", 
        SetBool,
        cb,
    )
    rospy.loginfo("toggle_rosbag_process server started")
    rospy.spin()
