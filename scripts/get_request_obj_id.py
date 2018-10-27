#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from smach_based_introspection_framework.srv import (
    Intereset_pose,
    Intereset_poseRequest,
    Intereset_poseResponse,
)
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import threading
import copy
import numpy
import ipdb


shared_msg = None

def cb(msg):
    global shared_msg
    shared_msg = msg.markers

def call_back(req):
    global shared_msg

    request = copy.deepcopy(req.request_id)
    rsp = Intereset_poseResponse()
    env_id = []
    found_id = []
    missing_id = []
    pose_list = []
    for msg in shared_msg:
        env_id.append(msg.id)

    for idx in id_request:
        if idx in env_id:
            found_id.append(idx)
        else:
            missing_id.append(idx)

    rsp.missing_id = missing_id
    rsp.found_id = found_id
   
    for idx in found_id:
        for msg in shared_msg:
            if msg.id == idx:
                pose_list.append(msg.pose.pose)
    # ipdb.set_trace()
    rsp.pose_list = pose_list
    return rsp

def main():
    rospy.init_node("get_request_obj_id")
    rospy.wait_for_message("ar_pose_marker",AlvarMarkers)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
    s = rospy.Service("get_request_obj_id",Intereset_pose,call_back)
    rospy.spin()
if __name__=="__main__":
    main()