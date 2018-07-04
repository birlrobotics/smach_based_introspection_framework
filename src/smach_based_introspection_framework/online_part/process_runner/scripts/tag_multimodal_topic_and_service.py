#!/usr/bin/env python
import argparse
import struct
import sys
import copy
import ipdb

import rospy
from std_msgs.msg import (
    Empty,
    Header,
    Int64
)

import copy

from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from smach_based_introspection_framework.msg import (
    Tag_MultiModal, tactile_static
)
from smach_based_introspection_framework.srv import (
    State_Switch,
    State_SwitchResponse
)

hmm_state = None
def state_switch_handle(req):
    global hmm_state
    hmm_state = req.state
    rospy.loginfo("tag is changed to %d" %req.state)
    resp = State_SwitchResponse()
    resp.finish.data = True
    return resp


def main():
    global hmm_state
    global shared_endpoint_state
    global shared_joint_state
    global shared_wrench_stamped

    hmm_state = 0

    publishing_rate = 100

    rospy.init_node("topic_multimodal", anonymous=True)

    rospy.loginfo("tag_multimodal_topic_and_service.py starts")

    pub = rospy.Publisher("/tag_multimodal",Tag_MultiModal, queue_size=10)
    state_switch = rospy.Service('/hmm_state_switch', State_Switch, state_switch_handle)
    r = rospy.Rate(publishing_rate)

    while not rospy.is_shutdown():
        tag_multimodal = Tag_MultiModal()
        tag_multimodal.tag = hmm_state
        tag_multimodal.header = Header()
        tag_multimodal.header.stamp = rospy.Time.now()
        pub.publish(tag_multimodal)

        try:
            r.sleep()
        except rospy.exceptions.ROSInterruptException:
            break

    rospy.loginfo("tag_multimodal_topic_and_service.py exits")

if __name__ == '__main__':
    main()
