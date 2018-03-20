#!/usr/bin/env python
import rospy
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node("send_manual_anomaly_signal_py")
    anomaly_detection_signal_pub = rospy.Publisher("/anomaly_detection_signal", Header, queue_size=0)

    while not rospy.is_shutdown():
        rospy.loginfo("Press [Enter] to send a signal")
        raw_input()
        msg = Header()
        msg.stamp = rospy.Time.now()
        anomaly_detection_signal_pub.publish(msg)
        rospy.loginfo("Sent")
        
        
    
