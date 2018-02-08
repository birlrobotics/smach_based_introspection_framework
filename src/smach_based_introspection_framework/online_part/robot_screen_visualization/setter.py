import os
import cv2
import cv_bridge
from sensor_msgs.msg import (
    Image,
)
import rospy

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
image_dir = os.path.join(dir_of_this_script, '..', '..', '..', '..', 'image')
def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(os.path.join(image_dir, path))
    if img is not None:
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
    else:
        rospy.logerr("Failed to update robot screen, imread returns None.")

def show_everyhing_is_good():
    send_image("green.jpg")

def show_anomaly_detected():
    send_image("red.jpg")

def show_introspection_model_not_found():
    send_image("yellow.jpg")
