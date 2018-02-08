from smach_based_introspection_framework.online_part.smach_modifier import dmp_execute
import sys
import moveit_commander
from sklearn.externals import joblib
from geometry_msgs.msg import (
    Pose,
    Quaternion,
)
import rospy
import random

pick_object_pose = Pose()
pick_object_pose.position.x = 0.604200717675+random.uniform(-0.10, +0.10)
pick_object_pose.position.y = -0.312913829206+random.uniform(-0.10, +0.10)
pick_object_pose.position.z = -0.235318105119
pick_object_pose.orientation = Quaternion(
    x= 0.492980327492,
    y= 0.870019950792,
    z= 0.00502515484191,
    w= 0.00322951215831,
)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_interruptatble_moveit_dmp_exec")
    dmp_model = joblib.load('dmp_model')    
    dmp_execute.execute(dmp_model, pick_object_pose)

