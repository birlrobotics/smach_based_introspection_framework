from smach_modifier import modify_user_sm
import data_collection.main
import rospy
import os
import smach_ros

def shutdown():
    rospy.loginfo("Shuting down, PID: %s"%os.getpid())
    data_collection.main.stop() 
    pass

def run(sm):
    try: 
        rospy.loginfo("PID: %s"%os.getpid())
        rospy.init_node("smach_based_introspection_framework_node")
        rospy.on_shutdown(shutdown)
        sm = modify_user_sm.run(sm)

        sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
        sis.start()

        data_collection.main.start()
        rospy.loginfo("Gonna wait 5 secs after starting online_data_collection")
        rospy.sleep(5)

        #outcome = sm.execute()
        #print 'sm.execute() returns', outcome

        sis.stop()
    except:
        os.killpg(os.getpid())
