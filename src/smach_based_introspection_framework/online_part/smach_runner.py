from smach_modifier import modify_user_sm
import rospy
import os
import smach_ros
from std_srvs.srv import SetBool, SetBoolRequest

def shutdown():
    rospy.loginfo("Shuting down, PID: %s"%os.getpid())
    pass

def control_service_processes(start):
    snames = [
        'toggle_anomaly_classification_process_service',
        'toggle_rosbag_process_service',
        'toggle_tag_multimodal_topic_process_service',     
    ]
    if start:
        req = SetBoolRequest()
        req.data = True
        for sname in snames:
            sp = rospy.ServiceProxy(sname, SetBool)
            resp = sp(req)
            if not resp.success:
                raise Exception("Failed to start process using service: %s. Errmsg: %s"%(snamei, resp.message))
    else:
        req = SetBoolRequest()
        req.data = False
        for sname in snames:
            sp = rospy.ServiceProxy(sname, SetBool)
            resp = sp(req)

def run(sm):
    try: 
        rospy.loginfo("PID: %s"%os.getpid())
        rospy.init_node("smach_based_introspection_framework_node")
        rospy.on_shutdown(shutdown)

        sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
        sis.start()
        sm = modify_user_sm.run(sm)

        try:
            control_service_processes(True)
        except Exception as e:
            control_service_processes(False)
            rospy.loginfo(str(e))
            raise
        rospy.loginfo("service processes up, start sm in 5 secs.")
        rospy.sleep(5)

        rospy.loginfo("start sm.")
        outcome = sm.execute()
        rospy.loginfo('sm.execute() returns %s'%outcome)

        rospy.loginfo("tear down service processes in 5 secs.")
        rospy.sleep(5)
        control_service_processes(False)
        rospy.loginfo("teared down.")

        sis.stop()
    except:
        os.killpg(os.getpid())
