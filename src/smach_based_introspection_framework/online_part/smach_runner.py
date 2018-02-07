from smach_modifier import modify_user_sm
import rospy
import os
import smach_ros
from std_srvs.srv import SetBool, SetBoolRequest
from smach_based_introspection_framework._constant import (
    latest_experiment_record_folder,
    experiment_record_folder, 
    folder_time_fmt,
)
from smach_based_introspection_framework.online_part.process_runner.rosbag_process import (
    RosbagProc
)
from smach_based_introspection_framework.online_part.process_runner.anomaly_classification_process import (
    AnomalyClassificationProc 
)
from smach_based_introspection_framework.online_part.process_runner.tag_multimodal_topic_process import (
   TagMultimodalTopicProc, 
)
import shutil
import datetime

def shutdown():
    rospy.loginfo("Shuting down, PID: %s"%os.getpid())
    pass

rosbag_proc = None
ac_proc = None
tmt_proc = None
sis = None
def toggle_introspection(start):
    global rosbag_proc, ac_proc, tmt_proc, sis
    if start:
        if not os.path.isdir(latest_experiment_record_folder):
            os.makedirs(latest_experiment_record_folder)
        rosbag_proc = RosbagProc(
            os.path.join(latest_experiment_record_folder, "record.bag"),
            ['/tag_multimodal', '/anomaly_detection_signal']
        )
        rosbag_proc.start()
        ac_proc = AnomalyClassificationProc()
        ac_proc.start()
        tmt_proc = TagMultimodalTopicProc()
        tmt_proc.start()
        sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
        sis.start()
        rospy.sleep(2)
    else:
        rospy.sleep(2)
        rosbag_proc.stop()
        ac_proc.stop()
        tmt_proc.stop()
        sis.stop()
        rospy.sleep(2)
        shutil.move(
            latest_experiment_dir, 
            os.path.join(
                experiment_record_folder,
                'experiment_at_%s'%datetime.datetime.now().strftime(folder_time_fmt),
            )
        )

def run(sm):
    try: 
        rospy.loginfo("PID: %s"%os.getpid())
        rospy.init_node("smach_based_introspection_framework_node")
        rospy.on_shutdown(shutdown)

        sm = modify_user_sm.run(sm)

        try:
            toggle_introspection(True)
        except Exception as e:
            toggle_introspection(False)
            rospy.loginfo(str(e))
            raise
        rospy.loginfo("introspection up.")

        rospy.loginfo("start sm.")
        outcome = sm.execute()
        rospy.loginfo('sm.execute() returns %s'%outcome)

        toggle_introspection(False)
        rospy.loginfo("introspection down.")

    except:
        os.killpg(os.getpid())
