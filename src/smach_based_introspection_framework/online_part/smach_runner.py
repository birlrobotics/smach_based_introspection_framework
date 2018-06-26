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
from smach_based_introspection_framework.online_part.process_runner.AnomalyDetectionProc import (
    AnomalyDetectionProc
)
from smach_based_introspection_framework.online_part.process_runner.tag_multimodal_topic_process import (
   TagMultimodalTopicProc, 
)
from smach_based_introspection_framework.online_part.process_runner.timeseries_process import (
   TimeseriesProc, 
)
from smach_based_introspection_framework.online_part.process_runner.goal_process import (
   GoalProc, 
)
import shutil
import datetime
import signal
from smach_based_introspection_framework.online_part.framework_core.states import (
    listen_HMM_anomaly_signal,
    set_reverting_statistics, 
)
import moveit_commander
import sys
from smach_based_introspection_framework.configurables import (
    topics_to_be_recorded_into_rosbag,
    HUMAN_AS_MODEL_MODE,
)
from smach_based_introspection_framework.srv import (
    ExperimentRecording,
    ExperimentRecordingRequest,
    ExperimentRecordingResponse,
)
import time

def shutdown():
    rospy.loginfo("Shuting down, PID: %s"%os.getpid())
    pass

rosbag_proc = None
ac_proc = None
tmt_proc = None
sis = None
ad_proc = None
ts_proc = None
goal_proc = None

def toggle_experiment_recording(start, experiment_name="Unnamed"):
    try:
        sp = rospy.ServiceProxy("experiment_recording_service", ExperimentRecording)
        req = ExperimentRecordingRequest()
        if start:
            req.start_recording = True
        else:
            req.start_recording = False
            req.experiment_name = experiment_name
        sp.call(req)
    except Exception as e:
        rospy.logerr("toggle_experiment_recording failed: %s"%e)

def toggle_introspection(start, sm=None):
    global rosbag_proc, ac_proc, tmt_proc, sis, ad_proc, ts_proc, goal_proc
    if start:
        toggle_experiment_recording(True)
        if not os.path.isdir(latest_experiment_record_folder):
            os.makedirs(latest_experiment_record_folder)
        rosbag_proc = RosbagProc(
            os.path.join(latest_experiment_record_folder, "record.bag"),
            topics_to_be_recorded_into_rosbag
        )
        rosbag_proc.start()
        # tmt_proc = TagMultimodalTopicProc()
        # tmt_proc.start()
        sis = smach_ros.IntrospectionServer('MY_SERVER', sm, '/SM_ROOT')
        sis.start()

        if not HUMAN_AS_MODEL_MODE:
            # ad_proc = AnomalyDetectionProc()
            # ad_proc.start()
            # ac_proc = AnomalyClassificationProc()
            # ac_proc.start()
            pass

        # ts_proc = TimeseriesProc()
        # ts_proc.start()
        goal_proc = GoalProc()
        goal_proc.start()
        listen_HMM_anomaly_signal()
    else:
        experiment_name = 'experiment_at_%s'%datetime.datetime.now().strftime(folder_time_fmt)
        toggle_experiment_recording(False, experiment_name)
        if rosbag_proc:
            rospy.loginfo("Tring to tear down rosbag_proc")
            rosbag_proc.stop()
            shutil.move(
                latest_experiment_record_folder, 
                os.path.join(
                    experiment_record_folder,
                    experiment_name
                )
            )
        if ac_proc:
            rospy.loginfo("Tring to tear down ac_proc")
            ac_proc.stop()
        if tmt_proc:
            rospy.loginfo("Tring to tear down tmt_proc")
            tmt_proc.stop()
        if sis:
            rospy.loginfo("Tring to tear down sis")
            sis.stop()
        if ad_proc:
            rospy.loginfo("Tring to tear down ad_proc")
            ad_proc.stop()
        if ts_proc:
            rospy.loginfo("Tring to tear down ts_proc")
            ts_proc.stop()
        if goal_proc:
            rospy.loginfo("Tring to tear down goal_proc")
            goal_proc.stop()
            

def run(sm, reverting_statistics=None):
    try: 
        set_reverting_statistics(reverting_statistics)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("smach_based_introspection_framework_node", log_level=rospy.INFO)
        rospy.loginfo("PID: %s"%os.getpid())
        rospy.on_shutdown(shutdown)

        sm = modify_user_sm.run(sm)

        try:
            toggle_introspection(True, sm)
        except Exception as e:
            toggle_introspection(False)
            rospy.loginfo(str(e))
            raise

        rospy.loginfo("introspection up.")

        try:
            rospy.loginfo("start sm.")
            outcome = sm.execute()
            rospy.loginfo('sm.execute() returns %s'%outcome)
        except Exception as e:
            rospy.logerr(str(e))

        toggle_introspection(False)
        rospy.loginfo("introspection down.")

    except:
        pass
    finally:
        time.sleep(2)
        os.killpg(os.getpid(), signal.SIGINT)
