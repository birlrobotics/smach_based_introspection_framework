from smach_based_introspection_framework.online_part.process_runner.shell_process_runner import ShellProcessRunner
from smach_based_introspection_framework._constant import experiment_video_folder
import os
import rospy
import shutil
import gtk, pygtk
from smach_based_introspection_framework.srv import (
    ExperimentRecording,
    ExperimentRecordingRequest,
    ExperimentRecordingResponse,
)
TEST_MODE = False

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
default_mp4_path = os.path.join(experiment_video_folder, "latest.mp4")
class ExperimentVideoProc(ShellProcessRunner):
    def __init__(self):
        super(ExperimentVideoProc, self).__init__()
        script_path = os.path.join(
            dir_of_this_script,
            'scripts',
            'anomaly_detection.py',
        )
        if not os.path.isdir(experiment_video_folder):
            os.makedirs(experiment_video_folder)

        default_mp4_path = os.path.join(experiment_video_folder, "latest.mp4")
        if os.path.isfile(default_mp4_path):
            os.remove(default_mp4_path)
        window = gtk.Window()
        screen = window.get_screen()
        self.cmd = "avconv -an -f x11grab -s %sx%s -r 20 -i :0.0 -r 20 %s"%(
            screen.get_width(),
            screen.get_height(),
            os.path.realpath(default_mp4_path))
evp = None
def cb(req):
    global evp
    if req.start_recording:
        if evp is not None and evp.started:
            evp.stop()
        evp = ExperimentVideoProc()
        evp.start()
    else:
        if evp is None or not evp.started:
            pass
        else:
            evp.stop() 
            if req.experiment_name == "":
                req.experiment_name = "None"
            experiment_mp4_path = os.path.join(experiment_video_folder, "%s.mp4"%req.experiment_name)
            shutil.move(
                default_mp4_path,
                experiment_mp4_path 
            )

    return ExperimentRecordingResponse(True)


if __name__ == '__main__':
    if TEST_MODE:
        import time
        evp = ExperimentVideoProc()
        evp.start()
        print 'Gonna sleep 5 secs'
        time.sleep(5)
        evp.stop()
        print 'Done'
    else:
        rospy.init_node("start_cam_recording_service_node")
        ss = rospy.Service("experiment_recording_service", ExperimentRecording, cb) 
        rospy.spin()
    
