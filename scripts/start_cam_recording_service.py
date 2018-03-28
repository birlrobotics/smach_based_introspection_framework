from smach_based_introspection_framework.online_part.process_runner.shell_process_runner import ShellProcessRunner
from smach_based_introspection_framework._constant import experiment_video_folder
import os
import rospy
import shutil
import gtk, pygtk

window = gtk.Window()
screen = window.get_screen()

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
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

        mp4_path = os.path.join(experiment_video_folder, "latest.mp4")
        if os.path.isfile(mp4_path):
            os.remove(mp4_path)
        self.cmd = "avconv -an -f x11grab -s %sx%s -r 20 -i :0.0 -r 20 %s"%(
            screen.get_width(),
            screen.get_height(),
            os.path.realpath(mp4_path))


TEST_MODE = True

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
        
    
