from smach_based_introspection_framework.online_part.framework_master import get_latest_experiment_dir
import os
from experiment_recording_via_rosbag import RosbagRecord

def control_rosbag_recording(start):
    if start:
        if hasattr(control_rosbag_recording, 'o'):
            return
        o = RosbagRecord(
            os.path.join(get_latest_experiment_dir(), "record.bag"),
            ['/tag_multimodal', '/anomaly_detection_signal']
        )
        o.start()
        control_rosbag_recording.o = o
    else:
        if not hasattr(control_rosbag_recording, 'o'):
            return
        control_rosbag_recording.o.stop()
        delattr(control_rosbag_recording, 'o')

def start():
    control_rosbag_recording(start=True)

def stop():
    control_rosbag_recording(start=False)
