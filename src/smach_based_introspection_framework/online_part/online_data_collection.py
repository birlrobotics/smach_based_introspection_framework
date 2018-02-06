from framework_master import get_latest_experiment_dir
from _core import (
    listen_HMM_anomaly_signal,
)
import os
from data_collection.experiment_recording_via_rosbag import RosbagRecord

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

def run(
    task_id=None,
    no_state_trainsition_report=False, 
    no_anomaly_detection=False , 
    use_manual_anomaly_signal=False,
):
    import _core
    _core.mode_no_state_trainsition_report = no_state_trainsition_report
    if not no_anomaly_detection:
        listen_HMM_anomaly_signal(use_manual_anomaly_signal)

    control_rosbag_recording(start=True)

def stop():
    control_rosbag_recording(start=False)
