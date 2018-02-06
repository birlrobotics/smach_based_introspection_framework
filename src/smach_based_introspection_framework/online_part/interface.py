from _core import (
    get_event_flag,
    set_event_flag,
    AnomalyDiagnosis,
    HumanTeachingRecovery,
    RollBackRecovery,
    write_exec_hist,
    hmm_state_switch_client,
    listen_HMM_anomaly_signal,
)
import copy
from _smach_execute_decorator import smach_execute_decorator
import types
import ipdb
import os
import datetime

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))


def start_instrospection(
    task_id=None,
    no_state_trainsition_report=False, 
    no_anomaly_detection=False , 
    use_manual_anomaly_signal=False,
):
    import _core
    _core.mode_no_state_trainsition_report = no_state_trainsition_report
    if not no_anomaly_detection:
        listen_HMM_anomaly_signal(use_manual_anomaly_signal)

    from smach_based_introspection_framework._constant import experiment_record_folder
    if not os.path.isdir(experiment_record_folder):
        os.makedirs(experiment_record_folder)

    from smach_based_introspection_framework._constant import folder_time_fmt
    experiment_folder = os.path.join(
        experiment_record_folder,
        'experiment_at_%s'%datetime.datetime.now().strftime(folder_time_fmt),
    )
    if not os.path.isdir(experiment_folder):
        os.makedirs(experiment_folder)

    import _experiment_recording_via_rosbag
    o = _experiment_recording_via_rosbag.RosbagRecord(
        os.path.join(experiment_folder, "record.bag"),
        ['/tag_multimodal', '/anomaly_detection_signal']
    )
    o.start()

