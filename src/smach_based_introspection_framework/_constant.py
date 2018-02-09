ANOMALY_DETECTED = 0
ANOMALY_DETECTION_BLOCKED = -1
ANOMALY_NOT_DETECTED = 1
RECOVERY_JUST_DONE = -2

folder_time_fmt = "%Yy%mm%dd%HH%MM%SS" 
import os
dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
introspection_data_folder = os.path.join(
    dir_of_this_script, 
    '..',
    '..',
    "introspection_data_folder")

experiment_record_folder = os.path.join(
    introspection_data_folder,
    "experiment_record_folder")
latest_experiment_record_folder = os.path.join(
    experiment_record_folder,
    'latest',
)

dataset_folder = os.path.join(
    introspection_data_folder,
    "dataset_folder")
latest_dataset_folder = os.path.join(
    dataset_folder,
    'latest',
)

model_folder = os.path.join(
    introspection_data_folder,
    "model_folder")
latest_model_folder = os.path.join(
    model_folder,
    'latest',
)

realtime_anomaly_plot_folder = os.path.join(
    introspection_data_folder,
    "realtime_anomaly_plot_folder")

anomaly_label_file = "anomaly_labels.txt"

SUCCESSULLY_EXECUTED_SKILL = 0
UNSUCCESSFULLY_EXECUTED_SKILL = 1

ROLLBACK_RECOVERY_TAG = -2
RECOVERY_DEMONSTRATED_BY_HUMAN_TAG = -3

RECOVERY_SKILL_BEGINS_AT = 1000


