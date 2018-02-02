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

skill_sensory_data_folder_folder = os.path.join(
    introspection_data_folder,
    "skill_sensory_data_folder_folder")

SUCCESSULLY_EXECUTED_SKILL = 0
UNSUCCESSFULLY_EXECUTED_SKILL = 1

ROLLBACK_RECOVERY_TAG = -2
RECOVERY_DEMONSTRATED_BY_HUMAN_TAG = -3
