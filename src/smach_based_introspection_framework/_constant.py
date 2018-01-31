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
