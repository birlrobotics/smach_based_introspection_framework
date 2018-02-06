import os
import datetime

def get_latest_experiment_dir():
    if hasattr(get_latest_experiment_dir, "latest_experiment_dir"):
        return get_latest_experiment_dir.latest_experiment_dir

    from smach_based_introspection_framework._constant import experiment_record_folder, folder_time_fmt
    if not os.path.isdir(experiment_record_folder):
        os.makedirs(experiment_record_folder)

    latest_experiment_dir = os.path.join(
        experiment_record_folder,
        'experiment_at_%s'%datetime.datetime.now().strftime(folder_time_fmt),
    )
    if not os.path.isdir(latest_experiment_dir):
        os.makedirs(latest_experiment_dir)

    get_latest_experiment_dir.latest_experiment_dir = latest_experiment_dir

    return latest_experiment_dir
