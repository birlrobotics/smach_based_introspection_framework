import process_experiment_record_to_dataset as e2d
import glob
import os
from _constant import (
    folder_time_fmt,
    dataset_folder,
    anomaly_label_file,
)
import re

def train_introspection_models():
    pass

def run():
    latest_dataset_dir = os.path.join(dataset_folder, 'latest')
    if not os.path.isdir(latest_dataset_dir):
        raise Exception("Not found %s"%latest_dataset_dir)

    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_dir, "skill_data", '*')):
        tag = prog.match(os.path.basename(i)).group(1)
    
