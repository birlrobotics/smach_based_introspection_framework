from smach_based_introspection_framework import ExperimentRecord
import glob
from smach_based_introspection_framework._constant import (
    experiment_record_folder, 
    anomaly_classification_feature_selection_folder,
)
import os
from rostopics_to_timeseries import OfflineRostopicsToTimeseries
from filtering_schemes import filtering_schemes
import itertools
import pandas as pd
import pickle
import shutil
import rospy
import coloredlogs, logging
coloredlogs.install()

def run():
    logger = logging.getLogger('AcfsGenAnoDataset')
    exp_dirs = [i for i in glob.glob(os.path.join(experiment_record_folder, '*')) if os.path.isdir(i)]
    exp_dirs.sort() 
    for exp_dir in exp_dirs:
        logger.info(exp_dir)
        er = ExperimentRecord(exp_dir)
        for anomaly_count, (anomaly_type, anomaly_time) in enumerate(er.anomaly_signals):
            logger.info("anomaly_type: %s, anomaly_time: %s"%(anomaly_type, anomaly_time)) 

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    run()
