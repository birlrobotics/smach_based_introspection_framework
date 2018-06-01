import itertools
from smach_based_introspection_framework._constant import (
    anomaly_classification_feature_selection_folder,
)
import glob
import os
import pandas as pd
import coloredlogs, logging
import matplotlib.pyplot as plt
import pickle
import re
import ipdb

def run():
    logger = logging.getLogger('GetAnomalyTpyes')
    folders = glob.glob(os.path.join(
        anomaly_classification_feature_selection_folder,'..',
        'experiment_record_folder','*'))
    logger.info(folders)
    dataset_info = open(os.path.join(anomaly_classification_feature_selection_folder, 'dataset_info.txt'), 'w')
    anomalies = []
    for folder in itertools.chain(folders):
        logger.info(folder)
        path_postfix = os.path.relpath(folder, os.path.join(folder, '..'))
        for txt in glob.glob(os.path.join(folder,'anomaly_labels.txt')):
            with open(txt, 'rb') as f:
                anomaly_type = f.readlines()
                dataset_info.write(path_postfix + ' :\n')
                for _type in anomaly_type:
                     dataset_info.write(_type)
                     anomalies.append(_type)
                dataset_info.write('\n')
    dataset_info.close()

    ntypes = [anomalies.count(atype) for atype in set(anomalies)]
    dtype  = [a_t.strip('\n') +": "+str(n_t)+"\n" for a_t, n_t in zip(set(anomalies), ntypes)]
    with open(os.path.join(anomaly_classification_feature_selection_folder, 'dataset_info.txt'), 'r+') as f:
        file_data = f.read()
        f.seek(0,0)
        f.write(''.join(dtype) + '\n' + file_data)
if __name__ == "__main__":
    run()
