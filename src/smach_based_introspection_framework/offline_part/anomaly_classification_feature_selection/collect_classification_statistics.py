from smach_based_introspection_framework.offline_part.model_training import train_anomaly_classifier
from smach_based_introspection_framework._constant import (
    anomaly_classification_feature_selection_folder,
)
from smach_based_introspection_framework.configurables import model_type, model_config, score_metric
import glob
import os
import pandas as pd
import pprint
import coloredlogs, logging
import sys, traceback
from sklearn.externals import joblib
import json
import re


def get_models_of_scheme(scheme_folder, logger):
    at_extractor = re.compile(r'anomaly_type_\((.*)\)')

    models_grouped_by_type = {}

    for model_file in glob.glob(os.path.join(scheme_folder, 'anomaly_type_(*)', 'classifier_model')):
        logger.info(model_file)

        anomaly_type = at_extractor.search(model_file).group(1)

        logger.info(anomaly_type)

        with open(model_file, 'rb') as f:
            models_grouped_by_type[anomaly_type] = joblib.load(f)

    return models_grouped_by_type

def run():
    logger = logging.getLogger('CollectClassificationStats')

    scheme_folders = glob.glob(os.path.join(
        anomaly_classification_feature_selection_folder,
        'classifier_models',
        'No.* filtering scheme',
    ))



    for scheme_folder in scheme_folders:
        logger.info(scheme_folder)

        models_grouped_by_type = get_models_of_scheme(scheme_folder, logger)

        path_postfix = os.path.relpath(scheme_folder, os.path.join(anomaly_classification_feature_selection_folder, 'classifier_models'))
        
        logger.info(path_postfix)
        logger.info(models_grouped_by_type)

if __name__ == '__main__':
    run()
    
