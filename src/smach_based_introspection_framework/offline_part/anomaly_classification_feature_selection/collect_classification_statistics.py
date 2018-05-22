from smach_based_introspection_framework.offline_part.model_training import train_anomaly_classifier
from smach_based_introspection_framework._constant import (
    anomaly_classification_feature_selection_folder,
)
from smach_based_introspection_framework.configurables import model_type, model_config, score_metric
from smach_based_introspection_framework.online_part.anomaly_classifier.Classifier import NormalDistributedConfidenceClassifier
import glob
import os
import pandas as pd
import pprint
import coloredlogs, logging
import sys, traceback
from sklearn.externals import joblib
import json
import re
import numpy as np
import copy


at_extractor = re.compile(r'anomaly_type_\((.*)\)')
def get_models_of_scheme(scheme_folder, logger):
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
        c = NormalDistributedConfidenceClassifier(models_grouped_by_type)

        path_postfix = os.path.relpath(scheme_folder, os.path.join(anomaly_classification_feature_selection_folder, 'classifier_models'))
        
        anomaly_csvs = glob.glob(os.path.join(
            anomaly_classification_feature_selection_folder,
            path_postfix,
            'anomalies_grouped_by_type',
            'anomaly_type_(*)',
            '*',
            '*.csv',
        ))

        stat_df = pd.DataFrame()

        for anomaly_csv in anomaly_csvs:
            anomaly_type_given_by_human = at_extractor.search(anomaly_csv).group(1)
            with open(anomaly_csv, 'r') as f:
                ano_df = pd.read_csv(f)
                mat = ano_df.values[:, 1:]

            predict_label = c.predict(mat)
            
            d = {}
            d['anomaly_csv'] = os.path.basename(anomaly_csv)
            d['anomaly_type_given_by_human'] = anomaly_type_given_by_human

            if predict_label == anomaly_type_given_by_human: 
                l2_d = copy.deepcopy(d)
                l2_d['anomaly_type_being_tested'] = anomaly_type_given_by_human
                l2_d['TP'] = 1
                stat_df = stat_df.append(l2_d, ignore_index=True)
            else:
                l2_d = copy.deepcopy(d)
                l2_d['anomaly_type_being_tested'] = anomaly_type_given_by_human
                l2_d['FN'] = 1
                stat_df = stat_df.append(l2_d, ignore_index=True)

                l2_d = copy.deepcopy(d)
                l2_d['anomaly_type_being_tested'] = predict_label
                l2_d['FP'] = 1
                stat_df = stat_df.append(l2_d, ignore_index=True)

        output_dir = os.path.join(
            anomaly_classification_feature_selection_folder,
            'classification_statistics',
            path_postfix,
        )
        stat_file = os.path.join(output_dir, 'stat.csv')
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        stat_df.to_csv(stat_file)

if __name__ == '__main__':
    run()
    
