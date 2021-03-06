from smach_based_introspection_framework.offline_part.model_training import train_anomaly_classifier
from smach_based_introspection_framework._constant import (
    anomaly_classification_feature_selection_folder,
)
from smach_based_introspection_framework.configurables import model_type, model_config, score_metric
import glob
import os,ipdb
import pandas as pd
import pprint
import coloredlogs, logging
import sys, traceback
from sklearn.externals import joblib
import json

coloredlogs.install()
pp = pprint.PrettyPrinter(indent=4)

def run():
    logger = logging.getLogger('GenClassificationModels')

    folders = glob.glob(os.path.join(
        anomaly_classification_feature_selection_folder,
        'No.* filtering scheme',
        'anomalies_grouped_by_type',
        'anomaly_type_(*)',
    )) 

    for folder in folders:
        logger.info(folder)
        path_postfix = os.path.relpath(folder, anomaly_classification_feature_selection_folder).replace("anomalies_grouped_by_type"+os.sep, "")

        output_dir = os.path.join(
            anomaly_classification_feature_selection_folder,
            'classifier_models',
            path_postfix,
        )
        model_file = os.path.join(output_dir, 'classifier_model')
        if os.path.isfile(model_file):
            logger.info("Model already exists. Gonna skip.")    
            continue

        csvs = glob.glob(os.path.join(
            folder,
            '*', '*.csv',
        ))
        list_of_mat = []
        for j in csvs:
            df = pd.read_csv(j, sep=',')
            # Exclude 1st column which is time index
            list_of_mat.append(df.values[:, 1:])

        try:
            result = train_anomaly_classifier.run(list_of_mat, model_type, model_config, score_metric)
            logger.info("Successfully trained classification model")
        except Exception as e:
            traceback.print_exc(file=sys.stdout)
            logger.error("Failed to train_anomaly_classifier: %s"%e)
            continue

        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        joblib.dump(
            result['model'],
            model_file,
        )
        model_info = { 
            'model_type': model_type,
            'find_best_model_in_this_config': model_config,
            'score_metric': score_metric,
        }
        model_info.update(result['model_info']),
        json.dump(
            model_info,
            open(os.path.join(output_dir, 'classifier_model_info'), 'w'),
            indent=4,
        )

if __name__ == '__main__':
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)
    run()
    
