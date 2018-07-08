import os
import glob
import pandas as pd
from sklearn.externals import joblib
import json
import traceback
from model_training import train_anomaly_classifier
import smach_based_introspection_framework.configurables as configurables 
model_type  = configurables.model_type  
score_metric  = configurables.score_metric  
model_config  = configurables.model_config  

def run(logger, anomaly_folder, output_dir):
    model_file = os.path.join(output_dir, 'classifier_model')
    if os.path.isfile(model_file):
        logger.info("Model already exists. Gonna skip.")    
        return 
    csvs = glob.glob(os.path.join(
        anomaly_folder,
        '*',
    ))
    list_of_mat = []
    for j in csvs:
        df = pd.read_csv(j, sep=',')
        # Exclude 1st column which is time index
        list_of_mat.append(df.values[:, 1:])
    try:
        result = train_anomaly_classifier.run(list_of_mat, model_type, model_config, score_metric, logger)
        logger.info("Successfully trained classification model")
    except Exception as e:
        logger.error("Failed to train_anomaly_classifier: %s"%e)
        logger.error("traceback: %s"%(traceback.format_exc()))
        return

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
