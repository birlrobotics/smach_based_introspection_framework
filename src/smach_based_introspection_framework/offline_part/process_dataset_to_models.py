import process_experiment_record_to_dataset as e2d
import glob
import os
from smach_based_introspection_framework._constant import (
    folder_time_fmt,
    latest_dataset_folder,
    latest_model_folder,
    anomaly_label_file,
    model_folder,
    RECOVERY_SKILL_BEGINS_AT
)
import re
import pandas as pd
from sklearn.externals import joblib
import shutil
import datetime
import json
import numpy as np
from model_training import train_introspection_model
from model_training import train_anomaly_classifier
import birl_baxter_dmp.dmp_train 
import ipdb
import smach_based_introspection_framework.configurables as configurables 
import logging

dmp_cmd_fields  = configurables.dmp_cmd_fields  
data_type_chosen  = configurables.data_type_chosen  
interested_data_fields  = configurables.interested_data_fields  
model_type  = configurables.model_type  
score_metric  = configurables.score_metric  
model_config  = configurables.model_config  

def get_latest_model_folder():
    if hasattr(get_latest_model_folder, "latest_model_folder"):
        return get_latest_model_folder.latest_model_folder
    if os.path.isdir(latest_model_folder):
       shutil.move(
            latest_model_folder,  
            os.path.join(model_folder, "model_folder.old.%s"%datetime.datetime.now().strftime(folder_time_fmt))
        )
        
    os.makedirs(latest_model_folder)
    get_latest_model_folder.latest_model_folder = latest_model_folder
    return latest_model_folder

def run():
    if not os.path.isdir(latest_dataset_folder):
        raise Exception("Not found %s"%latest_dataset_folder)

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger()

    log_file = os.path.join(get_latest_model_folder(), 'run.log')
    fileHandler = logging.FileHandler(os.path.realpath(log_file))
    logger.addHandler(fileHandler)

    consoleHandler = logging.StreamHandler()
    logger.addHandler(consoleHandler)

    logger.info("Collecting skill_data to build introspection model")
    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_folder, "skill_data", '*')):
        logger.info('Processing %s'%i)
        tag = prog.match(os.path.basename(i)).group(1)

        list_of_mat = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_mat.append(df[interested_data_fields].values)

        try:
            result = train_introspection_model.run(list_of_mat, model_type, model_config, score_metric)
        except Exception as e:
            logger.error("Failed to train introspection model for tag %s: %s"%(tag, e))
            continue
        logger.info("Successfully trained introspection model for tag %s"%(tag))
    
            
        d = os.path.join(get_latest_model_folder(), 'tag_%s'%tag)
        if not os.path.isdir(d):
            os.makedirs(d)
        joblib.dump(
            result['model'],
            os.path.join(d, 'introspection_model')
        )
    
        model_info = { 
            'data_type_chosen': data_type_chosen,
            'model_type': model_type,
            'find_best_model_in_this_config': model_config,
            'score_metric': score_metric,
        }
        model_info.update(result['model_info']),
        json.dump(
            model_info,
            open(os.path.join(d, 'introspection_model_info'), 'w'),
            indent=4,
        )

    logger.info("Collecting anomaly_data to build anomaly classifier")
    prog = re.compile(r'nominal_skill_(\d+)_anomaly_type_(.*)')
    for i in glob.glob(os.path.join(latest_dataset_folder, "anomaly_data", '*')):
        logger.info('Processing %s'%i)
        m = prog.match(os.path.basename(i))
        tag = m.group(1)
        anomaly_type = m.group(2)
        logger.info(tag, anomaly_type)
        if anomaly_type == 'Unlabeled':
            logger.info("Skip Unlabeled anomalies in skill %s"%tag)
            continue
        list_of_mat = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_mat.append(df[interested_data_fields].values)
        try:
            result = train_anomaly_classifier.run(list_of_mat, model_type, model_config, score_metric)
        except Exception as e:
            logger.error("Failed to train train_anomaly_classifier for anomaly_type %s in skill %s: %s"%(anomaly_type, tag, e))
            continue
        logger.info("Successfully trained classifier for anomaly type %s in skill %s"%(anomaly_type, tag))

        d = os.path.join(get_latest_model_folder(), os.path.basename(i))
        if not os.path.isdir(d):
            os.makedirs(d)
        joblib.dump(
            result['model'],
            os.path.join(d, 'classifier_model')
        )
        model_info = { 
            'data_type_chosen': data_type_chosen,
            'model_type': model_type,
            'find_best_model_in_this_config': model_config,
            'score_metric': score_metric,
        }
        model_info.update(result['model_info']),
        json.dump(
            model_info,
            open(os.path.join(d, 'classifier_model_info'), 'w'),
            indent=4,
        )


    logger.info("Collecting skill_data to build dmp")
    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_folder, "skill_data", '*')):
        tag = prog.match(os.path.basename(i)).group(1)
        if int(tag) < RECOVERY_SKILL_BEGINS_AT:
            logger.info("Skip nominal skill %s"%tag)
            continue
        logger.info("Gonna build dmp model for skill %s"%tag)
        list_of_mat = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_mat.append(df[dmp_cmd_fields].values)



        basis_weight, basis_function_type = birl_baxter_dmp.dmp_train.train(list_of_mat)
        logger.info("Built dmp model for skill %s"%tag)
        model = {
            "basis_weight": basis_weight,
            "basis_function_type": basis_function_type,
        }
        d = os.path.join(get_latest_model_folder(), os.path.basename(i))
        if not os.path.isdir(d):
            os.makedirs(d)
        joblib.dump(
            model,
            os.path.join(d, 'dmp_model')
        )
        model_info = { 
            'dmp_cmd_fields': dmp_cmd_fields,
        }
        json.dump(
            model_info,
            open(os.path.join(d, 'dmp_model_info'), 'w'),
            indent=4,
        )

    fileHandler.close()









