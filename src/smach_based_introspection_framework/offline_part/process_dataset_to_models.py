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
from smach_based_introspection_framework.offline_part.model_training import train_dmp_model
from smach_based_introspection_framework.offline_part import util 
import ipdb
import smach_based_introspection_framework.configurables as configurables 
import logging
import dill

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
    logger = logging.getLogger('process_dataset_to_models')

    log_file = os.path.join(get_latest_model_folder(), 'run.log')
    fileHandler = logging.FileHandler(os.path.realpath(log_file))
    logger.addHandler(fileHandler)

    consoleHandler = logging.StreamHandler()
    logger.addHandler(consoleHandler)

    '''
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
            logger.info("Successfully trained introspection model for tag %s"%(tag))
        except Exception as e:
            logger.error("Failed to train_introspection_model for skill %s: %s"%(tag, e))
            continue
    
            
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
    '''
    
    logger.info("Collecting anomaly_data to build anomaly classifier")
    prog = re.compile(r'nominal_skill_(\d+)_anomaly_type_(.*)')
    anomaly_paths = glob.glob(os.path.join(latest_dataset_folder, "anomaly_data", '*'))
    all_anomaly_type = []
    for i in anomaly_paths:
        m = prog.match(os.path.basename(i))
        all_anomaly_type.append(m.group(2))
    for anomaly_type in np.unique(all_anomaly_type):
        logger.info('Processing %s'%anomaly_type)
        if anomaly_type == 'Unlabeled':
            logger.info("Skip Unlabeled anomalies in skill")
            continue
        list_of_mat = []
        for ipath in filter(lambda x: anomaly_type in x, anomaly_paths):
            for j in glob.glob(os.path.join(ipath, "*")):
                df = pd.read_csv(j, sep=',')
                list_of_mat.append(df[interested_data_fields].values)            
        try:
            result = train_anomaly_classifier.run(list_of_mat, model_type, model_config, score_metric)
        except Exception as e:
            logger.error("Failed to train train_anomaly_classifier for anomaly_type %s in skill: %s"%(anomaly_type,  e))
            continue
        logger.info("Successfully trained classifier for anomaly type %s"%(anomaly_type))

        for ipath in filter(lambda x: anomaly_type in x, anomaly_paths):
            d = os.path.join(get_latest_model_folder(), os.path.basename(ipath))
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
            logger.info("Skip nominal skill %s without recovery dmp data"%tag)
            continue
        logger.info("Gonna build dmp model for skill %s"%tag)
        one_shot_mat = None
        goal_info = {}
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            cmd_df =  df[dmp_cmd_fields]
            dem_goal = cmd_df.iloc[-1].values
            ori_goal = np.array(eval(df['.goal_vector'].iloc[-1]))
            one_shot_mat = cmd_df.values
            goal_info['original_goal'] = ori_goal
            goal_info['demonstration_goal'] = dem_goal
            break

        goal_modification_info = {}
        pxyz_idx = util.get_position_xyz_index(dmp_cmd_fields)
        if None not in pxyz_idx:
            goal_modification_info['translation'] = {
                'index': pxyz_idx,
                'value': dem_goal[pxyz_idx]-ori_goal[pxyz_idx],
            }

        qxyzw_idx = util.get_quaternion_xyzw_index(dmp_cmd_fields)
        if None not in qxyzw_idx:
            from tf.transformations import (
                quaternion_inverse,
                quaternion_multiply,
            )
            ori_q = ori_goal[qxyzw_idx]
            dem_q = dem_goal[qxyzw_idx]
            rot_q = quaternion_multiply(dem_q, quaternion_inverse(ori_q))
            goal_modification_info['quaternion_rotation'] = {
                'index': qxyzw_idx,
                'value': rot_q,
            }

        result = train_dmp_model.run(one_shot_mat)

        model_info = { 
            'dmp_cmd_fields': dmp_cmd_fields,
            'model_info': result['model_info']
        }

        d = os.path.join(get_latest_model_folder(), os.path.basename(i))
        if not os.path.isdir(d):
            os.makedirs(d)
        dill.dump(
            goal_modification_info,
            open(os.path.join(d, 'goal_modification_info'), 'w')
        )
        dill.dump(
            result['model'],
            open(os.path.join(d, 'dmp_model'), 'w')
        )
        json.dump(
            model_info,
            open(os.path.join(d, 'dmp_model_info'), 'w'),
            indent=4,
        )

    fileHandler.close()
