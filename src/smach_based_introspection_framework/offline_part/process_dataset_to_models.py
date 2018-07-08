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
from model_training import train_introspection_model, train_anomaly_classifier, train_dmp_model
from smach_based_introspection_framework.offline_part import util 
import ipdb
import smach_based_introspection_framework.configurables as configurables 
import logging
import dill
import sys
import traceback

model_type  = configurables.model_type  
score_metric  = configurables.score_metric  
model_config  = configurables.model_config  

def get_latest_model_folder():
    if not os.path.isdir(latest_model_folder):
        os.makedirs(latest_model_folder)
    return latest_model_folder

def gen_introspection_model(logger, skill_folder, output_dir):
    model_file = os.path.join(output_dir, 'introspection_model')
    if os.path.isfile(model_file):
        logger.info("Model already exists. Gonna skip.")    
        return 
    csvs = glob.glob(os.path.join(
        skill_folder,
        '*',
    ))
    list_of_mat = []
    for j in csvs:
        df = pd.read_csv(j, sep=',')
        # Exclude 1st column which is time index
        list_of_mat.append(df.values[:, 1:])
    try:
        result = train_introspection_model.run(list_of_mat, model_type, model_config, score_metric, logger)
        logger.info("Successfully trained introspection model")
    except Exception as e:
        logger.error("Failed to train_introspection_model: %s"%e)
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
        open(os.path.join(output_dir, 'introspection_model_info'), 'w'),
        indent=4,
    )

def gen_classification_model(logger, anomaly_folder, output_dir):
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

def run():
    if not os.path.isdir(latest_dataset_folder):
        raise Exception("Not found %s"%latest_dataset_folder)

    logger = logging.getLogger('process_dataset_to_models')
    logger.setLevel(logging.DEBUG)
    log_file = os.path.join(get_latest_model_folder(), 'run.log')
    fileHandler = logging.FileHandler(os.path.realpath(log_file))
    fileHandler.setLevel(logging.DEBUG)
    logger.addHandler(fileHandler)

    skill_folders = glob.glob(os.path.join(
        latest_dataset_folder,
        'skill_data',
        "tag_*",
    ))
    prog = re.compile(r'tag_(.*)')
    for skill_folder in skill_folders:
        skill_id = prog.match(os.path.basename(skill_folder)).group(1)
        logger.debug("Processing skill %s"%skill_id)
        output_dir = os.path.join(
            get_latest_model_folder(), 
            'skill_%s'%skill_id,
        )
        gen_introspection_model(logger, skill_folder, output_dir)

    anomaly_folders = glob.glob(os.path.join(
        latest_dataset_folder,
        'anomaly_data',
        "anomaly_type_*",
    ))
    prog = re.compile(r'anomaly_type_(.*)')
    for anomaly_folder in anomaly_folders:
        m = prog.search(os.path.basename(anomaly_folder))
        anomaly_label = m.group(1)
        logger.debug("Processing anomaly %s"%(anomaly_label, ))

        output_dir = os.path.join(
            get_latest_model_folder(), 
            'anomaly_%s'%anomaly_label,
        )
        gen_classification_model(logger, anomaly_folder, output_dir)

    demonstration_folders = glob.glob(os.path.join(
        latest_dataset_folder,
        'demonstration_data',
        "nominal_skill_*_anomaly_type_*_tag_*",
    ))
    prog = re.compile(r'nominal_skill_(.*)_anomaly_type_(.*)_tag_(.*)')
    for demonstration_folder in demonstration_folders:
        m = prog.match(os.path.basename(demonstration_folder))
        skill_id = m.group(1)
        anomaly_label = m.group(2)
        demonstration_skill_id = m.group(3)
        logger.debug("Processing skill %s's anomaly %s's demonstration skill %s"%(skill_id, anomaly_label, demonstration_skill_id))
        # TODO: train dmp models

    sys.exit(1)

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
