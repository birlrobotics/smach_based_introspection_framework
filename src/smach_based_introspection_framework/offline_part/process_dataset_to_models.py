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
