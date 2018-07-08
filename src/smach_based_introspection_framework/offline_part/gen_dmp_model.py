import os
import glob
import pandas as pd
import json
import traceback
from model_training import train_dmp_model
import dill
import numpy as np
from smach_based_introspection_framework.offline_part import util 

def run(logger, demonstration_folder, output_dir):
    model_file = os.path.join(output_dir, 'dmp_model')
    if os.path.isfile(model_file):
        logger.info("Model already exists. Gonna skip.")    
        return 

    try:
        demonstration_trial = next(glob.iglob(os.path.join(demonstration_folder, "*")))
    except StopIteration:
        logger.error("No demonstration data in %s"%demonstration_folder) 
        return

    df = pd.read_csv(os.path.join(
        demonstration_trial,
        "dmp_cmd_timeseries.csv",
    ), sep=',')

    # Exclude 1st column which is time index
    df = df.drop(axis=1, columns=df.columns[0])

    cmd_matrix = df.values

    dem_goal = cmd_matrix[-1]
    with open(os.path.join(demonstration_trial, "original_goal.json"), 'r') as f:
        ori_goal = np.array(json.load(f))


    goal_info = {}
    goal_info['original_goal'] = ori_goal
    goal_info['demonstration_goal'] = dem_goal

    goal_modification_info = {}
    pxyz_idx = util.get_position_xyz_index(df.columns)
    if None not in pxyz_idx:
        goal_modification_info['translation'] = {
            'index': pxyz_idx,
            'value': dem_goal[pxyz_idx]-ori_goal[pxyz_idx],
        }

    qxyzw_idx = util.get_quaternion_xyzw_index(df.columns)
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

    one_shot_mat = cmd_matrix
    result = train_dmp_model.run(one_shot_mat)

    model_info = { 
        'dmp_cmd_fields': df.columns.tolist(),
        'model_info': result['model_info']
    }
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    dill.dump(
        goal_modification_info,
        open(os.path.join(output_dir, 'goal_modification_info'), 'w')
    )
    json.dump(
        model_info,
        open(os.path.join(output_dir, 'dmp_model_info'), 'w'),
        indent=4,
    )
    dill.dump(
        result['model'],
        open(model_file, 'w')
    )
