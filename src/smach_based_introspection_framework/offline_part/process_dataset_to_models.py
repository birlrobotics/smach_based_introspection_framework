import process_experiment_record_to_dataset as e2d
import glob
import os
from _constant import (
    folder_time_fmt,
    dataset_folder,
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

dmp_cmd_fields = [
    '.endpoint_state.pose.position.x',
    '.endpoint_state.pose.position.y',
    '.endpoint_state.pose.position.z',
    '.endpoint_state.pose.orientation.x',
    '.endpoint_state.pose.orientation.y',
    '.endpoint_state.pose.orientation.z',
    '.endpoint_state.pose.orientation.w',
]

data_fields_store = {
    "endpoint_pose": [
        '.endpoint_state.pose.position.x',
        '.endpoint_state.pose.position.y',
        '.endpoint_state.pose.position.z',
        '.endpoint_state.pose.orientation.x',
        '.endpoint_state.pose.orientation.y',
        '.endpoint_state.pose.orientation.z',
        '.endpoint_state.pose.orientation.w'
    ],
    'wrench': [
         '.wrench_stamped.wrench.force.x',
         '.wrench_stamped.wrench.force.y',
         '.wrench_stamped.wrench.force.z',
         '.wrench_stamped.wrench.torque.x',
         '.wrench_stamped.wrench.torque.y',
         '.wrench_stamped.wrench.torque.z',
    ] 
}
data_type_chosen = 'endpoint_pose_and_wrench'        
data_type_split = data_type_chosen.split("_and_")
interested_data_fields = []
for data_type in data_type_split:
    interested_data_fields += data_fields_store[data_type]
model_type = 'hmmlearn\'s HMM'
score_metric = '_score_metric_sum_of_loglik_'
model_config = {
    'hmm_max_train_iteration': 1000,
    'hmm_max_hidden_state_amount': 7,
    'gaussianhmm_covariance_type_string': ['diag', 'spherical', 'full', 'tied'],
}


def get_latest_model_dir():
    if hasattr(get_latest_model_dir, "latest_model_dir"):
        return get_latest_model_dir.latest_model_dir
    latest_model_dir = os.path.join(model_folder, 'latest')
    if os.path.isdir(latest_model_dir):
       shutil.move(
            latest_model_dir,  
            os.path.join(model_folder, "model_folder.old.%s"%datetime.datetime.now().strftime(folder_time_fmt))
        )
        
    os.makedirs(latest_model_dir)
    get_latest_model_dir.latest_model_dir = latest_model_dir
    return latest_model_dir

def run():
    latest_dataset_dir = os.path.join(dataset_folder, 'latest')
    if not os.path.isdir(latest_dataset_dir):
        raise Exception("Not found %s"%latest_dataset_dir)

    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_dir, "skill_data", '*')):
        tag = prog.match(os.path.basename(i)).group(1)

        list_of_mat = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_mat.append(df[interested_data_fields].values)

        try:
            result = train_introspection_model.run(list_of_mat, model_type, model_config, score_metric)
        except Exception as e:
            print "Failed to train introspection model for tag %s: %s"%(tag, e)  
            continue
        print "Successfully trained introspection model for tag %s"%(tag)  
    
            
        d = os.path.join(get_latest_model_dir(), 'tag_%s'%tag)
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

    prog = re.compile(r'nominal_skill_(\d+)_anomaly_type_(.*)')
    for i in glob.glob(os.path.join(latest_dataset_dir, "anomaly_data", '*')):
        m = prog.match(os.path.basename(i))
        tag = m.group(1)
        anomaly_type = m.group(2)
        print tag, anomaly_type
        if anomaly_type == 'Unlabeled':
            print "Skip Unlabeled anomalies in skill %s"%tag
            continue
        list_of_mat = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_mat.append(df[interested_data_fields].values)
        try:
            result = train_anomaly_classifier.run(list_of_mat, model_type, model_config, score_metric)
        except Exception as e:
            print "Failed to train train_anomaly_classifier for anomaly_type %s in skill %s: %s"%(anomaly_type, tag, e)  
            continue
        print "Successfully trained classifier for anomaly type %s in skill %s"%(anomaly_type, tag)  

        d = os.path.join(get_latest_model_dir(), os.path.basename(i))
        if not os.path.isdir(d):
            os.makedirs(d)
        joblib.dump(
            result['model'],
            os.path.join(d, 'classifer_model')
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


    prog = re.compile(r'tag_(\d+)')
    for i in glob.glob(os.path.join(latest_dataset_dir, "skill_data", '*')):
        tag = prog.match(os.path.basename(i)).group(1)
        if int(tag) < RECOVERY_SKILL_BEGINS_AT:
            print "Skip nominal skill %s"%tag
            continue
        print "Gonna build dmp model for skill %s"%tag
        list_of_mat = []
        for j in glob.glob(os.path.join(i, "*")):
            df = pd.read_csv(j, sep=',')
            list_of_mat.append(df[dmp_cmd_fields].values)



        basis_weight, basis_function_type = birl_baxter_dmp.dmp_train.train(list_of_mat)
        print "Built dmp model for skill %s"%tag
        model = {
            "basis_weight": basis_weight,
            "basis_function_type": basis_function_type,
        }
        d = os.path.join(get_latest_model_dir(), os.path.basename(i))
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









